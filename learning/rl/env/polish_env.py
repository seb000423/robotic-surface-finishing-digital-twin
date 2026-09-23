"""PolishEnv — Isaac Lab DirectRLEnv 폴리싱 환경.

구성:
  기준 제어기  = raster 경로 추종 + 가상 스프링·어드미턴스 힘제어 (contact.py, 검증 완료)
  잔차 action  = [Δforce_ratio, Δfeed_ratio] ∈ [-1,1]² — action=0 이면 기준 제어기 그대로
  품질 모델    = LiteraturePolishingModel (제거·Ra/Rz·scratch·clearcoat, 3μm 앵커)
  process ctx = BO recipe JSON

한 control step:
  physics substep(60Hz)마다 어드미턴스 힘 적분 → control step(20Hz) 끝에 평균 힘 산출
  → PolishingModel 1회 갱신 → reward/observation.
"""
from __future__ import annotations

import json
from collections.abc import Sequence

import numpy as np
import torch

import isaaclab.sim as sim_utils
from isaaclab.envs import DirectRLEnv
from isaaclab.sim.spawners.from_files import GroundPlaneCfg, spawn_ground_plane

from learning.digital_twin import config as PC
from learning.digital_twin.gloss_proxy import LiteratureGlossProxyModel
from learning.digital_twin.path_executor import Recipe, load_calibrated_config, raster_waypoints
from learning.digital_twin.polishing_model import ContactState, LiteraturePolishingModel
from learning.digital_twin.surface_state import (curve_height_normal, make_curved_patch,
                                              make_flat_patch)

from .contact import VirtualPadContact
from .polish_env_cfg import PolishEnvCfg


def _load_recipe(path: str) -> Recipe:
    try:
        with open(path, encoding="utf-8") as f:
            d = json.load(f)
        return Recipe(
            target_contact_force_n=float(d["target_contact_force_n"]),
            feed_speed_mm_s=float(d["feed_speed_mm_s"]),
            rpm=float(d["rpm"]),
            step_over_spacing_ratio=float(d["step_over_spacing_ratio"]),
            n_passes=int(d.get("n_passes", 1)),
        )
    except FileNotFoundError:
        # 폴백: reference 기준 recipe
        # 조용히 폴백하면 새 클론에서 BO recipe(5.78N) 대신 8N 으로 학습·판정하게 된다
        #   반드시 눈에 띄게 경고한다 (bo_runner 재실행 또는 recipe JSON 복원이 정공법).
        print(f"\n{'!' * 70}\n"
              f"[PolishEnv] recipe JSON 없음: {path}\n"
              f"[PolishEnv] REFERENCE 폴백(8.0N/5mm/s/1pass)으로 실행 — BO recipe 와 다르다!\n"
              f"{'!' * 70}\n")
        r = PC.REFERENCE
        return Recipe(r.target_force_n, r.feed_speed_mm_s, r.rpm_schedule[0],
                      r.step_over_spacing_ratio, n_passes=1)


class PolishEnv(DirectRLEnv):
    cfg: PolishEnvCfg

    def __init__(self, cfg: PolishEnvCfg, render_mode: str | None = None, **kwargs):
        super().__init__(cfg, render_mode, **kwargs)
        E = self.num_envs

        self.recipe = _load_recipe(cfg.recipe_json_path)
        self.quality_dt = cfg.sim.dt * cfg.decimation           # 20 Hz

        # 경로: 전 env 공통 raster (표면 seed 만 다름)
        spacing = self.recipe.step_over_spacing_ratio * PC.PAD_DIAMETER_M
        lines = raster_waypoints(cfg.patch_size_m, spacing)
        pts, line_len = [], []
        for p0, p1, _ in lines:
            pts.append((p0, p1))
            line_len.append(float(np.hypot(p1[0] - p0[0], p1[1] - p0[1])))
        self._lines = pts
        self._line_len = np.array(line_len)
        self._path_len = float(self._line_len.sum()) * self.recipe.n_passes

        # 접촉 (torch, 병렬) — 새 환경은 RMPFlow 가 없으므로 lag_offset=0 (완전 추종)
        self.contact = VirtualPadContact(E, self.device, dt=cfg.sim.dt, lag_offset=0.0)
        self._is_side = torch.zeros(E, dtype=torch.bool, device=self.device)
        n_side = int(round(E * cfg.side_env_ratio))
        if n_side > 0:                      # 앞쪽 n_side 개 env 를 side 접촉으로 (결정적 배정)
            self._is_side[:n_side] = True
        if getattr(cfg, "carcell_is_side", None):
            self._is_side[:] = torch.tensor([bool(v) for v in cfg.carcell_is_side][:E],
                                            dtype=torch.bool, device=self.device)
            print(f"[PolishEnv] 차 셀 모드 side env {int(self._is_side.sum())}/{E}")
            print(f"[PolishEnv] side 접촉 env {n_side}/{E} (side_env_ratio={cfg.side_env_ratio})")

        # 품질 모델 (numpy, env 별)
        self._cal = load_calibrated_config()
        self._model = LiteraturePolishingModel(self._cal)
        self._gloss = LiteratureGlossProxyModel()
        self._surfaces = [None] * E
        self._episode_count = np.zeros(E, dtype=int)
        # 종말 보상용 전/후 품질 (env 별). last_episode_results 는 판정 스크립트가 읽는다.
        self._before_metrics: dict[int, dict] = {}
        self._final_metrics: dict[int, dict] = {}
        self.last_episode_results: dict[int, dict] = {}

        # 진행 상태
        self._arc = torch.zeros(E, device=self.device)          # 경로 누적 arc length [m]
        self._sim_time = torch.zeros(E, device=self.device)
        self._prev_action = torch.zeros(E, 2, device=self.device)
        self._prev_force = torch.zeros(E, device=self.device)
        self._force_hard_violated = torch.zeros(E, dtype=torch.bool, device=self.device)
        self._force_accum = torch.zeros(E, device=self.device)
        self._force_mean = torch.zeros(E, device=self.device)
        self._force_cmd = torch.full((E,), self.recipe.target_contact_force_n, device=self.device)
        self._feed_cmd = torch.full((E,), self.recipe.feed_speed_mm_s / 1000.0, device=self.device)
        self._action_rate = torch.zeros(E, device=self.device)
        self._defect_removal = torch.zeros(E, device=self.device)
        self._healthy_over = torch.zeros(E, device=self.device)
        self._thermal_damage_delta = torch.zeros(E, device=self.device)
        self._thermal_hard_violated = torch.zeros(E, dtype=torch.bool, device=self.device)
        self._substep_n = 0
        # 로그
        self.step_log: list[dict] = []
        self.log_raw_steps = False

    # ── scene ─────────────────────────────────────────────────────────────
    def _setup_scene(self):
        spawn_ground_plane(prim_path="/World/ground", cfg=GroundPlaneCfg())
        self.scene.clone_environments(copy_from_source=False)
        light = sim_utils.DomeLightCfg(intensity=2000.0, color=(0.8, 0.8, 0.8))
        light.func("/World/Light", light)

    # ── helpers ───────────────────────────────────────────────────────────
    def _pos_at_arc(self, arc_m: float) -> tuple:
        """누적 arc length → patch 국소 (u,v). 경로 끝이면 마지막 점."""
        one_pass = float(self._line_len.sum())
        a = arc_m % one_pass if arc_m < self._path_len else one_pass - 1e-9
        for (p0, p1), L in zip(self._lines, self._line_len):
            if a <= L:
                t = a / L
                return (p0[0] + (p1[0] - p0[0]) * t, p0[1] + (p1[1] - p0[1]) * t)
            a -= L
        return self._lines[-1][1]

    def _quality_uv(self, i: int, arc_m: float) -> tuple[float, float]:
        """Quality-model pad center. Robot-coupled env overrides this with measured pad pose."""
        return self._pos_at_arc(arc_m)

    def _footprint_stats(self, i: int, uv) -> tuple:
        """관측용 국소 crop 통계.

        반경은 패드의 절반(코어)이다 — 전체 footprint(Ø110mm)는 patch(120mm)를 거의
          다 덮어 "스크래치 없음" 신호가 사실상 안 나온다 (3차 학습 진단: off-scratch
          스텝 12000 중 34개). 제거가 gaussian 중심에 집중되므로 코어 감지가 물리와도 맞다.
        scratch 는 정적 초기맵이 아니라 잔여 추정치 (초기 − 누적제거, clip 0)
          이미 지운 자리를 계속 "스크래치 있음"으로 보고하지 않게.
        """
        s = self._surfaces[i]
        res, R = s.resolution_m, 0.5 * PC.PAD_RADIUS_M
        i0 = max(int((uv[0] - R) / res), 0); i1 = min(int((uv[0] + R) / res) + 1, s.shape[0])
        j0 = max(int((uv[1] - R) / res), 0); j1 = min(int((uv[1] + R) / res) + 1, s.shape[1])
        if i0 >= i1 or j0 >= j1:
            return 0.0, 0.0, 0.0, 20.0, PC.AMBIENT_TEMPERATURE_C, PC.AMBIENT_TEMPERATURE_C, 0.0
        sl = (slice(i0, i1), slice(j0, j1))
        remaining = np.clip(s.initial_scratch_depth_um[sl] - s.cumulative_removal_um[sl],
                            0.0, None)
        return (float(remaining.mean()), float(remaining.max()),
                float(s.cumulative_removal_um[sl].mean()),
                float(s.clearcoat_remaining_um[sl].min() - self.cfg.clearcoat_safety_limit_um),
                float(s.temperature_c[sl].mean()),
                float(s.peak_temperature_c[sl].max()),
                float(s.thermal_damage_proxy[sl].mean()))

    def _remaining_crop(self, i: int, uv, R: float):
        """uv 주변 반경 R crop 의 잔여 scratch (mean, max) — lookahead 용 경량 판."""
        s = self._surfaces[i]
        res = s.resolution_m
        i0 = max(int((uv[0] - R) / res), 0); i1 = min(int((uv[0] + R) / res) + 1, s.shape[0])
        j0 = max(int((uv[1] - R) / res), 0); j1 = min(int((uv[1] + R) / res) + 1, s.shape[1])
        if i0 >= i1 or j0 >= j1:
            return 0.0, 0.0
        sl = (slice(i0, i1), slice(j0, j1))
        rem = np.clip(s.initial_scratch_depth_um[sl] - s.cumulative_removal_um[sl], 0.0, None)
        return float(rem.mean()), float(rem.max())

    def _lookahead_stats(self, i: int, arc: float):
        """경로 진행 방향 예견: near(0~1 패드반경)/far(1~3 반경)
        구간을 각 3점 샘플해 잔여 scratch mean/max. 정책이 감속을 선제적으로 걸 수 있게."""
        R = 0.5 * PC.PAD_RADIUS_M
        out = []
        for d0, d1 in ((0.0, PC.PAD_RADIUS_M), (PC.PAD_RADIUS_M, 3.0 * PC.PAD_RADIUS_M)):
            means, maxs = [], []
            for t in range(3):
                a = min(arc + d0 + (d1 - d0) * (t + 0.5) / 3.0, self._path_len)
                m, mx = self._remaining_crop(i, self._pos_at_arc(a), R)
                means.append(m); maxs.append(mx)
            out += [float(np.mean(means)), float(np.max(maxs))]
        return out          # [near_mean, near_max, far_mean, far_max]

    def _evaluate_quality(self, i: int) -> dict:
        """표면 i 의 품질 요약 (SYNTHETIC — 논문 기반 GU proxy). 전/후 공용."""
        q = self._model.evaluate(self._surfaces[i])
        g = self._gloss.evaluate(self._surfaces[i])["summary"]
        return {"gu": float(g["gu_mean"]), "scratch": float(q["max_residual_scratch_um"]),
                "ra": float(q["ra_um"]), "rz": float(q["rz_um"]),
                "cc_min": float(q["clearcoat_min_um"]),
                "temperature_mean_c": float(q["temperature_mean_c"]),
                "temperature_peak_c": float(q["temperature_peak_c"]),
                "thermal_damage_mean": float(q["thermal_damage_mean"]),
                "thermal_damage_peak": float(q["thermal_damage_peak"])}

    # ── DirectRLEnv hooks ─────────────────────────────────────────────────
    def _pre_physics_step(self, actions: torch.Tensor) -> None:
        a = torch.clamp(actions, -1.0, 1.0)
        self._action_rate = (a - self._prev_action).square().mean(dim=1)
        self._prev_action = a.clone()
        # 임피던스형 잔차: recipe 원본은 그대로, 배율만 보정
        self._force_cmd = self.recipe.target_contact_force_n * (
            1.0 + a[:, 0] * self.cfg.force_ratio_limit)
        self._feed_cmd = (self.recipe.feed_speed_mm_s / 1000.0) * (
            1.0 + a[:, 1] * self.cfg.feed_ratio_limit)
        self._force_accum = torch.zeros(self.num_envs, device=self.device)
        self._substep_n = 0

    def _apply_action(self) -> None:
        """physics substep(60Hz)마다: 어드미턴스 힘 적분 + 경로 전진."""
        f = self.contact.step(self._force_cmd, self._is_side)
        self._force_accum += f
        self._substep_n += 1
        self._arc += self._feed_cmd * self.cfg.sim.dt
        self._sim_time += self.cfg.sim.dt

    def _quality_update(self):
        """control step 끝: 평균 힘으로 품질 모델 1회 갱신.

        호출 위치가 정확성의 핵심이다. DirectRLEnv.step 훅 순서는
           _get_dones → _get_rewards → _reset_idx → _get_observations 이므로
           여기 계산은 _get_dones 첫머리에서 수행한다. (수정 전엔 _get_observations
           에서 수행 → ① 보상이 직전 스텝의 품질/힘으로 계산되는 1-스텝 지연,
           ② _reset_idx 직후 호출되어 막 생성된 새 표면을 이전 에피소드의 힘으로
           1회 연마하는 오염이 있었다.)
        """
        if self._substep_n == 0:      # env.reset() 직후 관측 경로 방어 — 아직 폴리싱 전
            return
        force_mean = self._force_accum / self._substep_n
        self._force_hard_violated |= force_mean > self.cfg.force_hard_limit_n
        arcs = self._arc.cpu().numpy()
        times = self._sim_time.cpu().numpy()
        forces = force_mean.cpu().numpy()
        feeds = self._feed_cmd.cpu().numpy()
        defect_rm = np.zeros(self.num_envs, dtype=np.float32)
        healthy_over = np.zeros(self.num_envs, dtype=np.float32)
        thermal_damage_delta = np.zeros(self.num_envs, dtype=np.float32)
        for i in range(self.num_envs):
            uv = self._quality_uv(i, float(arcs[i]))
            out = self._model.step(self._surfaces[i], ContactState(
                pad_center_uv_m=uv,
                contact_force_n=float(forces[i]),       # 달성 힘 — 명령이 아니라 (핵심)
                rpm=self.recipe.rpm,
                feed_speed_m_s=float(feeds[i]),
            ), dt_s=self.quality_dt, sim_time_s=float(times[i]))
            defect_rm[i] = out.get("defect_removal_um", 0.0)
            healthy_over[i] = out.get("healthy_over_removal_um", 0.0)
            thermal_damage_delta[i] = out.get("thermal_damage_delta_mean", 0.0)
            if bool(self._surfaces[i].peak_temperature_c.max()
                    > self.cfg.thermal_hard_limit_c):
                self._thermal_hard_violated[i] = True
        self._defect_removal = torch.as_tensor(defect_rm, device=self.device)
        self._healthy_over = torch.as_tensor(healthy_over, device=self.device)
        self._thermal_damage_delta = torch.as_tensor(thermal_damage_delta, device=self.device)
        self._force_mean = force_mean
        if self.log_raw_steps:
            self.step_log.append({
                "t": float(times[0]), "arc": float(arcs[0]),
                "force_cmd": float(self._force_cmd[0]), "force_meas": float(forces[0]),
                "z_offset": float(self.contact.z_offset[0]),
            })

    def _get_observations(self) -> dict:
        # 품질 갱신은 _get_dones 에서 이미 수행됨 — 여기서는 결과를 읽기만 한다.
        E = self.num_envs
        arcs = self._arc.cpu().numpy()
        stats = np.zeros((E, 7), dtype=np.float32)
        for i in range(E):
            # 관측용 잔존 scratch = 초기맵 − 누적제거 근사 (_footprint_stats 참고).
            # evaluate() 의 기하학적 valley 재계산은 무거워서 종료 시에만 수행한다.
            stats[i] = self._footprint_stats(i, self._quality_uv(i, float(arcs[i])))
        st = torch.as_tensor(stats, device=self.device)
        progress = (self._arc / self._path_len).clamp(0, 1).unsqueeze(1)
        f = self._force_mean.unsqueeze(1)
        obs = torch.cat([
            f / 10.0,
            (f - self._force_cmd.unsqueeze(1)) / 5.0,
            (f - self._prev_force.unsqueeze(1)) / 5.0,
            self._feed_cmd.unsqueeze(1) * 100.0,
            progress,
            st[:, 0:1] / 2.0, st[:, 1:2] / 2.0,     # 국소 scratch mean/max [μm]
            st[:, 2:3] / 5.0,                        # 국소 누적 제거 [μm]
            st[:, 3:4] / 20.0,                       # clearcoat 안전여유 [μm]
            self._prev_action,
        ] + ([
            (st[:, 4:5] - PC.AMBIENT_TEMPERATURE_C) / 40.0,  # 국소 현재온도 상승
            (st[:, 5:6] - PC.AMBIENT_TEMPERATURE_C) / 60.0,  # 국소 최고온도 상승
            st[:, 6:7] / PC.THERMAL_DAMAGE_MAX,               # 누적 열손상
        ] if self.cfg.use_thermal_obs else []) + ([
            torch.as_tensor(np.array(
                [self._lookahead_stats(i, float(arcs[i])) for i in range(E)],
                dtype=np.float32), device=self.device) / 2.0,  # lookahead 잔여 scratch /2
        ] if self.cfg.use_spatial_obs else []), dim=1)
        self._prev_force = self._force_mean.clone()
        return {"policy": obs}

    def _get_rewards(self) -> torch.Tensor:
        cfg = self.cfg
        force_err = (self._force_mean - self._force_cmd).abs() / cfg.force_tolerance_n
        r_force = -torch.clamp(force_err, 0.0, 2.0)
        # 품질항은 delta 보상:
        #   defect 셀 위 제거 = 유익 (스크래치를 실제로 깎는 중)
        #   healthy 셀 허용치 초과 제거 = 손상 (되돌릴 수 없음 → 더 큰 가중)
        r = (cfg.w_force * r_force
             + cfg.w_defect_removal * self._defect_removal
             - cfg.w_healthy_over * self._healthy_over
             - cfg.w_thermal_damage * self._thermal_damage_delta
             - cfg.w_action_rate * self._action_rate
             - cfg.w_time)

        # ── 종말 보상: 논문 기반 최종 GU proxy (실측 GU 아님 — cfg 주석 참고) ──
        # DirectRLEnv.step() 은 _get_dones → _get_rewards → _reset_idx 순서이므로
        # 여기서 reset_buf(종료 env)가 유효하고, 표면은 아직 리셋 전이다.
        if cfg.use_terminal_reward and self.reset_buf.any():
            terms = []
            for i in self.reset_buf.nonzero(as_tuple=False).squeeze(-1).cpu().tolist():
                fin = self._evaluate_quality(i)
                self._final_metrics[i] = fin
                b = self._before_metrics.get(i)
                if b is None:
                    continue
                cc_ok = fin["cc_min"] >= cfg.clearcoat_safety_limit_um
                thermal_safe = fin["temperature_peak_c"] <= cfg.thermal_hard_limit_c
                scr_improved = fin["scratch"] < b["scratch"] or b["scratch"] < 0.05
                all_pass = (fin["gu"] >= 70.0
                            and fin["ra"] <= cfg.t_ra_pass_max_um
                            and fin["rz"] <= cfg.t_rz_pass_max_um
                            and cc_ok and thermal_safe and scr_improved)
                rt = (cfg.t_gu_final * (fin["gu"] - 70.0) / 10.0
                      + cfg.t_gu_delta * (fin["gu"] - b["gu"]) / 10.0
                      + cfg.t_scratch * (b["scratch"] - fin["scratch"]) / 2.0
                      + cfg.t_ra * (b["ra"] - fin["ra"]) / 0.20
                      + cfg.t_rz * (b["rz"] - fin["rz"]) / 2.0)
                rt -= cfg.t_cc_use * max(b["cc_min"] - fin["cc_min"], 0.0)
                rt -= cfg.t_thermal_damage * fin["thermal_damage_peak"]
                overheat_span = max(80.0 - PC.THERMAL_PROFILE_TG_C, 1.0)
                rt -= cfg.t_overheat * max(
                    0.0, fin["temperature_peak_c"] - PC.THERMAL_PROFILE_TG_C) / overheat_span
                if not cc_ok:
                    rt -= cfg.t_cc_fail
                if all_pass:
                    rt += cfg.t_pass_bonus
                r[i] += rt
                terms.append(rt)
                self.last_episode_results[i] = {
                    "before": b, "after": fin, "terminal_reward": float(rt),
                    "all_pass": bool(all_pass), "cc_ok": bool(cc_ok),
                    "scr_improved": bool(scr_improved),
                    "thermal_safe": bool(thermal_safe)}
            if terms:
                log = self.extras.setdefault("log", {})
                log["Episode/terminal_reward"] = float(np.mean(terms))
        return r

    def _get_dones(self) -> tuple[torch.Tensor, torch.Tensor]:
        # step 훅 중 가장 먼저 불린다 — 이번 스텝의 품질/힘을 여기서 확정해야
        #   _get_rewards 가 같은 스텝의 결과로 계산된다 (_quality_update docstring 참고).
        self._quality_update()
        done_path = self._arc >= self._path_len
        died = self._force_hard_violated | self._thermal_hard_violated
        time_out = self.episode_length_buf >= self.max_episode_length - 1
        return died | done_path, time_out

    def _quad_coeffs(self, i: int):
        q = getattr(self.cfg, "carcell_quads", None)
        return (q[i] if q is not None and i < len(q) else None)

    def _make_quad_surface(self, i: int, seed: int):
        """차 셀 모드: 셀 2차곡면 + 셀 초기 상태(ra·scratch·n_scr·clearcoat) — 판정 파이프라인의
        synthesize_cell_patch 와 같은 스케일링을 곡면 패치에 적용."""
        init = (self.cfg.carcell_init[i] if getattr(self.cfg, "carcell_init", None) else None)
        ra = float(init["ra"]) if init else None
        n_scr = int(init["n_scr"]) if init and init.get("n_scr") is not None else None
        kw = {"target_ra_um": ra} if ra is not None else {}
        st = make_curved_patch("quad", 0.0, self.cfg.patch_size_m, self.cfg.patch_resolution_m,
                               seed=int(init["seed"]) if init and init.get("seed") is not None else seed,
                               n_scratches=n_scr, with_scratches=True,
                               quad_coeffs=self._quad_coeffs(i), **kw)
        if init:
            scr = float(init["scratch"]); cc = float(init["clearcoat"])
            if scr > 0.0 and st.initial_scratch_depth_um.max() > 1e-9:
                k = scr / float(st.initial_scratch_depth_um.max())
                st.micro_height_um += (1.0 - k) * st.initial_scratch_depth_um
                st.initial_micro_height_um = st.micro_height_um.copy()
                st.initial_scratch_depth_um *= k
                st.residual_scratch_depth_um = st.initial_scratch_depth_um.copy()
                st.defect_mask = st.initial_scratch_depth_um > (0.5 * PC.SCRATCH_DEPTH_MIN_UM)
                st.healthy_mask = ~st.defect_mask
            st.clearcoat_remaining_um += cc - float(st.clearcoat_remaining_um.mean())
            st.initial_clearcoat_um = st.clearcoat_remaining_um.copy()
        return st

    def _reset_idx(self, env_ids: Sequence[int] | None):
        if env_ids is None:
            env_ids = torch.arange(self.num_envs, device=self.device)
        ids = torch.as_tensor(env_ids, device=self.device).long()

        # 종료 직전 품질 요약을 extras 로 ( episode summary).
        # 종말 보상 경로(_get_rewards)에서 이미 평가했으면 재사용 — 이중 evaluate 방지.
        log = self.extras.setdefault("log", {})
        gus, scratches = [], []
        for i in ids.cpu().tolist():
            if self._surfaces[i] is not None:
                fin = self._final_metrics.pop(i, None) or self._evaluate_quality(i)
                gus.append(fin["gu"]); scratches.append(fin["scratch"])
        if gus:
            log["Metrics/gu_mean"] = float(np.mean(gus))
            log["Metrics/max_residual_scratch_um"] = float(np.mean(scratches))

        super()._reset_idx(ids)

        for i in ids.cpu().tolist():
            seed = self.cfg.surface_seed_base + 97 * i + int(self._episode_count[i])
            if self.cfg.surface_kind == "flat":
                self._surfaces[i] = make_flat_patch(
                    self.cfg.patch_size_m, self.cfg.patch_resolution_m,
                    seed=seed, with_scratches=True)
            elif self.cfg.surface_kind == "quad":
                self._surfaces[i] = self._make_quad_surface(i, seed)
            else:
                self._surfaces[i] = make_curved_patch(
                    self.cfg.surface_kind, self.cfg.curvature_radius_m,
                    self.cfg.patch_size_m, self.cfg.patch_resolution_m,
                    seed=seed, with_scratches=True)
                if self.cfg.surface_kind == "freeform":
                    # 형상은 고정(충돌 메시와 일치), 미세층만 에피소드 seed — 형상 seed 를
                    # cfg.freeform_seed 로 재생성해 nominal/법선을 덮어쓴다.
                    ref = make_curved_patch("freeform", 0.0, self.cfg.patch_size_m,
                                            self.cfg.patch_resolution_m,
                                            seed=self.cfg.freeform_seed,
                                            with_scratches=False)
                    self._surfaces[i].nominal_surface_xyz_m = ref.nominal_surface_xyz_m
                    self._surfaces[i].normal_xyz = ref.normal_xyz
            self._episode_count[i] += 1
            # 새 에피소드의 "전" 품질 — 종말 보상의 Δ(개선량) 기준점 (같은 에피소드 전·후)
            if self.cfg.use_terminal_reward:
                self._before_metrics[i] = self._evaluate_quality(i)
        self.contact.reset(ids, is_side=self._is_side)
        self._arc[ids] = 0.0
        self._sim_time[ids] = 0.0
        self._prev_action[ids] = 0.0
        self._prev_force[ids] = 0.0
        self._force_hard_violated[ids] = False
        self._thermal_hard_violated[ids] = False
        # 이전 에피소드 잔재가 새 에피소드의 관측/보상에 새지 않게 env 별로 청소.
        # (_quality_update 를 _get_dones 로 옮겨 표면 오염은 구조적으로 사라졌지만,
        #  관측의 힘 항과 보상 델타 항은 버퍼값을 읽으므로 명시적으로 0 이어야 한다.)
        self._force_mean[ids] = 0.0
        self._force_accum[ids] = 0.0
        self._defect_removal[ids] = 0.0
        self._healthy_over[ids] = 0.0
        self._thermal_damage_delta[ids] = 0.0
        self._action_rate[ids] = 0.0
        self._force_cmd[ids] = self.recipe.target_contact_force_n
        self._feed_cmd[ids] = self.recipe.feed_speed_mm_s / 1000.0
