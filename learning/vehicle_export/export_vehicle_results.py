"""차량 검사 시스템 연동 — 150셀 BC 정책 추론 + 논문 기반 물리모델 실행 → 결과 CSV.

    ~/isaacsim/kit/python/bin/python3 learning/vehicle_export/export_vehicle_results.py \
        --checkpoint learning/rl/champion/model_bc.pt \
        --input  learning/vehicle_export/vehicle_150_cells.csv \
        --output learning/vehicle_export/rl_vehicle_150_results.csv \
        [--workers 8] [--limit N]

Isaac Sim 앱 없이 돈다 (torch + rsl_rl + numpy 만). 셀마다:
  입력 초기상태(Ra·scratch·clearcoat)에 맞춘 대표 patch(120×120 mm) 합성
  → BC 정책이 20 Hz 로 잔차 [Δforce, Δfeed] 출력
  → 검증된 가상스프링·어드미턴스 접촉(60 Hz) + LiteraturePolishingModel 로 에피소드 실행
  → 전/후 품질(Rq·scratch·Ra·Rz·clearcoat·GU proxy) + 판정 출력.

정확성 계약:
  · 제어 루프의 모든 식은 learning/rl/env/polish_env.py 와 동일해야 한다 (검증:
    tests 의 obs/명령 대조). 여기 수치를 바꾸면 폴백이 아니라 버그다.
  · 모든 품질 수치는 SYNTHETIC — 논문 근거 모델 출력이지 실측 아님.
  정책은 120×120 mm 평면 patch 학습본. 차량 곡면/수직면 적용은 추론일 뿐
    검증된 곡면 일반화가 아니다.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import os
import sys

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
_REPO = os.path.dirname(os.path.dirname(_HERE))
sys.path.insert(0, _REPO)

import torch                                             # noqa: E402
torch.set_num_threads(1)                                 # multiprocessing 과 경합 방지

from learning.digital_twin import config as PC               # noqa: E402
from learning.digital_twin.gloss_proxy import TARGET_GU, LiteratureGlossProxyModel  # noqa: E402
from learning.digital_twin.path_executor import (            # noqa: E402
    Recipe, load_calibrated_config, raster_waypoints)
from learning.digital_twin.polishing_model import ContactState, LiteraturePolishingModel  # noqa: E402
from learning.digital_twin.roughness_metrics import (        # noqa: E402
    _detrend, ra_um, residual_scratch_depth_um, rz_um)
from learning.digital_twin.surface_state import make_flat_patch  # noqa: E402
from learning.rl.env.contact import VirtualPadContact    # noqa: E402

RECIPE_JSON = os.path.join(_REPO, "learning", "digital_twin", "outputs", "bo_best_recipe.json")
DEFAULT_CKPT = os.path.join(_REPO, "learning", "rl", "champion",
                            "model_terminal_ppo_14ch_it800.pt")   #  챔피언


def policy_label(ckpt_path: str) -> tuple:
    """(사람용 표기, evaluation_mode) — 체크포인트에서 유도해 표기가 어긋나지 않게 한다."""
    name = os.path.basename(ckpt_path)
    if "terminal_ppo" in name:
        return ("BC 부트스트랩 + 종말보상 PPO (논문 기반 최종 GU proxy 종말 보상)",
                "flat_trained_bc_ppo_terminal_policy_inference")
    return ("BC (behavior cloning of handmade dwell policy) — PPO 아님",
            "flat_trained_bc_policy_inference")

# ── 판정 기준 — literature-derived project target ─────────────────────────
# 4종 모두 논문 기반 디지털 트윈 판정값 (제공된 자동차 도장·연마 논문의 실험 결과를
# 근거로 프로젝트가 채택한 문헌 기반 목표값). 프로젝트 자체 실측·보정값이 아니다.
# 원칙: 결과가 기준에 미달해도 목표값을 낮추지 않는다 — 그대로 실패로 판정한다.
GU_PASS_MIN = 70.0                       # 20° GU proxy ≥ 70 ( 목표)
CLEARCOAT_SAFE_MIN_UM = PC.CLEARCOAT_SAFETY_LIMIT_UM     # 잔여 ≥ 35 μm — 프로젝트 통일 기준
RA_PASS_MAX_UM = 0.20                    # Ra ≤ 0.20 μm — literature-derived project target
RZ_PASS_MAX_UM = 2.0                     # Rz ≤ 2.0 μm — literature-derived project target

# ── 제어 상수 (polish_env_cfg 와 동일값) ──────────────────────────────────
PHYSICS_DT = 1.0 / 60.0
DECIMATION = 3                           # 20 Hz control
FORCE_RATIO_LIMIT = 0.3
FEED_RATIO_LIMIT = 0.5
FORCE_HARD_LIMIT_N = 14.0
MAX_CONTROL_STEPS = 10000                # 500 s
PATCH_SIZE = (0.12, 0.12)               # 학습 patch 와 동일한 대표 patch
RESOLUTION_M = 0.002

INPUT_COLUMNS = [
    "region_id", "region_name", "cell_id",
    "position_x_m", "position_y_m", "position_z_m",
    "normal_x", "normal_y", "normal_z",
    "init_roughness_um", "init_scratch_um", "init_ra_um", "init_rz_um",
    "init_clearcoat_um", "init_gu_proxy", "surface_seed", "init_scratch_count",
]

OUTPUT_COLUMNS = [
    # 셀 식별
    "region_id", "region_name", "cell_id",
    "position_x_m", "position_y_m", "position_z_m",
    "normal_x", "normal_y", "normal_z",
    # 정책·공정 조건
    "force_n", "rpm", "feed_speed_mm_s", "step_over_ratio", "pass_count",
    "policy_action_force", "policy_action_feed",
    # 전후 품질
    "roughness_before", "roughness_after",
    "scratch_before_um", "scratch_after_um",
    "ra_before_um", "ra_after_um", "rz_before_um", "rz_after_um",
    "clearcoat_initial_um", "clearcoat_removed_um",
    "clearcoat_remaining_um", "clearcoat_remaining_min_um",
    "gu_proxy_before", "gu_proxy_after",
    # 판정
    "gu_target_pass", "ra_target_pass", "rz_target_pass", "warranty_removal_ok",
    "scratch_improved", "clearcoat_safe", "overall_pass", "failure_reason", "disposition",
    # 추적·한계 명시
    "tilt_deg", "is_side", "evaluation_mode", "surface_seed",
    "process_time_s", "episode_completed", "repolish_episodes",
]

# evaluation_mode 는 policy_label 이 체크포인트에서 유도한다
EVALUATION_MODE = "flat_trained_policy_inference"


def rq_um(height_um: np.ndarray) -> float:
    """전역 Rq (RMS, detrend 후) — roughness_before/after 열의 정의."""
    z = _detrend(height_um)
    return float(np.sqrt(((z - z.mean()) ** 2).mean()))


def load_recipe(path: str = RECIPE_JSON) -> Recipe:
    with open(path, encoding="utf-8") as f:      # 폴백 없음 — 파일 필수 (조용한 기본값 금지)
        d = json.load(f)
    return Recipe(float(d["target_contact_force_n"]), float(d["feed_speed_mm_s"]),
                  float(d["rpm"]), float(d["step_over_spacing_ratio"]),
                  n_passes=int(d.get("n_passes", 1)))


def load_bc_policy(ckpt_path: str, obs_dim: int | None = None, act_dim: int = 2):
    """rsl_rl checkpoint(actor_state_dict)에서 actor 재구성 — demo_arm._load_policy 와 동일.

    obs_dim 은 체크포인트의 첫 Linear 가중치 shape 에서 자동 감지한다
    """
    from rsl_rl.models.mlp_model import MLPModel
    from tensordict import TensorDict

    ck_probe = torch.load(ckpt_path, map_location="cpu", weights_only=False)
    if obs_dim is None:
        for k, v in ck_probe["actor_state_dict"].items():
            if k.endswith("mlp.0.weight"):
                obs_dim = int(v.shape[1])
                break
        assert obs_dim in (11, 14, 15, 18), f"관측 차원 감지 실패: {obs_dim}"
    dummy = TensorDict({"policy": torch.zeros(1, obs_dim)}, batch_size=[1])
    actor = MLPModel(dummy, {"actor": ["policy"]}, "actor", act_dim,
                     hidden_dims=[128, 128], activation="elu", obs_normalization=True,
                     distribution_cfg={"class_name": "GaussianDistribution",
                                       "init_std": 0.3, "std_type": "scalar"})
    actor.load_state_dict(ck_probe["actor_state_dict"])
    actor.eval()

    def policy(obs_np: np.ndarray) -> np.ndarray:
        td = TensorDict({"policy": torch.as_tensor(obs_np, dtype=torch.float32).unsqueeze(0)},
                        batch_size=[1])
        with torch.no_grad():
            a = actor(td)                      # deterministic mean
        return a.clamp(-1.0, 1.0).squeeze(0).numpy()

    policy.obs_dim = obs_dim
    return policy


def synthesize_cell_patch(seed: int, ra_target_um: float, scratch_max_um: float,
                          clearcoat_mean_um: float, n_scratches: int | None = None):
    """검사 시스템 입력(Ra·최대 scratch·clearcoat 평균)에 맞춘 대표 patch 합성.

    입력의 roughness(Rq)/Rz/GU 는 참고값 — 합성 표면에서 twin 이 재계산한 값을 출력한다
    (전/후가 같은 정의로 비교되도록). scratch 는 절차 생성 후 최대 깊이를 목표값으로 스케일.
    """
    st = make_flat_patch(PATCH_SIZE, RESOLUTION_M, seed=seed,
                         target_ra_um=ra_target_um, with_scratches=scratch_max_um > 0.0,
                         n_scratches=n_scratches)
    if scratch_max_um > 0.0 and st.initial_scratch_depth_um.max() > 1e-9:
        k = scratch_max_um / float(st.initial_scratch_depth_um.max())
        # micro = base − scratch 이므로, scratch 를 k 배로 바꾸면 micro 에 (1−k)·scratch 를 더한다
        st.micro_height_um += (1.0 - k) * st.initial_scratch_depth_um
        st.initial_micro_height_um = st.micro_height_um.copy()
        st.initial_scratch_depth_um *= k
        st.residual_scratch_depth_um = st.initial_scratch_depth_um.copy()
        st.defect_mask = st.initial_scratch_depth_um > (0.5 * PC.SCRATCH_DEPTH_MIN_UM)
        st.healthy_mask = ~st.defect_mask
    shift = clearcoat_mean_um - float(st.clearcoat_remaining_um.mean())
    st.clearcoat_remaining_um += shift
    st.initial_clearcoat_um = st.clearcoat_remaining_um.copy()
    return st


class _Path:
    """raster 경로 (polish_env 의 경로 로직과 동일)."""

    def __init__(self, recipe: Recipe):
        spacing = recipe.step_over_spacing_ratio * PC.PAD_DIAMETER_M
        lines = raster_waypoints(PATCH_SIZE, spacing)
        self.pts = [(p0, p1) for p0, p1, _ in lines]
        self.len = np.array([math.hypot(p1[0] - p0[0], p1[1] - p0[1])
                             for p0, p1 in self.pts])
        self.one_pass = float(self.len.sum())
        self.total = self.one_pass * recipe.n_passes

    def pos_at(self, arc_m: float):
        a = arc_m % self.one_pass if arc_m < self.total else self.one_pass - 1e-9
        for (p0, p1), L in zip(self.pts, self.len):
            if a <= L:
                t = a / L
                return (p0[0] + (p1[0] - p0[0]) * t, p0[1] + (p1[1] - p0[1]) * t)
            a -= L
        return self.pts[-1][1]


def spatial_lookahead(st, path, arc: float):
    """env._lookahead_stats 와 동일 정의 — near/far 구간 잔여 scratch mean/max."""
    R = 0.5 * PC.PAD_RADIUS_M
    res = st.resolution_m
    out = []
    for d0, d1 in ((0.0, PC.PAD_RADIUS_M), (PC.PAD_RADIUS_M, 3.0 * PC.PAD_RADIUS_M)):
        means, maxs = [], []
        for t in range(3):
            a = min(arc + d0 + (d1 - d0) * (t + 0.5) / 3.0, path.total)
            uv = path.pos_at(a)
            i0 = max(int((uv[0] - R) / res), 0); i1 = min(int((uv[0] + R) / res) + 1, st.shape[0])
            j0 = max(int((uv[1] - R) / res), 0); j1 = min(int((uv[1] + R) / res) + 1, st.shape[1])
            if i0 >= i1 or j0 >= j1:
                means.append(0.0); maxs.append(0.0); continue
            sl = (slice(i0, i1), slice(j0, j1))
            rem = np.clip(st.initial_scratch_depth_um[sl] - st.cumulative_removal_um[sl], 0.0, None)
            means.append(float(rem.mean())); maxs.append(float(rem.max()))
        out += [float(np.mean(means)), float(np.max(maxs))]
    return out


def footprint_stats(st, uv, clearcoat_limit: float):
    """polish_env._footprint_stats 와 동일 — 코어 crop 의 scratch/제거/여유 + 열 3통계."""
    res, R = st.resolution_m, 0.5 * PC.PAD_RADIUS_M
    i0 = max(int((uv[0] - R) / res), 0); i1 = min(int((uv[0] + R) / res) + 1, st.shape[0])
    j0 = max(int((uv[1] - R) / res), 0); j1 = min(int((uv[1] + R) / res) + 1, st.shape[1])
    if i0 >= i1 or j0 >= j1:
        return (0.0, 0.0, 0.0, 20.0,
                PC.AMBIENT_TEMPERATURE_C, PC.AMBIENT_TEMPERATURE_C, 0.0)
    sl = (slice(i0, i1), slice(j0, j1))
    remaining = np.clip(st.initial_scratch_depth_um[sl] - st.cumulative_removal_um[sl],
                        0.0, None)
    return (float(remaining.mean()), float(remaining.max()),
            float(st.cumulative_removal_um[sl].mean()),
            float(st.clearcoat_remaining_um[sl].min() - clearcoat_limit),
            float(st.temperature_c[sl].mean()),
            float(st.peak_temperature_c[sl].max()),
            float(st.thermal_damage_proxy[sl].mean()))


def run_cell_episode(row: dict, recipe: Recipe, policy, cal, max_repolish: int = 2) -> dict:
    """셀 하나: patch 합성 → 정책 실행 → 판정 미달 시 재폴리싱(예산 내) → 전/후 품질 + 판정.

    재폴리싱 규칙 (실제 공정의 검사-재작업 루프 재현):
      · 에피소드 후 5종 판정 전부 통과 → 종료
      · 미달 → 다음 에피소드의 예상 최대 국소 제거량(직전 에피소드 실측)을 빼도
        잔여 clearcoat 최소가 35 μm 이상일 때만 반복. 아니면 정직하게 실패
        (failure_reason: clearcoat_budget_exhausted). 목표값은 낮추지 않는다.
    """
    seed = int(float(row.get("surface_seed") or 0)) or (
        7000 + 101 * int(row["region_id"]) + int(row["cell_id"]))
    ra0 = float(row["init_ra_um"]); scr0 = float(row["init_scratch_um"])
    cc0 = float(row["init_clearcoat_um"])
    nx, ny, nz = (float(row["normal_x"]), float(row["normal_y"]), float(row["normal_z"]))
    n = np.array([nx, ny, nz]); n /= max(np.linalg.norm(n), 1e-9)
    tilt_deg = math.degrees(math.acos(np.clip(n[2], -1.0, 1.0)))
    is_side = tilt_deg > 45.0            #  — 문/펜더 수직면은 side 접촉 상수

    n_scr = row.get("init_scratch_count")
    st = synthesize_cell_patch(seed, ra0, scr0, cc0,
                               n_scratches=int(n_scr) if n_scr not in (None, "",) else None)
    model = LiteraturePolishingModel(cal)
    gloss = LiteratureGlossProxyModel()

    # ── before (합성 표면에서 twin 재계산 — 전/후 동일 정의) ──
    before = {
        "roughness": rq_um(st.micro_height_um),
        "ra": ra_um(st.micro_height_um),
        "rz": rz_um(st.micro_height_um),
        "scratch": float(residual_scratch_depth_um(
            st.micro_height_um, st.defect_mask, st.resolution_m).max()),
        "gu": float(gloss.evaluate(st)["summary"]["gu_mean"]),
    }

    # ── episode 루프 (polish_env 과 동일 제어 + 재폴리싱) ──
    path = _Path(recipe)
    contact = VirtualPadContact(1, "cpu", dt=PHYSICS_DT, lag_offset=0.0)
    side_t = torch.tensor([is_side])
    quality_dt = PHYSICS_DT * DECIMATION
    feed0 = recipe.feed_speed_mm_s / 1000.0

    sim_t = 0.0
    a_sum = np.zeros(2); f_sum = 0.0; feed_sum = 0.0; n_steps_total = 0
    hard_violated = False
    completed = False
    episodes_used = 0
    budget_exhausted = False
    gloss_mid = gloss   # 판정용 재사용

    def _judge():
        q = model.evaluate(st)
        gu = float(gloss_mid.evaluate(st)["summary"]["gu_mean"])
        ok = (gu >= GU_PASS_MIN and q["ra_um"] <= RA_PASS_MAX_UM
              and q["rz_um"] <= RZ_PASS_MAX_UM
              and float(st.clearcoat_remaining_um.min()) >= CLEARCOAT_SAFE_MIN_UM
              and (before["scratch"] < 0.05
                   or q["max_residual_scratch_um"] < before["scratch"]))
        return ok, gu

    last_gu = None
    for _ep in range(1 + max_repolish):
        episodes_used += 1
        contact.reset(is_side=side_t)
        arc = 0.0
        prev_force = 0.0
        prev_a = np.zeros(2, dtype=np.float32)
        force_mean = 0.0
        obs_scr_mean = obs_scr_max = obs_removal = 0.0
        obs_cc_margin = 20.0
        obs_temp = obs_peak = PC.AMBIENT_TEMPERATURE_C
        obs_tdmg = 0.0
        completed = False
        cc_min_ep_start = float(st.clearcoat_remaining_um.min())

        for step in range(MAX_CONTROL_STEPS):
            core = [
                force_mean / 10.0,
                0.0,                                   # (f − cmd)/5 — 아래서 채움
                (force_mean - prev_force) / 5.0,
                0.0,                                   # feed*100 — 아래서 채움
                min(arc / path.total, 1.0),
                obs_scr_mean / 2.0, obs_scr_max / 2.0,
                obs_removal / 5.0, obs_cc_margin / 20.0,
            ]
            # 채널 순서는 polish_env 와 동일: core(0-8), prev_action(9-10),
            # thermal(+3, use_thermal_obs), spatial(+4, use_spatial_obs).
            # 9.14/9.15 의 첫 판정은 thermal 을 prev 앞에 넣은 배선 오류로 오염 — 수정판.
            od = getattr(policy, "obs_dim", 11)
            tail = []
            if od in (14, 18):
                tail += [(obs_temp - PC.AMBIENT_TEMPERATURE_C) / 40.0,
                         (obs_peak - PC.AMBIENT_TEMPERATURE_C) / 60.0,
                         obs_tdmg / PC.THERMAL_DAMAGE_MAX]
            if od in (15, 18):
                la = spatial_lookahead(st, path, arc)
                tail += [v / 2.0 for v in la]
            obs = np.array(core + [prev_a[0], prev_a[1]] + tail, dtype=np.float32)
            # 직전 스텝의 명령 기준 오차/이송 (polish_env 관측과 동일한 시점 정렬)
            cmd_force_prev = recipe.target_contact_force_n * (1.0 + prev_a[0] * FORCE_RATIO_LIMIT)
            cmd_feed_prev = feed0 * (1.0 + prev_a[1] * FEED_RATIO_LIMIT)
            obs[1] = (force_mean - cmd_force_prev) / 5.0
            obs[3] = cmd_feed_prev * 100.0

            a = policy(obs)
            force_cmd = recipe.target_contact_force_n * (1.0 + a[0] * FORCE_RATIO_LIMIT)
            feed_cmd = feed0 * (1.0 + a[1] * FEED_RATIO_LIMIT)

            f_accum = 0.0
            tgt = torch.tensor([force_cmd], dtype=torch.float32)
            for _ in range(DECIMATION):
                f_accum += float(contact.step(tgt, side_t)[0])
                arc += feed_cmd * PHYSICS_DT
                sim_t += PHYSICS_DT
            prev_force = force_mean
            force_mean = f_accum / DECIMATION
            if force_mean > FORCE_HARD_LIMIT_N:
                hard_violated = True

            uv = path.pos_at(arc)
            model.step(st, ContactState(
                pad_center_uv_m=uv,
                contact_force_n=force_mean,        # 달성 힘 — 명령이 아니라 (env 와 동일)
                rpm=recipe.rpm,
                feed_speed_m_s=feed_cmd,
            ), dt_s=quality_dt, sim_time_s=sim_t)
            (obs_scr_mean, obs_scr_max, obs_removal, obs_cc_margin,
             obs_temp, obs_peak, obs_tdmg) = footprint_stats(st, uv, CLEARCOAT_SAFE_MIN_UM)

            prev_a = a
            a_sum += a; f_sum += force_mean; feed_sum += feed_cmd
            if hard_violated:
                break
            if arc >= path.total:
                completed = True
                break
        n_steps_total += step + 1

        if hard_violated or not completed:
            break
        ok, gu_now = _judge()
        if ok:                             # 5종 판정 통과 — 더 닦을 이유 없음
            break
        # 과연마 가드: 직전 에피소드보다 GU 가 나빠졌으면 더 갈아도 손해다 — 중단.
        # (실물 공정은 되돌릴 수 없으므로 '멈춤'이 유일한 올바른 대응)
        if last_gu is not None and gu_now <= last_gu + 0.05:
            break
        last_gu = gu_now
        # 재폴리싱 예산: 다음 에피소드가 직전보다 다른 곳을 더 깎을 수 있으므로
        # 직전 최대 국소 제거량의 1.5배 여유를 두고도 35 μm 유지돼야 반복한다.
        cc_min_now = float(st.clearcoat_remaining_um.min())
        ep_removal_est = max(cc_min_ep_start - cc_min_now, 0.5)
        if cc_min_now - 1.5 * ep_removal_est < CLEARCOAT_SAFE_MIN_UM:
            budget_exhausted = _ep < max_repolish   # 반복 여지가 있었는데 예산으로 중단
            break
    n_steps = n_steps_total

    # ── after ──
    q = model.evaluate(st)
    gu_after = float(gloss.evaluate(st)["summary"]["gu_mean"])
    after = {
        "roughness": rq_um(st.micro_height_um),
        "ra": q["ra_um"], "rz": q["rz_um"],
        "scratch": q["max_residual_scratch_um"],
        "cc_removed": float(st.cumulative_removal_um.mean()),
        "cc_remaining": float(st.clearcoat_remaining_um.mean()),
        "cc_remaining_min": float(st.clearcoat_remaining_um.min()),
        "gu": gu_after,
    }

    # ── 판정 ──
    gu_pass = after["gu"] >= GU_PASS_MIN
    ra_pass = after["ra"] <= RA_PASS_MAX_UM
    rz_pass = after["rz"] <= RZ_PASS_MAX_UM
    scr_improved = (before["scratch"] < 0.05) or (after["scratch"] < before["scratch"])
    cc_safe = after["cc_remaining_min"] >= CLEARCOAT_SAFE_MIN_UM
    # OEM 보증 제거한도: Ford 0.3mil=7.5μm (최엄격 기준 채택).
    # 판정 5종과 별개의 정보 플래그 — 통과셀의 보증 준수 여부를 명시 (기준 아님).
    warranty_ok = (float(row["init_clearcoat_um"]) - after["cc_remaining_min"]) <= 7.5
    reasons = []
    if not gu_pass: reasons.append(f"gu_below_target({after['gu']:.1f}<{GU_PASS_MIN:.0f})")
    if not ra_pass: reasons.append(f"ra_above_target({after['ra']:.3f}>{RA_PASS_MAX_UM:.2f})")
    if not rz_pass: reasons.append(f"rz_above_target({after['rz']:.2f}>{RZ_PASS_MAX_UM:.1f})")
    if not scr_improved: reasons.append("scratch_not_improved")
    if not cc_safe: reasons.append(
        f"clearcoat_below_safe({after['cc_remaining_min']:.1f}<{CLEARCOAT_SAFE_MIN_UM:.0f})")
    if hard_violated: reasons.append("force_hard_limit_violated")
    if not completed and not hard_violated: reasons.append("episode_timeout")
    overall = gu_pass and ra_pass and rz_pass and scr_improved and cc_safe \
        and completed and not hard_violated
    if budget_exhausted and not overall:
        reasons.append("clearcoat_budget_exhausted")
    # 처분(disposition) — 실패를 현장 행동으로 번역:
    #   spot_repaint_review = 예산 소진/바닥 근접 — 폴리싱으로 더 못 가는 셀.
    #     (q_clearcoat 는 광학이 아니라 "보전 위험" 항 — 얇게 출고된 셀은 표면이 좋아도
    #      여기 걸린다. 산업 관행상 스팟 재도장 검토가 올바른 처분)
    #   rework_candidate = 예산 여유가 남은 근소 미달 — 재작업/공정조정 후보.
    if overall:
        disposition = "pass"
    elif budget_exhausted or (after["cc_remaining_min"] - CLEARCOAT_SAFE_MIN_UM) < 1.0             or hard_violated:
        disposition = "spot_repaint_review"
    else:
        disposition = "rework_candidate"

    return {
        "region_id": row["region_id"], "region_name": row.get("region_name", ""),
        "cell_id": row["cell_id"],
        "position_x_m": row["position_x_m"], "position_y_m": row["position_y_m"],
        "position_z_m": row["position_z_m"],
        "normal_x": f"{n[0]:.4f}", "normal_y": f"{n[1]:.4f}", "normal_z": f"{n[2]:.4f}",
        "force_n": f"{f_sum / n_steps:.3f}", "rpm": f"{recipe.rpm:.0f}",
        "feed_speed_mm_s": f"{feed_sum / n_steps * 1000.0:.3f}",
        "step_over_ratio": f"{recipe.step_over_spacing_ratio:.4f}",
        "pass_count": recipe.n_passes * episodes_used,   # 실제 수행한 raster pass 총수
        "policy_action_force": f"{a_sum[0] / n_steps:+.3f}",
        "policy_action_feed": f"{a_sum[1] / n_steps:+.3f}",
        "roughness_before": f"{before['roughness']:.4f}",
        "roughness_after": f"{after['roughness']:.4f}",
        "scratch_before_um": f"{before['scratch']:.4f}",
        "scratch_after_um": f"{after['scratch']:.4f}",
        "ra_before_um": f"{before['ra']:.4f}", "ra_after_um": f"{after['ra']:.4f}",
        "rz_before_um": f"{before['rz']:.4f}", "rz_after_um": f"{after['rz']:.4f}",
        "clearcoat_initial_um": f"{float(row['init_clearcoat_um']):.2f}",
        "clearcoat_removed_um": f"{after['cc_removed']:.4f}",
        "clearcoat_remaining_um": f"{after['cc_remaining']:.2f}",
        "clearcoat_remaining_min_um": f"{after['cc_remaining_min']:.2f}",
        "gu_proxy_before": f"{before['gu']:.2f}", "gu_proxy_after": f"{after['gu']:.2f}",
        "gu_target_pass": gu_pass, "ra_target_pass": ra_pass, "rz_target_pass": rz_pass,
        "warranty_removal_ok": warranty_ok,
        "scratch_improved": scr_improved, "clearcoat_safe": cc_safe,
        "overall_pass": overall,
        "failure_reason": ";".join(reasons) if reasons else "",
        "disposition": disposition,
        "tilt_deg": f"{tilt_deg:.1f}", "is_side": is_side,
        "evaluation_mode": EVALUATION_MODE, "surface_seed": seed,
        "process_time_s": f"{sim_t:.1f}", "episode_completed": completed,
        "repolish_episodes": episodes_used - 1,
    }


# ── multiprocessing worker ────────────────────────────────────────────────
_G = {}

def _init_worker(ckpt, max_repolish=2, recipe_json=RECIPE_JSON, recipe_json_side=None):
    global EVALUATION_MODE
    EVALUATION_MODE = policy_label(ckpt)[1]
    _G["max_repolish"] = max_repolish
    _G["recipe"] = load_recipe(recipe_json)
    _G["recipe_side"] = load_recipe(recipe_json_side) if recipe_json_side else _G["recipe"]
    _G["policy"] = load_bc_policy(ckpt)
    _G["cal"] = load_calibrated_config()

def _work(row):
    import math as _m
    n = np.array([float(row["normal_x"]), float(row["normal_y"]), float(row["normal_z"])])
    n /= max(np.linalg.norm(n), 1e-9)
    tilt = _m.degrees(_m.acos(float(np.clip(n[2], -1.0, 1.0))))
    recipe = _G["recipe_side"] if tilt > 45.0 else _G["recipe"]
    return run_cell_episode(row, recipe, _G["policy"], _G["cal"],
                            max_repolish=_G["max_repolish"])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--checkpoint", default=DEFAULT_CKPT)
    ap.add_argument("--recipe_json", default=RECIPE_JSON,
                    help="공정 recipe JSON — BO outer loop 후보 검증 시 교체")
    ap.add_argument("--recipe_json_side", default=None,
                    help="side(수직면, tilt>45°) 셀 전용 recipe JSON. 자세별 공정 분리 — "
                         "원 시뮬도 top/side 목표힘을 다르게 뒀다 (contact.py 상수). "
                         "미지정 시 전 셀 --recipe_json 사용")
    ap.add_argument("--input", required=True)
    ap.add_argument("--output", required=True)
    ap.add_argument("--summary", default=None, help="기본: <output 이름>_summary.json")
    ap.add_argument("--workers", type=int, default=8)
    ap.add_argument("--limit", type=int, default=0, help="스모크용 — 앞 N 셀만")
    ap.add_argument("--max_repolish", type=int, default=2,
                    help="판정 미달 시 재폴리싱 최대 횟수 (총 에피소드 = 1+N). "
                         "clearcoat 예산이 허용할 때만 반복 — 목표 하향이 아니라 공정 반복이다")
    args = ap.parse_args()

    with open(args.input, newline="", encoding="utf-8") as f:
        rows = list(csv.DictReader(f))
    if args.limit:
        rows = rows[:args.limit]
    ptype, emode = policy_label(args.checkpoint)
    global EVALUATION_MODE
    EVALUATION_MODE = emode
    print(f"[vehicle] 입력 {len(rows)} 셀 | 정책 {ptype} ({os.path.basename(args.checkpoint)}) "
          f"| clearcoat 안전기준 {CLEARCOAT_SAFE_MIN_UM:.0f} μm | mode={emode}")

    if args.workers > 1:
        import multiprocessing as mp
        with mp.get_context("fork").Pool(args.workers, _init_worker,
                                         (args.checkpoint, args.max_repolish,
                                          args.recipe_json, args.recipe_json_side)) as pool:
            results = pool.map(_work, rows, chunksize=2)
    else:
        _init_worker(args.checkpoint, args.max_repolish, args.recipe_json,
                     args.recipe_json_side)
        results = [_work(r) for r in rows]

    with open(args.output, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=OUTPUT_COLUMNS, extrasaction="ignore")
        w.writeheader(); w.writerows(results)
    print(f"[vehicle] {len(results)} rows → {args.output}")

    # ── 요약 JSON: 셀별 실패 원인 + 자세(top/side)별 성능 — 곡면 검증 대체물 아님 ──
    fails = [r for r in results if not r["overall_pass"]]
    def agg(rs, key):
        if not rs:
            return None
        v = np.array([float(r[key]) for r in rs])
        return {"mean": round(float(v.mean()), 3), "min": round(float(v.min()), 3),
                "max": round(float(v.max()), 3)}
    def group(rs):
        return {
            "n_cells": len(rs),
            "n_pass": sum(1 for r in rs if r["overall_pass"]),
            "gu_after": agg(rs, "gu_proxy_after"),
            "scratch_after_um": agg(rs, "scratch_after_um"),
            "scratch_reduction_pct": round(100.0 * (1.0 - (
                sum(float(r["scratch_after_um"]) for r in rs)
                / max(sum(float(r["scratch_before_um"]) for r in rs), 1e-9))), 1) if rs else None,
            "clearcoat_remaining_min_um": agg(rs, "clearcoat_remaining_min_um"),
        }
    top = [r for r in results if not r["is_side"]]
    side = [r for r in results if r["is_side"]]
    summary = {
        "policy_type": ptype,
        "checkpoint": args.checkpoint,
        "evaluation_mode": EVALUATION_MODE,
        "curved_generalization_validated": False,
        "note_curved": "정책은 120×120 mm 평면 patch 학습본. 아래 top/side 구분은 같은 평면 "
                       "학습 정책의 자세별 추론 결과이지 곡면 재학습·검증 결과가 아니다.",
        "clearcoat_safety_limit_um": CLEARCOAT_SAFE_MIN_UM,
        "thresholds": {"gu_min": GU_PASS_MIN, "ra_max_um": RA_PASS_MAX_UM,
                       "rz_max_um": RZ_PASS_MAX_UM,
                       "clearcoat_min_um": CLEARCOAT_SAFE_MIN_UM,
                       "provenance": "literature-derived project target — 논문 기반 디지털 트윈 "
                                     "판정값. 프로젝트 자체 실측·보정값 아님. 미달 시 목표를 "
                                     "낮추지 않고 실패로 판정"},
        "all": group(results),
        "top_facing_cells (보닛/루프 등, tilt≤45°)": group(top),
        "side_facing_cells (도어/펜더 등, tilt>45°)": group(side),
        "per_region": {},
        "failed_cells": [
            {"region_id": r["region_id"], "region_name": r["region_name"],
             "cell_id": r["cell_id"], "failure_reason": r["failure_reason"]}
            for r in fails],
        "synthetic_disclaimer": "모든 품질 수치는 논문 근거 모델의 SYNTHETIC 출력 — 실측 아님.",
    }
    regions = sorted({r["region_id"] for r in results}, key=str)
    for rid in regions:
        rs = [r for r in results if r["region_id"] == rid]
        summary["per_region"][f"{rid}:{rs[0]['region_name']}"] = group(rs)

    out_json = args.summary or os.path.splitext(args.output)[0] + "_summary.json"
    with open(out_json, "w", encoding="utf-8") as f:
        json.dump(summary, f, ensure_ascii=False, indent=2, default=str)
    n_pass = sum(1 for r in results if r["overall_pass"])
    print(f"[vehicle] overall_pass {n_pass}/{len(results)} | 요약 → {out_json}")


if __name__ == "__main__":
    main()
