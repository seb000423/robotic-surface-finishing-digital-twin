"""v5 시뮬레이션 ↔ 학습 스택 연결: 차 전체 셀 격자 + 잔차 정책 + 셀별 품질/판정.

역할:
  1. CellRegistry  — 스캔 점군을 윗면/좌·우 측면/전·후면으로 분류해 12 cm 셀 격자를 깔고,
     셀마다 학습·판정과 동일한 SurfaceState(디지털 트윈 패치)를 합성한다.
     lookup(world_xyz) → (cell, (u, v)) 로 패드 접촉점을 셀 로컬 좌표로 사영.
  2. ResidualPolicyBridge — 챔피언 잔차 정책(14ch)을 로드하고, 로봇 에이전트마다 20 Hz 로
     관측을 조립해 [Δforce ±30%, Δfeed ±50%] 를 낸다. 같은 주기로 현재 셀의 품질 모델을
     달성 힘(명령 아님)으로 스텝한다 — polish_env / export_vehicle_results 와 동일 규약.
  3. judge_cells() — 셀별 5종 판정 + 보증 플래그 + 처분(disposition) → CSV.

v5 시뮬레이션 쪽 훅은 agent.py 의 두 줄(목표 힘 ×, 경로 전진 ×)과 20 Hz 품질 스텝 호출뿐이다.
접촉은 v5 의 가상 스프링(CPU) 그대로 — 해석식 환경 정책과 같은 물리 규약.
"""
from __future__ import annotations

import csv
import math
import os
import sys
from dataclasses import dataclass, field

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
_REPO = os.path.dirname(os.path.dirname(_HERE))
if _REPO not in sys.path:
    sys.path.insert(0, _REPO)

from learning.digital_twin import config as PC                                   # noqa: E402
from learning.digital_twin.gloss_proxy import LiteratureGlossProxyModel           # noqa: E402
from learning.digital_twin.path_executor import load_calibrated_config            # noqa: E402
from learning.digital_twin.polishing_model import ContactState, LiteraturePolishingModel  # noqa: E402
from learning.digital_twin.roughness_metrics import (                             # noqa: E402
    ra_um, residual_scratch_depth_um, rz_um)
from learning.vehicle_export.export_vehicle_results import (                  # noqa: E402
    CLEARCOAT_SAFE_MIN_UM, DEFAULT_CKPT, FEED_RATIO_LIMIT, FORCE_HARD_LIMIT_N,
    FORCE_RATIO_LIMIT, GU_PASS_MIN, RA_PASS_MAX_UM, RZ_PASS_MAX_UM, RECIPE_JSON,
    footprint_stats, load_bc_policy, load_recipe, rq_um, synthesize_cell_patch)
from learning.vehicle_export.make_example_input import draw_initial_state    # noqa: E402

CELL_SIZE_M = 0.12            # 학습/판정 patch 와 동일
PHYSICS_DT = 1.0 / 60.0
DECIMATION = 3                # 20 Hz 제어 (polish_env 와 동일)
SIDE_TILT_DEG = 45.0          # 판정 파이프라인의 side 정의
# 시간 스케일: v5 시뮬레이션의 경로 전진(데모 속도, ~0.45 m/s)은 레시피 이송(~5.7 mm/s)보다 수십 배
# 빠르다. 로봇은 데모 속도로 움직이되 해석 품질 모델은 "레시피 시간"으로 스텝한다 —
# dt_model = dt_ctrl × (레시피 이송 / 실제 이송). 셀당 노출(제거량)이 레시피와 같아진다.
TIME_SCALE_MAX = float(os.environ.get("POLISH_RL_TIME_SCALE_MAX", "200.0"))


# ── 1. 셀 격자 ────────────────────────────────────────────────────────────
@dataclass
class Cell:
    cell_id: int
    region: str                 # top / side_left / side_right / front / rear
    origin: np.ndarray          # 셀 로컬 (0,0) 의 세계 좌표
    u_axis: np.ndarray          # 단위벡터 (셀 로컬 u 방향)
    v_axis: np.ndarray
    normal: np.ndarray          # 평균 법선 (세계)
    center: np.ndarray
    n_points: int
    tilt_deg: float
    is_side: bool
    init: dict                  # 초기 상태 (ra, scratch, n_scr, clearcoat)
    seed: int
    surface: object = None      # SurfaceState
    before: dict = field(default_factory=dict)
    visits: int = 0             # 품질 스텝 횟수
    force_sum: float = 0.0
    feed_sum: float = 0.0
    a_sum: np.ndarray = field(default_factory=lambda: np.zeros(2))
    hard_violated: bool = False


class CellRegistry:
    """스캔 점군 → 셀 격자. 법선은 셀 점군 PCA."""

    def __init__(self, points: np.ndarray, profile: str = "new_car", seed: int = 7000,
                 cell_size: float = CELL_SIZE_M, min_points: int = 40):
        self.points = np.asarray(points, float)
        self.cell_size = cell_size
        self.cells: list[Cell] = []
        self._index: dict[tuple, Cell] = {}
        rng = np.random.default_rng(seed)
        cal = load_calibrated_config()
        self.model = LiteraturePolishingModel(cal)
        self.gloss = LiteratureGlossProxyModel()

        P = self.points
        n_all = _estimate_normals(P, k=24)
        # 외향 정렬: 차 중심에서 바깥으로
        c = P.mean(axis=0)
        flip = np.einsum("ij,ij->i", n_all, P - c) < 0
        n_all[flip] *= -1.0
        tilt = np.degrees(np.arccos(np.clip(n_all[:, 2], -1.0, 1.0)))

        # 면 분류: 윗면 / 좌·우 측면(|n_x| 우세) / 전·후면(|n_y| 우세)
        cls = np.full(len(P), "top", dtype=object)
        side_mask = tilt > SIDE_TILT_DEG
        nx, ny = n_all[:, 0], n_all[:, 1]
        cls[side_mask & (np.abs(nx) >= np.abs(ny)) & (nx > 0)] = "side_right"
        cls[side_mask & (np.abs(nx) >= np.abs(ny)) & (nx <= 0)] = "side_left"
        cls[side_mask & (np.abs(nx) < np.abs(ny)) & (ny > 0)] = "front"
        cls[side_mask & (np.abs(nx) < np.abs(ny)) & (ny <= 0)] = "rear"

        # 면별 격자 평면: top → (x, y), side → (y, z), front/rear → (x, z)
        plane_axes = {"top": (0, 1), "side_left": (1, 2), "side_right": (1, 2),
                      "front": (0, 2), "rear": (0, 2)}
        groups: dict[tuple, list[int]] = {}
        for i in range(len(P)):
            a, b = plane_axes[cls[i]]
            key = (cls[i], int(math.floor(P[i, a] / cell_size)), int(math.floor(P[i, b] / cell_size)))
            groups.setdefault(key, []).append(i)

        cid = 0
        for key, idx in sorted(groups.items(), key=lambda kv: (kv[0][0], kv[0][1], kv[0][2])):
            if len(idx) < min_points:
                continue
            region, ia, ib = key
            a, b = plane_axes[region]
            pts = P[idx]
            nrm = n_all[idx].mean(axis=0); nrm /= max(np.linalg.norm(nrm), 1e-9)
            # 로컬 축: u = 격자 첫 축을 법선에 직교화, v = n × u
            e_a = np.zeros(3); e_a[a] = 1.0
            u = e_a - nrm * float(nrm @ e_a); u /= max(np.linalg.norm(u), 1e-9)
            v = np.cross(nrm, u); v /= max(np.linalg.norm(v), 1e-9)
            center = pts.mean(axis=0)
            origin = center - u * (cell_size / 2) - v * (cell_size / 2)
            t_deg = float(np.degrees(np.arccos(np.clip(nrm[2], -1.0, 1.0))))
            ra0, scr0, n_scr, cc0 = draw_initial_state(rng, profile)
            cell = Cell(cell_id=cid, region=region, origin=origin, u_axis=u, v_axis=v,
                        normal=nrm, center=center, n_points=len(idx), tilt_deg=t_deg,
                        is_side=t_deg > SIDE_TILT_DEG,
                        init={"ra": ra0, "scratch": scr0, "n_scr": n_scr, "clearcoat": cc0},
                        seed=seed + cid)
            cell.surface = synthesize_cell_patch(cell.seed, ra0, scr0, cc0, n_scratches=n_scr)
            st = cell.surface
            cell.before = {
                "roughness": rq_um(st.micro_height_um), "ra": ra_um(st.micro_height_um),
                "rz": rz_um(st.micro_height_um),
                "scratch": float(residual_scratch_depth_um(
                    st.micro_height_um, st.defect_mask, st.resolution_m).max()),
                "gu": float(self.gloss.evaluate(st)["summary"]["gu_mean"]),
            }
            self.cells.append(cell)
            self._index[(region, ia, ib)] = cell
            cid += 1
        self._plane_axes = plane_axes
        self._cls_of_point = cls
        self._normals = n_all
        self._kd = None
        try:
            from scipy.spatial import KDTree
            self._kd = KDTree(P)
        except Exception:
            pass
        by_region = {}
        for cl in self.cells:
            by_region[cl.region] = by_region.get(cl.region, 0) + 1
        print(f"[rl_bridge] 셀 격자: {len(self.cells)}셀 ({cell_size*100:.0f}cm) — "
              + ", ".join(f"{k}:{v}" for k, v in sorted(by_region.items())))

    def lookup(self, xyz) -> tuple[Cell | None, tuple[float, float]]:
        """세계 좌표 → (셀, 셀 로컬 (u,v) [m]). 가장 가까운 스캔 점의 면 분류로 격자를 고른다."""
        p = np.asarray(xyz, float)
        if self._kd is None:
            return None, (0.0, 0.0)
        _, i = self._kd.query(p)
        region = self._cls_of_point[i]
        a, b = self._plane_axes[region]
        key = (region, int(math.floor(p[a] / self.cell_size)), int(math.floor(p[b] / self.cell_size)))
        cell = self._index.get(key)
        if cell is None:        # 점이 적어 셀이 없는 곳 → 같은 면의 최근접 셀
            best, bd = None, 1e9
            for cl in self.cells:
                if cl.region != region:
                    continue
                d = float(np.linalg.norm(cl.center - p))
                if d < bd:
                    best, bd = cl, d
            cell = best
            if cell is None:
                return None, (0.0, 0.0)
        d = p - cell.origin
        u = float(np.clip(d @ cell.u_axis, 0.0, self.cell_size))
        v = float(np.clip(d @ cell.v_axis, 0.0, self.cell_size))
        return cell, (u, v)


def _estimate_normals(P: np.ndarray, k: int = 24) -> np.ndarray:
    from scipy.spatial import KDTree
    kd = KDTree(P)
    _, nb = kd.query(P, k=min(k, len(P)))
    normals = np.zeros_like(P)
    for i in range(len(P)):
        q = P[nb[i]] - P[nb[i]].mean(axis=0)
        w, vec = np.linalg.eigh(q.T @ q)
        normals[i] = vec[:, 0]
    return normals


# ── 2. 잔차 정책 브리지 ────────────────────────────────────────────────────
class AgentRLState:
    """로봇 에이전트 하나의 20 Hz 제어 상태."""

    def __init__(self):
        self.prev_action = np.zeros(2, dtype=np.float32)
        self.action = np.zeros(2, dtype=np.float32)
        self.prev_force = 0.0
        self.force_mean = 0.0
        self.force_accum = 0.0
        self.substep = 0
        self.cmd_force_prev = 0.0
        self.cmd_feed_prev = 0.0
        self.force_scale = 1.0
        self.feed_scale = 1.0
        self.fp = (0.0, 0.0, 0.0, 20.0, PC.AMBIENT_TEMPERATURE_C, PC.AMBIENT_TEMPERATURE_C, 0.0)
        self.sim_t = 0.0
        self.time_scale = 1.0


class ResidualPolicyBridge:
    def __init__(self, registry: CellRegistry, ckpt: str = DEFAULT_CKPT,
                 recipe_json: str = RECIPE_JSON, recipe_json_side: str | None = None,
                 enabled: bool = True):
        self.reg = registry
        self.enabled = enabled
        self.policy = load_bc_policy(ckpt) if enabled else None
        self.recipe_top = load_recipe(recipe_json)
        self.recipe_side = load_recipe(recipe_json_side) if recipe_json_side else self.recipe_top
        self.states: dict[str, AgentRLState] = {}
        od = getattr(self.policy, "obs_dim", 14) if self.policy else 14
        print(f"[rl_bridge] 정책 {os.path.basename(ckpt) if enabled else '(off)'} obs_dim={od}, "
              f"레시피 top {self.recipe_top.target_contact_force_n:.2f}N/"
              f"{self.recipe_top.feed_speed_mm_s:.2f}mm/s, side "
              f"{self.recipe_side.target_contact_force_n:.2f}N/{self.recipe_side.feed_speed_mm_s:.2f}mm/s")

    def state(self, label: str) -> AgentRLState:
        return self.states.setdefault(label, AgentRLState())

    def recipe_for(self, is_side: bool):
        return self.recipe_side if is_side else self.recipe_top

    def substep(self, label: str, measured_force_n: float, pad_xyz, feed_mps: float,
                progress: float, is_side: bool, in_contact: bool, dt: float = PHYSICS_DT):
        """물리 스텝(60 Hz)마다 호출. 3스텝마다 정책 갱신 + 품질 모델 스텝.

        반환 (force_scale, feed_scale) — v5 시뮬레이션은 이를 목표 힘·경로 전진에 곱한다.
        """
        s = self.state(label)
        s.sim_t += dt
        s.force_accum += float(measured_force_n)
        s.substep += 1
        if s.substep < DECIMATION:
            return s.force_scale, s.feed_scale
        s.substep = 0
        s.prev_force = s.force_mean
        s.force_mean = s.force_accum / DECIMATION
        s.force_accum = 0.0
        recipe = self.recipe_for(is_side)

        # ── 품질 모델 스텝 (달성 힘, 명령 아님; 레시피 시간으로 스케일) ──
        cell, uv = self.reg.lookup(pad_xyz)
        recipe_feed = (recipe.feed_speed_mm_s / 1000.0) * s.feed_scale
        time_scale = float(np.clip(recipe_feed / max(float(feed_mps), 1e-4), 0.05, TIME_SCALE_MAX)) \
            if feed_mps > 0 else 1.0
        s.time_scale = time_scale
        if cell is not None and in_contact:
            self.reg.model.step(cell.surface, ContactState(
                pad_center_uv_m=uv, contact_force_n=s.force_mean, rpm=recipe.rpm,
                feed_speed_m_s=recipe_feed), dt_s=PHYSICS_DT * DECIMATION * time_scale,
                sim_time_s=s.sim_t * time_scale)
            s.fp = footprint_stats(cell.surface, uv, CLEARCOAT_SAFE_MIN_UM)
            cell.visits += 1
            cell.force_sum += s.force_mean
            cell.feed_sum += recipe_feed
            cell.a_sum += s.action
            if s.force_mean > FORCE_HARD_LIMIT_N:
                cell.hard_violated = True

        if not self.enabled:
            return 1.0, 1.0

        # ── 관측 14ch (polish_env / export 와 동일 순서) ──
        core = [
            s.force_mean / 10.0,
            (s.force_mean - s.cmd_force_prev) / 5.0,
            (s.force_mean - s.prev_force) / 5.0,
            s.cmd_feed_prev * 100.0,
            float(np.clip(progress, 0.0, 1.0)),
            s.fp[0] / 2.0, s.fp[1] / 2.0, s.fp[2] / 5.0, s.fp[3] / 20.0,
        ]
        od = getattr(self.policy, "obs_dim", 14)
        tail = []
        if od in (14, 18):
            tail += [(s.fp[4] - PC.AMBIENT_TEMPERATURE_C) / 40.0,
                     (s.fp[5] - PC.AMBIENT_TEMPERATURE_C) / 60.0,
                     s.fp[6] / PC.THERMAL_DAMAGE_MAX]
        if od in (15, 18):
            tail += [0.0, 0.0, 0.0, 0.0]        # spatial lookahead 는 v5 시뮬레이션 경로에선 미정의
        obs = np.array(core + [s.prev_action[0], s.prev_action[1]] + tail, dtype=np.float32)
        a = self.policy(obs)
        s.n_ctrl = getattr(s, "n_ctrl", 0) + 1
        if os.environ.get("POLISH_RL_DEBUG") and s.n_ctrl % 100 == 0:
            print(f"[rl_bridge][{label}] ctrl {s.n_ctrl} feed_v5={feed_mps*1000:.1f}mm/s ts={time_scale:.1f} "
                  f"in_contact={in_contact} obs={np.round(obs, 2).tolist()} a={np.round(a, 2).tolist()}", flush=True)
        s.prev_action = s.action
        s.action = np.asarray(a, dtype=np.float32)
        s.force_scale = float(1.0 + a[0] * FORCE_RATIO_LIMIT)
        s.feed_scale = float(1.0 + a[1] * FEED_RATIO_LIMIT)
        s.cmd_force_prev = recipe.target_contact_force_n * s.force_scale
        s.cmd_feed_prev = (recipe.feed_speed_mm_s / 1000.0) * s.feed_scale
        return s.force_scale, s.feed_scale


# ── 3. 셀별 판정 ──────────────────────────────────────────────────────────
JUDGE_COLUMNS = [
    "cell_id", "region", "center_x_m", "center_y_m", "center_z_m", "tilt_deg", "is_side",
    "n_points", "visits", "force_n", "feed_speed_mm_s", "policy_action_force", "policy_action_feed",
    "scratch_before_um", "scratch_after_um", "ra_before_um", "ra_after_um",
    "rz_before_um", "rz_after_um", "clearcoat_initial_um", "clearcoat_removed_um",
    "clearcoat_remaining_min_um", "gu_proxy_before", "gu_proxy_after",
    "gu_target_pass", "ra_target_pass", "rz_target_pass", "scratch_improved", "clearcoat_safe",
    "warranty_removal_ok", "overall_pass", "failure_reason", "disposition",
]


def judge_cells(reg: CellRegistry) -> list[dict]:
    rows = []
    for cell in reg.cells:
        st = cell.surface
        q = reg.model.evaluate(st)
        gu_after = float(reg.gloss.evaluate(st)["summary"]["gu_mean"])
        b = cell.before
        after = {"ra": q["ra_um"], "rz": q["rz_um"], "scratch": q["max_residual_scratch_um"],
                 "cc_removed": float(st.cumulative_removal_um.mean()),
                 "cc_min": float(st.clearcoat_remaining_um.min()), "gu": gu_after}
        visited = cell.visits > 0
        gu_pass = after["gu"] >= GU_PASS_MIN
        ra_pass = after["ra"] <= RA_PASS_MAX_UM
        rz_pass = after["rz"] <= RZ_PASS_MAX_UM
        scr_ok = (b["scratch"] < 0.05) or (after["scratch"] < b["scratch"])
        cc_safe = after["cc_min"] >= CLEARCOAT_SAFE_MIN_UM
        warranty_ok = (cell.init["clearcoat"] - after["cc_min"]) <= 7.5
        reasons = []
        if not visited: reasons.append("not_polished")
        if not gu_pass: reasons.append(f"gu_below_target({after['gu']:.1f}<{GU_PASS_MIN:.0f})")
        if not ra_pass: reasons.append(f"ra_above_target({after['ra']:.3f}>{RA_PASS_MAX_UM:.2f})")
        if not rz_pass: reasons.append(f"rz_above_target({after['rz']:.2f}>{RZ_PASS_MAX_UM:.1f})")
        if not scr_ok: reasons.append("scratch_not_improved")
        if not cc_safe: reasons.append(f"clearcoat_below_safe({after['cc_min']:.1f})")
        if cell.hard_violated: reasons.append("force_hard_limit_violated")
        overall = visited and gu_pass and ra_pass and rz_pass and scr_ok and cc_safe and not cell.hard_violated
        if overall:
            disp = "pass"
        elif not visited:
            disp = "not_reached"
        elif (after["cc_min"] - CLEARCOAT_SAFE_MIN_UM) < 1.0 or cell.hard_violated:
            disp = "spot_repaint_review"
        else:
            disp = "rework_candidate"
        n = max(cell.visits, 1)
        rows.append({
            "cell_id": cell.cell_id, "region": cell.region,
            "center_x_m": f"{cell.center[0]:.3f}", "center_y_m": f"{cell.center[1]:.3f}",
            "center_z_m": f"{cell.center[2]:.3f}", "tilt_deg": f"{cell.tilt_deg:.1f}",
            "is_side": cell.is_side, "n_points": cell.n_points, "visits": cell.visits,
            "force_n": f"{cell.force_sum / n:.3f}", "feed_speed_mm_s": f"{cell.feed_sum / n * 1000:.3f}",
            "policy_action_force": f"{cell.a_sum[0] / n:+.3f}", "policy_action_feed": f"{cell.a_sum[1] / n:+.3f}",
            "scratch_before_um": f"{b['scratch']:.4f}", "scratch_after_um": f"{after['scratch']:.4f}",
            "ra_before_um": f"{b['ra']:.4f}", "ra_after_um": f"{after['ra']:.4f}",
            "rz_before_um": f"{b['rz']:.4f}", "rz_after_um": f"{after['rz']:.4f}",
            "clearcoat_initial_um": f"{cell.init['clearcoat']:.2f}",
            "clearcoat_removed_um": f"{after['cc_removed']:.4f}",
            "clearcoat_remaining_min_um": f"{after['cc_min']:.2f}",
            "gu_proxy_before": f"{b['gu']:.2f}", "gu_proxy_after": f"{after['gu']:.2f}",
            "gu_target_pass": gu_pass, "ra_target_pass": ra_pass, "rz_target_pass": rz_pass,
            "scratch_improved": scr_ok, "clearcoat_safe": cc_safe,
            "warranty_removal_ok": warranty_ok, "overall_pass": overall,
            "failure_reason": ";".join(reasons), "disposition": disp,
        })
    return rows


def write_judgement(reg: CellRegistry, out_csv: str) -> dict:
    rows = judge_cells(reg)
    os.makedirs(os.path.dirname(os.path.abspath(out_csv)), exist_ok=True)
    with open(out_csv, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=JUDGE_COLUMNS)
        w.writeheader(); w.writerows(rows)
    n = len(rows)
    summary = {
        "cells": n,
        "polished": sum(1 for r in rows if r["visits"] > 0),
        "pass": sum(1 for r in rows if r["overall_pass"]),
        "spot_repaint_review": sum(1 for r in rows if r["disposition"] == "spot_repaint_review"),
        "rework_candidate": sum(1 for r in rows if r["disposition"] == "rework_candidate"),
        "not_reached": sum(1 for r in rows if r["disposition"] == "not_reached"),
    }
    print(f"[rl_bridge] 판정 → {out_csv}: {summary}")
    return summary
