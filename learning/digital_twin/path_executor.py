"""Raster 경로 실행기 + reference 캘리브레이션.

Isaac Lab 없이 순수 기구학으로 패드 중심을 raster 경로로 굴린다.
BO 초기 탐색과 k 캘리브레이션 전용 — 완벽한 추종을 가정하므로,
이 실행기로 뽑은 레시피는 잠정치이며 PPO 연결 후 재탐색 대상.
"""
from __future__ import annotations

import json
import os
from dataclasses import asdict, dataclass

import numpy as np

from . import config as C
from .polishing_model import ContactState, LiteraturePolishingModel
from .surface_state import SurfaceState, make_flat_patch


@dataclass
class Recipe:
    """공정 레시피. BO 의 탐색 대상."""
    target_contact_force_n: float
    feed_speed_mm_s: float
    rpm: float
    step_over_spacing_ratio: float
    tool_path_type: str = "raster"
    n_passes: int = 1


def raster_waypoints(patch_size_m, spacing_m: float, margin_m: float = 0.0):
    """raster 라인들의 (시작점, 끝점, line_id) 목록."""
    lines = []
    y = margin_m + spacing_m / 2.0
    line_id = 0
    while y < patch_size_m[1] - margin_m:
        p0 = (margin_m, y)
        p1 = (patch_size_m[0] - margin_m, y)
        if line_id % 2 == 1:
            p0, p1 = p1, p0        # serpentine
        lines.append((p0, p1, line_id))
        y += spacing_m
        line_id += 1
    return lines


def run_episode(model: LiteraturePolishingModel,
                state: SurfaceState,
                recipe: Recipe,
                patch_size_m,
                quality_dt_s: float = C.REFERENCE.quality_dt_s,
                rpm_schedule=None,
                stage_time_ratio=None,
                total_time_s: float | None = None) -> dict:
    """레시피 하나를 patch 에서 실행한다.

    기본: 경로를 recipe.n_passes 회 주행하고 이송속도가 시간을 결정한다.
    reference 모드: total_time_s 와 rpm_schedule 을 주면 그 시간 예산을 단계별로 배분하고
    각 단계에서 경로를 반복 주행한다.
    """
    spacing_m = recipe.step_over_spacing_ratio * C.PAD_DIAMETER_M
    lines = raster_waypoints(patch_size_m, spacing_m)
    feed_m_s = recipe.feed_speed_mm_s / 1000.0
    step_len_m = feed_m_s * quality_dt_s

    def polish_lines(rpm: float, time_budget_s: float | None, sim_t: float) -> float:
        """경로를 주행한다. time_budget_s 가 있으면 소진까지 반복, 없으면 1회."""
        while True:
            for p0, p1, _ in lines:
                p0 = np.asarray(p0); p1 = np.asarray(p1)
                seg_len = float(np.linalg.norm(p1 - p0))
                n_steps = max(int(seg_len / step_len_m), 1)
                direction = (p1 - p0) / seg_len
                for k in range(n_steps):
                    pos = p0 + direction * (k + 0.5) * (seg_len / n_steps)
                    model.step(state, ContactState(
                        pad_center_uv_m=(float(pos[0]), float(pos[1])),
                        contact_force_n=recipe.target_contact_force_n,
                        rpm=rpm,
                        feed_speed_m_s=feed_m_s,
                    ), dt_s=quality_dt_s, sim_time_s=sim_t)
                    sim_t += quality_dt_s
                    if time_budget_s is not None and sim_t >= time_budget_s:
                        return sim_t
            if time_budget_s is None:
                return sim_t

    sim_t = 0.0
    if rpm_schedule is not None:
        # reference 모드 — 시간예산을 stage 별로 배분
        assert total_time_s is not None and stage_time_ratio is not None
        budget = 0.0
        for rpm, ratio in zip(rpm_schedule, stage_time_ratio):
            budget += total_time_s * ratio
            sim_t = polish_lines(rpm, budget, sim_t)
    else:
        for _ in range(recipe.n_passes):
            sim_t = polish_lines(recipe.rpm, None, sim_t)

    result = model.evaluate(state)
    result["process_time_s"] = sim_t
    return result


# ── k 캘리브레이션 ───────────────────────────────────────────
def run_reference_simulation(k: float, seed: int = 0) -> dict:
    ref = C.REFERENCE
    cfg = C.PolishingModelConfig(k_literature_synthetic=k)
    model = LiteraturePolishingModel(cfg)
    state = make_flat_patch(patch_size_m=ref.patch_size_m,
                            resolution_m=ref.grid_resolution_m,
                            seed=seed, with_scratches=False)   # 계수 산정은 깨끗한 면에서
    recipe = Recipe(
        target_contact_force_n=ref.target_force_n,
        feed_speed_mm_s=ref.feed_speed_mm_s,
        rpm=ref.rpm_schedule[0],            # rpm_schedule 이 덮어씀
        step_over_spacing_ratio=ref.step_over_spacing_ratio,
    )
    return run_episode(model, state, recipe, ref.patch_size_m,
                       quality_dt_s=ref.quality_dt_s,
                       rpm_schedule=ref.rpm_schedule,
                       stage_time_ratio=ref.stage_time_ratio,
                       total_time_s=ref.total_time_s)


def calibrate_k(out_path: str | None = None, seed: int = 0) -> C.PolishingModelConfig:
    """reference 를 돌려 평균 제거량이 정확히 3 μm 가 되도록 k 를 역산한다.

     k=1 그대로는 쓸 수 없다 — 제거량이 천문학적이라 Clearcoat 클램프(40~50μm)에
    포화돼 선형 역산이 무너진다. 클램프가 물리지 않는 작은 probe 로 재고 선형 스케일한 뒤,
    검산 1회로 3.00 μm 를 확인한다 (검산은 클램프·비선형이 물리는 경우를 잡는 안전망).
    """
    probe = 1e-12
    unit = run_reference_simulation(k=probe, seed=seed)
    k = probe * C.REFERENCE.target_mean_removal_um / unit["mean_removal_um"]

    check = run_reference_simulation(k=k, seed=seed)
    err = abs(check["mean_removal_um"] - C.REFERENCE.target_mean_removal_um)
    if err > 0.01:
        raise RuntimeError(
            f"k 검산 실패: mean_removal={check['mean_removal_um']:.4f} μm (목표 3.0). "
            f"클램프/비선형이 물렸는지 확인할 것.")

    cfg = C.PolishingModelConfig(k_literature_synthetic=float(k))
    if out_path:
        os.makedirs(os.path.dirname(out_path), exist_ok=True)
        payload = {kk: vv for kk, vv in asdict(cfg).items() if kk != "tags"}
        payload["tags"] = cfg.tags
        payload["reference"] = asdict(C.REFERENCE)
        payload["calibration_check"] = check
        with open(out_path, "w", encoding="utf-8") as f:
            json.dump(payload, f, ensure_ascii=False, indent=2)
    return cfg


DEFAULT_CALIBRATION_PATH = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "outputs", "polishing_model_config.json")


def load_calibrated_config(path: str = DEFAULT_CALIBRATION_PATH) -> C.PolishingModelConfig:
    with open(path, encoding="utf-8") as f:
        data = json.load(f)
    fields = {k: v for k, v in data.items() if k in C.PolishingModelConfig.__dataclass_fields__}
    return C.PolishingModelConfig(**fields)
