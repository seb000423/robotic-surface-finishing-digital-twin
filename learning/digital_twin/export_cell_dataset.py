"""셀 단위 before/after 폴리싱 데이터셋 export.

    ~/isaacsim/kit/python/bin/python3 -m learning.digital_twin.export_cell_dataset \
        [--episodes 3] [--patch 0.12] [--out <csv>]

기존 digital_twin 코드는 건드리지 않는다 (읽기 전용). run_episode 를 그대로 쓰지 않고
루프를 복제한 이유는 스텝마다 셀별 힘/rpm/이송을 dwell 가중으로 누적하기 위해서다 —
kinematic 실행기에서는 상수지만, 나중에 PPO 환경 리플레이로 바꿔도 스키마가 그대로 유지된다.

전부 SYNTHETIC (논문 근거 모델의 출력, 실측 아님).
"""
from __future__ import annotations

import argparse
import csv
import json
import os

import numpy as np
from numpy.lib.stride_tricks import sliding_window_view

from . import config as C
from .path_executor import Recipe, load_calibrated_config, raster_waypoints
from .polishing_model import ContactState, LiteraturePolishingModel
from .roughness_metrics import residual_scratch_depth_um
from .surface_state import make_flat_patch

OUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "outputs")
RECIPE_JSON = os.path.join(OUT_DIR, "bo_best_recipe.json")


def load_recipe(path: str = RECIPE_JSON) -> tuple[Recipe, str]:
    try:
        with open(path, encoding="utf-8") as f:
            d = json.load(f)
        return Recipe(float(d["target_contact_force_n"]), float(d["feed_speed_mm_s"]),
                      float(d["rpm"]), float(d["step_over_spacing_ratio"]),
                      n_passes=int(d.get("n_passes", 1))), d.get("recipe_id", "unknown")
    except FileNotFoundError:
        r = C.REFERENCE
        return Recipe(r.target_force_n, r.feed_speed_mm_s, r.rpm_schedule[0],
                      r.step_over_spacing_ratio, n_passes=1), "REFERENCE_FALLBACK"


def local_roughness(height_um: np.ndarray, window: int):
    """셀별 국소 Ra / Rz / Rq [μm].

    전역 Ra/Rz(roughness_metrics)는 patch 전체를 1차 평면 detrend 하지만, 셀 단위 값이
    필요하므로 여기서는 window×window 창의 평균 제거로 국소 detrend 한다.
    국소 Rz 는 창 안 상위/하위 5샘플 차 — 전역 Rz 의 국소극값 필터와 정의가 다르다.
    """
    pad = window // 2
    hp = np.pad(height_um, pad, mode="reflect")
    win = sliding_window_view(hp, (window, window))
    win = win.reshape(height_um.shape[0], height_um.shape[1], window * window)
    z = win - win.mean(-1, keepdims=True)
    ra = np.abs(z).mean(-1)
    rq = np.sqrt((z ** 2).mean(-1))
    s = np.sort(z, axis=-1)
    rz = s[..., -5:].mean(-1) - s[..., :5].mean(-1)
    return ra, rz, rq


def run_and_record(model, state, recipe: Recipe, patch_size_m, dt_s: float):
    """경로를 주행하며 셀별 힘/rpm/이송을 dwell 가중 누적한다."""
    spacing_m = recipe.step_over_spacing_ratio * C.PAD_DIAMETER_M
    lines = raster_waypoints(patch_size_m, spacing_m)
    feed_m_s = recipe.feed_speed_mm_s / 1000.0
    step_len_m = feed_m_s * dt_s

    acc_w = np.zeros(state.shape)      # Σ shape01·dt  (가중치)
    acc_f = np.zeros(state.shape)      # Σ shape01·dt·force
    acc_r = np.zeros(state.shape)
    acc_v = np.zeros(state.shape)
    n_contact = np.zeros(state.shape, dtype=np.int32)

    sim_t, n_steps_total = 0.0, 0
    for _ in range(recipe.n_passes):
        for p0, p1, _ in lines:
            p0 = np.asarray(p0, float); p1 = np.asarray(p1, float)
            seg_len = float(np.linalg.norm(p1 - p0))
            n_steps = max(int(seg_len / step_len_m), 1)
            direction = (p1 - p0) / seg_len
            for k in range(n_steps):
                pos = p0 + direction * (k + 0.5) * (seg_len / n_steps)
                uv = (float(pos[0]), float(pos[1]))
                model.step(state, ContactState(
                    pad_center_uv_m=uv, contact_force_n=recipe.target_contact_force_n,
                    rpm=recipe.rpm, feed_speed_m_s=feed_m_s), dt_s=dt_s, sim_time_s=sim_t)
                sl, shape01, _w, _a = model._footprint(state, uv)
                if sl is not None:
                    wdt = shape01 * dt_s
                    acc_w[sl] += wdt
                    acc_f[sl] += wdt * recipe.target_contact_force_n
                    acc_r[sl] += wdt * recipe.rpm
                    acc_v[sl] += wdt * recipe.feed_speed_mm_s
                    n_contact[sl] += (shape01 > 0.0)
                sim_t += dt_s
                n_steps_total += 1
    denom = np.where(acc_w > 0.0, acc_w, 1.0)
    return {
        "force_n": np.where(acc_w > 0.0, acc_f / denom, 0.0),
        "rpm": np.where(acc_w > 0.0, acc_r / denom, 0.0),
        "feed_mm_s": np.where(acc_w > 0.0, acc_v / denom, 0.0),
        "dwell_w_s": acc_w,
        "n_contact": n_contact,
        "process_time_s": sim_t,
        "n_steps": n_steps_total,
        "spacing_m": spacing_m,
    }


COLUMNS = [
    "episode_id", "cell_id", "x", "y", "z",
    "surface_normal_x", "surface_normal_y", "surface_normal_z",
    "force_N", "rpm", "feed_mm_s", "step_over_mm", "step_over_ratio", "pass_count",
    "roughness_before", "roughness_after", "scratch_before", "scratch_after",
    "ra_before_um", "ra_after_um", "rz_before_um", "rz_after_um",
    "clearcoat_before_um", "clearcoat_after_um", "clearcoat_removed_um",
    # ── 해석에 필요한 보조 컬럼 ──
    "is_defect", "dwell_weighted_s", "contact_steps", "recipe_id", "surface_seed",
]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--episodes", type=int, default=3)
    ap.add_argument("--patch", type=float, default=0.12)
    ap.add_argument("--resolution", type=float, default=0.002)
    ap.add_argument("--seed_base", type=int, default=1000)
    ap.add_argument("--window", type=int, default=11, help="국소 Ra/Rz/Rq 창 [셀]")
    ap.add_argument("--out", type=str, default=os.path.join(OUT_DIR, "cell_dataset.csv"))
    args = ap.parse_args()

    recipe, recipe_id = load_recipe()
    cal = load_calibrated_config()
    patch = (args.patch, args.patch)
    feed_m_s = recipe.feed_speed_mm_s / 1000.0
    dt_s = float(np.clip(0.001 / feed_m_s, 0.02, 0.5))   # RecipeEvaluator 와 동일 규칙

    print(f"recipe {recipe_id}: {recipe.target_contact_force_n:.3f} N / "
          f"{recipe.feed_speed_mm_s:.3f} mm/s / {recipe.rpm:.0f} rpm / "
          f"spacing_ratio {recipe.step_over_spacing_ratio} / {recipe.n_passes} pass")
    print(f"patch {patch[0]:.2f}×{patch[1]:.2f} m @ {args.resolution*1000:.0f} mm, dt {dt_s:.3f} s")

    os.makedirs(os.path.dirname(args.out), exist_ok=True)
    f = open(args.out, "w", newline="", encoding="utf-8")
    w = csv.writer(f); w.writerow(COLUMNS)
    n_rows = 0

    for ep in range(args.episodes):
        seed = args.seed_base + ep
        state = make_flat_patch(patch, args.resolution, seed=seed, with_scratches=True)
        model = LiteraturePolishingModel(cal)

        h_before = state.initial_micro_height_um.copy()
        cc_before = state.initial_clearcoat_um.copy()
        # before/after 를 같은 정의로 — 생성 깊이가 아니라 기하학적 valley 깊이
        scratch_before = residual_scratch_depth_um(h_before, state.defect_mask, state.resolution_m)

        rec = run_and_record(model, state, recipe, patch, dt_s)
        summary = model.evaluate(state)          # residual_scratch_depth_um 갱신
        h_after = state.micro_height_um
        scratch_after = state.residual_scratch_depth_um

        ra_b, rz_b, rq_b = local_roughness(h_before, args.window)
        ra_a, rz_a, rq_a = local_roughness(h_after, args.window)

        cc_after = state.clearcoat_remaining_um
        cc_removed = state.cumulative_removal_um
        assert np.allclose(cc_before - cc_after, cc_removed, atol=1e-9), "물질수지 불일치"

        nx, ny = state.shape
        xyz = state.nominal_surface_xyz_m
        nrm = state.normal_xyz
        step_over_mm = rec["spacing_m"] * 1000.0
        for i in range(nx):
            for j in range(ny):
                w.writerow([
                    ep, i * ny + j,
                    f"{xyz[i,j,0]:.6f}", f"{xyz[i,j,1]:.6f}", f"{xyz[i,j,2]:.6f}",
                    f"{nrm[i,j,0]:.4f}", f"{nrm[i,j,1]:.4f}", f"{nrm[i,j,2]:.4f}",
                    f"{rec['force_n'][i,j]:.4f}", f"{rec['rpm'][i,j]:.1f}",
                    f"{rec['feed_mm_s'][i,j]:.4f}", f"{step_over_mm:.3f}",
                    f"{recipe.step_over_spacing_ratio:.4f}", int(state.pass_count[i,j]),
                    f"{rq_b[i,j]:.6f}", f"{rq_a[i,j]:.6f}",
                    f"{scratch_before[i,j]:.6f}", f"{scratch_after[i,j]:.6f}",
                    f"{ra_b[i,j]:.6f}", f"{ra_a[i,j]:.6f}",
                    f"{rz_b[i,j]:.6f}", f"{rz_a[i,j]:.6f}",
                    f"{cc_before[i,j]:.4f}", f"{cc_after[i,j]:.4f}", f"{cc_removed[i,j]:.6f}",
                    int(state.defect_mask[i,j]), f"{rec['dwell_w_s'][i,j]:.4f}",
                    int(rec["n_contact"][i,j]), recipe_id, seed,
                ])
        n_rows += nx * ny
        print(f"  ep {ep} (seed {seed}): {nx*ny} cells | 결함셀 {int(state.defect_mask.sum())} | "
              f"Ra {summary['ra_um']:.4f} Rz {summary['rz_um']:.3f} | "
              f"잔존scratch max {summary['max_residual_scratch_um']:.3f} μm | "
              f"평균제거 {summary['mean_removal_um']:.3f} μm | "
              f"공정 {rec['process_time_s']:.0f} s ({rec['n_steps']} steps)")

    f.close()
    print(f"\n{n_rows} rows → {args.out}")


if __name__ == "__main__":
    main()
