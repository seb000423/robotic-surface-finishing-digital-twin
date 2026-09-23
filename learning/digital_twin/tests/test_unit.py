""" 단위시험. Isaac Sim 불필요.

    ~/IsaacLab/.venv/bin/python -m learning.digital_twin.tests.test_unit
"""
import sys
import numpy as np

from learning.digital_twin import config as C
from learning.digital_twin.surface_state import make_flat_patch
from learning.digital_twin.polishing_model import ContactState, LiteraturePolishingModel
from learning.digital_twin.roughness_metrics import ra_um, rz_um
from learning.digital_twin.path_executor import Recipe, run_episode, calibrate_k, run_reference_simulation

results = []
def check(name, ok, detail=""):
    results.append(ok)
    print(f"  [{'PASS' if ok else 'FAIL'}] {name}" + (f" — {detail}" if detail else ""))

cfg = C.PolishingModelConfig(k_literature_synthetic=1e-9)  # 단위시험용 임시 (캘리브레이션은 별도)
model = LiteraturePolishingModel(cfg)
PATCH = (0.20, 0.20)

def fresh(seed=1, scratches=True):
    return make_flat_patch(PATCH, resolution_m=0.002, seed=seed, with_scratches=scratches)

print("\n단위시험")

# 1. Force=0 / 비접촉이면 제거 0
s = fresh()
model.step(s, ContactState((0.1, 0.1), 0.0, 4000, 0.005), 0.05)
model.step(s, ContactState((0.1, 0.1), 8.0, 4000, 0.005, in_contact=False), 0.05)
check("1. Force=0·비접촉 → 제거 0", s.cumulative_removal_um.max() == 0.0)

# 2. dt 반분할 불변성
s_a, s_b = fresh(2), fresh(2)
model.step(s_a, ContactState((0.1, 0.1), 8.0, 4000, 0.005), 0.10)
for _ in range(2):
    model.step(s_b, ContactState((0.1, 0.1), 8.0, 4000, 0.005), 0.05)
diff = abs(s_a.cumulative_removal_um.mean() - s_b.cumulative_removal_um.mean())
rel = diff / max(s_a.cumulative_removal_um.mean(), 1e-30)
check("2. dt 분할 불변", rel < 0.02, f"상대차 {rel:.2e}")

# 3. RPM 증가 → 제거율 증가
def removal_at(rpm):
    s = fresh(3, scratches=False)
    model.step(s, ContactState((0.1, 0.1), 8.0, rpm, 0.005), 0.05)
    return s.cumulative_removal_um.sum()
check("3. RPM↑ → 제거↑", removal_at(5000) > removal_at(3000))

# 4. Feed 증가 → 같은 경로에서 총 dwell·제거 감소
def episode_removal(feed):
    s = fresh(4, scratches=False)
    m = LiteraturePolishingModel(C.PolishingModelConfig(k_literature_synthetic=1e-9))
    r = Recipe(8.0, feed, 4000, 0.4, n_passes=1)
    run_episode(m, s, r, PATCH, quality_dt_s=0.05)
    return s.cumulative_removal_um.mean(), s.dwell_time_s.sum()
rem_slow, dwell_slow = episode_removal(2.0)
rem_fast, dwell_fast = episode_removal(8.0)
check("4. Feed↑ → dwell·제거↓", rem_fast < rem_slow and dwell_fast < dwell_slow,
      f"2mm/s: {rem_slow:.2e}μm vs 8mm/s: {rem_fast:.2e}μm")

# 6. 돌출부가 valley 보다 빨리 제거 → Ra 감소
s = fresh(6, scratches=False)
ra0 = ra_um(s.micro_height_um)
m6 = LiteraturePolishingModel(C.PolishingModelConfig(k_literature_synthetic=2e-9))
run_episode(m6, s, Recipe(8.0, 5.0, 4000, 0.4, n_passes=3), PATCH, quality_dt_s=0.05)
ra1 = ra_um(s.micro_height_um)
check("6. 폴리싱 후 Ra 감소", ra1 < ra0, f"{ra0:.4f} → {ra1:.4f} μm")

# 6b. 스크래치 patch 에서 잔존 스크래치 감소
s = fresh(7, scratches=True)
before = s.initial_scratch_depth_um.max()
m6b = LiteraturePolishingModel(C.PolishingModelConfig(k_literature_synthetic=2e-9))
out = run_episode(m6b, s, Recipe(8.0, 5.0, 4000, 0.4, n_passes=3), PATCH, quality_dt_s=0.05)
check("6b. 잔존 scratch < 초기", out["max_residual_scratch_um"] < before,
      f"{before:.3f} → {out['max_residual_scratch_um']:.3f} μm")

# 8. 물질수지: 제거량 합 == 초기 clearcoat − 현재 clearcoat
mass_in = s.initial_clearcoat_um.sum() - s.clearcoat_remaining_um.sum()
mass_rm = s.cumulative_removal_um.sum()
check("8. 물질수지 일치", abs(mass_in - mass_rm) / max(mass_rm, 1e-12) < 1e-6)

# 9. 같은 seed 재현성
a = fresh(9).micro_height_um; b = fresh(9).micro_height_um
check("9. 동일 seed 재현", np.array_equal(a, b))

# 10. 해상도 변경 시 평균 결과 안정 (±20%)
def mean_removal_at_res(res):
    s = make_flat_patch(PATCH, resolution_m=res, seed=10, with_scratches=False)
    m = LiteraturePolishingModel(C.PolishingModelConfig(k_literature_synthetic=1e-9))
    run_episode(m, s, Recipe(8.0, 5.0, 4000, 0.4, n_passes=1), PATCH, quality_dt_s=0.05)
    return s.cumulative_removal_um.mean()
r2, r4 = mean_removal_at_res(0.002), mean_removal_at_res(0.004)
check("10. 해상도 불변(±20%)", abs(r2 - r4) / r2 < 0.2, f"2mm {r2:.2e} vs 4mm {r4:.2e}")

# 7. reference regression — 평균 제거량 3 μm (본 캘리브레이션, 1mm 해상도)
print("\nreference 캘리브레이션 — 수 분 걸릴 수 있음")
from learning.digital_twin.path_executor import DEFAULT_CALIBRATION_PATH
cal = calibrate_k(out_path=DEFAULT_CALIBRATION_PATH)
final = run_reference_simulation(k=cal.k_literature_synthetic)
check("7. reference 평균 제거량 = 3 μm", abs(final["mean_removal_um"] - 3.0) < 0.01,
      f"{final['mean_removal_um']:.4f} μm, k={cal.k_literature_synthetic:.4e}")
print(f"     Ra {final['ra_um']:.4f} μm | Rz {final['rz_um']:.4f} μm | "
      f"clearcoat min {final['clearcoat_min_um']:.1f} μm | coverage {final['coverage_ratio']:.2f}")

n_fail = results.count(False)
print(f"\n{len(results)-n_fail}/{len(results)} 통과")
sys.exit(1 if n_fail else 0)
