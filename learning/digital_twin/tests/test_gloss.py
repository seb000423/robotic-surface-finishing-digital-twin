""" 단위시험 (RTX 없이 가능한 항목).

    ~/IsaacLab/.venv/bin/python -m learning.digital_twin.tests.test_gloss
"""
import sys
import numpy as np

from learning.digital_twin import config as C
from learning.digital_twin.surface_state import make_flat_patch
from learning.digital_twin.polishing_model import LiteraturePolishingModel
from learning.digital_twin.path_executor import Recipe, run_episode, load_calibrated_config
from learning.digital_twin.gloss_proxy import (
    LiteratureGlossProxyModel, GlossProxyConfig, gu_from_relative)

results = []
def check(name, ok, detail=""):
    results.append(ok)
    print(f"  [{'PASS' if ok else 'FAIL'}] {name}" + (f" — {detail}" if detail else ""))

print("\nGU proxy 단위시험")

# 8. anchor mapping 회귀
b, a = float(gu_from_relative(0.2180)), float(gu_from_relative(0.8815))
check("8. mapping 회귀 0.2180→36.6 / 0.8815→71.7",
      abs(b - 36.6) < 0.1 and abs(a - 71.7) < 0.1, f"{b:.1f} / {a:.1f} GU")

check("경계: rel=0→25, rel=1→78",
      float(gu_from_relative(0.0)) == 25.0 and float(gu_from_relative(1.0)) == 78.0)

gm = LiteratureGlossProxyModel()
PATCH = (0.20, 0.20)
cal = load_calibrated_config()

# 5. roughness/scratch 증가 시 GU 감소 (더 거친/긁힌 표면일수록 낮은 GU)
s_clean = make_flat_patch(PATCH, 0.002, seed=20, with_scratches=False)
s_scr   = make_flat_patch(PATCH, 0.002, seed=20, with_scratches=True, n_scratches=12)
s_rough = make_flat_patch(PATCH, 0.002, seed=20, with_scratches=False, target_ra_um=0.5)
gu_clean = gm.evaluate(s_clean)["summary"]["gu_mean"]
gu_scr   = gm.evaluate(s_scr)["summary"]["gu_mean"]
gu_rough = gm.evaluate(s_rough)["summary"]["gu_mean"]
check("5a. scratch → GU 감소", gu_scr < gu_clean, f"clean {gu_clean:.1f} vs scratched {gu_scr:.1f}")
check("5b. roughness↑ → GU 감소", gu_rough < gu_clean, f"clean {gu_clean:.1f} vs rough {gu_rough:.1f}")

# before/after — 폴리싱하면 GU 가 올라야 한다
s = make_flat_patch(PATCH, 0.002, seed=21, with_scratches=True, n_scratches=8)
gu_before = gm.evaluate(s)["summary"]["gu_mean"]
model = LiteraturePolishingModel(cal)
run_episode(model, s, Recipe(8.0, 5.0, 4000, 0.4, n_passes=2), PATCH, quality_dt_s=0.05)
r_after = gm.evaluate(s)
gu_after = r_after["summary"]["gu_mean"]
check("폴리싱 후 GU 상승", gu_after > gu_before, f"{gu_before:.1f} → {gu_after:.1f} GU")

# 6. Clearcoat 과다 제거 시 GU 항이 무너져 전체가 실패
s_over = make_flat_patch(PATCH, 0.002, seed=22, with_scratches=False)
s_over.clearcoat_remaining_um[:] = C.CLEARCOAT_SAFETY_LIMIT_UM - 1.0   # 안전한계 아래로 강제
r_over = gm.evaluate(s_over)
check("6. clearcoat 과다제거 → q_clearcoat=0·불합격",
      r_over["term_maps"]["q_clearcoat"].max() == 0.0
      and not r_over["summary"]["gloss_pass"],
      f"GU {r_over['summary']['gu_mean']:.1f}")

# 7. 같은 표면 재평가 시 결과 동일 (결정론)
r1 = gm.evaluate(make_flat_patch(PATCH, 0.002, seed=23))
r2 = gm.evaluate(make_flat_patch(PATCH, 0.002, seed=23))
check("7. 결정론 재현", np.array_equal(r1["gu_map"], r2["gu_map"]))

# optical 훅 — w_optical>0 + optical_map 이 결과에 반영되는가
gm_opt = LiteratureGlossProxyModel(GlossProxyConfig(w_optical=1.0))
s = make_flat_patch(PATCH, 0.002, seed=24, with_scratches=False)
lo = gm_opt.evaluate(s, optical_map=np.full((5, 5), 0.2))["summary"]["gu_mean"]
hi = gm_opt.evaluate(s, optical_map=np.full((5, 5), 1.0))["summary"]["gu_mean"]
check("optical 훅 동작 (0.2 < 1.0)", lo < hi, f"{lo:.1f} vs {hi:.1f} GU")

print(f"\nbefore/after 데모 (scratched patch, 8N/5mm/s/4000rpm/0.4 spacing, 2 pass):")
sm = r_after["summary"]
print(f"  GU {gu_before:.1f} → {gu_after:.1f} | p10 {sm['gu_p10']:.1f} | "
      f"std {sm['gu_std']:.1f} | pass={sm['gloss_pass']} | bands {sm['band_counts']}")

n_fail = results.count(False)
print(f"\n{len(results)-n_fail}/{len(results)} 통과")
sys.exit(1 if n_fail else 0)
