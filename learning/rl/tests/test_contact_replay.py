"""이식 검증 — 가상 스프링 이식이 원본과 같은 물건인지 확인한다.

RL_ISAACLAB_.md M1 의 사전 검사. Isaac Sim 없이 돈다 (torch + numpy 만 필요).

    ~/IsaacLab/.venv/bin/python learning/rl/tests/test_contact_replay.py

검사 4개:
  A. 상수 대조   — scripts/polishing_v5_modules/common.py 원본과 값이 같은가 (원본 수정 감지)
  B. 로그 대조   — 원 시뮬 로그의 virtual_force 를 이식한 스프링식이 재현하는가
  C. 폐루프      — 어드미턴스가 목표힘에 수렴하는가 / 압입 포화율은 얼마인가 (M2 가설)
  D. BC 전이 — 폐루프가 만드는 힘 분포가 BC 학습 분포와 호환인가
"""
import csv
import os
import re
import sys

import numpy as np
import torch

HERE = os.path.dirname(os.path.abspath(__file__))
RL_DIR = os.path.dirname(HERE)
PROJECT_ROOT = os.path.dirname(os.path.dirname(RL_DIR))
sys.path.insert(0, os.path.join(RL_DIR, "env"))
sys.path.insert(0, os.path.join(PROJECT_ROOT, "learning", "bc"))

import contact as C  # noqa: E402

COMMON_PY = os.path.join(PROJECT_ROOT, "scripts", "polishing_v5_modules", "common.py")
LOG_C = os.path.join(PROJECT_ROOT, "scripts", "force_log_rail_C.csv")

results = []


def check(name, ok, detail=""):
    results.append((name, ok, detail))
    print(f"  [{'PASS' if ok else 'FAIL'}] {name}" + (f" — {detail}" if detail else ""))


# ── A. 상수 대조 ───────────────────────────────────────────────────────────
def read_const(src, name):
    m = re.search(rf"^{name}\s*=\s*([-\d.]+)", src, re.M)
    return float(m.group(1)) if m else None


print("\nA. 상수 대조 (원본 common.py vs 이식본)")
src = open(COMMON_PY, encoding="utf-8").read()
PAIRS = [
    ("VIRTUAL_PAD_STIFFNESS", C.VIRTUAL_PAD_STIFFNESS),
    ("VIRTUAL_PAD_DAMPING", C.VIRTUAL_PAD_DAMPING),
    ("FORCE_FILTER_ALPHA", C.FORCE_FILTER_ALPHA),
    ("FORCE_CONTROL_CLIP_N", C.FORCE_CONTROL_CLIP_N),
    ("ADMITTANCE_MASS", C.ADMITTANCE_MASS),
    ("ADMITTANCE_DAMPING", C.ADMITTANCE_DAMPING),
    ("ADMITTANCE_MAX_VEL", C.ADMITTANCE_MAX_VEL),
    ("SURFACE_GUARD_MIN_CLEARANCE", C.SURFACE_GUARD_MIN_CLEARANCE),
    ("VIRTUAL_PAD_CONTACT_DISTANCE", C.CONTACT_DIST_TOP),
    ("SIDE_VIRTUAL_PAD_CONTACT_DISTANCE", C.CONTACT_DIST_SIDE),
    ("PRESS_OFFSET_MIN", C.PRESS_MIN_TOP),
    ("PRESS_OFFSET_MAX", C.PRESS_MAX_TOP),
    ("SIDE_PRESS_OFFSET_MIN", C.PRESS_MIN_SIDE),
    ("SIDE_PRESS_OFFSET_MAX", C.PRESS_MAX_SIDE),
    ("PHYSICAL_FORCE_SOFT_LIMIT_TOP_N", C.SOFT_LIMIT_TOP),
    ("PHYSICAL_FORCE_SOFT_LIMIT_SIDE_N", C.SOFT_LIMIT_SIDE),
    ("TARGET_FORCE_TOP_FLAT", C.TARGET_FORCE_TOP_FLAT),
    ("TARGET_FORCE_TOP_STEEP", C.TARGET_FORCE_TOP_STEEP),
    ("TARGET_FORCE_SIDE_FLAT", C.TARGET_FORCE_SIDE_FLAT),
    ("TARGET_FORCE_SIDE_STEEP", C.TARGET_FORCE_SIDE_STEEP),
    ("ADAPTIVE_FORCE_TILT_SPAN", C.ADAPTIVE_FORCE_TILT_SPAN),
]
drift = [(n, read_const(src, n), v) for n, v in PAIRS if read_const(src, n) != v]
check("상수 21개 원본 일치", not drift,
      "일치" if not drift else f"불일치 {drift}")

# ── B. 로그 대조 ───────────────────────────────────────────────────────────
print("\nB. 로그 대조 (원 시뮬 C 레일 force_log)")
rows = list(csv.DictReader(open(LOG_C)))
col = lambda k: np.array([float(r[k]) for r in rows])
vf, ac = col("virtual_force"), col("actual_clearance")
comp = np.clip(C.CONTACT_DIST_TOP - ac, 0.0, None)
pred = np.clip(C.VIRTUAL_PAD_STIFFNESS * comp, 0.0, C.SOFT_LIMIT_TOP)
m = vf > 0
r = float(np.corrcoef(pred[m], vf[m])[0, 1])
mae = float(np.abs(pred[m] - vf[m]).mean())
# 정적항만 비교한다. 로그에 z_vel 이 없어 댐핑항(−35·z_vel)은 뺄 수 없고, 그게 잔차의 정체다.
check("스프링식 재현 (정적항, r > 0.99)", r > 0.99, f"r={r:.4f}, MAE={mae:.3f}N ({m.sum()}행)")

lag = float((ac - np.clip(col("z_offset") - C.CONTACT_DIST_TOP, 0.0, C.PRESS_MAX_TOP)).mean())
check("추종지연 실측 확인 (기대값 ~2cm)", 0.010 < lag < 0.030, f"{lag * 100:.2f} cm")

# ── C. 폐루프 ──────────────────────────────────────────────────────────────
print("\nC. 폐루프 — 원본 병리 재현 / 제거 (M2 가설)")
dev = "cuda" if torch.cuda.is_available() else "cpu"
STEPS = 600  # 10초 @ 60Hz


def run(target_n, lag_offset, n=512, side=False):
    ct = C.VirtualPadContact(n, dev, lag_offset=lag_offset)
    is_side = torch.full((n,), side, dtype=torch.bool, device=dev)
    ct.reset(is_side=is_side)
    tgt = torch.full((n,), float(target_n), device=dev)
    hist = [ct.step(tgt, is_side).clone() for _ in range(STEPS)]
    f = torch.stack(hist)                      # (T, E)
    tail = f[-120:]                            # 마지막 2초 = 정상상태
    return (float(tail.mean()), float((tail.mean(0) - tgt).abs().mean()),
            float(ct.saturation(is_side).float().mean()), f)


# C-1. 실측 지연을 넣으면 원본의 병리(힘 부족 + 압입 포화)가 재현되어야 한다.
#      이게 재현되지 않으면 이식이 원본과 다른 물건이라는 뜻이다.
mean_lag, err_lag, sat_lag, _ = run(5.82, C.MEASURED_LAG_OFFSET)   # 규칙 명령 평균 5.82N
check("실측 지연 시 힘 부족 재현 (명령 5.82N > 실측)", mean_lag < 5.82,
      f"명령 5.82N → 달성 {mean_lag:.2f}N (시연 실측 3.12N)")
check("실측 지연 시 압입 포화 재현 (시연 65.7%)", sat_lag > 0.5,
      f"포화율 {sat_lag * 100:.1f}%")

# C-2. 지연을 제거하면(새 환경) 폐루프가 목표힘에 수렴하고 포화가 사라져야 한다.
mean_b, err_b, sat_b, _ = run(3.12, 0.0)       # BC 목표 = 시연 실측 평균
check("지연 제거 시 BC 목표 3.12N 수렴", err_b < 0.10,
      f"정상상태 {mean_b:.3f}N, 오차 {err_b:.4f}N")
check("M2 가설 — 포화율이 시연 65.7%보다 낮음", sat_b < 0.657,
      f"{sat_b * 100:.1f}% (지연 있을 때 {sat_lag * 100:.1f}%)")

# C-3. 도달 가능 상한 — 완전추종 top 레일에서 press_min 까지 눌렀을 때의 힘.
#      정적항  K·(2·cdist − press_min) = 350·(0.036 − 0.012)          = 8.40 N
#      댐핑항  −D·z_vel, 포화 시 z_vel 은 하한 −ADMITTANCE_MAX_VEL 에 물림
#              −35·(−0.02)                                            = +0.70 N
#      합계                                                            = 9.10 N
#      원본은 z_offset 을 클램프할 때 z_vel 을 리셋하지 않는다(안티와인드업 없음).
#        그래서 포화 상태에 +0.7N 의 유령 힘이 남는다. 원본 거동이므로 그대로 이식했다.
#        새 환경(lag_offset=0)에서는 포화가 거의 없어 영향이 없지만, 지연을 켜고 실험할 때는
#        이 항이 힘에 섞인다는 것을 알고 있을 것.
mean_hi, _, sat_hi, _ = run(12.0, 0.0)
check("도달 상한 ≈ 9.1N (정적 8.4 + 댐핑 0.7)", abs(mean_hi - 9.1) < 0.15,
      f"목표 12N 요구 시 달성 {mean_hi:.2f}N, 포화율 {sat_hi * 100:.1f}%")

# ── D. BC 전이 ─────────────────────────────────────────────────────────────
print("\nD. BC 전이 사전 점검")
from bc_policy import BCPolicy  # noqa: E402

_, _, _, f_hist = run(3.12, 0.0)
forces = f_hist[-300:].flatten().cpu().numpy()
policy = BCPolicy(os.path.join(PROJECT_ROOT, "learning", "bc", "bc_mlp.pt"), device=dev)
rep = policy.check_transfer(forces)
check("check_transfer 호환", bool(rep["compatible"]),
      f"{rep['verdict']} | 평균 {rep['observed_mean']:.2f}N (학습 {rep['train_mean']}N), "
      f"배율 {rep['scale_ratio']:.2f}, 범위내 {rep['in_range_ratio'] * 100:.0f}%")

# ── 요약 ───────────────────────────────────────────────────────────────────
n_fail = sum(1 for _, ok, _ in results if not ok)
print(f"\n{'=' * 66}\n{len(results) - n_fail}/{len(results)} 통과" +
      ("" if n_fail == 0 else f" — 실패 {n_fail}건"))
sys.exit(1 if n_fail else 0)
