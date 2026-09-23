"""— Isaac Lab PolishEnv action=0 baseline 검증.

    ~/isaacsim/python.sh learning/rl/tests/test_lab_env_baseline.py --headless
"""
import argparse, sys, os

import os as _os
_REPO_ROOT = _os.path.dirname(_os.path.dirname(_os.path.dirname(_os.path.dirname(_os.path.abspath(__file__)))))
sys.path.insert(0, _REPO_ROOT)

from isaaclab.app import AppLauncher
parser = argparse.ArgumentParser()
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
app = AppLauncher(args).app

import numpy as np
import torch

from learning.rl.env.polish_env import PolishEnv
from learning.rl.env.polish_env_cfg import PolishEnvCfg

results = []
def check(name, ok, detail=""):
    results.append(ok)
    print(f"  [{'PASS' if ok else 'FAIL'}] {name}" + (f" — {detail}" if detail else ""))

cfg = PolishEnvCfg()
cfg.scene.num_envs = 4
env = PolishEnv(cfg, render_mode=None)
env.log_raw_steps = True
print(f"\nrecipe: force {env.recipe.target_contact_force_n}N feed {env.recipe.feed_speed_mm_s}mm/s "
      f"rpm {env.recipe.rpm:.0f} spacing {env.recipe.step_over_spacing_ratio} "
      f"passes {env.recipe.n_passes} | path {env._path_len:.2f} m")

obs, _ = env.reset()
check("reset: obs shape", tuple(obs["policy"].shape) == (4, 14), str(tuple(obs["policy"].shape)))
check("reset: thermal obs neutral", torch.allclose(obs["policy"][:, 11:14],
                                                   torch.zeros_like(obs["policy"][:, 11:14])))

# 열손상 dense reward는 누적 절대값이 아니라 이번 step의 증가량만 감점한다.
env._force_mean = env._force_cmd.clone()
env._defect_removal.zero_(); env._healthy_over.zero_(); env._action_rate.zero_()
env._thermal_damage_delta.zero_()
r_without_damage = env._get_rewards().clone()
env._thermal_damage_delta.fill_(0.001)
r_with_damage = env._get_rewards().clone()
check("thermal damage delta lowers dense reward",
      torch.all(r_with_damage < r_without_damage),
      f"delta reward={float((r_with_damage-r_without_damage).mean()):.3f}")
env._thermal_damage_delta.zero_()

# 초기 표면 저장 (재현성 비교용)
init_surf = env._surfaces[0].micro_height_um.copy()
zero = torch.zeros(4, 2, device=env.device)
cmd_orig = env.recipe.target_contact_force_n

# ── 영역 1: 도달 가능한 명령 (6.0 N < 정적 상한 8.4 N) → 정확 수렴해야 한다 ──
# 리셋 시 z_offset=press_max(0.08)에서 seek 상한 0.02 m/s 로 하강 — 평형 도달에 ~3 s.
# 정착 판정은 도착 후 과도가 끝난 8 s 이후 구간에서 한다.
env.recipe.target_contact_force_n = 6.0
f6 = []
for t in range(240):                     # 12 s
    obs, *_ = env.step(zero)
    f6.append(float(env._force_mean[0]))
s6 = np.array(f6[160:])                  # 8 s 이후
check("도달가능 명령 6.0N 정확 수렴", abs(s6.mean() - 6.0) < 0.05 and s6.std() < 0.05,
      f"달성 {s6.mean():.3f} ± {s6.std():.4f} N")
check("contact updates thermal observations",
      bool(torch.isfinite(obs["policy"][:, 11:14]).all()
           and (obs["policy"][:, 11] > 0).all()),
      f"thermal obs env0={obs['policy'][0, 11:14].tolist()}")

# ── 영역 2: 도달불가 명령 → 포화 limit cycle (화된 원본 거동) ──
# 정적 상한 = K·(2·cdist − press_min) = 350·0.024 = 8.4 N. z_offset 클램프 시 z_vel 을
# 리셋하지 않으므로(원본과 동일 — 안티와인드업 없음) 포화 경계에서 진동한다.
# 재탐색된 BO recipe(5.78N)는 상한 아래라, 포화 검증은 명시적으로 10 N 을 강제 주입한다.
#   (구버전 테스트는 recipe 8.86N 자체가 포화 표본이었다 — recipe 갱신과 함께 분리.)
env.recipe.target_contact_force_n = 10.0
forces = []
for t in range(240):                     # 12 s
    env.step(zero)
    forces.append(float(env._force_mean[0]))
f = np.array(forces)[160:]
check("도달불가 명령 10N → 포화 진동 (달성 < 명령)",
      8.0 < f.mean() < 9.5 and f.mean() < 9.9,
      f"달성 {f.mean():.2f} ± {f.std():.2f} N (정적 상한 8.4 N) — kinematic '명령=달성' 가정 반증")

# ── 영역 3: recipe 원복 후 에피소드 완주 + 품질 산출 ──
env.recipe.target_contact_force_n = cmd_orig
done_step, gu, scr = None, None, None
for t in range(12000):
    obs, rew, terminated, truncated, extras = env.step(zero)
    if (terminated | truncated).any() and done_step is None:
        done_step = t
        gu = extras.get("log", {}).get("Metrics/gu_mean")
        scr = extras.get("log", {}).get("Metrics/max_residual_scratch_um")
        break

check("에피소드 완주 (경로 끝 도달)", done_step is not None, f"{done_step} control steps")

# 품질 갱신 확인
s = env._surfaces  # reset 후라 이미 새 표면 — extras 의 종료 직전 요약을 사용
check("종료 시 GU 산출", gu is not None and 0 < gu < 100, f"GU {gu:.1f}")
check("제거 발생 (scratch 감소)", scr is not None and scr < 2.0, f"잔존 scratch {scr:.2f} μm")

# 리셋 재현성 — env0 의 두 번째 에피소드 seed 는 base+1, 첫 에피소드와 달라야 함
diff_surface = not np.array_equal(env._surfaces[0].micro_height_um, init_surf)
check("reset 후 새 표면 생성", diff_surface)

# 상태 누적 없음 — 접촉 상태 리셋 확인
check("reset 후 접촉 상태 초기화", float(env._arc[0]) == 0.0 and float(env.contact.filtered[0]) == 0.0)

# 병렬 독립성 — env 별 표면 seed 다름
check("env 간 표면 독립", not np.array_equal(env._surfaces[0].micro_height_um,
                                             env._surfaces[1].micro_height_um))

# raw step log 존재
check("raw step log 기록", len(env.step_log) > 100, f"{len(env.step_log)} rows")

n_fail = results.count(False)
print(f"\n{len(results)-n_fail}/{len(results)} 통과")
env.close(); app.close()
sys.exit(1 if n_fail else 0)
