""" — 같은 표면에서 baseline(action=0) vs PPO 잔차 비교.

    ~/isaacsim/python.sh learning/rl/eval_ppo.py --headless \
        --checkpoint <model_final.pt 경로> [--num_envs 8]

env i 는 두 조건에서 같은 surface seed 를 받는다 (짝지은 비교).
"""
import argparse
import os
import sys

import os as _os
_REPO_ROOT = _os.path.dirname(_os.path.dirname(_os.path.dirname(_os.path.abspath(__file__))))
sys.path.insert(0, _REPO_ROOT)

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
parser.add_argument("--checkpoint", type=str, required=True)
parser.add_argument("--num_envs", type=int, default=8)
parser.add_argument("--episodes", type=int, default=1, help="조건당 에피소드 라운드 수")
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
app = AppLauncher(args).app

from importlib import metadata  # noqa: E402

import numpy as np  # noqa: E402
import torch  # noqa: E402
from rsl_rl.runners import OnPolicyRunner  # noqa: E402

from isaaclab_rl.rsl_rl import RslRlVecEnvWrapper, handle_deprecated_rsl_rl_cfg  # noqa: E402

from learning.rl.env.polish_env import PolishEnv  # noqa: E402
from learning.rl.env.polish_env_cfg import PolishEnvCfg  # noqa: E402
from learning.rl.ppo_cfg import PolishPPORunnerCfg  # noqa: E402


def run_condition(env, policy_fn, rounds: int) -> dict:
    """전 env 를 리셋하고 rounds 라운드 완주. env 별 종료 품질을 모은다."""
    out = {"gu": [], "scratch": []}
    for _ in range(rounds):
        obs, _ = env.reset()
        done_envs = torch.zeros(env.num_envs, dtype=torch.bool, device=env.device)
        for _ in range(12000):     # episode_length_s=500 → 10000 control step + 여유
            with torch.no_grad():
                actions = policy_fn(obs)
            obs, _, terminated, truncated, extras = env.step(actions)
            newly = (terminated | truncated) & ~done_envs
            if newly.any():
                log = extras.get("log", {})
                if "Metrics/gu_mean" in log:
                    out["gu"].append(log["Metrics/gu_mean"])
                    out["scratch"].append(log["Metrics/max_residual_scratch_um"])
                done_envs |= (terminated | truncated)
            if done_envs.all():
                break
    return out


def main():
    env_cfg = PolishEnvCfg()
    env_cfg.scene.num_envs = args.num_envs
    env = PolishEnv(env_cfg, render_mode=None)
    wrapped = RslRlVecEnvWrapper(env, clip_actions=1.0)

    agent_cfg = handle_deprecated_rsl_rl_cfg(PolishPPORunnerCfg(),
                                             metadata.version("rsl-rl-lib"))
    runner = OnPolicyRunner(wrapped, agent_cfg.to_dict(), log_dir=None,
                            device=str(env.device))
    runner.load(args.checkpoint)
    policy = runner.get_inference_policy(device=str(env.device))

    print(f"\n=== 조건 1: baseline (action=0) — 표면 seed 고정 재현 ===")
    ep0 = int(env._episode_count[0])
    zero = torch.zeros(env.num_envs, 2, device=env.device)
    base = run_condition(env, lambda _obs: zero, args.episodes)

    # 같은 seed 재사용: episode_count 를 조건 1 시작 시점으로 되감는다
    env._episode_count[:] = ep0
    print(f"=== 조건 2: PPO 잔차 ({os.path.basename(args.checkpoint)}) ===")
    ppo = run_condition(env, lambda obs: policy(obs), args.episodes)

    def stats(name, d):
        gu, sc = np.array(d["gu"]), np.array(d["scratch"])
        print(f"{name:10s} GU {gu.mean():.2f} ± {gu.std():.2f}   "
              f"잔존 scratch {sc.mean():.3f} ± {sc.std():.3f} μm   (n={len(gu)})")
        return gu.mean(), sc.mean()

    print(f"\n{'=' * 64}")
    g0, s0 = stats("baseline", base)
    g1, s1 = stats("PPO", ppo)
    print(f"{'-' * 64}\nΔGU {g1 - g0:+.2f}  |  Δscratch {s1 - s0:+.3f} μm")
    verdict = "개선" if (g1 > g0 and s1 <= s0 + 0.05) else "미개선 — 보상/스케일 재점검"
    print(f"판정: {verdict}")

    env.close()


if __name__ == "__main__":
    main()
    app.close()
