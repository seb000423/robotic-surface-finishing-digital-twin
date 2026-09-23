"""Runtime gate for the M0609+v5 pad coupled RL environment.

This is an executable Isaac Lab smoke/stability test, not a pytest unit test.
"""
from __future__ import annotations

import argparse
import os
import sys

_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
sys.path.insert(0, _REPO_ROOT)

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
parser.add_argument("--num_envs", type=int, default=1)
parser.add_argument("--steps", type=int, default=900)
parser.add_argument("--log_every", type=int, default=120)
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
app = AppLauncher(args).app

import torch  # noqa: E402

from learning.rl.env.robot_polish_env import RobotPolishEnv  # noqa: E402
from learning.rl.env.robot_polish_env_cfg import RobotPolishEnvCfg  # noqa: E402


def main():
    cfg = RobotPolishEnvCfg()
    cfg.scene.num_envs = args.num_envs
    cfg.episode_length_s = max(cfg.episode_length_s, args.steps * cfg.sim.dt * cfg.decimation + 1.0)
    env = RobotPolishEnv(cfg, render_mode=None)
    if getattr(args, "viz", "none") != "none":
        env.sim.set_camera_view(eye=(1.05, -0.90, 1.10), target=(0.45, 0.0, 0.40))
    env.log_raw_steps = True
    obs, _ = env.reset()
    assert obs["policy"].shape == (args.num_envs, cfg.observation_space)

    max_force = torch.zeros(args.num_envs, device=env.device)
    min_gap = torch.full((args.num_envs,), 1.0, device=env.device)
    finite = True
    for step in range(args.steps):
        if getattr(args, "viz", "none") != "none" and step in (0, 10, 30):
            env.sim.set_camera_view(eye=(1.05, -0.90, 1.10), target=(0.45, 0.0, 0.40))
        action = torch.zeros((args.num_envs, cfg.action_space), device=env.device)
        obs, reward, terminated, truncated, _ = env.step(action)
        finite = finite and bool(torch.isfinite(obs["policy"]).all()) and bool(torch.isfinite(reward).all())
        max_force = torch.maximum(max_force, env._force_used_n)
        min_gap = torch.minimum(min_gap, env._pad_gap_m)
        if step % args.log_every == 0 or step == args.steps - 1:
            print(
                f"[robot-gate] step={step:04d} envs={args.num_envs} "
                f"pad_uv0=({float(env._pad_uv_actual[0,0]):+.4f},{float(env._pad_uv_actual[0,1]):+.4f})m "
                f"gap0={float(env._pad_gap_m[0]):+.4f}m "
                f"force_cmd0={float(env._force_cmd[0]):.3f}N "
                f"force_sensor0={float(env._force_sensor_n[0]):.3f}N "
                f"force_model0={float(env._force_model_n[0]):.3f}N "
                f"force_used0={float(env._force_used_n[0]):.3f}N "
                f"temp0={float(env._surfaces[0].temperature_c.mean()):.2f}C",
                flush=True,
            )
        if bool((terminated | truncated).any()):
            print(f"[robot-gate] reset observed at step {step}")

    pad_count = sum(name == "polishing_contact_pad" for name in env.robot.body_names)
    passed = finite and pad_count == 1 and bool((max_force > 0.5).all())
    print(
        f"[robot-gate] RESULT={'PASS' if passed else 'FAIL'} envs={args.num_envs} "
        f"pad_body_count={pad_count} finite={finite} "
        f"max_force_N={max_force.detach().cpu().numpy().round(3).tolist()} "
        f"min_gap_m={min_gap.detach().cpu().numpy().round(4).tolist()} "
        f"gpu={env.device}",
        flush=True,
    )
    env.close()
    if not passed:
        raise RuntimeError("M0609+pad runtime gate failed")


if __name__ == "__main__":
    try:
        main()
    except BaseException as exc:
        import traceback
        print(f"[robot-gate] EXCEPTION: {type(exc).__name__}: {exc}", flush=True)
        traceback.print_exc()
    finally:
        app.close()
