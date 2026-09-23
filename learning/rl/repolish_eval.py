"""
한 pass만 도는 학습/평가와 달리, 여기서는 정책이 실제로 목표에 도달할 때까지
(또는 실패 조건에 걸릴 때까지) 같은 표면을 반복 폴리싱한다:

    1 pass 수행 → 품질·안전 평가
      → 통과: 종료(success)
      → 미달 + 안전: 냉각 후 같은 표면에서 다음 pass
      → 안전 위반 / 최대 pass 초과 / 개선 없음: 실패 종료

이 순환은 RobotPolishEnv._get_dones()(repolish_mode=True일 때)가 내부에서 수행하고,
이 스크립트는 env.step() 을 반복 호출하며 env._repolish_log 에 쌓이는 시퀀스별
결과를 읽어 집계·CSV로 저장하는 바깥 루프다.

    ~/isaacsim/python.sh learning/rl/repolish_eval.py --headless \
        --checkpoint learning/rl/robot/logs/_13-14-15/model_700.pt \
        --num_envs 8 --num_sequences 3 --max_passes 6 --cooldown_s 20

주의: 재폴리싱은 반드시 물리 접촉 모드(PhysX 센서힘)로만 수행한다 — force_model_n
기반으로는 "실제로 목표에 도달했다"는 성공 판정을 신뢰할 수 없다.
"""
from __future__ import annotations

import argparse
import csv
import os
import sys

import os as _os
_REPO_ROOT = _os.path.dirname(_os.path.dirname(_os.path.abspath(__file__)))
sys.path.insert(0, _REPO_ROOT)

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
parser.add_argument("--checkpoint", type=str, required=True)
parser.add_argument("--surface_kind", default="flat",
                    choices=["flat", "cylinder", "sphere"],
                    help="작업면 곡률 종류")
parser.add_argument("--curvature_radius", type=float, default=0.5)
parser.add_argument("--num_envs", type=int, default=8)
parser.add_argument("--num_sequences", type=int, default=3,
                    help="env 당 반복할 (새 표면 → 재폴리싱 완료까지) 시퀀스 수")
parser.add_argument("--max_passes", type=int, default=6)
parser.add_argument("--cooldown_s", type=float, default=20.0)
parser.add_argument("--pass_time_factor", type=float, default=1.7,
                    help="pass 시간 여유계수 — dwell 정책은 공칭보다 오래 걸림 (9.25 타임아웃 2건 교정)")
parser.add_argument("--max_control_steps", type=int, default=200000,
                    help="안전 상한 — 이 스텝 안에 목표 시퀀스 수를 못 채우면 중단")
parser.add_argument("--out", type=str,
                    default=os.path.join(_REPO_ROOT, "learning", "rl", "robot", "results",
                                         "repolish_eval.csv"))
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
app = AppLauncher(args).app

import numpy as np  # noqa: E402
import torch  # noqa: E402
from rsl_rl.models.mlp_model import MLPModel  # noqa: E402
from tensordict import TensorDict  # noqa: E402

from learning.rl.env.robot_polish_env import RobotPolishEnv  # noqa: E402
from learning.rl.env.robot_polish_env_cfg import RobotPolishEnvCfg  # noqa: E402


def load_policy(checkpoint, device):
    ck = torch.load(checkpoint, map_location=device, weights_only=False)
    state = ck["actor_state_dict"]
    obs_dim = int(state["mlp.0.weight"].shape[1])
    dummy = TensorDict({"policy": torch.zeros(1, obs_dim, device=device)}, batch_size=[1])
    actor = MLPModel(
        dummy, {"actor": ["policy"]}, "actor", 2,
        hidden_dims=[128, 128], activation="elu", obs_normalization=True,
        distribution_cfg={"class_name": "GaussianDistribution", "init_std": 0.3,
                          "std_type": "scalar"}).to(device)
    actor.load_state_dict(state)
    actor.eval()

    def policy(obs):
        x = obs["policy"][:, :obs_dim]
        td = TensorDict({"policy": x}, batch_size=[len(x)])
        return actor(td).clamp(-1.0, 1.0)

    print(f"[repolish] loaded {checkpoint} (obs_dim={obs_dim})")
    return policy


def main():
    env_cfg = RobotPolishEnvCfg()
    env_cfg.surface_kind = args.surface_kind
    env_cfg.curvature_radius_m = args.curvature_radius
    env_cfg.scene.num_envs = args.num_envs
    env_cfg.enable_pad_physical_contact = True   # repolish는 반드시 실측 PhysX 힘 기준
    env_cfg.repolish_max_passes = args.max_passes
    env_cfg.repolish_cooldown_s = args.cooldown_s
    # 한 시퀀스(최대 pass 수)를 다 담을 수 있도록 넉넉히: 공칭 1 pass 완주 시간(BO recipe
    # 기준 raster 총 길이/feed, 대략 200~250s) + pass당 냉각시간을 max_passes 배 확보.
    nominal_pass_s = 260.0
    env_cfg.episode_length_s = args.max_passes * (nominal_pass_s * args.pass_time_factor + args.cooldown_s) + 60.0

    env = RobotPolishEnv(env_cfg, render_mode=None)
    env._repolish_mode = True
    print(f"[repolish] envs={args.num_envs} num_sequences={args.num_sequences} "
          f"max_passes={args.max_passes} cooldown={args.cooldown_s}s "
          f"episode_length_s={env_cfg.episode_length_s:.0f}")

    policy = load_policy(args.checkpoint, env.device)

    obs, _ = env.reset()
    seq_done = np.zeros(args.num_envs, dtype=int)
    rows = []
    step = 0
    target = args.num_envs * args.num_sequences
    while len(rows) < target and step < args.max_control_steps:
        with torch.no_grad():
            actions = policy(obs)
        obs, _, terminated, truncated, _ = env.step(actions)
        step += 1
        done_ids = (terminated | truncated).nonzero(as_tuple=False).squeeze(-1).cpu().tolist()
        for i in done_ids:
            if seq_done[i] >= args.num_sequences:
                continue
            log = env._repolish_log.pop(i, None)
            if log is None:
                continue
            b, f = log["before"], log["final"]
            rows.append({
                "checkpoint": os.path.basename(args.checkpoint), "env": i,
                "sequence": seq_done[i], "outcome": log["outcome"], "passes": log["passes"],
                "gu_before": round(b["gu"], 2), "gu_final": round(f["gu"], 2),
                "ra_final_um": round(f["ra"], 4), "rz_final_um": round(f["rz"], 3),
                "scratch_before_um": round(b["scratch"], 3), "scratch_final_um": round(f["scratch"], 3),
                "clearcoat_min_um": round(f["cc_min"], 2),
                "temperature_peak_c": round(f["temperature_peak_c"], 2),
                "thermal_damage_peak": round(f["thermal_damage_peak"], 6),
                "quality_ok": log["quality_ok"], "safety_ok": log["safety_ok"],
            })
            seq_done[i] += 1
        if step % 2000 == 0:
            print(f"[repolish] step={step} sequences={len(rows)}/{target} "
                  f"(min per-env={int(seq_done.min())})", flush=True)

    if step >= args.max_control_steps and len(rows) < target:
        print(f"[repolish] 안전 상한({args.max_control_steps} step) 도달 — "
              f"{len(rows)}/{target} 시퀀스만 수집됨")

    os.makedirs(os.path.dirname(args.out), exist_ok=True)
    if rows:
        with open(args.out, "w", newline="", encoding="utf-8") as fh:
            w = csv.DictWriter(fh, fieldnames=list(rows[0].keys()))
            w.writeheader(); w.writerows(rows)
        print(f"\n에피소드(시퀀스)별 CSV → {args.out}")

    n = len(rows)
    outcomes = {}
    for r in rows:
        outcomes[r["outcome"]] = outcomes.get(r["outcome"], 0) + 1
    succ = [r for r in rows if r["outcome"] == "success"]
    print(f"\n{'=' * 60}\n재폴리싱 결과 — n={n} 시퀀스 (env={args.num_envs} × seq={args.num_sequences})")
    for k, v in sorted(outcomes.items(), key=lambda kv: -kv[1]):
        print(f"  {k:24s} {v:3d}/{n}  ({v / max(n,1):.0%})")
    if succ:
        passes = np.array([r["passes"] for r in succ])
        gu = np.array([r["gu_final"] for r in succ])
        print(f"\n성공({len(succ)}건) 평균 pass 수: {passes.mean():.2f} (min {passes.min()}, max {passes.max()})")
        print(f"성공 시 최종 GU proxy 평균: {gu.mean():.2f}")
    print(f"전체 성공률: {len(succ)}/{n} = {len(succ) / max(n,1):.0%}")

    env.close()


if __name__ == "__main__":
    main()
    app.close()
