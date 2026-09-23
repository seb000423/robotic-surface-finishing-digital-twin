"""같은 표면 seed 에서 여러 정책 조건을 짝지어 비교 — 에피소드별 전·후 품질 CSV.

    ~/isaacsim/python.sh learning/rl/eval_conditions.py --headless \
        --conditions "baseline=,bc_champion=learning/rl/champion/model_bc.pt,..." \
        [--num_envs 8] [--episodes 2] [--out learning/rl/results/eval_conditions.csv]

`name=` (경로 없음) 은 action=0 baseline. 각 조건은 같은 seed 시퀀스를 재사용한다.
품질은 전부 SYNTHETIC — 논문 기반 디지털 트윈(GU proxy) 출력. 판정 기준은
literature-derived project target (GU≥70 / Ra≤0.20 / Rz≤2.0 / CC≥35 / scratch 감소).
"""
import argparse
import csv
import os
import sys

import os as _os
_REPO_ROOT = _os.path.dirname(_os.path.dirname(_os.path.dirname(_os.path.abspath(__file__))))
sys.path.insert(0, _REPO_ROOT)

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
parser.add_argument("--conditions", type=str, required=True,
                    help="쉼표구분 name=ckpt (ckpt 비우면 baseline)")
parser.add_argument("--num_envs", type=int, default=8)
parser.add_argument("--episodes", type=int, default=2)
parser.add_argument("--recipe_json", type=str, default=None,
                    help="env recipe 재정의 — 로봇 레시피 스크리닝용")
parser.add_argument("--surface_kind", default="flat",
                    choices=["flat", "cylinder", "sphere"],
                    help="작업면 곡률 종류")
parser.add_argument("--curvature_radius", type=float, default=0.5)
parser.add_argument("--contact_mode", type=str, default="physical",
                    choices=["physical", "model"],
                    help="physical=PhysX 센서힘(검증 완료, C단계 기본) / model=가상 스프링힘")
parser.add_argument("--out", type=str,
                    default=os.path.join(_REPO_ROOT, "learning", "rl", "results",
                                         "eval_conditions.csv"))
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
app = AppLauncher(args).app

import numpy as np  # noqa: E402
import torch  # noqa: E402
from rsl_rl.models.mlp_model import MLPModel  # noqa: E402
from tensordict import TensorDict  # noqa: E402

from learning.rl.env.robot_polish_env import RobotPolishEnv  # noqa: E402
from learning.rl.env.robot_polish_env_cfg import RobotPolishEnvCfg  # noqa: E402

RA_MAX, RZ_MAX, GU_MIN = 0.20, 2.0, 70.0


def load_policy(checkpoint, device):
    """Load either legacy 11-D or thermal 14-D actor into the same thermal env."""
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

    print(f"[eval] loaded {checkpoint} (obs_dim={obs_dim})")
    return policy


def run_condition(env, policy_fn, rounds, cond_name):
    """rounds 라운드 완주. env.last_episode_results 에서 에피소드별 전·후 품질을 수집."""
    rows = []
    for rnd in range(rounds):
        obs, _ = env.reset()
        done_envs = torch.zeros(env.num_envs, dtype=torch.bool, device=env.device)
        for _ in range(12000):
            with torch.no_grad():
                actions = policy_fn(obs)
            obs, _, terminated, truncated, extras = env.step(actions)
            newly = (terminated | truncated) & ~done_envs
            for i in newly.nonzero(as_tuple=False).squeeze(-1).cpu().tolist():
                r = env.last_episode_results.get(i)
                if r is None:
                    continue
                b, a = r["before"], r["after"]
                cc_ok = a["cc_min"] >= env.cfg.clearcoat_safety_limit_um
                rows.append({
                    "condition": cond_name, "round": rnd, "env": i,
                    "gu_before": round(b["gu"], 2), "gu_after": round(a["gu"], 2),
                    "scratch_before_um": round(b["scratch"], 3),
                    "scratch_after_um": round(a["scratch"], 3),
                    "ra_before_um": round(b["ra"], 4), "ra_after_um": round(a["ra"], 4),
                    "rz_before_um": round(b["rz"], 3), "rz_after_um": round(a["rz"], 3),
                    "clearcoat_min_um": round(a["cc_min"], 2),
                    "temperature_mean_c": round(a["temperature_mean_c"], 3),
                    "temperature_peak_c": round(a["temperature_peak_c"], 3),
                    "thermal_damage_peak": round(a["thermal_damage_peak"], 6),
                    "gu_pass": a["gu"] >= GU_MIN, "ra_pass": a["ra"] <= RA_MAX,
                    "rz_pass": a["rz"] <= RZ_MAX, "cc_pass": cc_ok,
                    "scratch_improved": r["scr_improved"],
                    "thermal_safe": r["thermal_safe"], "overall_pass": r["all_pass"],
                })
            done_envs |= (terminated | truncated)
            if done_envs.all():
                break
    return rows


def summarize(rows, name, ckpt):
    n = len(rows)
    f = lambda k: np.array([float(r[k]) for r in rows])
    c = lambda k: sum(1 for r in rows if r[k])
    print(f"\n── {name}  (ckpt: {ckpt or 'action=0'})  n={n} 에피소드 ──")
    print(f"  GU proxy   {f('gu_before').mean():6.2f} → {f('gu_after').mean():6.2f}"
          f"   | GU≥70   {c('gu_pass'):2d}/{n}")
    print(f"  scratch    {f('scratch_before_um').mean():6.3f} → {f('scratch_after_um').mean():6.3f} um"
          f" | 개선     {c('scratch_improved'):2d}/{n}")
    print(f"  Ra         {f('ra_before_um').mean():6.4f} → {f('ra_after_um').mean():6.4f} um"
          f" | <=0.20  {c('ra_pass'):2d}/{n}")
    print(f"  Rz         {f('rz_before_um').mean():6.3f} → {f('rz_after_um').mean():6.3f} um"
          f" | <=2.0   {c('rz_pass'):2d}/{n}")
    print(f"  CC 최소     {f('clearcoat_min_um').min():6.2f} um"
          f"           | >=35    {c('cc_pass'):2d}/{n}")
    print(f"  온도       mean {f('temperature_mean_c').mean():6.2f} °C | "
          f"peak {f('temperature_peak_c').max():6.2f} °C | thermal-safe {c('thermal_safe'):2d}/{n}")
    print(f"  열손상 peak 평균 {f('thermal_damage_peak').mean():.6f}")
    print(f"  전체 동시 통과 {c('overall_pass'):2d}/{n}")
    return {"name": name, "n": n, "gu_after": float(f('gu_after').mean()),
            "scratch_after": float(f('scratch_after_um').mean()),
            "overall_pass": c('overall_pass'), "gu_pass": c('gu_pass')}


def main():
    conds = []
    for tok in args.conditions.split(","):
        name, _, ckpt = tok.partition("=")
        conds.append((name.strip(), ckpt.strip() or None))

    env_cfg = RobotPolishEnvCfg()
    env_cfg.surface_kind = args.surface_kind
    env_cfg.curvature_radius_m = args.curvature_radius
    if args.recipe_json:
        env_cfg.recipe_json_path = args.recipe_json
        print(f"[cfg] recipe_json = {args.recipe_json}")
    env_cfg.scene.num_envs = args.num_envs
    env_cfg.enable_pad_physical_contact = (args.contact_mode == "physical")
    env = RobotPolishEnv(env_cfg, render_mode=None)
    print(f"[eval] contact_mode={args.contact_mode} "
          f"(enable_pad_physical_contact={env_cfg.enable_pad_physical_contact})")
    ep0 = env._episode_count.copy()
    zero = torch.zeros(env.num_envs, 2, device=env.device)

    all_rows, summaries = [], []
    for name, ckpt in conds:
        env._episode_count[:] = ep0            # 같은 seed 시퀀스 재사용 (짝지은 비교)
        if ckpt is None:
            rows = run_condition(env, lambda _o: zero, args.episodes, name)
        else:
            policy = load_policy(ckpt, env.device)
            rows = run_condition(env, lambda o: policy(o), args.episodes, name)
        for r in rows:
            r["checkpoint"] = ckpt or "action=0"
            r["contact_mode"] = args.contact_mode
        all_rows += rows
        summaries.append(summarize(rows, name, ckpt))

    os.makedirs(os.path.dirname(args.out), exist_ok=True)
    cols = list(all_rows[0].keys())
    with open(args.out, "w", newline="", encoding="utf-8") as fh:
        w = csv.DictWriter(fh, fieldnames=cols)
        w.writeheader(); w.writerows(all_rows)
    print(f"\n{'=' * 64}\n에피소드별 CSV → {args.out}")
    best = max(summaries, key=lambda s: (s["overall_pass"], s["gu_after"]))
    print(f"품질 판정 1위: {best['name']} (동시통과 {best['overall_pass']}/{best['n']}, "
          f"GU {best['gu_after']:.2f})")
    env.close()


if __name__ == "__main__":
    main()
    app.close()
