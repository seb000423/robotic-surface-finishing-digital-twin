"""수제 dwell 정책 → actor 모방학습(BC) 부트스트랩.

    ~/isaacsim/python.sh learning/rl/bootstrap_bc.py --headless

왜: 백지 PPO 는 4차 만에 "개선"까지 왔지만 수제 정책(+0.81 GU / scratch −29%)에 못 미친다.
    이기는 정책이 손에 있으니 그걸 출발점으로 주고 PPO 로 다듬는다 —
    프로젝트의 BC→잔차 RL 철학을 이 단계에도 그대로 적용.

산출: learning/rl/logs/polish_ppo/bootstrap/model_bc.pt  (rsl_rl runner 형식 — resume 가능)
"""
import argparse
import os
import sys

import os as _os
_REPO_ROOT = _os.path.dirname(_os.path.dirname(_os.path.dirname(_os.path.abspath(__file__))))
sys.path.insert(0, _REPO_ROOT)

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
parser.add_argument("--num_envs", type=int, default=16)
parser.add_argument("--collect_steps", type=int, default=2500, help="env 당 수집 control step")
parser.add_argument("--bc_epochs", type=int, default=60)
parser.add_argument("--recipe_json", type=str, default=None,
                    help="학습 env 의 process-context recipe 재정의 (새 레시피 분포로 재학습). 기본: 레거시 bo_best_recipe")
parser.add_argument("--t_cc_use", type=float, default=None,
                    help="clearcoat 소모 벌점 가중치 재정의 (기본 cfg=0; 14ch 챔피언 재현엔 30)")
parser.add_argument("--obs", default="thermal",
                    choices=["basic", "thermal", "spatial", "full"],
                    help="관측 프로파일 (polish_env_cfg.apply_obs_profile)")
parser.add_argument("--side_ratio", type=float, default=0.0,
                    help="side(수직면) 접촉 env 비율 — side 학습 시 0.5")
parser.add_argument("--out_name", type=str, default="model_bc.pt",
                    help="champion/ 에 저장할 파일명 — 변형 학습 시 덮어쓰기 방지용")
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
app = AppLauncher(args).app

from importlib import metadata  # noqa: E402

import torch  # noqa: E402
from rsl_rl.runners import OnPolicyRunner  # noqa: E402

from isaaclab_rl.rsl_rl import RslRlVecEnvWrapper, handle_deprecated_rsl_rl_cfg  # noqa: E402

from learning.rl.env.polish_env import PolishEnv  # noqa: E402
from learning.rl.env.polish_env_cfg import PolishEnvCfg, apply_obs_profile  # noqa: E402
from learning.rl.ppo_cfg import PolishPPORunnerCfg  # noqa: E402


def scripted_action(obs_policy: torch.Tensor) -> torch.Tensor:
    """존재 증명에서 이긴 수제 정책. obs[:,6] = 코어 잔여 scratch max / 2.0."""
    scratch_max = obs_policy[:, 6] * 2.0
    on = scratch_max > 0.3
    a = torch.empty(obs_policy.shape[0], 2, device=obs_policy.device)
    a[:, 0] = torch.where(on, 1.0, -0.3)     # Δforce ratio
    a[:, 1] = torch.where(on, -1.0, 0.5)     # Δfeed ratio
    return a


def main():
    env_cfg = PolishEnvCfg()
    env_cfg.scene.num_envs = args.num_envs
    env_cfg.side_env_ratio = args.side_ratio
    apply_obs_profile(env_cfg, args.obs)
    if args.recipe_json:
        env_cfg.recipe_json_path = args.recipe_json
        print(f"[cfg] recipe_json = {args.recipe_json}")
    if args.t_cc_use is not None:
        env_cfg.t_cc_use = args.t_cc_use
        print(f"[cfg] t_cc_use = {args.t_cc_use}")
    env = PolishEnv(env_cfg, render_mode=None)
    wrapped = RslRlVecEnvWrapper(env, clip_actions=1.0)
    agent_cfg = handle_deprecated_rsl_rl_cfg(PolishPPORunnerCfg(),
                                             metadata.version("rsl-rl-lib"))
    runner = OnPolicyRunner(wrapped, agent_cfg.to_dict(), log_dir=None,
                            device=str(env.device))
    actor = runner.alg.actor

    # ── 1) 수제 정책으로 (obs, action) 수집 — 탐색 다양성 위해 약간의 노이즈 섞음 ──
    print(f"[bc] 데이터 수집: {args.num_envs} env × {args.collect_steps} steps")
    obs, _ = env.reset()
    obs_buf, act_buf = [], []
    for t in range(args.collect_steps):
        a = scripted_action(obs["policy"])
        obs_buf.append(obs["policy"].clone())
        act_buf.append(a.clone())
        noisy = (a + 0.1 * torch.randn_like(a)).clamp(-1, 1)
        obs, _, term, trunc, _ = env.step(noisy)
    X = torch.cat(obs_buf)                       # (N, 11)
    Y = torch.cat(act_buf)                       # (N, 2)
    on_ratio = float((X[:, 6] * 2.0 > 0.3).float().mean())
    print(f"[bc] {len(X)} 샘플 (스크래치-위 비율 {on_ratio:.2f})")

    # ── 2) actor 지도학습 — 정규화 통계 먼저 흡수, 그 뒤 MSE ──
    from tensordict import TensorDict
    td_all = TensorDict({"policy": X}, batch_size=[len(X)])
    actor.train()
    if hasattr(actor, "update_normalization"):
        actor.update_normalization(td_all)

    opt = torch.optim.Adam(actor.parameters(), lr=1e-3)
    n = len(X)
    for ep in range(args.bc_epochs):
        perm = torch.randperm(n, device=X.device)
        tot = 0.0
        for i in range(0, n, 4096):
            idx = perm[i:i + 4096]
            td = TensorDict({"policy": X[idx]}, batch_size=[len(idx)])
            pred = actor(td)                     # deterministic mean
            loss = torch.nn.functional.mse_loss(pred, Y[idx])
            opt.zero_grad(); loss.backward(); opt.step()
            tot += float(loss) * len(idx)
        if ep % 10 == 0 or ep == args.bc_epochs - 1:
            print(f"[bc] epoch {ep:3d}  MSE {tot / n:.4f}")

    # ── 3) 검증: 모방된 actor 가 수제 정책과 같은 구분을 내는가 ──
    actor.eval()
    with torch.no_grad():
        pred = actor(td_all).clamp(-1, 1)
    on = X[:, 6] * 2.0 > 0.3
    print(f"[bc] 모방 검증 — 스크래치 위  Δforce {pred[on, 0].mean():+.2f} Δfeed {pred[on, 1].mean():+.2f} "
          f"(목표 +1.0/-1.0)")
    print(f"[bc]             스크래치 밖  Δforce {pred[~on, 0].mean():+.2f} Δfeed {pred[~on, 1].mean():+.2f} "
          f"(목표 -0.3/+0.5)")

    out_dir = os.path.join(_REPO_ROOT, "learning", "rl", "champion")
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, args.out_name)
    # rsl_rl 5.0.1: log_dir=None 이면 logger.writer 미정의 → runner.save 가 torch.save 후
    # 업로드 훅에서 AttributeError (파일은 이미 저장됨). writer 를 명시해 훅을 무해화한다.
    runner.logger.writer = None
    runner.logger.logger_type = "none"
    runner.save(path)
    print(f"[bc] saved {path}")
    env.close()


if __name__ == "__main__":
    main()
    app.close()
