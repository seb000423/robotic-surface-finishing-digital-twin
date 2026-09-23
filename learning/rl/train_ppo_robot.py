"""— PolishEnv 잔차 PPO 학습 (rsl_rl).

    ~/isaacsim/python.sh learning/rl/train_ppo.py --headless \
        [--num_envs 8] [--max_iterations 300]

검증 게이트: actor 초기 출력 std 를 작게 → 첫 정책 ≈ 기준 제어기.
결과: learning/rl/logs/polish_ppo/<날짜>/ 에 checkpoint + tensorboard.
"""
import argparse
import os
import sys
from datetime import datetime

import os as _os
_REPO_ROOT = _os.path.dirname(_os.path.dirname(_os.path.dirname(_os.path.abspath(__file__))))
sys.path.insert(0, _REPO_ROOT)

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
parser.add_argument("--num_envs", type=int, default=8)
parser.add_argument("--max_iterations", type=int, default=300)
parser.add_argument("--seed", type=int, default=42)
parser.add_argument("--resume", type=str, default=None,
                    help="이 checkpoint 에서 이어서 학습 (BC 부트스트랩 미세조정용)")
# ── BC 보호 ──
parser.add_argument("--lr", type=float, default=None, help="learning rate 재정의 (BC 보호: 1e-4)")
parser.add_argument("--clip_param", type=float, default=None, help="PPO clip 재정의 (BC 보호: 0.1)")
parser.add_argument("--desired_kl", type=float, default=None,
                    help="adaptive KL 목표 재정의 (BC 보호: 0.005 — 업데이트당 이탈 제한)")
parser.add_argument("--gamma", type=float, default=None,
                    help="할인율 재정의. 종말 보상은 에피소드(~4800스텝) 끝에만 나오므로 "
                         "0.99 로는 초반 상태에 신호가 닿지 않는다 (0.99^4800≈1e-21) → 0.9995 권장")
parser.add_argument("--entropy_coef", type=float, default=None,
                    help="entropy 계수 재정의 — BC 미세조정 시 후반 정책 붕괴 완화용 하향 "
                         "")
parser.add_argument("--freeze_actor_iters", type=int, default=0,
                    help="첫 N iter 동안 actor 동결 — critic 워밍업. bootstrap_bc 는 critic 을 "
                         "랜덤인 채 저장하므로, 워밍업 없이 미세조정하면 초기 advantage 노이즈가 "
                         "BC 정책을 먼저 파괴할 수 있다.")
parser.add_argument("--surface_kind", default="flat",
                    choices=["flat", "cylinder", "sphere"],
                    help="작업면 곡률 종류")
parser.add_argument("--curvature_radius", type=float, default=0.5)
parser.add_argument("--contact_mode", type=str, default="physical",
                    choices=["physical", "model"],
                    help="physical=PhysX 센서힘(검증 완료, C단계 기본) / model=가상 스프링힘")
parser.add_argument(
    "--output_root", type=str,
    default=os.path.join(_REPO_ROOT, "learning", "rl", "robot", "logs"),
    help="새 학습 출력 루트. 기존 PPO/thermal 결과는 덮어쓰지 않는다.")
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
app = AppLauncher(args).app

import torch  # noqa: E402
from rsl_rl.runners import OnPolicyRunner  # noqa: E402

from importlib import metadata  # noqa: E402

from isaaclab_rl.rsl_rl import RslRlVecEnvWrapper, handle_deprecated_rsl_rl_cfg  # noqa: E402

from learning.rl.env.robot_polish_env import RobotPolishEnv  # noqa: E402
from learning.rl.env.robot_polish_env_cfg import RobotPolishEnvCfg  # noqa: E402
from learning.rl.ppo_cfg import PolishPPORunnerCfg  # noqa: E402


def main():
    env_cfg = RobotPolishEnvCfg()
    env_cfg.surface_kind = args.surface_kind
    env_cfg.curvature_radius_m = args.curvature_radius
    env_cfg.scene.num_envs = args.num_envs
    env_cfg.seed = args.seed
    env_cfg.enable_pad_physical_contact = (args.contact_mode == "physical")
    env = RobotPolishEnv(env_cfg, render_mode=None)
    print(f"[train_ppo] contact_mode={args.contact_mode} "
          f"(enable_pad_physical_contact={env_cfg.enable_pad_physical_contact})")

    agent_cfg = PolishPPORunnerCfg()
    agent_cfg.max_iterations = args.max_iterations
    agent_cfg.seed = args.seed
    for name, dst in (("lr", "learning_rate"), ("clip_param", "clip_param"),
                      ("desired_kl", "desired_kl"), ("gamma", "gamma"),
                      ("entropy_coef", "entropy_coef")):
        v = getattr(args, name)
        if v is not None:
            setattr(agent_cfg.algorithm, dst, v)
            print(f"[train_ppo] algorithm.{dst} = {v}")
    # deprecated 필드(stochastic 등)를 설치된 rsl_rl 버전에 맞게 정리 — 공식 train.py 와 동일 경로
    agent_cfg = handle_deprecated_rsl_rl_cfg(agent_cfg, metadata.version("rsl-rl-lib"))

    log_dir = os.path.join(os.path.abspath(args.output_root),
                           datetime.now().strftime("%Y-%m-%d_%H-%M-%S"))
    os.makedirs(log_dir, exist_ok=True)
    print(f"[train_ppo] recipe: {env.recipe} | envs {args.num_envs} | log {log_dir}")

    wrapped = RslRlVecEnvWrapper(env, clip_actions=1.0)
    runner = OnPolicyRunner(wrapped, agent_cfg.to_dict(), log_dir=log_dir,
                            device=str(env.device))
    if args.resume:
        runner.load(args.resume)
        print(f"[train_ppo] resumed from {args.resume}")
    else:
        # actor 마지막(mean) 층 zero 초기화 → 초기 정책 = 기준 제어기(잔차 0).
        #   ppo_cfg 의 init_std 는 탐색 노이즈 폭일 뿐 평균출력을 0 으로 만들지 않는다.
        #   (실측: rsl_rl 5.0.1 기본 초기화의 초기 평균출력 |a| ≈ 0.03~0.14, 최대 0.4 —
        #    힘 최대 12%/이송 최대 20% 편차이고 seed 마다 방향이 달라 잡음원이었다.)
        last = [m for m in runner.alg.actor.mlp
                if isinstance(m, torch.nn.Linear)][-1]
        torch.nn.init.zeros_(last.weight)
        torch.nn.init.zeros_(last.bias)
        print("[train_ppo] actor mean 층 zero 초기화 — 초기 정책 = 기준 제어기")
    warm = args.freeze_actor_iters
    if warm > 0:
        # ── phase 1: actor 동결, critic 만 학습 ──
        # 동결 중 KL=0 이라 adaptive 스케줄은 lr 을 건드리지 않지만(kl>0 가드),
        # 해제 직후 첫 업데이트가 설정 lr 에서 출발하도록 명시 복원한다.
        for p in runner.alg.actor.parameters():
            p.requires_grad_(False)
        print(f"[train_ppo] critic 워밍업 — 첫 {warm} iter actor 동결")
        runner.learn(num_learning_iterations=warm, init_at_random_ep_len=False)
        for p in runner.alg.actor.parameters():
            p.requires_grad_(True)
        runner.alg.learning_rate = agent_cfg.algorithm.learning_rate
        for g in runner.alg.optimizer.param_groups:
            g["lr"] = agent_cfg.algorithm.learning_rate
        print(f"[train_ppo] actor 동결 해제 (lr={agent_cfg.algorithm.learning_rate}) — 본 학습")
    runner.learn(num_learning_iterations=agent_cfg.max_iterations - warm,
                 init_at_random_ep_len=False)

    final = os.path.join(log_dir, "model_final.pt")
    runner.save(final)
    print(f"[train_ppo] saved {final}")
    env.close()


if __name__ == "__main__":
    main()
    app.close()
