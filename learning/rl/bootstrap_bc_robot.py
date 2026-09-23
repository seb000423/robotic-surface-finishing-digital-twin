"""수제 dwell 정책 → actor 모방학습(BC) 부트스트랩 — RobotPolishEnv(물리 접촉) 판.

    ~/isaacsim/python.sh learning/rl/bootstrap_bc.py --headless

왜: 백지 PPO 는 수제 정책에 못 미친다 — 이기는 규칙 정책을 출발점으로 주고 PPO 로
    다듬는다. C단계는 검증된 PhysX 접촉력 환경
    (contact_validation 게이트 통과)에서 데이터를 새로 수집한다 — 기존 해석식 환경
    체크포인트(learning/rl/champion, learning/rl/thermal)는 재사용·덮어쓰기하지 않는다.

에피소드 접근)이 데이터에 포함되는지 스텝 계수로 확인해 출력한다.

산출: learning/rl/robot/champion/model_bc_robot.pt  (rsl_rl checkpoint schema — resume 가능)
"""
import argparse
import os
import sys
import time

import os as _os
_REPO_ROOT = _os.path.dirname(_os.path.dirname(_os.path.dirname(_os.path.abspath(__file__))))
sys.path.insert(0, _REPO_ROOT)

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
parser.add_argument("--num_envs", type=int, default=16)
parser.add_argument("--collect_steps", type=int, default=5500,
                    help="env 당 수집 control step. 5500 이면 공칭 완주(~4800)+재접근 포함")
parser.add_argument("--bc_epochs", type=int, default=60)
parser.add_argument("--seed", type=int, default=42)
parser.add_argument("--surface_kind", default="flat",
                    choices=["flat", "cylinder", "sphere"],
                    help="작업면 곡률 종류")
parser.add_argument("--curvature_radius", type=float, default=0.5)
parser.add_argument("--contact_mode", type=str, default="physical",
                    choices=["physical", "model"],
                    help="physical=PhysX 센서힘(검증 완료, C단계 기본) / model=가상 스프링힘")
parser.add_argument(
    "--output", type=str,
    default=os.path.join(_REPO_ROOT, "learning", "rl", "robot", "champion",
                         "model_bc_robot.pt"),
    help="새 BC checkpoint 경로. 기존 champion(해석식 환경)은 덮어쓰지 않는다.")
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
app = AppLauncher(args).app

from importlib import metadata  # noqa: E402

import torch  # noqa: E402
from rsl_rl.runners import OnPolicyRunner  # noqa: E402

from isaaclab_rl.rsl_rl import RslRlVecEnvWrapper, handle_deprecated_rsl_rl_cfg  # noqa: E402

from learning.rl.env.robot_polish_env import RobotPolishEnv  # noqa: E402
from learning.rl.env.robot_polish_env_cfg import RobotPolishEnvCfg  # noqa: E402
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
    torch.manual_seed(args.seed)
    path = os.path.abspath(args.output)
    os.makedirs(os.path.dirname(path), exist_ok=True)
    env_cfg = RobotPolishEnvCfg()
    env_cfg.surface_kind = args.surface_kind
    env_cfg.curvature_radius_m = args.curvature_radius
    env_cfg.scene.num_envs = args.num_envs
    env_cfg.seed = args.seed
    env_cfg.enable_pad_physical_contact = (args.contact_mode == "physical")
    env = RobotPolishEnv(env_cfg, render_mode=None)
    print(f"[bc] contact_mode={args.contact_mode} "
          f"(enable_pad_physical_contact={env_cfg.enable_pad_physical_contact})")
    wrapped = RslRlVecEnvWrapper(env, clip_actions=1.0)
    agent_cfg = handle_deprecated_rsl_rl_cfg(PolishPPORunnerCfg(),
                                             metadata.version("rsl-rl-lib"))
    bootstrap_log_dir = os.path.join(os.path.dirname(path), "bootstrap_log")
    os.makedirs(bootstrap_log_dir, exist_ok=True)
    runner = OnPolicyRunner(wrapped, agent_cfg.to_dict(), log_dir=bootstrap_log_dir,
                            device=str(env.device))
    actor = runner.alg.actor

    # ── 1) 수제 정책으로 (obs, action) 수집 — 탐색 다양성 위해 약간의 노이즈 섞음 ──
    print(f"[bc] 데이터 수집: {args.num_envs} env × {args.collect_steps} steps")
    obs, _ = env.reset()
    obs_buf, act_buf = [], []
    n_contact = 0          # force_used > 0.05 N (접촉 중 가공)
    n_free = 0             # 자유공간/접근
    n_episode_end = 0      # 경로 끝/종료 → 다음 스텝부터 재접근
    n_fault = 0            # 센서 fault (fallback 사용 스텝 — 결과에 명시)
    t0 = time.time()
    for t in range(args.collect_steps):
        a = scripted_action(obs["policy"])
        obs_buf.append(obs["policy"].clone())
        act_buf.append(a.clone())
        noisy = (a + 0.1 * torch.randn_like(a)).clamp(-1, 1)
        obs, _, term, trunc, _ = env.step(noisy)
        in_contact = env._force_used_n > 0.05
        n_contact += int(in_contact.sum())
        n_free += int((~in_contact).sum())
        n_episode_end += int((term | trunc).sum())
        n_fault += int(env._sensor_fault.sum())
        if t % 500 == 0 or t == args.collect_steps - 1:
            rate = (t + 1) / max(time.time() - t0, 1e-6)
            print(f"[bc] collect {t:5d}/{args.collect_steps} "
                  f"({rate:.1f} steps/s, 잔여 {((args.collect_steps - t - 1) / max(rate, 1e-6)) / 60:.1f}분) "
                  f"force_used0={float(env._force_used_n[0]):.2f}N "
                  f"sensor0={float(env._force_sensor_filt_n[0]):.2f}N", flush=True)
    X = torch.cat(obs_buf)                       # (N, 14: 기존 11 + thermal 3)
    Y = torch.cat(act_buf)                       # (N, 2)
    assert X.shape[1] == 14, f"관측 차원 {X.shape[1]} != 14 — 환경 불일치"
    on_ratio = float((X[:, 6] * 2.0 > 0.3).float().mean())
    total = args.collect_steps * args.num_envs
    print(f"[bc] {len(X)} 샘플 (스크래치-위 비율 {on_ratio:.2f})")
    print(f"[bc] 커버리지 — 접촉 {n_contact}/{total} ({n_contact / total:.1%}) | "
          f"자유공간·접근 {n_free}/{total} ({n_free / total:.1%}) | "
          f"에피소드 종료(재접근 유발) {n_episode_end}회 | 센서 fault {n_fault}스텝")
    if n_episode_end == 0:
        print("[bc] 경로 끝/재접근 상태가 데이터에 없음 — collect_steps 를 늘려야 한다")

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
            print(f"[bc] epoch {ep:3d}  MSE {tot / n:.4f}", flush=True)

    # ── 3) 검증: 모방된 actor 가 수제 정책과 같은 구분을 내는가 ──
    actor.eval()
    with torch.no_grad():
        pred = actor(td_all).clamp(-1, 1)
    on = X[:, 6] * 2.0 > 0.3
    print(f"[bc] 모방 검증 — 스크래치 위  Δforce {pred[on, 0].mean():+.2f} Δfeed {pred[on, 1].mean():+.2f} "
          f"(목표 +1.0/-1.0)")
    print(f"[bc]             스크래치 밖  Δforce {pred[~on, 0].mean():+.2f} Δfeed {pred[~on, 1].mean():+.2f} "
          f"(목표 -0.3/+0.5)")

    # rsl_rl 5.0.1의 Logger는 learn()을 호출하지 않은 BC 전용 runner에서 writer 속성을
    # 만들지 않아 runner.save()가 torch.save 이후 예외를 낸다. 동일 checkpoint schema를
    # 직접 저장해 불필요한 외부 logger 경로를 건드리지 않는다.
    saved = runner.alg.save()
    saved["iter"] = runner.current_learning_iteration
    saved["infos"] = {
        "source": "robot_bc_physical", "env": "RobotPolishEnv",
        "contact_mode": args.contact_mode, "observation_dim": int(X.shape[1]),
        "coverage": {"contact_steps": n_contact, "free_steps": n_free,
                     "episode_ends": n_episode_end, "sensor_fault_steps": n_fault},
    }
    torch.save(saved, path)
    print(f"[bc] saved {path}")
    env.close()


if __name__ == "__main__":
    main()
    app.close()
