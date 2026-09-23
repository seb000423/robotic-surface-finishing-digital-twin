"""PPO 러너 설정 — train/eval 공용 (train_ppo 는 import 시 argparse 를 실행하므로 분리)."""
from isaaclab.utils.configclass import configclass
from isaaclab_rl.rsl_rl import (
    RslRlMLPModelCfg, RslRlOnPolicyRunnerCfg, RslRlPpoAlgorithmCfg)


@configclass
class PolishPPORunnerCfg(RslRlOnPolicyRunnerCfg):
    num_steps_per_env = 48          # 20 Hz 에서 2.4 s rollout
    max_iterations = 4000
    save_interval = 50
    experiment_name = "polish_ppo"
    obs_groups = {"policy": ["policy"], "critic": ["policy"]}
    actor = RslRlMLPModelCfg(
        hidden_dims=[128, 128],
        activation="elu",
        obs_normalization=True,
        # 초기 정책 ≈ 기준 제어기 — 작은 init_std 로 시작
        distribution_cfg=RslRlMLPModelCfg.GaussianDistributionCfg(init_std=0.3),
    )
    critic = RslRlMLPModelCfg(hidden_dims=[128, 128], activation="elu",
                              obs_normalization=True)
    algorithm = RslRlPpoAlgorithmCfg(
        value_loss_coef=1.0, use_clipped_value_loss=True, clip_param=0.2,
        entropy_coef=0.01, num_learning_epochs=5, num_mini_batches=4,
        learning_rate=3.0e-4,       # / 값
        schedule="adaptive", gamma=0.99, lam=0.95,
        desired_kl=0.01, max_grad_norm=1.0,
    )
