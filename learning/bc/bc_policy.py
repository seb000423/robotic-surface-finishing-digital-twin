"""BC 정책 — 단독 추론 모듈.

의존성: torch, numpy 뿐. 저장소 구조에 의존하지 않는다.
이 파일과 bc_mlp.pt 두 개만 있으면 어디서든 동작한다.

사용:
    from bc_policy import BCPolicy

    policy = BCPolicy("bc_mlp.pt")            # 로드 (기본 eval + no_grad)
    force, speed = policy.predict_one(obs9)   # 단일 관측 → (N, m/s)
    actions = policy.predict(obs_batch)       # (B, 9) → (B, 2)

Isaac Lab 병렬 환경:
    actions = policy.predict(obs)             # obs: (num_envs, 9) numpy/torch 모두 가능
    # GPU에 올리려면:  policy.to("cuda")

입력 순서가 고정이다. policy.state_columns 로 확인할 것.
정규화는 이 모듈이 내부에서 처리한다. 밖에서 따로 정규화하지 말 것.
"""
from __future__ import annotations

import numpy as np
import torch
import torch.nn as nn

__all__ = ["BCPolicy", "STATE_COLUMNS", "ACTION_COLUMNS"]

# ── 입력 순서 (고정) ───────────────────────────────────────────────────────
#   normal_x, normal_y, normal_z : 표면 법선 (단위벡터, normal_z >= 0)
#   tilt_deg                     : acos(normal_z) [도], 0~90
#   curvature                    : 점군 국소 PCA의 λ_min / Σλ, 평면 ≈ 0
#   progress                     : 구간 내 경로 진행률 0~1
#   is_side                      : 측면 = 1 / 상면 = 0
#                                  ※ 로봇 1대 환경에서는 (tilt_deg > 45) 로 대체 (합의됨)
#   phase                        : 연마 단계. 현재 0 고정 (예약 슬롯)
#   prev_force                   : 직전 스텝 실측 접촉력 [N]
STATE_COLUMNS = [
    "normal_x", "normal_y", "normal_z",
    "tilt_deg", "curvature", "progress",
    "is_side", "phase", "prev_force",
]

# ── 출력 ──────────────────────────────────────────────────────────────────
#   contact_force_n : 접촉력 [N].
#       배포 계약: 이 값을 어드미턴스/힘 제어기의 목표힘 setpoint 자리에 넣는다.
#         값의 출처는 "시연에서 실제로 달성된 힘"이지 "규칙이 명령한 힘"이 아니다.
#         (규칙 명령 평균 5.82N은 물리적으로 도달 불가능했다 — 시연의 65.7%가 압입 한계 포화)
#   feed_speed_mps  : 경로 추종 이송속도 [m/s]
ACTION_COLUMNS = ["contact_force_n", "feed_speed_mps"]

# 학습 데이터 분포 — 새 환경 접촉력이 이 범위와 자릿수가 다르면 이 체크포인트는 쓸 수 없다.
# 그 경우 새 환경 로그로 BC를 재학습할 것 (파이프라인 그대로 재사용 가능).
TRAIN_FORCE_RANGE_N = (0.27, 6.92)
TRAIN_FORCE_MEAN_N = 3.12
TRAIN_SPEED_RANGE_MPS = (0.0, 0.793)


class _MLP(nn.Module):
    """학습 시 사용한 구조와 동일 — state_dict 키(net.0/net.2/net.4)가 여기에 맞춰져 있다."""

    def __init__(self, state_dim: int, action_dim: int, hidden_sizes):
        super().__init__()
        layers, prev = [], state_dim
        for h in hidden_sizes:
            layers += [nn.Linear(prev, h), nn.ReLU()]
            prev = h
        layers.append(nn.Linear(prev, action_dim))
        self.net = nn.Sequential(*layers)

    def forward(self, x):
        return self.net(x)


class BCPolicy:
    """BC 정책 추론기. 정규화 → 신경망 → 역정규화를 한 번에 처리한다."""

    def __init__(self, checkpoint_path: str, device: str = "cpu"):
        ckpt = torch.load(checkpoint_path, map_location="cpu", weights_only=False)

        self.state_columns = list(ckpt["state_columns"])
        self.action_columns = list(ckpt["action_columns"])
        self.state_dim = int(ckpt["state_dim"])
        self.action_dim = int(ckpt["action_dim"])

        if self.state_columns != STATE_COLUMNS:
            raise ValueError(
                "체크포인트의 입력 순서가 이 모듈의 STATE_COLUMNS와 다르다.\n"
                f"  체크포인트: {self.state_columns}\n  모듈: {STATE_COLUMNS}"
            )

        self._model = _MLP(self.state_dim, self.action_dim, ckpt["hidden_sizes"])
        self._model.load_state_dict(ckpt["model"])
        self._model.eval()
        for p in self._model.parameters():   # 잔차 RL에서 BC는 동결이다
            p.requires_grad_(False)

        self._s_mean = ckpt["state_normalizer"]["mean"].clone()
        self._s_std = torch.clamp(ckpt["state_normalizer"]["std"].clone(), min=1e-6)
        self._a_mean = ckpt["action_normalizer"]["mean"].clone()
        self._a_std = torch.clamp(ckpt["action_normalizer"]["std"].clone(), min=1e-6)

        self.val_loss = float(ckpt.get("best_val_loss", float("nan")))
        self.to(device)

    def to(self, device):
        self.device = torch.device(device)
        self._model.to(self.device)
        self._s_mean = self._s_mean.to(self.device)
        self._s_std = self._s_std.to(self.device)
        self._a_mean = self._a_mean.to(self.device)
        self._a_std = self._a_std.to(self.device)
        return self

    @torch.no_grad()
    def predict(self, obs):
        """(B, 9) 관측 → (B, 2) 행동 [N, m/s].

        obs 는 numpy 배열 또는 torch 텐서. 반환 타입은 입력 타입을 따른다.
        """
        was_numpy = isinstance(obs, np.ndarray)
        x = torch.as_tensor(obs, dtype=torch.float32, device=self.device)
        if x.ndim == 1:
            x = x.unsqueeze(0)
        if x.shape[-1] != self.state_dim:
            raise ValueError(f"입력 차원이 {x.shape[-1]}이다. {self.state_dim}이어야 한다 "
                             f"({self.state_columns}).")

        y = self._model((x - self._s_mean) / self._s_std) * self._a_std + self._a_mean
        return y.cpu().numpy() if was_numpy else y

    def predict_one(self, obs):
        """단일 관측 (9,) → (force_n, speed_mps) 튜플."""
        out = self.predict(np.asarray(obs, dtype=np.float32).reshape(1, -1))
        out = out[0] if isinstance(out, np.ndarray) else out[0].cpu().numpy()
        return float(out[0]), float(out[1])

    def check_transfer(self, observed_forces) -> dict:
        """새 환경의 접촉력 분포가 학습 분포와 호환되는지 사전 점검.

        RL을 며칠 돌린 뒤에 "BC가 쓸모없었다"를 알면 낭비다. 학습 시작 전에 호출할 것.
        """
        f = np.asarray(observed_forces, dtype=float)
        f = f[np.isfinite(f)]
        lo, hi = TRAIN_FORCE_RANGE_N
        report = {
            "observed_mean": float(f.mean()) if f.size else float("nan"),
            "observed_range": (float(f.min()), float(f.max())) if f.size else (float("nan"),) * 2,
            "train_mean": TRAIN_FORCE_MEAN_N,
            "train_range": (lo, hi),
            "in_range_ratio": float(((f >= lo) & (f <= hi)).mean()) if f.size else 0.0,
        }
        ratio = report["observed_mean"] / TRAIN_FORCE_MEAN_N if f.size else float("inf")
        report["scale_ratio"] = float(ratio)
        report["compatible"] = bool(0.4 < ratio < 2.5 and report["in_range_ratio"] > 0.5)
        report["verdict"] = (
            "호환 — 이 체크포인트를 그대로 사용 가능"
            if report["compatible"] else
            "불일치 — 새 환경 로그로 BC 재학습 권장"
        )
        return report

    def __repr__(self):
        return (f"BCPolicy(in={self.state_dim}, out={self.action_dim}, "
                f"device={self.device}, val_loss={self.val_loss:.4f})")


if __name__ == "__main__":
    import os
    here = os.path.dirname(os.path.abspath(__file__))
    policy = BCPolicy(os.path.join(here, "bc_mlp.pt"))
    print(policy)
    print("입력:", policy.state_columns)
    print("출력:", policy.action_columns)

    # 예시: 82도 경사면(측면), 평평, 경로 40% 지점, 직전 힘 3.38N
    obs = [-0.9885, 0.0580, 0.1393, 81.99, 0.0005, 0.4029, 1.0, 0.0, 3.381]
    f, v = policy.predict_one(obs)
    print(f"\n예시 추론 → 접촉력 {f:.3f} N, 이송속도 {v * 1000:.1f} mm/s")
