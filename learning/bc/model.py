"""BC 정책 네트워크 — 상태 → (목표 접촉력, 이송속도) 회귀 MLP."""
import torch
import torch.nn as nn


class BCPolicy(nn.Module):
    def __init__(self, state_dim, action_dim, hidden_sizes=(128, 128)):
        super().__init__()
        layers = []
        prev = state_dim
        for h in hidden_sizes:
            layers += [nn.Linear(prev, h), nn.ReLU()]
            prev = h
        layers.append(nn.Linear(prev, action_dim))
        self.net = nn.Sequential(*layers)

    def forward(self, x):
        return self.net(x)


class Normalizer:
    """입출력 표준화(z-score). 체크포인트에 함께 저장해 추론 시 재사용."""

    def __init__(self, mean, std):
        self.mean = mean
        self.std = torch.clamp(std, min=1e-6)

    @classmethod
    def fit(cls, data):
        return cls(data.mean(dim=0), data.std(dim=0))

    def normalize(self, x):
        return (x - self.mean) / self.std

    def denormalize(self, x):
        return x * self.std + self.mean

    def state_dict(self):
        return {"mean": self.mean, "std": self.std}

    @classmethod
    def from_state_dict(cls, d):
        return cls(d["mean"], d["std"])
