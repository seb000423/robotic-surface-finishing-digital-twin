"""BC 평가 — 예측 vs 실측 시각화.

라벨이 실측 힘(filtered)으로 바뀌었으므로 규칙 곡선 재현 패널 대신
구간 궤적을 따라 실측/예측/규칙명령 힘을 겹쳐 그린다.

실행:  ~/isaacsim_venv/bin/python learning/bc/evaluate.py
결과:  learning/outputs/bc_eval.png
"""
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import torch

_BC_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(_BC_DIR))
import config
from bc.model import BCPolicy, Normalizer
from bc.train import split_by_group

RAIL_NAMES = {0: "C", 1: "SL", 2: "SR"}


def load_model():
    ckpt = torch.load(config.CHECKPOINT_PATH, map_location="cpu", weights_only=False)
    model = BCPolicy(ckpt["state_dim"], ckpt["action_dim"], ckpt["hidden_sizes"])
    model.load_state_dict(ckpt["model"])
    model.eval()
    s_norm = Normalizer.from_state_dict(ckpt["state_normalizer"])
    a_norm = Normalizer.from_state_dict(ckpt["action_normalizer"])
    return model, s_norm, a_norm, ckpt


def predict(model, s_norm, a_norm, states):
    with torch.no_grad():
        return a_norm.denormalize(model(s_norm.normalize(states))).numpy()


def pick_segment(aux):
    """샘플이 가장 많은 (rail, seg) 궤적을 대표로 선택."""
    groups = aux[:, 0] * 1000 + aux[:, 1]
    unique, counts = np.unique(groups, return_counts=True)
    g = unique[np.argmax(counts)]
    mask = groups == g
    order = np.argsort(aux[mask, 3])  # step 순 정렬
    return mask, order, int(g // 1000), int(g % 1000)


def main():
    data = np.load(config.DATASET_PATH, allow_pickle=True)
    states = torch.tensor(data["states"], dtype=torch.float32)
    actions = data["actions"]
    aux = data["aux"]
    model, s_norm, a_norm, ckpt = load_model()
    pred = predict(model, s_norm, a_norm, states)

    # train/val을 반드시 구분해서 그린다 — 전체 평균 MAE는 학습 데이터가 섞여 낙관적이다.
    # 인용할 숫자는 언제나 val 쪽 (train.py가 출력하는 값과 동일).
    tr_mask, va_mask, _, _ = split_by_group(aux, config.VAL_FRACTION, config.SEED)

    fig, axes = plt.subplots(2, 2, figsize=(14, 10))

    # 1) 접촉력: 예측 vs 실측
    ax = axes[0, 0]
    ax.scatter(actions[tr_mask, 0], pred[tr_mask, 0], s=6, alpha=0.25,
               color="0.6", label="train")
    ax.scatter(actions[va_mask, 0], pred[va_mask, 0], s=10, alpha=0.6,
               color="tab:red", label="val (held-out)")
    lim = [actions[:, 0].min() - 0.3, actions[:, 0].max() + 0.3]
    ax.plot(lim, lim, "k--", lw=1)
    mae = np.abs(pred[va_mask, 0] - actions[va_mask, 0]).mean()
    ax.set_xlabel("label force (smoothed measured) [N]")
    ax.set_ylabel("BC predicted [N]")
    ax.set_title(f"Contact force  —  val MAE {mae:.3f} N "
                 f"(label std {actions[va_mask, 0].std():.2f} N)")
    ax.legend(fontsize=8, loc="upper left")

    # 2) 이송속도: 예측 vs 실측
    ax = axes[0, 1]
    ax.scatter(actions[tr_mask, 1] * 1000, pred[tr_mask, 1] * 1000, s=6, alpha=0.25,
               color="0.6", label="train")
    ax.scatter(actions[va_mask, 1] * 1000, pred[va_mask, 1] * 1000, s=10, alpha=0.6,
               color="tab:red", label="val (held-out)")
    lim = [0, max(actions[:, 1].max(), pred[:, 1].max()) * 1000 * 1.05]
    ax.plot(lim, lim, "k--", lw=1)
    mae_v = np.abs(pred[va_mask, 1] - actions[va_mask, 1]).mean() * 1000
    ax.set_xlabel("label feed speed (smoothed) [mm/s]")
    ax.set_ylabel("BC predicted [mm/s]")
    ax.set_title(f"Feed speed  —  val MAE {mae_v:.2f} mm/s")
    ax.legend(fontsize=8, loc="upper left")

    # 3~4) 대표 구간 궤적: step 순서대로 실측 vs 예측
    mask, order, rail_id, seg = pick_segment(aux)
    steps = aux[mask, 3][order]
    raw_f = aux[mask, 4][order]
    label_f = actions[mask, 0][order]
    pred_f = pred[mask, 0][order]
    rule_f = aux[mask, 2][order]
    raw_v = aux[mask, 5][order] * 1000
    label_v = actions[mask, 1][order] * 1000
    pred_v = pred[mask, 1][order] * 1000

    ax = axes[1, 0]
    ax.plot(steps, raw_f, lw=0.7, alpha=0.35, color="tab:blue", label="raw filtered(t)")
    ax.plot(steps, label_f, lw=1.4, color="tab:blue", label="label (smoothed)")
    ax.plot(steps, pred_f, lw=1.4, color="tab:orange", label="BC predicted")
    ax.plot(steps, rule_f, ls="--", lw=1, color="gray", label="rule commanded (ref)")
    ax.set_xlabel("sim step")
    ax.set_ylabel("force [N]")
    ax.set_title(f"Force along trajectory — rail {RAIL_NAMES[rail_id]} seg {seg}")
    ax.legend(fontsize=8)

    ax = axes[1, 1]
    ax.plot(steps, raw_v, lw=0.7, alpha=0.35, color="tab:blue", label="raw observed")
    ax.plot(steps, label_v, lw=1.4, color="tab:blue", label="label (smoothed)")
    ax.plot(steps, pred_v, lw=1.4, color="tab:orange", label="BC predicted")
    ax.set_xlabel("sim step")
    ax.set_ylabel("feed speed [mm/s]")
    ax.set_title(f"Feed speed along trajectory — rail {RAIL_NAMES[rail_id]} seg {seg}")
    ax.legend(fontsize=8)

    fig.suptitle(f"BC evaluation  —  n={len(states)} "
                 f"(train {int(tr_mask.sum())} / val {int(va_mask.sum())}), "
                 f"val loss {ckpt['best_val_loss']:.4f}")
    fig.tight_layout()
    os.makedirs(config.OUTPUT_DIR, exist_ok=True)
    out = os.path.join(config.OUTPUT_DIR, "bc_eval.png")
    fig.savefig(out, dpi=130)
    print(f"저장: {out}")
    print(f"[검증셋 기준] 접촉력 MAE {mae:.4f} N / 이송속도 MAE {mae_v:.3f} mm/s")


if __name__ == "__main__":
    main()
