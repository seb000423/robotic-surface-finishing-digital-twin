"""BC 학습 — 데이터셋을 (rail, seg) 그룹 단위로 나눠 MLP를 지도학습.

실행:  ~/isaacsim_venv/bin/python learning/bc/train.py
"""
import os
import sys

import numpy as np
import torch
from torch.utils.data import DataLoader, TensorDataset

_BC_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(_BC_DIR))
import config
from bc.model import BCPolicy, Normalizer


def split_by_group(aux, val_fraction, seed):
    """(rail, seg) 그룹째로 train/val 분할 — 같은 구간이 양쪽에 섞이는 누수 방지."""
    groups = aux[:, 0] * 1000 + aux[:, 1]
    unique = np.unique(groups)
    rng = np.random.default_rng(seed)
    rng.shuffle(unique)
    n_val = max(1, int(round(len(unique) * val_fraction)))
    val_groups = set(unique[:n_val].tolist())
    val_mask = np.isin(groups, list(val_groups))
    return ~val_mask, val_mask, len(unique) - n_val, n_val


def main():
    torch.manual_seed(config.SEED)
    data = np.load(config.DATASET_PATH, allow_pickle=True)
    states = torch.tensor(data["states"], dtype=torch.float32)
    actions = torch.tensor(data["actions"], dtype=torch.float32)

    train_mask, val_mask, n_tr_g, n_va_g = split_by_group(
        data["aux"], config.VAL_FRACTION, config.SEED)
    print(f"샘플 {len(states)}개 → train {train_mask.sum()} / val {val_mask.sum()} "
          f"(구간 그룹 {n_tr_g}/{n_va_g})")

    s_norm = Normalizer.fit(states[train_mask])
    a_norm = Normalizer.fit(actions[train_mask])

    device = "cuda" if torch.cuda.is_available() else "cpu"
    train_ds = TensorDataset(s_norm.normalize(states[train_mask]),
                             a_norm.normalize(actions[train_mask]))
    val_x = s_norm.normalize(states[val_mask]).to(device)
    val_y = a_norm.normalize(actions[val_mask]).to(device)
    loader = DataLoader(train_ds, batch_size=config.BATCH_SIZE, shuffle=True)

    model = BCPolicy(states.shape[1], actions.shape[1], config.HIDDEN_SIZES).to(device)
    optim = torch.optim.Adam(model.parameters(), lr=config.LEARNING_RATE)
    loss_fn = torch.nn.MSELoss()

    best_val, best_state, patience = float("inf"), None, 0
    for epoch in range(1, config.MAX_EPOCHS + 1):
        model.train()
        train_loss = 0.0
        for xb, yb in loader:
            xb, yb = xb.to(device), yb.to(device)
            optim.zero_grad()
            loss = loss_fn(model(xb), yb)
            loss.backward()
            optim.step()
            train_loss += loss.item() * len(xb)
        train_loss /= len(train_ds)

        model.eval()
        with torch.no_grad():
            val_loss = loss_fn(model(val_x), val_y).item()

        if val_loss < best_val - 1e-6:
            best_val = val_loss
            best_state = {k: v.detach().cpu().clone() for k, v in model.state_dict().items()}
            patience = 0
        else:
            patience += 1

        if epoch % 20 == 0 or patience == 0 and epoch <= 5:
            print(f"epoch {epoch:4d}  train {train_loss:.5f}  val {val_loss:.5f}  best {best_val:.5f}")
        if patience >= config.EARLY_STOP_PATIENCE:
            print(f"조기 종료 (epoch {epoch}, best val {best_val:.5f})")
            break

    model.load_state_dict(best_state)

    # 물리 단위(N, m/s) 기준 검증 오차 리포트
    model_cpu = model.to("cpu").eval()
    with torch.no_grad():
        pred = a_norm.denormalize(model_cpu(s_norm.normalize(states[val_mask])))
    err = (pred - actions[val_mask]).abs()
    for i, name in enumerate(data["action_columns"]):
        print(f"검증 MAE  {name}: {err[:, i].mean():.4f} "
              f"(정답 std {actions[val_mask][:, i].std():.4f})")

    # 착시 점검: 힘 라벨은 자기상관이 커서 "prev_force 복사"만으로도 MAE가 낮게 나온다.
    # 모델이 이 나이브 베이스라인보다 나은지 확인.
    cols = list(data["state_columns"])
    if "prev_force" in cols:
        naive = (states[val_mask][:, cols.index("prev_force")] - actions[val_mask][:, 0]).abs()
        print(f"참고 — prev_force 복사 베이스라인 힘 MAE: {naive.mean():.4f}")

    os.makedirs(config.CHECKPOINT_DIR, exist_ok=True)
    torch.save({
        "model": model_cpu.state_dict(),
        "state_normalizer": s_norm.state_dict(),
        "action_normalizer": a_norm.state_dict(),
        "state_dim": states.shape[1],
        "action_dim": actions.shape[1],
        "hidden_sizes": config.HIDDEN_SIZES,
        "state_columns": list(data["state_columns"]),
        "action_columns": list(data["action_columns"]),
        "best_val_loss": best_val,
    }, config.CHECKPOINT_PATH)
    print(f"저장: {config.CHECKPOINT_PATH}")


if __name__ == "__main__":
    main()
