"""v5 시뮬레이션 로그 → BC 학습 데이터셋 (state, action) 추출.

선생님(teacher) = 규칙 컨트롤러 + 물리엔진이 실제로 낸 결과:
  - 접촉력 라벨: 로그의 실측 `filtered`(t)  ← 규칙 공식 재계산 아님 (입력의 결정적 함수 = 정보 0)
    이 값의 배포 시 역할(어드미턴스 제어기 목표힘)은 config.ACTION_COLUMNS 주석 참고.
  - 이송속도:   로그의 path_idx 진행량에서 실측 역산
  - 입력에는 직전 스텝 실측 힘 filtered(t-1)을 넣는다 — 같은 시점 힘을 넣으면
    라벨을 그대로 베끼는 누수이므로 반드시 1스텝 지연.

로그의 (seg, path_idx)를 상태로 되돌리기 위해 agent.py와 동일한 절차로
경로 파일 로드 → CAR_LIFT_Z 적용 → filter_safe_waypoints 필터링을 재현한다.

로그 소스: scripts/의 최신 로그 + learning/data/raw/<날짜>/에 복사해둔 과거 실행 로그.
동일 내용 파일(해시 일치)은 한 번만 읽는다.

실행:  ~/isaacsim_venv/bin/python learning/bc/extract_dataset.py
"""
import csv
import glob
import hashlib
import json
import os
import re
import sys

import numpy as np
from scipy.spatial import cKDTree

_BC_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(_BC_DIR))          # learning/
import config

sys.path.insert(0, config.SCRIPTS_DIR)                # scripts/ → polishing_v5_modules
from polishing_v5_modules.common import (
    CAR_LIFT_Z,
    NORMAL_QUERY_K,
    adaptive_target_force,
    estimate_surface_normal,
    filter_safe_waypoints,
    load_ply_points,
)


# press_min — 최대 압입 깊이(작을수록 깊게 누름). 측면 0.012, 천장 0.012로 동일.
PRESS_OFFSET_FLOOR = 0.012


def load_surface():
    """runner.py와 동일: 점군 로드 + 리프트 좌표계 변환."""
    points = load_ply_points(config.PLY_PATH).astype(float)
    points[:, 2] += CAR_LIFT_Z
    return points, cKDTree(points)


def build_segments(label, rail_cfg, raw_points, kdtree):
    """agent.py __init__과 동일한 순서/조건으로 구간별 필터링된 경로 재구성.

    로그의 seg 컬럼은 '빈 구간을 건너뛴 뒤'의 인덱스이므로 순서를 그대로 따라야 한다.
    """
    cfg = rail_cfg[label]
    is_overhead = cfg["mount_mode"] == "overhead"
    is_side = cfg["mount_mode"] == "side"
    rail_x = float(cfg["rail_x"])

    segments = []
    for stop_idx, (y_stop, z_stop) in enumerate(cfg["yz_stops"]):
        fname = os.path.join(config.SCAN_DIR, f"path_{label}{stop_idx}.npy")
        if not os.path.exists(fname):
            continue
        raw_path = np.load(fname).astype(float)
        raw_path[:, 2] += CAR_LIFT_Z
        # agent.py:30 — 정지 위치 z도 리프트 좌표계로 (+CAR_LIFT_Z)
        base_at_stop = np.array([rail_x, float(y_stop), float(z_stop) + CAR_LIFT_Z])
        path = filter_safe_waypoints(
            raw_path, raw_points, kdtree, base_at_stop,
            is_side=is_side, is_overhead=is_overhead,
        )
        if len(path) == 0:
            continue
        segments.append(np.asarray(path, dtype=float))
    return segments, is_side


def local_curvature(kdtree, raw_points, pos, k=NORMAL_QUERY_K):
    """PCA surface variation: λ_min / (λ0+λ1+λ2). 평면=0, 급곡면일수록 큼."""
    sample_count = min(k, len(raw_points))
    _, indices = kdtree.query(pos, k=sample_count)
    neighbors = raw_points[np.atleast_1d(indices)]
    centered = neighbors - neighbors.mean(axis=0)
    eigenvalues = np.linalg.eigvalsh(centered.T @ centered)
    total = float(eigenvalues.sum())
    return float(eigenvalues[0] / total) if total > 0 else 0.0


def path_position(path, idx_float):
    """path_idx(실수)를 웨이포인트 사이 선형보간 위치로 변환."""
    idx_float = min(max(idx_float, 0.0), len(path) - 1.0)
    i0 = int(np.floor(idx_float))
    i1 = min(i0 + 1, len(path) - 1)
    frac = idx_float - i0
    return path[i0] * (1.0 - frac) + path[i1] * frac


def arc_length_at(cumlen, idx_float):
    """경로 누적 호길이를 path_idx 위치에서 선형보간."""
    idx_float = min(max(idx_float, 0.0), len(cumlen) - 1.0)
    i0 = int(np.floor(idx_float))
    i1 = min(i0 + 1, len(cumlen) - 1)
    frac = idx_float - i0
    return cumlen[i0] * (1.0 - frac) + cumlen[i1] * frac


def smooth_trajectory(values, window):
    """궤적 내 centered rolling mean (edge pad). window는 홀수 권장."""
    values = np.asarray(values, dtype=float)
    if len(values) <= 2 or window <= 1:
        return values
    kernel = np.ones(window) / window
    padded = np.pad(values, window // 2, mode="edge")
    return np.convolve(padded, kernel, mode="valid")[:len(values)]


def _dedup_existing(candidates):
    """존재하는 파일만, 내용 해시 기준 중복 제거해 반환."""
    paths, seen = [], set()
    for p in candidates:
        if not os.path.exists(p):
            continue
        digest = hashlib.md5(open(p, "rb").read()).hexdigest()
        if digest in seen:
            continue
        seen.add(digest)
        paths.append(p)
    return paths


def rail_log_paths(label):
    """이 레일의 CSV 로그 목록: scripts/ 최신본 + data/raw/ 누적본 (내용 중복 제거)."""
    return _dedup_existing(
        [config.RAIL_LOGS[label]]
        + sorted(glob.glob(os.path.join(config.RAW_LOG_DIR, "*", f"force_log_rail_{label}.csv"))))


def status_log_paths():
    """상태 로그 목록 — CSV가 유실된 레일의 대체 소스 (config.STATUS_LOG_PATH 주석 참고)."""
    return _dedup_existing(
        [config.STATUS_LOG_PATH]
        + sorted(glob.glob(os.path.join(config.RAW_LOG_DIR, "*", config.STATUS_LOG_NAME))))


def read_status_log(path, label):
    """status_log.txt의 tag=POLISH 진단 라인을 CSV와 같은 dict 형태로 파싱.

    idx는 정수(current_target_idx)이므로 웨이포인트 중앙(+0.5)으로 보정한다
    (C 레일 실측 평균 오프셋 0.473). 잔여 양자화는 라벨 스무딩이 흡수한다.
    """
    rows = []
    for line in open(path):
        if f"rail={label} " not in line or "tag=POLISH" not in line:
            continue
        d = dict(re.findall(r"(\w+)=([-\d.]+)", line))
        if not {"t", "seg", "filt", "idx"} <= d.keys():
            continue
        rows.append({
            "step": d["t"],
            "seg": d["seg"],
            "filtered": d["filt"],
            "path_idx": str(float(d["idx"]) + 0.5),
            "z_offset": d.get("zoff", "nan"),
            "state": "POLISH",
        })
    return rows


def read_log_rows(path, label):
    """확장자로 분기 — .csv는 그대로, status_log.txt는 파싱해서 같은 형태로."""
    if os.path.basename(path) == config.STATUS_LOG_NAME:
        return read_status_log(path, label)
    with open(path) as f:
        return list(csv.DictReader(f))


def segment_total_map(label):
    """status_log에서 {step: 그 시점 구간 경로 길이(total)} 를 만든다.

    재폴리싱 패스 판별용 — agent._regen_segs_from_red가 "아직 안 닦인 점"만으로
    경로를 다시 만들기 때문에, 재폴리싱 중의 path_idx는 우리가 재구성한 원본 경로가
    아니라 그때그때 새로 만들어진 짧은 경로를 가리킨다. 그대로 쓰면 위치·법선·곡률·
    속도가 전부 엉뚱한 값이 된다 (실측: C seg 5는 원본 66점인데 로그엔 7·28·66점 패스가 섞임).
    CSV에는 total 컬럼이 없어 status_log에서 step 기준으로 가져온다 (두 로그는 1:1 대응).
    """
    total_map = {}
    for path in status_log_paths():
        for line in open(path):
            if f"rail={label} " not in line or "tag=POLISH" not in line:
                continue
            d = dict(re.findall(r"(\w+)=([-\d.]+)", line))
            if "t" in d and "total" in d:
                total_map[int(d["t"])] = int(d["total"])
    return total_map


def extract_rail(label, log_path, segments, is_side, raw_points, kdtree, total_map=None):
    """한 레일 로그에서 POLISH 상태 행만 (state, action) 샘플로 변환.

    total_map이 있으면 재폴리싱 패스(경로가 재생성된 구간)를 걸러낸다 — segment_total_map 참고.
    """
    rows = read_log_rows(log_path, label)
    polish = [r for r in rows if r["state"] == "POLISH"]
    if not polish:
        print(f"[{label}] POLISH 행 없음 — 건너뜀 ({len(rows)}행, {os.path.basename(os.path.dirname(log_path))})")
        return [], [], []

    if total_map:
        kept = []
        for r in polish:
            seg = int(r["seg"])
            total = total_map.get(int(r["step"]))
            # 경로 길이가 재구성본과 일치하는 행만 = 원본 경로를 도는 첫 패스
            if total is not None and seg < len(segments) and total == len(segments[seg]):
                kept.append(r)
        n_drop = len(polish) - len(kept)
        if n_drop:
            print(f"[{label}] 재폴리싱 패스 제외: {n_drop}행 (경로 재생성으로 path_idx 대응 불가)")
        polish = kept
        if not polish:
            print(f"[{label}] 남은 행 없음 — 건너뜀")
            return [], [], []

    cumlens = [np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(p, axis=0), axis=1))])
               for p in segments]

    states, actions, aux = [], [], []
    run_ids, run_id = [], 0   # 같은 구간을 연속 주행한 궤적 단위 (스무딩 경계)
    dropped = 0
    for prev, cur in zip(polish, polish[1:]):
        seg = int(cur["seg"])
        # 이송속도는 연속 두 행의 진행량으로 실측 — 같은 구간 안에서만 유효
        if int(prev["seg"]) != seg or seg >= len(segments):
            dropped += 1
            run_id += 1
            continue
        d_step = int(cur["step"]) - int(prev["step"])
        if d_step <= 0:
            dropped += 1
            run_id += 1
            continue

        path = segments[seg]
        idx = float(cur["path_idx"])
        pos = path_position(path, idx)
        normal, tilt_deg = estimate_surface_normal(kdtree, raw_points, pos)
        curvature = local_curvature(kdtree, raw_points, pos)
        progress = idx / max(len(path) - 1, 1)

        ds = arc_length_at(cumlens[seg], idx) - arc_length_at(cumlens[seg], float(prev["path_idx"]))
        feed_speed = max(ds, 0.0) / (d_step / config.PHYSICS_HZ)  # m/s

        # 라벨 = 이번 스텝 실측 힘. 입력 = 직전 스텝 실측 힘 (1스텝 지연 — 누수 방지)
        measured_force = float(cur["filtered"])
        prev_force = float(prev["filtered"])
        # 규칙이 명령한 힘은 학습에 안 쓰고 aux에 보관 (평가 그래프의 참조선용)
        rule_force = adaptive_target_force(tilt_deg, mode="side" if is_side else "top")

        # v2: 절대 위치(pos)는 입력에서 제외 — 차량 좌표 암기 방지 (config.STATE_COLUMNS 주석).
        # pos 자체는 위 법선·곡률·이송속도 계산에 필요하므로 계속 구하고, aux에만 남긴다.
        states.append([normal[0], normal[1], normal[2],
                       tilt_deg, curvature, progress,
                       1.0 if is_side else 0.0,
                       0.0,  # phase: 로그에 없어 0 고정
                       prev_force])
        actions.append([measured_force, feed_speed])
        aux.append([{"C": 0, "SL": 1, "SR": 2}[label], seg,
                    rule_force, int(cur["step"]),
                    measured_force, feed_speed,      # raw 원본 보존 (평가용)
                    float(cur.get("z_offset", "nan")),  # 압입 깊이 — 배포 계약 검증용
                    pos[0], pos[1], pos[2]])         # v2: 입력에선 뺐지만 디버깅용 보존
        run_ids.append(run_id)

    if not states:
        print(f"[{label}] 유효 샘플 없음 — 건너뜀 (제외 {dropped})")
        return [], [], []

    # 라벨 스무딩 — 궤적(연속 주행) 단위 rolling mean. config.LABEL_SMOOTH_WINDOW 참고.
    actions = np.asarray(actions, dtype=float)
    run_ids = np.asarray(run_ids)
    for r in np.unique(run_ids):
        m = run_ids == r
        for col in range(actions.shape[1]):
            actions[m, col] = smooth_trajectory(actions[m, col], config.LABEL_SMOOTH_WINDOW)
    actions = actions.tolist()

    print(f"[{label}] 전체 {len(rows)}행 → POLISH {len(polish)}행 → 샘플 {len(states)}개 (제외 {dropped})")
    return states, actions, aux


def main():
    rail_cfg = json.load(open(config.RAIL_CONFIG_PATH))
    raw_points, kdtree = load_surface()
    print(f"점군 로드: {len(raw_points)}점 ({config.PLY_PATH})")

    all_states, all_actions, all_aux = [], [], []
    for label in config.RAIL_LOGS:
        segments, is_side = build_segments(label, rail_cfg, raw_points, kdtree)
        log_paths = rail_log_paths(label)
        print(f"[{label}] 구간 {len(segments)}개 재구성, CSV 로그 {len(log_paths)}개")

        total_map = segment_total_map(label)
        collected = []
        for log_path in log_paths:
            collected.append(extract_rail(label, log_path, segments, is_side,
                                          raw_points, kdtree, total_map))

        # CSV에서 한 건도 못 얻은 레일만 status_log로 대체 (얻었다면 중복이므로 쓰지 않는다)
        if not any(s for s, _, _ in collected):
            for sp in status_log_paths():
                s, a, x = extract_rail(label, sp, segments, is_side,
                                       raw_points, kdtree, total_map)
                if s:
                    print(f"[{label}] ↑ CSV 유실 — status_log에서 복구 ({os.path.basename(sp)})")
                collected.append((s, a, x))

        for s, a, x in collected:
            all_states += s
            all_actions += a
            all_aux += x

    states = np.array(all_states, dtype=np.float32)
    actions = np.array(all_actions, dtype=np.float32)
    aux = np.array(all_aux, dtype=np.float32)

    os.makedirs(config.DATA_DIR, exist_ok=True)
    np.savez(config.DATASET_PATH,
             states=states, actions=actions, aux=aux,
             state_columns=np.array(config.STATE_COLUMNS),
             action_columns=np.array(config.ACTION_COLUMNS),
             aux_columns=np.array(["rail_id", "seg", "rule_force", "step",
                                   "raw_force", "raw_speed", "z_offset",
                                   "pos_x", "pos_y", "pos_z"]))

    print(f"\n저장: {config.DATASET_PATH}")
    print(f"  states  {states.shape}  actions {actions.shape}")

    # 배포 계약 근거 점검 — 시연이 압입 깊이 한계에 얼마나 붙어 있었나.
    # 포화율이 높다는 건 규칙의 목표힘이 도달 불가능했다는 뜻이고,
    # 그래서 라벨로 '명령'이 아니라 '실측'을 쓴다. (config.ACTION_COLUMNS 주석)
    z = aux[:, 6]
    valid = np.isfinite(z)
    if valid.any():
        sat = z[valid] <= PRESS_OFFSET_FLOOR + 1e-4
        print(f"  [배포계약] 시연 압입 포화율 {100 * sat.mean():.1f}% "
              f"(포화 시 평균 실측힘 {actions[valid][sat, 0].mean():.2f}N)"
              if sat.any() else "  [배포계약] 시연 압입 포화 없음")
        print(f"             규칙 명령 평균 {aux[:, 2].mean():.2f}N vs 실측 평균 {actions[:, 0].mean():.2f}N")
    for i, name in enumerate(config.ACTION_COLUMNS):
        col = actions[:, i]
        print(f"  {name}: mean={col.mean():.4f} std={col.std():.4f} "
              f"min={col.min():.4f} max={col.max():.4f}")


if __name__ == "__main__":
    main()
