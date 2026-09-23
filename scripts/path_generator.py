"""
path_generator.py — 3D 표면 경로 생성 알고리즘 (멀티 로봇 지원)

로봇 위치 변경 시 ROBOT_BASE_POSITIONS 와 polishing_v4.py의 ROBOT_CONFIGS 를 함께 수정해야 함.
"""
import os
import sys
import argparse
import numpy as np

# ─────────────────────────────────────────────
# 로봇 베이스 위치 목록 (polishing_v4.py의 ROBOT_CONFIGS와 반드시 동일해야 함)
# 형식: [x, y, z] (월드 좌표, 단위: m)
# ─────────────────────────────────────────────
ROBOT_BASE_POSITIONS = [
    np.array([-1.0, -0.65, 0.5], dtype=np.float32),  # Robot 0: 왼쪽 앞
    np.array([-1.0, -0.95, 0.5], dtype=np.float32),  # Robot 1: 왼쪽 뒤
    np.array([ 1.0, -0.65, 0.5], dtype=np.float32),  # Robot 2: 오른쪽 앞
    np.array([ 1.0, -0.95, 0.5], dtype=np.float32),  # Robot 3: 오른쪽 뒤
]

HOOD_Y_MAX = 9999.0   # 레일 모드: 차량 전체 Y 범위 사용 (필터 없음)
MIN_RADIUS = 0.35     # 특이점(singularity) 회피 최소 반경
MAX_RADIUS = 0.85   # M0609 최대 도달 거리 기준 (실제 max 0.9m, 여유 0.05m)
EDGE_MARGIN = 0.03
MAX_SURFACE_TILT_DEG = 15.0   # 유리/급경사 제외 강화 (윗면 평탄부만 폴리싱)
NORMAL_QUERY_K = 35
OVERHEAD_Y_MARGIN = 1.00        # 오버헤드 레일이 앞/뒤 범퍼 바깥까지 접근할 여유 (0.75→1.0: 전/후면 수직면 도달)
FRONT_REAR_MIN_TILT = 50.0      # 앞/뒤 수직면 후보 최소 tilt
FRONT_REAR_NORMAL_MIN_DOT = 0.58 # 앞/뒤 수직면 후보 최소 |normal_y|

# 4분할 경계 — 팔 충돌 방지
# X_SPLIT: 차량 좌우 중앙선 (Robot 0,1 ↔ Robot 2,3)
# Y_SPLIT: 앞/뒤 경계 = 앞 로봇 Y와 뒤 로봇 Y의 중점
#   (-0.65 + -0.95) / 2 = -0.80
X_SPLIT = 0.0
Y_SPLIT = (ROBOT_BASE_POSITIONS[0][1] + ROBOT_BASE_POSITIONS[1][1]) / 2.0  # -0.80


def load_ply_points(ply_path):
    points = []
    header_done = False
    with open(ply_path, "r") as f:
        for line in f:
            line = line.strip()
            if not header_done:
                if line == "end_header":
                    header_done = True
                continue
            parts = line.split()
            if len(parts) >= 3:
                try:
                    points.append([float(parts[0]), float(parts[1]), float(parts[2])])
                except ValueError:
                    continue
    return np.asarray(points, dtype=np.float32)


def generate_3d_raster_path(points, step=0.09, resolution=0.025):
    """주어진 3D 점들을 기반으로 지그재그(Raster) 경로를 추출"""
    if len(points) == 0:
        return np.array([])

    y_bins = np.arange(np.min(points[:, 1]), np.max(points[:, 1]), step)

    path_3d = []
    for i, y in enumerate(y_bins):
        y_mask = (points[:, 1] >= y) & (points[:, 1] < y + step)
        row_points = points[y_mask]
        if len(row_points) == 0:
            continue

        row_points = row_points[np.argsort(row_points[:, 0])]
        if i % 2 == 1:
            row_points = row_points[::-1]

        sampled_row = []
        last_x = None
        for p in row_points:
            if last_x is None or abs(p[0] - last_x) >= resolution:
                sampled_row.append(p)
                last_x = p[0]
        path_3d.extend(sampled_row)

    return np.array(path_3d)


def estimate_surface_normal_and_tilt(kdtree, surface_points, target_pos, k=NORMAL_QUERY_K):
    sample_count = min(k, len(surface_points))
    _, indices = kdtree.query(target_pos, k=sample_count)
    neighbors = surface_points[np.atleast_1d(indices)]
    centroid = np.mean(neighbors, axis=0)
    centered = neighbors - centroid
    cov = np.dot(centered.T, centered)
    _, eigenvectors = np.linalg.eigh(cov)
    normal = eigenvectors[:, 0]
    if normal[2] < 0:
        normal = -normal
    normal = normal / np.linalg.norm(normal)
    tilt_deg = np.degrees(np.arccos(np.clip(normal[2], -1.0, 1.0)))
    return normal, tilt_deg


def estimate_surface_tilt(kdtree, surface_points, target_pos, k=NORMAL_QUERY_K):
    _, tilt_deg = estimate_surface_normal_and_tilt(kdtree, surface_points, target_pos, k)
    return tilt_deg


def assign_points_to_robots(points, robot_positions):
    """
    4분할(X축 좌우 × Y축 앞뒤) 직접 배정 — 팔 교차 충돌 방지.

    Robot 0: X < X_SPLIT, Y >= Y_SPLIT  (왼쪽 앞)
    Robot 1: X < X_SPLIT, Y <  Y_SPLIT  (왼쪽 뒤)
    Robot 2: X >= X_SPLIT, Y >= Y_SPLIT (오른쪽 앞)
    Robot 3: X >= X_SPLIT, Y <  Y_SPLIT (오른쪽 뒤)

    각 사분면 내에서도 해당 로봇의 작업 반경 필터를 추가 적용.
    """
    from scipy.spatial import KDTree

    n_robots = len(robot_positions)
    assert n_robots == 4, "4대 로봇 전용"

    surface_min = np.min(points, axis=0)
    surface_max = np.max(points, axis=0)
    kdtree = KDTree(points)

    # ── 사전 필터: 가장자리·기울기 ──
    edge_ok = (
        (points[:, 0] >= surface_min[0] + EDGE_MARGIN) &
        (points[:, 0] <= surface_max[0] - EDGE_MARGIN) &
        (points[:, 1] >= surface_min[1] + EDGE_MARGIN) &
        (points[:, 1] <= surface_max[1] - EDGE_MARGIN)
    )
    tilt_ok = np.array([
        estimate_surface_tilt(kdtree, points, pt) <= MAX_SURFACE_TILT_DEG
        for pt in points
    ])
    valid_mask = edge_ok & tilt_ok
    print(f"[path_generator] 가장자리·기울기 필터 통과: {valid_mask.sum()}/{len(points)}점")
    print(f"[path_generator] 4분할 기준: X_SPLIT={X_SPLIT:.2f}, Y_SPLIT={Y_SPLIT:.2f}")

    # ── 4분할 배정 ──
    # quadrant_mask[r]: 해당 로봇의 공간 영역에 속하는 점 마스크
    quadrant_masks = [
        (points[:, 0] <  X_SPLIT) & (points[:, 1] >= Y_SPLIT),  # Robot 0: 왼쪽 앞
        (points[:, 0] <  X_SPLIT) & (points[:, 1] <  Y_SPLIT),  # Robot 1: 왼쪽 뒤
        (points[:, 0] >= X_SPLIT) & (points[:, 1] >= Y_SPLIT),  # Robot 2: 오른쪽 앞
        (points[:, 0] >= X_SPLIT) & (points[:, 1] <  Y_SPLIT),  # Robot 3: 오른쪽 뒤
    ]

    robot_positions_arr = np.array(robot_positions)
    per_robot = []
    for r_idx in range(n_robots):
        base = robot_positions_arr[r_idx]
        mask = valid_mask & quadrant_masks[r_idx]
        candidates = points[mask]

        # 반경 필터 (팔 길이 한계)
        dists = np.linalg.norm(candidates - base, axis=1)
        reachable = candidates[(dists >= MIN_RADIUS) & (dists <= MAX_RADIUS)]
        per_robot.append(reachable)
        print(f"[path_generator] Robot {r_idx}: {len(reachable)}점 배정 "
              f"(사분면 내 후보 {len(candidates)}점, 반경 통과 {len(reachable)}점)")

    total_assigned = sum(len(r) for r in per_robot)
    print(f"[path_generator] 총 배정: {total_assigned}점 / 유효 {valid_mask.sum()}점")
    return per_robot


# ─────────────────────────────────────────────
# 레일 모드 설정
# 왼쪽(X=-1.0) 레일에 두 대 나란히: A=앞끝, B=뒤끝 → 서로 충돌 없이 가운데로 진행
# ─────────────────────────────────────────────
RAIL_LEFT_X       = -1.07
RAIL_BASE_Z       =  0.5
RAIL_STOP_SPACING =  0.25   # 정지 위치 간격 (m)
RAIL_VISUAL_MARGIN =  0.15   # 시각 레일 여유 (m) — 차량보다 15cm 더 길게만 표시

# ─────────────────────────────────────────────
# 오버헤드 갠트리 모드 설정
# 로봇을 차량 중앙 상단(X=0)에 거꾸로 매달아 위에서 내려다보며 폴리싱.
# ※ 아래 4개 상수는 polishing_v5.py의 동일 상수와 반드시 일치해야 함
#   (경로-제어 좌표계 일치, 기존 ROBOT_BASE 커플링과 동일 주의사항)
# ─────────────────────────────────────────────
OVERHEAD_BASE_X   =  0.0    # 갠트리 중앙 X (차량 좌우 중앙선)
OVERHEAD_STANDOFF =  0.70   # 경로 생성/런타임 공통 standoff. agent는 rail_config stop Z를 따른다.
OVERHEAD_Z_MIN    =  0.70   # 베이스 Z 하한 (낮은 후드까지 내려가 실제 접촉)
OVERHEAD_Z_MAX    =  1.85   # 베이스 Z 상한 (갠트리 승강 상한)

# ─────────────────────────────────────────────
# 측면(옆면) 모드 설정 — 좌(SL, X=-1.07)/우(SR, X=+1.07) 레일에서 수평 접근
# ※ SIDE_RAIL_X 는 polishing_v5.py 의 동일 상수와 일치해야 함
# ─────────────────────────────────────────────
SIDE_RAIL_X      = 1.36     # 측면 레일 X 절대값: 팔을 더 펴 팔꿈치 차체충돌 회피
SIDE_MIN_TILT    = 40.0     # 50→40: 휠 아치 위 곡면(법선~45°)까지 측면 경로에 포함
SIDE_NORMAL_MIN_DOT = 0.58  # 측면 법선이 좌/우 바깥(±X)을 향하는 점만 사용(후면/모서리 제외)
SIDE_X_MIN       = 0.12     # 중앙 제외: |x| 이 값 이상인 바깥쪽 점만 측면으로
SIDE_BASE_Z_MIN  = 0.15   # 단상 리프트가 점 높이를 추적 → 낮은 옆면(z≈0.13)까지 도달
SIDE_BASE_Z_MAX  = 1.55
SIDE_BASE_Z_OFFSET = 0.06  # 표면보다 약간 높은 단상 자세로 링크 4/5 차체 관통을 줄임


def compute_y_stops(y_min, y_max, margin=0.0):
    """차량 Y 범위 내에서 등간격 정지 위치 (큰 Y → 작은 Y 순).
    margin을 주면 앞/뒤 범퍼 바깥까지 레일 정지점을 확장한다."""
    stops, y = [], y_max + float(margin)
    y_end = y_min - float(margin)
    while y >= y_end - 1e-6:
        stops.append(round(y, 4))
        y -= RAIL_STOP_SPACING
    return stops

def compute_yz_stops(left_pts, all_y_stops):
    """동적 Z 트래킹을 사용하므로, Y정지 위치 하나당 기본 Z높이(0.45m) 하나만 생성합니다."""
    yz_stops = []
    for y_stop in all_y_stops:
        yz_stops.append([round(y_stop, 4), 0.45])
    return yz_stops


def generate_rail_paths(points, scan_dir):
    """
    레일 모드: 왼쪽 레일에 A(앞끝) / B(뒤끝) 두 대.
    Y 정지 위치를 점군 Y 범위에서 자동 계산하여 균등 분할.
    결과를 rail_config.json에 저장 → polishing_v5.py가 로드.
    """
    import json, glob
    from scipy.spatial import KDTree

    # 이전 경로 파일 전부 삭제 (구 파일이 남아 좌표 불일치 유발 방지)
    for old_f in glob.glob(os.path.join(scan_dir, "path_A*.npy")) + \
                 glob.glob(os.path.join(scan_dir, "path_B*.npy")):
        os.remove(old_f)
        print(f"[rail] 기존 파일 삭제: {os.path.basename(old_f)}")

    kdtree = KDTree(points)
    surface_min = np.min(points, axis=0)
    surface_max = np.max(points, axis=0)

    edge_ok = (
        (points[:, 0] >= surface_min[0] + EDGE_MARGIN) &
        (points[:, 0] <= surface_max[0] - EDGE_MARGIN) &
        (points[:, 1] >= surface_min[1] + EDGE_MARGIN) &
        (points[:, 1] <= surface_max[1] - EDGE_MARGIN)
    )
    tilt_ok = np.array([
        estimate_surface_tilt(kdtree, points, pt) <= MAX_SURFACE_TILT_DEG
        for pt in points
    ])
    valid = edge_ok & tilt_ok

    # 왼쪽 면 유효 점 (x < 0)
    left_pts = points[valid & (points[:, 0] < 0)]
    if len(left_pts) == 0:
        print("[rail] 왼쪽 유효 점 없음")
        return False

    y_min = float(np.min(left_pts[:, 1]))
    y_max = float(np.max(left_pts[:, 1]))
    print(f"[rail] 점군 유효: {valid.sum()}/{len(points)}, 왼쪽: {len(left_pts)}점")
    print(f"[rail] 왼쪽 Y 범위: [{y_min:.3f}, {y_max:.3f}] (길이 {y_max - y_min:.3f}m)")

    # Y 정지 위치 전체 계산 (여유 포함 → 레일이 차량보다 길어짐)
    all_y_stops = compute_y_stops(y_min, y_max)
    all_yz_stops = compute_yz_stops(left_pts, all_y_stops)
    print(f"[rail] 전체 YZ 정지 ({len(all_yz_stops)}개): {all_yz_stops}")

    # A=앞절반(높은 Y), B=뒤절반(낮은 Y)
    mid = max(1, (len(all_yz_stops) + 1) // 2)
    stops_A = all_yz_stops[:mid]            # 앞끝부터 가운데 방향
    stops_B = all_yz_stops[mid:]            # 뒤끝부터 가운데 방향 (거꾸로 순서)
    if not stops_B:
        stops_B = [all_yz_stops[-1]]
    stops_B = list(reversed(stops_B))   # B는 낮은 Y에서 시작해 높은 Y 방향으로

    stops_map = {"A": stops_A, "B": stops_B}
    for label, stops in stops_map.items():
        print(f"[rail] Robot {label} 스탑 개수: {len(stops)}")

    rail_config = {}
    any_saved   = False

    for label, stops in stops_map.items():
        y_stops_arr = np.array([y for (y, z) in stops])
        dynamic_z = np.clip(left_pts[:, 2] - 0.15, 0.45, 1.15)
        
        num_stops = len(y_stops_arr)
        num_pts = len(left_pts)
        dists = np.zeros((num_pts, num_stops))
        
        for i, y_stop in enumerate(y_stops_arr):
            base_pts = np.column_stack((np.full(num_pts, RAIL_LEFT_X), np.full(num_pts, y_stop), dynamic_z))
            dists[:, i] = np.linalg.norm(left_pts - base_pts, axis=1)

        reachable_any   = np.any((dists >= MIN_RADIUS) & (dists <= MAX_RADIUS), axis=1)
        reachable_pts   = left_pts[reachable_any]
        reachable_dists = dists[reachable_any]

        if len(reachable_pts) == 0:
            print(f"[rail] Robot {label}: 반경 내 점 없음")
            continue

        in_range     = (reachable_dists >= MIN_RADIUS) & (reachable_dists <= MAX_RADIUS)
        masked_dists = np.where(in_range, reachable_dists, np.inf)
        assigned     = np.argmin(masked_dists, axis=1)

        rail_config[label] = {
            "rail_x":     RAIL_LEFT_X,
            "base_yaw":   round(-np.pi / 2, 8),
            "yz_stops":   stops,
            "min_radius": MIN_RADIUS,
            "max_radius": MAX_RADIUS,
        }

        for stop_idx, (y_stop, z_stop) in enumerate(stops):
            seg_pts = reachable_pts[assigned == stop_idx]
            if len(seg_pts) == 0:
                print(f"[rail] {label}{stop_idx} (Y={y_stop:.2f}, Z={z_stop:.2f}): 점 없음")
                continue
            path_3d = generate_3d_raster_path(seg_pts, step=0.09, resolution=0.025)
            np.save(os.path.join(scan_dir, f"path_{label}{stop_idx}.npy"), path_3d)
            print(f"[rail] {label}{stop_idx} (Y={y_stop:.2f}, Z={z_stop:.2f}): {len(seg_pts)}점 → {len(path_3d)}개 웨이포인트")

            any_saved = True

    # rail_config.json 저장
    config_path = os.path.join(scan_dir, "rail_config.json")
    with open(config_path, "w") as f:
        json.dump(rail_config, f, indent=2)
    print(f"[rail] 레일 설정 저장: {config_path}")

    return any_saved


def generate_overhead_paths(points, scan_dir, write=True):
    """
    오버헤드 갠트리 모드: 차량 중앙 상단(X=0)에 거꾸로 매달린 로봇 1대.
    기존 윗면뿐 아니라 normal_y가 큰 앞/뒤 수직면 후보까지 포함한다.
    좌우 옆면은 SL/SR 측면 로봇이 담당하므로 normal_x가 큰 점은 제외한다.
    """
    import json, glob
    from scipy.spatial import KDTree

    # 이전 오버헤드 경로 파일 전부 삭제
    for old_f in glob.glob(os.path.join(scan_dir, "path_C*.npy")):
        os.remove(old_f)
        print(f"[overhead] 기존 파일 삭제: {os.path.basename(old_f)}")

    kdtree = KDTree(points)
    surface_min = np.min(points, axis=0)
    surface_max = np.max(points, axis=0)

    edge_x_ok = (
        (points[:, 0] >= surface_min[0] + EDGE_MARGIN) &
        (points[:, 0] <= surface_max[0] - EDGE_MARGIN)
    )
    edge_y_ok = (
        (points[:, 1] >= surface_min[1] + EDGE_MARGIN) &
        (points[:, 1] <= surface_max[1] - EDGE_MARGIN)
    )

    normals = []
    tilts = []
    for pt in points:
        n, tilt = estimate_surface_normal_and_tilt(kdtree, points, pt)
        normals.append(n)
        tilts.append(tilt)
    normals = np.asarray(normals)
    tilts = np.asarray(tilts)

    # 윗면은 기존처럼 완만한 점만 사용.
    top_ok = edge_x_ok & edge_y_ok & (tilts <= MAX_SURFACE_TILT_DEG)

    # 앞/뒤면은 차량 양 끝 근처이면서, 법선이 Y방향으로 큰 수직면만 오버헤드가 담당.
    end_y_zone = (
        (points[:, 1] <= surface_min[1] + OVERHEAD_Y_MARGIN) |
        (points[:, 1] >= surface_max[1] - OVERHEAD_Y_MARGIN)
    )
    front_back_ok = (
        edge_x_ok &
        end_y_zone &
        (tilts >= FRONT_REAR_MIN_TILT) &
        (np.abs(normals[:, 1]) >= FRONT_REAR_NORMAL_MIN_DOT) &
        (np.abs(normals[:, 1]) >= np.abs(normals[:, 0]) + 0.12)
    )
    valid = top_ok | front_back_ok

    overhead_pts = points[valid]
    if len(overhead_pts) == 0:
        print("[overhead] 유효 점 없음")
        return False

    y_min = float(np.min(overhead_pts[:, 1]))
    y_max = float(np.max(overhead_pts[:, 1]))
    print(f"[overhead] 점군 유효: {valid.sum()}/{len(points)}, "
          f"윗면 {top_ok.sum()}점, 앞/뒤 {front_back_ok.sum()}점")
    print(f"[overhead] Y 범위: [{y_min:.3f}, {y_max:.3f}] "
          f"(레일 여유 ±{OVERHEAD_Y_MARGIN:.2f}m)")

    # 앞/뒤 수직면을 닦으려면 캐리지가 범퍼 바깥까지 나가야 하므로 margin 적용.
    all_y_stops = compute_y_stops(y_min, y_max, margin=OVERHEAD_Y_MARGIN)
    print(f"[overhead] Y 정지 ({len(all_y_stops)}개): {all_y_stops}")

    yz_stops = []
    for y_stop in all_y_stops:
        near = overhead_pts[np.abs(overhead_pts[:, 1] - y_stop) <= RAIL_STOP_SPACING * 1.2]
        if len(near):
            ref = near
        else:
            nearest = np.argsort(np.abs(overhead_pts[:, 1] - y_stop))[: min(250, len(overhead_pts))]
            ref = overhead_pts[nearest]
        rep_z = float(np.percentile(ref[:, 2], 85))
        base_z = float(np.clip(rep_z + OVERHEAD_STANDOFF, OVERHEAD_Z_MIN, OVERHEAD_Z_MAX))
        yz_stops.append([round(y_stop, 4), round(base_z, 4)])
    print(f"[overhead] YZ 정지: {yz_stops}")

    # 도달성: 각 점을 가장 가까운 도달 가능한 Y정지에 배정한다.
    y_arr = np.array([y for (y, z) in yz_stops])
    z_arr = np.array([z for (y, z) in yz_stops])
    num_pts = len(overhead_pts)
    num_stops = len(yz_stops)
    dists = np.zeros((num_pts, num_stops))
    for i in range(num_stops):
        base_pts = np.column_stack((
            np.full(num_pts, OVERHEAD_BASE_X),
            np.full(num_pts, y_arr[i]),
            np.full(num_pts, z_arr[i]),
        ))
        dists[:, i] = np.linalg.norm(overhead_pts - base_pts, axis=1)

    in_range = (dists >= MIN_RADIUS) & (dists <= MAX_RADIUS)
    y_dists = np.abs(overhead_pts[:, 1:2] - y_arr[None, :])
    reachable_any = np.any(in_range, axis=1)
    reachable_pts = overhead_pts[reachable_any]
    masked_y_dists = np.where(in_range[reachable_any], y_dists[reachable_any], np.inf)
    assigned = np.argmin(masked_y_dists, axis=1)
    print(f"[overhead] 도달 가능 점: {len(reachable_pts)}/{len(overhead_pts)}")

    any_saved = False
    for stop_idx, (y_stop, z_stop) in enumerate(yz_stops):
        seg_pts = reachable_pts[assigned == stop_idx]
        if len(seg_pts) == 0:
            print(f"[overhead] C{stop_idx} (Y={y_stop:.2f}, Z={z_stop:.2f}): 점 없음")
            continue
        path_3d = generate_3d_raster_path(seg_pts, step=0.09, resolution=0.025)
        np.save(os.path.join(scan_dir, f"path_C{stop_idx}.npy"), path_3d)
        print(f"[overhead] C{stop_idx} (Y={y_stop:.2f}, Z={z_stop:.2f}): {len(seg_pts)}점 → {len(path_3d)}개 웨이포인트")
        any_saved = True

    rail_config = {
        "C": {
            "mount_mode": "overhead",
            "rail_x":     OVERHEAD_BASE_X,
            "base_yaw":   round(-np.pi / 2, 8),
            "standoff":   OVERHEAD_STANDOFF,
            "z_min":      OVERHEAD_Z_MIN,
            "z_max":      OVERHEAD_Z_MAX,
            "yz_stops":   yz_stops,
            "min_radius": MIN_RADIUS,
            "max_radius": MAX_RADIUS,
        }
    }
    if write:
        config_path = os.path.join(scan_dir, "rail_config.json")
        with open(config_path, "w") as f:
            json.dump(rail_config, f, indent=2)
        print(f"[overhead] 설정 저장: {config_path}")

    return rail_config, any_saved

def generate_side_raster_path(points, step=0.05, resolution=0.02):
    """수직 옆면용 Y-Z 래스터: Y로 행을 나누고 각 행 안에서 Z를 따라 지그재그."""
    if len(points) == 0:
        return np.array([])
    y_bins = np.arange(np.min(points[:, 1]), np.max(points[:, 1]), step)
    path_3d = []
    for i, y in enumerate(y_bins):
        row = points[(points[:, 1] >= y) & (points[:, 1] < y + step)]
        if len(row) == 0:
            continue
        row = row[np.argsort(row[:, 2])]          # Z 오름차순
        if i % 2 == 1:
            row = row[::-1]
        sampled, last_z = [], None
        for p in row:
            if last_z is None or abs(p[2] - last_z) >= resolution:
                sampled.append(p)
                last_z = p[2]
        path_3d.extend(sampled)
    return np.array(path_3d)


def _side_entry(points, scan_dir, label, outward_sign):
    """한쪽 옆면(label=SL/SR) 경로 + config 엔트리 생성.
    outward_sign: 좌=-1(바깥 -X), 우=+1(바깥 +X). 반환: ({label:{...}}, any_saved)."""
    import glob
    from scipy.spatial import KDTree

    for old_f in glob.glob(os.path.join(scan_dir, f"path_{label}*.npy")):
        os.remove(old_f)
        print(f"[side {label}] 기존 파일 삭제: {os.path.basename(old_f)}")

    rail_x = outward_sign * SIDE_RAIL_X
    kdtree = KDTree(points)
    smin, smax = np.min(points, axis=0), np.max(points, axis=0)

    edge_ok = (
        (points[:, 1] >= smin[1] + EDGE_MARGIN) &
        (points[:, 1] <= smax[1] - EDGE_MARGIN)
    )
    side_ok = (points[:, 0] * outward_sign) >= SIDE_X_MIN   # 해당 바깥쪽 절반
    normals = []
    tilts = []
    for pt in points:
        n, tilt = estimate_surface_normal_and_tilt(kdtree, points, pt)
        if n[0] * outward_sign < 0:
            n = -n
        normals.append(n)
        tilts.append(tilt)
    normals = np.asarray(normals)
    tilts = np.asarray(tilts)
    side_axis = np.array([float(outward_sign), 0.0, 0.0])
    side_normal_dot = normals @ side_axis
    tilt_ok = tilts >= SIDE_MIN_TILT
    normal_ok = side_normal_dot >= SIDE_NORMAL_MIN_DOT
    side_pts = points[edge_ok & side_ok & tilt_ok & normal_ok]
    if len(side_pts) == 0:
        print(f"[side {label}] 유효 옆면 점 없음")
        return {}, False
    y_min, y_max = float(side_pts[:, 1].min()), float(side_pts[:, 1].max())
    print(f"[side {label}] 옆면 점 {len(side_pts)}개 "
          f"(법선필터 {normal_ok.sum()}/{len(points)}), Y[{y_min:.2f},{y_max:.2f}]")

    all_y_stops = compute_y_stops(y_min, y_max)
    yz_stops = [[round(y, 4), round(float(np.clip(
        (np.median(side_pts[np.abs(side_pts[:, 1] - y) <= RAIL_STOP_SPACING][:, 2])
         if np.any(np.abs(side_pts[:, 1] - y) <= RAIL_STOP_SPACING) else np.median(side_pts[:, 2]))
        + SIDE_BASE_Z_OFFSET,
        SIDE_BASE_Z_MIN, SIDE_BASE_Z_MAX)), 4)] for y in all_y_stops]

    # 도달성: 점마다 동적 베이스 Z(점 z를 따라감)로 가장 가까운 Y정지에 배정
    y_arr = np.array([y for (y, z) in yz_stops])
    dyn_z = np.clip(side_pts[:, 2] + SIDE_BASE_Z_OFFSET, SIDE_BASE_Z_MIN, SIDE_BASE_Z_MAX)
    n_pts, n_stops = len(side_pts), len(yz_stops)
    dists = np.zeros((n_pts, n_stops))
    for i in range(n_stops):
        base = np.column_stack((np.full(n_pts, rail_x), np.full(n_pts, y_arr[i]), dyn_z))
        dists[:, i] = np.linalg.norm(side_pts - base, axis=1)
    in_range = (dists >= MIN_RADIUS) & (dists <= MAX_RADIUS)
    y_dists = np.abs(side_pts[:, 1:2] - y_arr[None, :])
    reach_any = np.any(in_range, axis=1)
    reach_pts = side_pts[reach_any]
    assigned = np.argmin(np.where(in_range[reach_any], y_dists[reach_any], np.inf), axis=1)
    print(f"[side {label}] 도달 가능 점: {len(reach_pts)}/{len(side_pts)}")

    any_saved = False
    for stop_idx, (y_stop, z_stop) in enumerate(yz_stops):
        seg = reach_pts[assigned == stop_idx]
        if len(seg) == 0:
            continue
        path = generate_side_raster_path(seg, step=0.05, resolution=0.02)
        np.save(os.path.join(scan_dir, f"path_{label}{stop_idx}.npy"), path)
        print(f"[side {label}] {label}{stop_idx} (Y={y_stop:.2f},Z={z_stop:.2f}): {len(seg)}점 → {len(path)}wp")
        any_saved = True

    # base_yaw: 좌(outward -1)=-π/2(차량 +X 향함), 우(outward +1)=+π/2(차량 -X 향함)
    base_yaw = round(np.pi / 2 * outward_sign, 8)
    entry = {label: {
        "mount_mode": "side",
        "side":       "L" if outward_sign < 0 else "R",
        "rail_x":     rail_x,
        "outward_sign": int(outward_sign),
        "base_yaw":   base_yaw,
        "yz_stops":   yz_stops,
        "min_radius": MIN_RADIUS,
        "max_radius": MAX_RADIUS,
    }}
    return entry, any_saved


def generate_full_paths(points, scan_dir):
    """윗면(오버헤드 C) + 좌측면(SL) + 우측면(SR)을 한 rail_config.json에 병합 생성."""
    import json
    cfg, ok_top = generate_overhead_paths(points, scan_dir, write=False)
    sl, ok_l = _side_entry(points, scan_dir, "SL", outward_sign=-1)
    sr, ok_r = _side_entry(points, scan_dir, "SR", outward_sign=+1)
    cfg.update(sl)
    cfg.update(sr)
    config_path = os.path.join(scan_dir, "rail_config.json")
    with open(config_path, "w") as f:
        json.dump(cfg, f, indent=2)
    print(f"[full] 설정 저장: {config_path} (로봇 {list(cfg.keys())})")
    return ok_top or ok_l or ok_r


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--obj_name", type=str, default="car")
    parser.add_argument("--mode", type=str, default="multi",
                        choices=["multi", "rail", "overhead", "full"],
                        help="multi: 4대 고정, rail: 2대 레일, overhead: 1대 갠트리(상단), full: 윗면+양옆 3대")
    args, unknown = parser.parse_known_args()

    BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    scan_dir = os.path.join(BASE_DIR, "scan_result", args.obj_name)
    ply_path = os.path.join(scan_dir, "points", "real_camera_surface_points.ply")

    print(f"[path_generator] 로드 중: {ply_path}")
    if not os.path.exists(ply_path):
        print(f"[ERROR] PLY 파일을 찾을 수 없습니다: {ply_path}")
        sys.exit(1)

    points = load_ply_points(ply_path)
    print(f"[path_generator] 원본 점 개수: {len(points)}")

    if HOOD_Y_MAX < 999.0:
        points = points[points[:, 1] < HOOD_Y_MAX]
        print(f"[path_generator] 필터링 후 점 개수: {len(points)}")
    else:
        print(f"[path_generator] Y 필터 없음 — 차량 전체 사용 ({len(points)}점)")

    # 차량 실제 좌표 범위 출력 (로봇 배치 확인용)
    _pmin = np.min(points, axis=0)
    _pmax = np.max(points, axis=0)
    print(f"[path_generator] 차량 X:[{_pmin[0]:.2f},{_pmax[0]:.2f}] "
          f"Y:[{_pmin[1]:.2f},{_pmax[1]:.2f}] Z:[{_pmin[2]:.2f},{_pmax[2]:.2f}]")

    # ── 레일 모드 ──
    if args.mode == "rail":
        ok = generate_rail_paths(points, scan_dir)
        if not ok:
            print("[ERROR] 레일 경로 생성 실패")
            sys.exit(1)
        return

    # ── 오버헤드 갠트리 모드 ──
    if args.mode == "overhead":
        _, ok = generate_overhead_paths(points, scan_dir)
        if not ok:
            print("[ERROR] 오버헤드 경로 생성 실패")
            sys.exit(1)
        return

    # ── 윗면+양옆 (3대) 모드 ──
    if args.mode == "full":
        ok = generate_full_paths(points, scan_dir)
        if not ok:
            print("[ERROR] full 경로 생성 실패")
            sys.exit(1)
        return

    # ── 기존 4대 고정 모드 ──
    per_robot_points = assign_points_to_robots(points, ROBOT_BASE_POSITIONS)

    any_saved = False
    for r_idx, robot_pts in enumerate(per_robot_points):
        if len(robot_pts) == 0:
            print(f"[WARNING] Robot {r_idx}: 배정된 점이 없어 경로를 생성하지 않습니다.")
            continue

        path_3d = generate_3d_raster_path(robot_pts, step=0.03, resolution=0.02)
        print(f"[path_generator] Robot {r_idx}: 웨이포인트 {len(path_3d)}개 생성")

        path_output = os.path.join(scan_dir, f"path_robot_{r_idx}.npy")
        np.save(path_output, path_3d)
        print(f"[path_generator] 저장 완료: {path_output}")
        any_saved = True

    if not any_saved:
        print("[ERROR] 어느 로봇도 안전 작업영역 내에 작업할 점이 없습니다!")
        sys.exit(1)


if __name__ == "__main__":
    main()
