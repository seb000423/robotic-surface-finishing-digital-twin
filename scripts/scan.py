import os
import sys
import json
import argparse

# 경로 설정
BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

parser = argparse.ArgumentParser(description="Scan a USD object")
parser.add_argument("--obj_name", type=str, default="car", help="Name of the object to scan (e.g. car, cube)")
parser.add_argument("--gui", action="store_true", help="Open the Isaac Sim GUI while scanning")
parser.add_argument("--headless", action="store_true", help="Compatibility flag; scan is headless by default")
args, unknown = parser.parse_known_args()

USD_PATH = os.path.join(BASE_DIR, "scan_obj", f"{args.obj_name}.usd")
OUTPUT_DIR = os.path.join(BASE_DIR, "scan_result", args.obj_name)

if not os.path.exists(USD_PATH):
    print(f"[ERROR] USD 파일을 찾을 수 없습니다: {USD_PATH}")
    sys.exit(1)

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": not args.gui})

import numpy as np
import matplotlib.colors as mcolors
import omni
import omni.replicator.core as rep

from pxr import Usd, UsdGeom, Vt, Gf
from isaacsim.core.api import World
from omni.isaac.core.utils.prims import create_prim

DEPTH_DIR = os.path.join(OUTPUT_DIR, "depth")
POINTS_DIR = os.path.join(OUTPUT_DIR, "points")

os.makedirs(DEPTH_DIR, exist_ok=True)
os.makedirs(POINTS_DIR, exist_ok=True)

IMAGE_WIDTH = 640
IMAGE_HEIGHT = 480

def save_json(path, data):
    with open(path, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=2, ensure_ascii=False)

def depth_to_world_surface(depth: np.ndarray, sem_img, target_id, camera_view_transform: np.ndarray, fx: float, fy: float, cx: float, cy: float):
    h, w = depth.shape

    # step to subsample depth map for faster processing (optional, 1 for full resolution, 2 for 1/4)
    step = 2
    u_coords = np.arange(0, w, step)
    v_coords = np.arange(0, h, step)
    u, v = np.meshgrid(u_coords, v_coords)

    d = depth[v, u]

    # Valid depths (adjust max distance as needed, e.g., 10.0 meters)
    valid_mask = np.isfinite(d) & (d > 0.0) & (d <= 10.0)

    if sem_img is not None and target_id is not None:
        sem_vals = sem_img[v, u]
        valid_mask = valid_mask & (sem_vals == target_id)

    u = u[valid_mask]
    v = v[valid_mask]
    d = d[valid_mask]

    if len(d) == 0:
        return np.empty((0, 3), dtype=np.float32)

    ray_x = (u - cx) / fx
    ray_y = (v - cy) / fy
    ray_length = np.sqrt(ray_x**2 + ray_y**2 + 1.0)

    z_cam = d / ray_length
    x_cam = ray_x * z_cam
    y_cam = ray_y * z_cam

    # Points in camera coordinate frame.
    # OpenCV image coordinates: +X right, +Y down, +Z forward.
    # USD camera coordinates: +X right, +Y up, -Z forward.
    # Thus, the local coordinates are (x_cam, -y_cam, -z_cam).
    points_cam = np.stack([x_cam, -y_cam, -z_cam, np.ones_like(z_cam)], axis=-1)

    # cameraViewTransform is World-to-Camera. We need Camera-to-World.
    world_to_cam = camera_view_transform.reshape(4, 4)
    cam_to_world = np.linalg.inv(world_to_cam)

    # Multiply
    points_world = points_cam @ cam_to_world

    return points_world[:, :3].astype(np.float32)

def save_surface_points_ply(path: str, points: np.ndarray):
    if len(points) == 0:
        print(f"[WARNING] No points to save to {path}")
        return
    with open(path, "w", encoding="utf-8") as f:
        f.write("ply\n")
        f.write("format ascii 1.0\n")
        f.write(f"element vertex {points.shape[0]}\n")
        f.write("property float x\n")
        f.write("property float y\n")
        f.write("property float z\n")
        f.write("end_header\n")
        for p in points:
            f.write(f"{p[0]:.4f} {p[1]:.4f} {p[2]:.4f}\n")

def analyze_merged_surface(points: np.ndarray):
    if len(points) == 0:
        return {
            "description": "Empty surface points",
            "num_surface_points": 0
        }
    min_pt = np.min(points, axis=0)
    max_pt = np.max(points, axis=0)
    center = np.mean(points, axis=0)
    z_values = points[:, 2]
    return {
        "description": "Multi-camera based real upper surface geometry",
        "num_surface_points": int(points.shape[0]),
        "surface_center_m": center.tolist(),
        "surface_min_m": min_pt.tolist(),
        "surface_max_m": max_pt.tolist(),
        "surface_size_m": (max_pt - min_pt).tolist(),
        "height_min_m": float(np.min(z_values)),
        "height_max_m": float(np.max(z_values)),
        "height_mean_m": float(np.mean(z_values)),
        "height_range_m": float(np.max(z_values) - np.min(z_values)),
        "height_range_cm": float((np.max(z_values) - np.min(z_values)) * 100.0),
    }

def main():
    print("[DEBUG] Initializing World")
    world = World(stage_units_in_meters=1.0)

    print("[DEBUG] Loading USD Object:", USD_PATH)
    create_prim(
        prim_path=f"/World/{args.obj_name.capitalize()}",
        prim_type="Xform",
        usd_path=USD_PATH
    )

    print("[DEBUG] Getting USD Stage")
    stage = omni.usd.get_context().get_stage()

    print("[DEBUG] Setting Semantic Label for Target Mesh")
    from pxr import Semantics
    # 사용자가 클릭한 특정 파츠의 경로
    target_prim_path = f"/World/{args.obj_name.capitalize()}/tc/tc/bmw_z4_car_007_color_polySurface37"
    target_prim = stage.GetPrimAtPath(target_prim_path)
    if target_prim.IsValid():
        semAPI = Semantics.SemanticsAPI.Apply(target_prim, "Semantics")
        semAPI.CreateSemanticTypeAttr()
        semAPI.CreateSemanticDataAttr()
        semAPI.GetSemanticTypeAttr().Set("class")
        semAPI.GetSemanticDataAttr().Set("target_mesh")
        print(f"[INFO] 타겟 메쉬에 시맨틱 라벨 적용 성공: {target_prim_path}")
    else:
        print(f"[WARNING] 타겟 메쉬를 찾을 수 없습니다: {target_prim_path}")

    print("[DEBUG] Resetting World")
    world.reset()

    # 다중 카메라 포즈 정의 (기존 상단/측면 카메라 + 사각지대 보완용 하단 카메라 추가)
    camera_poses = {
        "top":   {"pos": (0.0, 0.0, 5.0),   "look_at": (0.0, 0.0, 0.0)},
        "left":  {"pos": (0.0, 4.0, 1.5),   "look_at": (0.0, 0.0, 1.0)},
        "right": {"pos": (0.0, -4.0, 1.5),  "look_at": (0.0, 0.0, 1.0)},
        "front": {"pos": (4.0, 0.0, 1.5),   "look_at": (0.0, 0.0, 1.0)},
        "back":  {"pos": (-4.0, 0.0, 1.5),  "look_at": (0.0, 0.0, 1.0)},
        # 하단 사각지대(번호판, 범퍼 아래쪽 등) 스캔용 카메라 추가
        "left_low":  {"pos": (0.0, 4.0, 0.5),   "look_at": (0.0, 0.0, 0.5)},
        "right_low": {"pos": (0.0, -4.0, 0.5),  "look_at": (0.0, 0.0, 0.5)},
        "front_low": {"pos": (4.0, 0.0, 0.5),   "look_at": (0.0, 0.0, 0.5)},
        "back_low":  {"pos": (-4.0, 0.0, 0.5),  "look_at": (0.0, 0.0, 0.5)}
    }

    camera_data_extractors = {}

    print("[DEBUG] Creating Cameras and Annotators")
    for name, pose in camera_poses.items():
        cam = rep.create.camera(
            position=pose["pos"],
            look_at=pose["look_at"],
            focal_length=24.0,
            horizontal_aperture=20.955
        )
        rp = rep.create.render_product(cam, resolution=(IMAGE_WIDTH, IMAGE_HEIGHT))

        depth_annot = rep.AnnotatorRegistry.get_annotator("distance_to_camera")
        depth_annot.attach([rp])

        rgb_annot = rep.AnnotatorRegistry.get_annotator("rgb")
        rgb_annot.attach([rp])

        cam_annot = rep.AnnotatorRegistry.get_annotator("camera_params")
        cam_annot.attach([rp])

        sem_annot = rep.AnnotatorRegistry.get_annotator("semantic_segmentation")
        sem_annot.attach([rp])

        camera_data_extractors[name] = {
            "depth_annot": depth_annot,
            "rgb_annot": rgb_annot,
            "cam_annot": cam_annot,
            "sem_annot": sem_annot
        }

    print("[DEBUG] Stepping Simulation to stabilize")
    for i in range(60):
        simulation_app.update()
        rep.orchestrator.step()

    all_surface_points = []

    for name, extractors in camera_data_extractors.items():
        print(f"[DEBUG] Processing camera view: {name}")
        depth = extractors["depth_annot"].get_data()
        if depth is None:
            print(f"[WARNING] Depth 데이터 획득 실패: {name}")
            continue

        depth = np.asarray(depth, dtype=np.float32)

        cam_params = extractors["cam_annot"].get_data()
        if not cam_params or "cameraViewTransform" not in cam_params:
            print(f"[WARNING] 카메라 파라미터 획득 실패: {name}")
            continue

        K = cam_params.get("cameraMatrix", None)
        if K is not None:
            fx = float(K[0][0])
            fy = float(K[1][1])
            cx = float(K[0][2])
            cy = float(K[1][2])
        else:
            fx = (IMAGE_WIDTH * 24.0) / 20.955
            fy = fx
            cx = IMAGE_WIDTH / 2.0
            cy = IMAGE_HEIGHT / 2.0

        view_transform = cam_params["cameraViewTransform"]

        # RGB 데이터 가져오기 및 저장
        rgb_data = extractors["rgb_annot"].get_data()
        if rgb_data is not None:
            from PIL import Image
            img = Image.fromarray(rgb_data, "RGBA")
            rgb_path = os.path.join(OUTPUT_DIR, f"real_camera_rgb_{name}.png")
            img.convert("RGB").save(rgb_path)

        # Semantic 데이터 가져오기
        sem_data = extractors["sem_annot"].get_data()
        sem_img = None
        target_id = None
        if sem_data and "info" in sem_data and "data" in sem_data:
            sem_img = sem_data["data"]
            id_to_labels = sem_data["info"].get("idToLabels", {})
            for id_str, meta in id_to_labels.items():
                if isinstance(meta, dict) and meta.get("class") == "target_mesh":
                    target_id = int(id_str)
                    break
                elif meta == "target_mesh":
                    target_id = int(id_str)
                    break

        # Point Cloud 변환
        surface_points = depth_to_world_surface(depth, sem_img, target_id, view_transform, fx, fy, cx, cy)

        # 필터링: 바닥 및 유효 범위 (전체 차량을 스캔하기 위해 X, Y 제한 확대)
        if len(surface_points) > 0:
            mask = (
                (surface_points[:, 0] >= -5.0) & (surface_points[:, 0] <= 5.0) &
                (surface_points[:, 1] >= -5.0) & (surface_points[:, 1] <= 5.0) &
                (surface_points[:, 2] > 0.01)  # 바닥 제거
            )
            surface_points = surface_points[mask]

        # 개별 포인트 클라우드 저장
        if len(surface_points) > 0:
            pts_path = os.path.join(POINTS_DIR, f"real_camera_surface_points_{name}.ply")
            save_surface_points_ply(pts_path, surface_points)
            all_surface_points.append(surface_points)
        else:
            print(f"[WARNING] 유효한 표면 포인트가 없습니다: {name}")

    if len(all_surface_points) > 0:
        merged_points = np.concatenate(all_surface_points, axis=0)

        # 합쳐진 포인트 클라우드 저장
        merged_pts_path = os.path.join(POINTS_DIR, "real_camera_surface_points.ply")
        save_surface_points_ply(merged_pts_path, merged_points)

        # 분석 데이터 저장
        surface_info = analyze_merged_surface(merged_points)
        surface_info_path = os.path.join(POINTS_DIR, "real_camera_surface_info.json")
        save_json(surface_info_path, surface_info)

        print("[OK] Multi-Camera Scan Completed!")
        print(f"[OK] Total Points: {len(merged_points)}")
        print(f"[OK] Saved points: {merged_pts_path}")
        print(f"[OK] Saved info:   {surface_info_path}")
        return True
    else:
        print("[ERROR] 병합할 수 있는 포인트 클라우드가 없습니다.")
        return False

if __name__ == "__main__":
    try:
        ok = main()
        if not ok:
            sys.exit(1)
        print("\n[INFO] Auto-exiting.")
    except Exception as e:
        import traceback
        traceback.print_exc()
        print(f"[ERROR] {e}")
        sys.exit(1)
    finally:
        simulation_app.close()
