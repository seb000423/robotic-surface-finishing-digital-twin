import sys
import os
import numpy as np
from isaacsim import SimulationApp

# 시스템 ROS2 경로 제거 (Isaac Sim 내부 rclpy 강제 사용)
import sys
import os
if "PYTHONPATH" in os.environ:
    os.environ["PYTHONPATH"] = ":".join([p for p in os.environ["PYTHONPATH"].split(":") if "/opt/ros" not in p])
sys.path = [p for p in sys.path if "/opt/ros" not in p]

simulation_app = SimulationApp({"headless": False})

# ROS2 브릿지 활성화 및 rclpy 임포트
from omni.isaac.core.utils.extensions import enable_extension
enable_extension("omni.isaac.ros2_bridge")

try:
    import rclpy
    from std_msgs.msg import Float64
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False
    print("[WARNING] rclpy 모듈을 찾을 수 없습니다. ROS2 퍼블리시가 비활성화됩니다.")

from omni.isaac.core import World
from omni.isaac.core.objects import VisualSphere
from isaacsim.core.prims import SingleArticulation
from omni.isaac.core.utils.prims import create_prim
from omni.isaac.sensor import ContactSensor

# 스크립트 위치 기준 경로 설정
_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
_SRC_DIR = os.path.dirname(_SCRIPT_DIR)

# RMPFlow 컨트롤러 경로 추가
sys.path.append(os.path.join(_SRC_DIR, "rmpflow"))
from m0609_rmpflow_controller import RMPFlowController

import argparse
parser = argparse.ArgumentParser()
parser.add_argument("--obj_name", type=str, default="car")
args, unknown = parser.parse_known_args()
obj_name = args.obj_name

# 로봇 모델 절대경로
ROBOT_USD_PATH = os.path.join(_SRC_DIR, "usd", "env", "Collected_m0609_with_polisher", "m0609_with_polisher.usd")
PLY_PATH = os.path.join(_SRC_DIR, "scan_result", obj_name, "points", "real_camera_surface_points.ply")

ROBOT_BASE_POSITION = np.array([-1.0, -0.7, 0.5])
ROBOT_BASE_YAW_RAD = -0.5 * np.pi  # initial pose -> right 90 deg, so the robot faces the car
ROBOT_BASE_ORIENTATION = np.array([
    np.cos(ROBOT_BASE_YAW_RAD * 0.5),
    0.0,
    0.0,
    np.sin(ROBOT_BASE_YAW_RAD * 0.5),
])
HOOD_Y_MAX = -0.35
PATH_MIN_RADIUS = 0.48
PATH_MAX_RADIUS = 0.66
PATH_EDGE_MARGIN = 0.03
MAX_SURFACE_TILT_DEG = 20.0
MAX_NORMAL_TILT_DEG = 20.0
NORMAL_QUERY_K = 35
NORMAL_SMOOTHING = 0.85
MAX_PATH_JUMP = 0.08
PRESS_OFFSET_MIN = 0.038
PRESS_OFFSET_MAX = 0.080
FALLBACK_PAD_CONTACT_OFFSET_LOCAL = np.array([-0.0025, -0.0200, -0.0040])
TARGET_NORMAL_FORCE = 1.5
CONTACT_FORCE_THRESHOLD = 0.5
CONTACT_GEOMETRY_MAX_OFFSET = 0.045
USE_VIRTUAL_SOFT_PAD = True
ENABLE_POLISHING_DISK_COLLIDER = True
VIRTUAL_PAD_CONTACT_DISTANCE = 0.045
VIRTUAL_PAD_STIFFNESS = 500.0
VIRTUAL_PAD_DAMPING = 6.0
POLISHING_COMPLIANT_STIFFNESS = 450.0
POLISHING_COMPLIANT_DAMPING = 65.0
POLISHING_DISK_RADIUS = 0.075
POLISHING_DISK_HEIGHT = 0.030
POLISHING_DISK_SIDES = 16
POLISHING_VISUAL_MAX_COMPRESSION = 0.018
SPONGE_VISUAL_UPDATE_INTERVAL_STEPS = 10
POLISHING_DISK_LOCAL_POSITION = np.array([0.0, -0.002, 0.0])
POLISHING_DYNAMIC_FRICTION = 0.15
POLISHING_STATIC_FRICTION = 0.25
POLISHING_RESTITUTION = 0.0
USE_ONLY_POLISHING_DISK_COLLIDER = True
PAD_SPIN_VELOCITY = 5.0
MAX_PRESS_VELOCITY = 0.006
FORCE_FILTER_ALPHA = 0.55
FORCE_CONTROL_CLIP_N = 15.0
FORCE_SPIKE_N = 25.0
FORCE_ADVANCE_MIN_N = 0.5
FORCE_ADVANCE_MAX_N = 3.5
PHYSICAL_FORCE_SOFT_LIMIT_N = 10.0
PHYSICAL_FORCE_ADVANCE_MIN_N = 0.5
PHYSICAL_FORCE_ADVANCE_MAX_N = 12.0
PHYSICAL_FORCE_HARD_LIMIT_N = 25.0
PHYSICAL_FORCE_SOFT_RETRACT_GAIN = 0.00020
PHYSICAL_FORCE_SOFT_RETRACT_MAX_STEP = 0.004
SURFACE_GUARD_MIN_CLEARANCE = -0.0015
SURFACE_GUARD_RECOVER_CLEARANCE = 0.010
SPIKE_RETRACT_STEP = 0.003
SPIKE_PAUSE_STEPS = 8
CONTACT_SETTLE_STEPS = 2
PATH_ADVANCE_PER_STEP = 1.0 / 60.0
PATH_CREEP_ADVANCE_PER_STEP = 0.0
STATUS_LOG_INTERVAL_STEPS = 15
HOME_JOINT_POSITIONS = np.array([0.0, -1.05, 1.45, 0.0, 1.15, 0.0])
HOME_INITIAL_SETTLE_STEPS = 90
APPROACH_SETTLE_STEPS = 120
RETRACT_SETTLE_STEPS = 90
RETURN_HOME_SETTLE_STEPS = 180
SAFE_APPROACH_CLEARANCE = 0.18
SAFE_RETRACT_CLEARANCE = 0.18

STATE_HOME = "HOME"
STATE_APPROACH = "APPROACH"
STATE_POLISH = "POLISH"
STATE_RETRACT = "RETRACT"
STATE_RETURN_HOME = "RETURN_HOME"
STATE_DONE = "DONE"

def load_ply_points(path):
    points = []
    with open(path, 'r') as f:
        lines = f.readlines()
        header_ended = False
        for line in lines:
            if header_ended:
                parts = line.strip().split()
                if len(parts) >= 3:
                    points.append([float(parts[0]), float(parts[1]), float(parts[2])])
            if line.strip() == "end_header":
                header_ended = True
    return np.array(points)

def z_align_quat(z_vec):
    """Z축을 원하는 방향(z_vec)으로 정렬하되, X축을 로봇 기준 정면(-X)으로 유지시켜 손목이 180도 꺾이는 특이점(Gimbal Lock)을 방지합니다."""
    z_vec = z_vec / np.linalg.norm(z_vec)
    
    # 샌더의 X축(전면)이 로봇이 뻗는 방향(+X)을 바라보도록 설정하여 손목이 꼬이는 것을 방지
    fwd = np.array([1.0, 0.0, 0.0])
    
    if abs(np.dot(fwd, z_vec)) > 0.99:
        fwd = np.array([0.0, -1.0, 0.0])
        
    y_vec = np.cross(z_vec, fwd)
    y_vec = y_vec / np.linalg.norm(y_vec)
    
    x_vec = np.cross(y_vec, z_vec)
    x_vec = x_vec / np.linalg.norm(x_vec)
    
    R_mat = np.column_stack((x_vec, y_vec, z_vec))
    from scipy.spatial.transform import Rotation
    q = Rotation.from_matrix(R_mat).as_quat() # [x, y, z, w]
    
    return np.array([q[3], q[0], q[1], q[2]]) # [w, x, y, z] 반환

def estimate_surface_normal(kdtree, surface_points, target_pos, k=NORMAL_QUERY_K):
    sample_count = min(k, len(surface_points))
    distances, indices = kdtree.query(target_pos, k=sample_count)
    neighbors = surface_points[np.atleast_1d(indices)]

    centroid = np.mean(neighbors, axis=0)
    centered = neighbors - centroid
    cov = np.dot(centered.T, centered)

    eigenvalues, eigenvectors = np.linalg.eigh(cov)
    normal = eigenvectors[:, 0]
    if normal[2] < 0:
        normal = -normal

    normal = normal / np.linalg.norm(normal)
    tilt_deg = np.degrees(np.arccos(np.clip(normal[2], -1.0, 1.0)))
    return normal, tilt_deg

def clamp_normal_tilt(normal, max_tilt_deg=MAX_NORMAL_TILT_DEG):
    vertical = np.array([0.0, 0.0, 1.0])
    cos_limit = np.cos(np.radians(max_tilt_deg))
    dot = float(np.dot(normal, vertical))
    if dot >= cos_limit:
        return normal

    horizontal = normal - vertical * dot
    horizontal_norm = np.linalg.norm(horizontal)
    if horizontal_norm < 1e-6:
        return vertical

    limited = vertical * cos_limit + (horizontal / horizontal_norm) * np.sin(np.radians(max_tilt_deg))
    return limited / np.linalg.norm(limited)

def filter_safe_waypoints(points, surface_points, kdtree):
    if len(points) == 0:
        return points

    surface_min = np.min(surface_points, axis=0)
    surface_max = np.max(surface_points, axis=0)
    safe_points = []
    reject_counts = {
        "reach": 0,
        "edge": 0,
        "tilt": 0,
    }

    for pt in points:
        dist = np.linalg.norm(pt - ROBOT_BASE_POSITION)
        if dist < PATH_MIN_RADIUS or dist > PATH_MAX_RADIUS:
            reject_counts["reach"] += 1
            continue

        inside_x = surface_min[0] + PATH_EDGE_MARGIN <= pt[0] <= surface_max[0] - PATH_EDGE_MARGIN
        inside_y = surface_min[1] + PATH_EDGE_MARGIN <= pt[1] <= surface_max[1] - PATH_EDGE_MARGIN
        if not (inside_x and inside_y):
            reject_counts["edge"] += 1
            continue

        _, tilt_deg = estimate_surface_normal(kdtree, surface_points, pt)
        if tilt_deg > MAX_SURFACE_TILT_DEG:
            reject_counts["tilt"] += 1
            continue

        safe_points.append(pt)

    if len(safe_points) == 0:
        print(
            "[ERROR] No safe waypoints after filtering "
            f"(reach rejected={reject_counts['reach']}, edge rejected={reject_counts['edge']}, tilt rejected={reject_counts['tilt']})."
        )
        return np.array([])

    safe_segments = []
    current_segment = [safe_points[0]]
    for prev, curr in zip(safe_points[:-1], safe_points[1:]):
        if np.linalg.norm(curr - prev) <= MAX_PATH_JUMP:
            current_segment.append(curr)
        else:
            safe_segments.append(current_segment)
            current_segment = [curr]
    safe_segments.append(current_segment)

    longest_segment = max(safe_segments, key=len)
    print(f"[Safe Path] Original waypoints: {len(points)}, filtered waypoints: {len(safe_points)}")
    print(
        "[Safe Path] Rejected "
        f"reach={reject_counts['reach']}, edge={reject_counts['edge']}, tilt={reject_counts['tilt']} "
        f"(radius {PATH_MIN_RADIUS:.2f}~{PATH_MAX_RADIUS:.2f}m, edge margin {PATH_EDGE_MARGIN:.2f}m)."
    )
    print(f"[Safe Path] Selected longest continuous segment: {len(longest_segment)} waypoints.")
    return np.array(longest_segment)

def compute_pad_contact_offset_local(stage, link_6_path, pad_path):
    from pxr import Usd, UsdGeom, Gf

    link_prim = stage.GetPrimAtPath(link_6_path)
    pad_prim = stage.GetPrimAtPath(pad_path)
    if not (link_prim.IsValid() and pad_prim.IsValid()):
        print("[WARNING] Cannot compute pad contact offset. Falling back to measured default offset.")
        return FALLBACK_PAD_CONTACT_OFFSET_LOCAL.copy()

    try:
        cache = UsdGeom.XformCache()
        link_to_world = cache.GetLocalToWorldTransform(link_prim)
        world_to_link = link_to_world.GetInverse()
        bbox = UsdGeom.Imageable(pad_prim).ComputeWorldBound(Usd.TimeCode.Default(), "default")
        aligned = bbox.ComputeAlignedBox()
        min_pt = aligned.GetMin()
        max_pt = aligned.GetMax()

        contact_world = Gf.Vec3d(
            float((min_pt[0] + max_pt[0]) * 0.5),
            float((min_pt[1] + max_pt[1]) * 0.5),
            float(min_pt[2]),
        )
        contact_local = world_to_link.Transform(contact_world)
        offset = np.array([contact_local[0], contact_local[1], contact_local[2]], dtype=float)
        print(f"[INFO] Pad contact offset in link_6 frame: {offset}")
        return offset
    except Exception as exc:
        print(f"[WARNING] Pad contact offset calculation failed: {exc}")
        return FALLBACK_PAD_CONTACT_OFFSET_LOCAL.copy()

def compute_cylinder_contact_offset_local(stage, link_6_path, cylinder_path, cylinder_height):
    from pxr import UsdGeom, Gf

    link_prim = stage.GetPrimAtPath(link_6_path)
    cylinder_prim = stage.GetPrimAtPath(cylinder_path)
    if not (link_prim.IsValid() and cylinder_prim.IsValid()):
        print("[WARNING] Cannot compute polishing disk contact offset. Falling back to measured default offset.")
        return FALLBACK_PAD_CONTACT_OFFSET_LOCAL.copy()

    cache = UsdGeom.XformCache()
    cylinder_to_world = cache.GetLocalToWorldTransform(cylinder_prim)
    world_to_link = cache.GetLocalToWorldTransform(link_prim).GetInverse()
    contact_world = cylinder_to_world.Transform(Gf.Vec3d(0.0, -0.5 * cylinder_height, 0.0))
    contact_local = world_to_link.Transform(contact_world)
    offset = np.array([contact_local[0], contact_local[1], contact_local[2]], dtype=float)
    print(f"[INFO] Polishing disk contact offset in link_6 frame: {offset}")
    return offset

def build_polishing_disk_mesh(radius, height, side_count, visual_compression=0.0):
    from pxr import Gf, Vt

    half_height = 0.5 * float(height)
    compression = float(np.clip(visual_compression, 0.0, max(0.0, float(height) * 0.8)))
    bottom_y = -half_height
    top_y = max(bottom_y + 0.002, half_height - compression)

    points = []
    for y in (bottom_y, top_y):
        for i in range(side_count):
            angle = 2.0 * np.pi * float(i) / float(side_count)
            points.append(Gf.Vec3f(radius * np.cos(angle), y, radius * np.sin(angle)))

    face_counts = [side_count, side_count]
    face_indices = list(reversed(range(side_count)))
    face_indices.extend(range(side_count, side_count * 2))
    for i in range(side_count):
        j = (i + 1) % side_count
        face_counts.append(4)
        face_indices.extend([i, j, side_count + j, side_count + i])

    extent = Vt.Vec3fArray([
        Gf.Vec3f(-radius, bottom_y, -radius),
        Gf.Vec3f(radius, top_y, radius),
    ])
    return Vt.Vec3fArray(points), Vt.IntArray(face_counts), Vt.IntArray(face_indices), extent

def update_polishing_sponge_visual(stage, visual_path, compression):
    from pxr import UsdGeom

    visual_prim = stage.GetPrimAtPath(visual_path)
    if not visual_prim.IsValid():
        return

    visual_mesh = UsdGeom.Mesh(visual_prim)
    visual_compression = float(np.clip(compression, 0.0, POLISHING_VISUAL_MAX_COMPRESSION))
    points, _, _, extent = build_polishing_disk_mesh(
        POLISHING_DISK_RADIUS,
        POLISHING_DISK_HEIGHT,
        POLISHING_DISK_SIDES,
        visual_compression,
    )
    visual_mesh.GetPointsAttr().Set(points)
    visual_mesh.GetExtentAttr().Set(extent)

def create_polishing_contact_disk(stage, old_pad_path, physics_material):
    from pxr import UsdGeom, UsdPhysics, UsdShade, Gf, Sdf, PhysxSchema, Vt

    sander_pad_path = "/World/M0609/m0609/m0609/sander_pad"
    robot_root_path = "/World/M0609/m0609/m0609"
    disk_path = robot_root_path + "/polishing_contact_pad"
    visual_path = disk_path + "/sponge_visual"
    joint_path = sander_pad_path + "/polishing_contact_pad_joint"

    for stale_path in (
        disk_path,
        visual_path,
        sander_pad_path + "/polishing_contact_pad",
        sander_pad_path + "/polishing_contact_pad/sponge_visual",
        joint_path,
        "/World/VirtualPolishingFootprint",
        "/World/PadSpinMarkerYellow",
        "/World/PadSpinMarkerBlack",
        "/World/PadSpinMarkerCenter",
    ):
        if stage.GetPrimAtPath(stale_path).IsValid():
            stage.RemovePrim(Sdf.Path(stale_path))

    side_count = int(POLISHING_DISK_SIDES)
    radius = float(POLISHING_DISK_RADIUS)
    points, face_counts, face_indices, extent = build_polishing_disk_mesh(
        radius,
        POLISHING_DISK_HEIGHT,
        side_count,
    )

    disk = UsdGeom.Mesh.Define(stage, disk_path)
    disk.GetPointsAttr().Set(points)
    disk.GetFaceVertexCountsAttr().Set(face_counts)
    disk.GetFaceVertexIndicesAttr().Set(face_indices)
    disk.GetExtentAttr().Set(extent)
    disk.CreateDisplayColorAttr([Gf.Vec3f(1.0, 1.0, 1.0)])
    disk.CreateDisplayOpacityAttr([0.0])

    sponge_visual = UsdGeom.Mesh.Define(stage, visual_path)
    sponge_visual.GetPointsAttr().Set(points)
    sponge_visual.GetFaceVertexCountsAttr().Set(face_counts)
    sponge_visual.GetFaceVertexIndicesAttr().Set(face_indices)
    sponge_visual.GetExtentAttr().Set(extent)
    sponge_visual.CreateDisplayColorAttr([Gf.Vec3f(1.0, 1.0, 1.0)])
    sponge_visual.CreateDisplayOpacityAttr([1.0])

    cache = UsdGeom.XformCache()
    sander_pad_prim = stage.GetPrimAtPath(sander_pad_path)
    robot_root_prim = stage.GetPrimAtPath(robot_root_path)
    if sander_pad_prim.IsValid() and robot_root_prim.IsValid():
        sander_pad_to_world = cache.GetLocalToWorldTransform(sander_pad_prim)
        robot_root_to_world = cache.GetLocalToWorldTransform(robot_root_prim)
        local_to_sander_pad = Gf.Matrix4d(1.0)
        local_to_sander_pad.SetTranslate(Gf.Vec3d(*[float(v) for v in POLISHING_DISK_LOCAL_POSITION]))
        disk_to_world = local_to_sander_pad * sander_pad_to_world
        disk_to_robot_root = disk_to_world * robot_root_to_world.GetInverse()
    else:
        disk_to_robot_root = Gf.Matrix4d(1.0)
        disk_to_robot_root.SetTranslate(Gf.Vec3d(*[float(v) for v in POLISHING_DISK_LOCAL_POSITION]))

    xform = UsdGeom.Xformable(disk.GetPrim())
    xform.ClearXformOpOrder()
    xform.AddTransformOp().Set(disk_to_robot_root)

    UsdPhysics.RigidBodyAPI.Apply(disk.GetPrim())
    mass_api = UsdPhysics.MassAPI.Apply(disk.GetPrim())
    mass_api.CreateMassAttr().Set(0.05)
    collision_api = UsdPhysics.CollisionAPI.Apply(disk.GetPrim())
    collision_api.CreateCollisionEnabledAttr().Set(bool(ENABLE_POLISHING_DISK_COLLIDER))
    mesh_collision = UsdPhysics.MeshCollisionAPI.Apply(disk.GetPrim())
    mesh_collision.CreateApproximationAttr().Set("convexHull")
    try:
        physx_collision = PhysxSchema.PhysxCollisionAPI.Apply(disk.GetPrim())
        physx_collision.CreateContactOffsetAttr().Set(0.006)
        physx_collision.CreateRestOffsetAttr().Set(0.0)
    except Exception:
        pass
    UsdShade.MaterialBindingAPI.Apply(disk.GetPrim()).Bind(
        physics_material,
        UsdShade.Tokens.weakerThanDescendants,
        "physics"
    )

    old_pad_prim = stage.GetPrimAtPath(old_pad_path)
    if old_pad_prim.IsValid():
        if USE_ONLY_POLISHING_DISK_COLLIDER:
            disabled_count = set_collision_enabled_recursive(stage, sander_pad_path, False)
            print(
                f"[INFO] Disabled original hard pad collisions under {sander_pad_path} "
                f"({disabled_count} prims). The lower polishing disk is the contact collider.",
                flush=True,
            )
        else:
            UsdPhysics.CollisionAPI.Apply(old_pad_prim)

    fixed_joint = UsdPhysics.FixedJoint.Define(stage, joint_path)
    fixed_joint.CreateBody0Rel().SetTargets([Sdf.Path(sander_pad_path)])
    fixed_joint.CreateBody1Rel().SetTargets([Sdf.Path(disk_path)])
    fixed_joint.CreateLocalPos0Attr().Set(Gf.Vec3f(*[float(v) for v in POLISHING_DISK_LOCAL_POSITION]))
    fixed_joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
    fixed_joint.CreateLocalRot0Attr().Set(Gf.Quatf(1.0, Gf.Vec3f(0.0, 0.0, 0.0)))
    fixed_joint.CreateLocalRot1Attr().Set(Gf.Quatf(1.0, Gf.Vec3f(0.0, 0.0, 0.0)))
    fixed_joint.CreateCollisionEnabledAttr().Set(False)

    print(
        "[INFO] Added polishing contact pad rigid body "
        f"(radius={POLISHING_DISK_RADIUS:.3f}m, height={POLISHING_DISK_HEIGHT:.3f}m): {disk_path} "
        f"fixed to {sander_pad_path}",
        flush=True,
    )
    return disk_path

def set_collision_enabled_recursive(stage, root_path, enabled):
    from pxr import Usd, UsdGeom, UsdPhysics

    root_prim = stage.GetPrimAtPath(root_path)
    if not root_prim.IsValid():
        return 0

    changed = 0
    for prim in Usd.PrimRange(root_prim):
        if prim.IsA(UsdGeom.Mesh) or prim.HasAPI(UsdPhysics.CollisionAPI):
            collision_api = UsdPhysics.CollisionAPI.Apply(prim)
            attr = collision_api.GetCollisionEnabledAttr()
            if not attr:
                attr = collision_api.CreateCollisionEnabledAttr()
            attr.Set(bool(enabled))
            changed += 1
    return changed

def configure_compliant_material(stage, material_path):
    from pxr import PhysxSchema, Sdf

    material_prim = stage.GetPrimAtPath(material_path)
    if not material_prim.IsValid():
        return False

    try:
        physx_material = PhysxSchema.PhysxMaterialAPI.Apply(material_prim)

        stiffness_attr = getattr(physx_material, "CreateCompliantContactStiffnessAttr", None)
        damping_attr = getattr(physx_material, "CreateCompliantContactDampingAttr", None)
        if stiffness_attr is not None:
            stiffness_attr().Set(float(POLISHING_COMPLIANT_STIFFNESS))
        else:
            material_prim.CreateAttribute(
                "physxMaterial:compliantContactStiffness",
                Sdf.ValueTypeNames.Float,
            ).Set(float(POLISHING_COMPLIANT_STIFFNESS))

        if damping_attr is not None:
            damping_attr().Set(float(POLISHING_COMPLIANT_DAMPING))
        else:
            material_prim.CreateAttribute(
                "physxMaterial:compliantContactDamping",
                Sdf.ValueTypeNames.Float,
            ).Set(float(POLISHING_COMPLIANT_DAMPING))
        return True
    except Exception as exc:
        print(f"[WARNING] Failed to configure compliant material: {exc}", flush=True)
        return False

def append_text_log(path, message):
    with open(path, "a") as f:
        f.write(message + "\n")

def load_ply_mesh(path):
    points = []
    faces = []
    num_vertices = 0
    num_faces = 0
    header_ended = False
    in_vertices = False
    in_faces = False
    vertex_count = 0
    face_count = 0
    
    with open(path, 'r') as f:
        lines = f.readlines()
        
    for line in lines:
        line = line.strip()
        if not line:
            continue
        if not header_ended:
            if line.startswith("element vertex"):
                num_vertices = int(line.split()[2])
            elif line.startswith("element face"):
                num_faces = int(line.split()[2])
            elif line == "end_header":
                header_ended = True
                if num_vertices > 0:
                    in_vertices = True
            continue
            
        if in_vertices:
            parts = line.split()
            points.append([float(parts[0]), float(parts[1]), float(parts[2])])
            vertex_count += 1
            if vertex_count == num_vertices:
                in_vertices = False
                if num_faces > 0:
                    in_faces = True
            continue
            
        if in_faces:
            parts = line.split()
            count = int(parts[0])
            face = [int(x) for x in parts[1:count+1]]
            if len(face) == 3: # 삼각형만 지원
                faces.append(face)
            elif len(face) == 4: # 사각형인 경우 2개의 삼각형으로 분할
                faces.append([face[0], face[1], face[2]])
                faces.append([face[0], face[2], face[3]])
            face_count += 1
            if face_count == num_faces:
                in_faces = False
            continue
            
    return np.array(points, dtype=np.float32), faces

def create_usd_mesh(stage, prim_path, points, faces):
    from pxr import UsdGeom, Vt, Gf
    mesh = UsdGeom.Mesh.Define(stage, prim_path)
    
    pts_list = [Gf.Vec3f(float(p[0]), float(p[1]), float(p[2])) for p in points]
    mesh.GetPointsAttr().Set(Vt.Vec3fArray(pts_list))
    
    face_vertex_counts = [int(len(f)) for f in faces]
    mesh.GetFaceVertexCountsAttr().Set(Vt.IntArray(face_vertex_counts))
    
    face_vertex_indices = [int(idx) for f in faces for idx in f]
    mesh.GetFaceVertexIndicesAttr().Set(Vt.IntArray(face_vertex_indices))
    
    min_pt = np.min(points, axis=0)
    max_pt = np.max(points, axis=0)
    extents = [Gf.Vec3f(float(min_pt[0]), float(min_pt[1]), float(min_pt[2])),
               Gf.Vec3f(float(max_pt[0]), float(max_pt[1]), float(max_pt[2]))]
    mesh.GetExtentAttr().Set(Vt.Vec3fArray(extents))
    
    # 포인트 클라우드(빨강)와 대비되도록 원본 형체는 파란색 계열로 렌더링
    mesh.GetDisplayColorAttr().Set(Vt.Vec3fArray([Gf.Vec3f(0.2, 0.5, 0.8)]))
    
    # 물리적 충돌 속성(Collision API) 추가 (진짜 껍데기 생성)
    from pxr import UsdPhysics
    UsdPhysics.CollisionAPI.Apply(mesh.GetPrim())
    mesh_collision = UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim())
    # 접촉 판정이 시각 메쉬와 어긋나지 않도록 triangle mesh를 사용합니다.
    mesh_collision.CreateApproximationAttr().Set("none")
    
    return mesh

def main():
    log_file_path = os.path.join(_SCRIPT_DIR, "force_log.csv")
    status_log_path = os.path.join(_SCRIPT_DIR, "status_log.txt")
    with open(status_log_path, "w") as f:
        f.write("[INFO] polishing_v1 boot started\n")

    def boot_log(message):
        print(message, flush=True)
        append_text_log(status_log_path, message)

    boot_log("[INFO] Creating Isaac world.")
    world = World(stage_units_in_meters=1.0)
    world.scene.add_default_ground_plane()
    
    # 로봇이 흰색이라 잘 보이도록 어두운 톤의 바닥 시각화용 박스 추가
    from omni.isaac.core.objects import VisualCuboid
    world.scene.add(
        VisualCuboid(
            prim_path="/World/DarkFloor",
            name="dark_floor",
            position=np.array([0.0, 0.0, -0.01]),
            scale=np.array([10.0, 10.0, 0.02]),
            color=np.array([0.15, 0.15, 0.15])
        )
    )
    
    # 조명
    create_prim(
        prim_path="/World/DomeLight",
        prim_type="DomeLight",
        attributes={"inputs:intensity": 1000.0, "inputs:color": (1.0, 1.0, 1.0)}
    )
    
    # 로봇 로드 (요청에 따라 로봇을 지상에서 50cm 위로 올리고 차에서 20cm 더 멀리 배치)
    create_prim(
        prim_path="/World/M0609",
        prim_type="Xform",
        position=ROBOT_BASE_POSITION,
        orientation=ROBOT_BASE_ORIENTATION,
        usd_path=ROBOT_USD_PATH
    )
    boot_log(
        "[INFO] Robot USD loaded. "
        f"base_position={ROBOT_BASE_POSITION.tolist()}, "
        f"base_yaw={np.degrees(ROBOT_BASE_YAW_RAD):.1f}deg"
    )
    
    # 실제 조인트 컨트롤을 위한 Articulation 생성
    # m0609_polishing.usd 파일 구조에 맞게 경로 지정
    robot_prim_path = "/World/M0609/m0609/m0609"
    robot_articulation = SingleArticulation(prim_path=robot_prim_path, name="m0609_robot")
    
    # 스캔 데이터 로드 (원본 포인트 클라우드)
    raw_points = load_ply_points(PLY_PATH)
    
    # ---------------------------------------------------------
    # 본네트(Hood) 부분만 추출 (앞유리창 제외)
    # ---------------------------------------------------------
    # Y축 기준으로 -0.35보다 큰 쪽은 위로 솟아오르는 앞유리(Windshield)이므로 잘라냅니다.
    filtered_points = []
    for p in raw_points:
        if p[1] < HOOD_Y_MAX:
            filtered_points.append(p)
    raw_points = np.array(filtered_points)
    print(f"Filtered to {len(raw_points)} points (Hood only, Windshield removed).")
    # ---------------------------------------------------------

    # [정밀 제어를 위한 실제 표면 법선 계산 준비]
    from scipy.spatial import KDTree
    print("[INFO] Building KDTree for accurate surface normal estimation...")
    kdtree = KDTree(raw_points)
    print("[INFO] KDTree build complete.")
    
    import omni.usd
    stage = omni.usd.get_context().get_stage()
    
    # 타겟 자동차 물체 로드 (원본 USD 파일)
    TARGET_USD_PATH = os.path.join(_SRC_DIR, "scan_obj", f"{obj_name}.usd")
    create_prim(
        prim_path="/World/Car",
        prim_type="Xform",
        position=np.array([0.0, 0.0, 0.0]),
        usd_path=TARGET_USD_PATH
    )
    boot_log("[INFO] Car USD loaded.")
    
    # 작업 공간(Room) 로드
    ROOM_USD_PATH = os.path.join(_SRC_DIR, "usd", "env", "room.usd")
    create_prim(
        prim_path="/World/Room",
        prim_type="Xform",
        position=np.array([0.0, 0.0, 0.0]),
        usd_path=ROOM_USD_PATH
    )
    
    # 3. 3D 포인트 클라우드 시각화 (UsdGeom.Points를 사용하여 빠르고 가볍게 렌더링)
    from pxr import UsdGeom, Vt, Gf
    pts_prim = UsdGeom.Points.Define(stage, "/World/Points")
    # 본네트 표면이 잘 보이도록 Z축 오프셋을 없애고(표면에 겹치게) 점 크기를 확 줄임
    pts_prim.CreatePointsAttr(Vt.Vec3fArray([Gf.Vec3f(float(p[0]), float(p[1]), float(p[2]) + 0.000) for p in raw_points]))
    pts_prim.CreateWidthsAttr(Vt.FloatArray([0.002] * len(raw_points)))
    pts_prim.CreateDisplayColorAttr(Vt.Vec3fArray([Gf.Vec3f(1.0, 0.0, 0.0)]))
        
    # 4. 3D 웨이포인트 경로 로드 (visualize_in_isaac.py와 동일하게 path.npy 참조)
    PATH_NPY = os.path.join(_SRC_DIR, "scan_result", obj_name, "path.npy")
    if os.path.exists(PATH_NPY):
        points = np.load(PATH_NPY)
        print(f"Loaded {len(points)} waypoints from {PATH_NPY} for robot tracking.")
        
        points = filter_safe_waypoints(points, raw_points, kdtree)
        # --------------------------------------------------------
    else:
        print(f"[ERROR] Cannot find {PATH_NPY}")
        points = np.array([])
    
    # 접촉 센서 부착 (어드미턴스 제어용)
    # 기존 패드 위치를 찾은 뒤, 아래쪽 폴리싱 collider를 그 자식으로 만들어 센서를 붙입니다.
    pad_path_candidates = [
        "/World/M0609/m0609/m0609/sander_pad/pad_visual/sander_ref/OnRobot_Sander_v2/tn__104327_",
        "/World/M0609/m0609/m0609/link_6/quick_mount/sanding_kit/OnRobot_Sander_v2/tn__104327_",
        "/World/M0609/m0609/m0609/link_6/sanding_kit/OnRobot_Sander_v2/tn__104327_",
    ]
    pad_path = pad_path_candidates[0]
    try:
        from omni.isaac.core.utils.prims import get_prim_at_path
        for candidate in pad_path_candidates:
            if get_prim_at_path(candidate):
                pad_path = candidate
                break
    except:
        pass
    print(f"[INFO] Using pad collision/contact prim: {pad_path}", file=sys.stderr, flush=True)
    # 1. ContactSensor를 생성하기 전에 먼저 ContactReportAPI를 적용합니다 (뒷북 방지)
    import omni.usd
    from pxr import PhysxSchema
    stage = omni.usd.get_context().get_stage()
    
    # --- 1번 개선책: 부드러운 물리 재질(Soft Material) 및 물리 씬 안정화 적용 ---
    from pxr import UsdShade, UsdPhysics
    
    # 1-1. 폭발적인 튕김 방지를 위해 Physics Scene의 최대 반발 속도 제한
    physics_scene_prim = stage.GetPrimAtPath("/physicsScene")
    if physics_scene_prim.IsValid():
        physx_scene = PhysxSchema.PhysxSceneAPI(physics_scene_prim)
        # 물체가 파고들었을 때 튕겨내는 최대 속도를 5cm/s로 제한 (기본값은 무제한이라 폭발함)
        from pxr import Sdf
        physx_scene.GetPrim().CreateAttribute("physxScene:maxDepenetrationVelocity", Sdf.ValueTypeNames.Float).Set(0.05)
        
    # 1-2. 스펀지처럼 반발력이 없는 물리 재질 생성 (PhysicsMaterial 사용)
    from omni.isaac.core.materials import PhysicsMaterial
    no_bounce_material = PhysicsMaterial(
        prim_path="/World/NoBounceMaterial",
        dynamic_friction=POLISHING_DYNAMIC_FRICTION,
        static_friction=POLISHING_STATIC_FRICTION,
        restitution=POLISHING_RESTITUTION
    )
    compliant_material_configured = configure_compliant_material(stage, "/World/NoBounceMaterial")
    
    # 1-3. 하드한 기존 샌더는 visual/회전 구조로 두고, 아래쪽 폴리싱 디스크만 접촉 collider로 사용합니다.
    original_pad_path = pad_path
    material_prim = stage.GetPrimAtPath("/World/NoBounceMaterial")
    pad_path = create_polishing_contact_disk(stage, original_pad_path, UsdShade.Material(material_prim))
    pad_visual_path = pad_path + "/sponge_visual"
    print(f"[INFO] Using polishing contact disk as contact prim: {pad_path}", file=sys.stderr, flush=True)
    contact_report_path = pad_path
    print(f"[INFO] Contact sensor/report prim: {contact_report_path}", file=sys.stderr, flush=True)
    boot_log(f"[INFO] Polishing contact disk prepared: {pad_path}")
        
    # 1-4. 타겟 물체(자동차)에 Rigid Body, Collider, 그리고 마찰력 0 재질 적용
    from pxr import UsdShade, UsdPhysics, Usd, UsdGeom
    target_prim = stage.GetPrimAtPath("/World/Car")
    if target_prim.IsValid():
        # 하위 메쉬들에 충돌체(Collider) 적용 및 재질 바인딩 (RigidBody를 적용하지 않아 Static Collider로 만듦)
        for prim in Usd.PrimRange(target_prim):
            if prim.IsA(UsdGeom.Mesh):
                UsdPhysics.CollisionAPI.Apply(prim)
                mesh_collision = UsdPhysics.MeshCollisionAPI.Apply(prim)
                # Convex 근사는 본네트보다 큰 보이지 않는 collider를 만들 수 있어
                # 접촉 판정이 시각 메쉬와 맞도록 triangle mesh를 사용합니다.
                mesh_collision.CreateApproximationAttr().Set("none")
                
                # 마찰력 0 재질 바인딩
                material_prim = stage.GetPrimAtPath("/World/NoBounceMaterial")
                UsdShade.MaterialBindingAPI.Apply(prim).Bind(UsdShade.Material(material_prim), UsdShade.Tokens.weakerThanDescendants, "physics")
    # -------------------------------------------------------------------------
    # (실제 물리 모터(Revolute Joint) 추가 코드는 로봇 Articulation 트리를 붕괴시켜 로봇이 주저앉는 원인이 되므로 삭제함)
    
    pad_prim = stage.GetPrimAtPath(contact_report_path)
    if pad_prim.IsValid():
        # 인스턴스 프록시(Instance Proxy) 오류 방지: 센서 부착 경로 상위의 모든 인스턴스 속성 해제
        parts = contact_report_path.split('/')
        for i in range(2, len(parts)+1):
            curr_path = '/'.join(parts[:i])
            p = stage.GetPrimAtPath(curr_path)
            if p.IsValid() and p.IsInstance():
                p.SetInstanceable(False)
                
        # 인스턴스가 해제된 후 다시 프림을 가져옴
        pad_prim = stage.GetPrimAtPath(contact_report_path)
        if pad_prim.IsValid():
            report_api = PhysxSchema.PhysxContactReportAPI.Apply(pad_prim)
            report_api.CreateThresholdAttr().Set(0.0)
    # 2. API가 적용된 프림에 센서를 생성합니다.
    contact_sensor = ContactSensor(
        prim_path=contact_report_path + "/contact_sensor",
        name="pad_contact_sensor",
        frequency=60,
        translation=np.array([0, 0, 0]),
    )
    
    boot_log("[INFO] Calling world.reset().")
    world.reset()
    boot_log("[INFO] world.reset() complete.")
    robot_articulation.initialize()
    boot_log("[INFO] Robot articulation initialized.")
    contact_sensor.initialize()
    boot_log("[INFO] Contact sensor initialized.")

    import omni.timeline
    timeline = omni.timeline.get_timeline_interface()
    timeline.play()
    print("[INFO] Isaac timeline auto-play started.", file=sys.stderr, flush=True)

    # 샌딩 패드가 로봇의 Articulation 트리에 병합되었으므로 추가적인 initialize()가 필요하지 않습니다.
    # 터미널에서 로봇 관절의 일원으로서 패드의 모터 속도를 추출할 것입니다.
    
    # --- 동적 TCP 변환 초기화 (하드코딩 제거) ---
    from omni.isaac.core.utils.xforms import get_world_pose
    from scipy.spatial.transform import Rotation as R

    link_6_path = "/World/M0609/m0609/m0609/link_6"
    p_link6_init, q_link6_init = get_world_pose(link_6_path)
    if pad_path.endswith("/polishing_contact_pad"):
        pad_contact_offset_local = compute_cylinder_contact_offset_local(
            stage,
            link_6_path,
            pad_path,
            POLISHING_DISK_HEIGHT
        )
    else:
        pad_contact_offset_local = compute_pad_contact_offset_local(stage, link_6_path, pad_path)
    # (동적 측정 삭제: USD Xform 원점이 실제 메쉬 끝단이 아니라 0.8cm 부근에 찍혀 있어서 오차가 발생함)
    # ---------------------------------------------
    
    # RMPFlow 제어기 초기화
    controller = RMPFlowController(
        name="polishing_controller",
        robot_articulation=robot_articulation,
        urdf_path=os.path.join(_SRC_DIR, "rmpflow", "m0609_isaac_sim.urdf"),
        end_effector_frame_name="link_6"
    )
    
    from omni.isaac.core.objects import VisualCuboid, VisualCylinder
    
    # 1. 로봇 단상 시각화 (Z=0 ~ Z=0.5)
    world.scene.add(
        VisualCylinder(
            prim_path="/World/Pedestal",
            name="robot_pedestal",
            position=np.array([-1.0, -0.7, 0.25]), # 0.5 높이의 중간 지점, 로봇 위치와 동일한 X=-1.0
            radius=0.12,
            height=0.5,
            color=np.array([0.3, 0.3, 0.3]) # 짙은 회색 단상
        )
    )
    

    # pyrefly: ignore [missing-import]
    import omni.isaac.core.utils.rotations as rot_utils
    
    current_target_idx = 0
    current_path_idx_float = 0.0 # 부드러운 보간을 위한 실수형 인덱스
    
    # 어드미턴스 제어용 변수 (안정성 강화)
    z_offset = 0.04  # 처음에는 표면에서 4cm 떨어져서 시작해 관통을 피한 뒤 천천히 눌러 들어갑니다.
    z_vel = 0.0
    admittance_mass = 1.0
    admittance_damping = 80.0 # 진동 방지 및 부드러운 힘 조절을 위한 댐핑
    admittance_stiffness = 0.0
    target_normal_force = TARGET_NORMAL_FORCE
    
    dt = 1.0 / 60.0 # 시뮬레이션 물리 스텝 (60Hz)
    
    # RViz 스타일의 '앞으로 갈 경로(Future Path)' 선분(BasisCurves) 생성
    import omni.usd
    from pxr import UsdGeom, Vt, Gf
    stage = omni.usd.get_context().get_stage()
    
    future_path_prim = UsdGeom.BasisCurves.Define(stage, "/World/FuturePath")
    future_path_prim.CreateTypeAttr().Set(UsdGeom.Tokens.linear)
    future_path_prim.CreateWidthsAttr().Set([0.004]) # 선 두께 얇게
    future_path_prim.CreateDisplayColorAttr().Set([(0.0, 1.0, 0.0)]) # 연두색 선

    completed_path_prim = UsdGeom.BasisCurves.Define(stage, "/World/CompletedPath")
    completed_path_prim.CreateTypeAttr().Set(UsdGeom.Tokens.linear)
    completed_path_prim.CreateWidthsAttr().Set([0.007])
    completed_path_prim.CreateDisplayColorAttr().Set([(1.0, 0.82, 0.05)])
    
    from omni.isaac.core.utils.xforms import get_world_pose
    from omni.isaac.core.utils.prims import get_prim_at_path
    
    if ROS2_AVAILABLE:
        rclpy.init(args=None)
        ros_node = rclpy.create_node('isaac_sim_polishing_node')
        force_pub = ros_node.create_publisher(Float64, '/contact_force/data', 10)
        print("[INFO] ROS2 /contact_force 토픽 퍼블리셔가 시작되었습니다.")

    try:
        import carb
    except Exception:
        carb = None

    def log_status(message):
        console_message = f"[POLISHING_STATUS] {message}"
        print(console_message, flush=True)
        append_text_log(status_log_path, message)
        if carb is not None:
            carb.log_warn(console_message)
        
    append_text_log(status_log_path, "[INFO] polishing_v1 status log started")
    append_text_log(status_log_path, f"[INFO] Using polishing contact disk as contact prim: {pad_path}")
    append_text_log(status_log_path, f"[INFO] Contact sensor/report prim: {contact_report_path}")
    append_text_log(status_log_path, "[INFO] Isaac timeline auto-play started.")
    append_text_log(
        status_log_path,
        "[INFO] Polishing material: "
        f"dynamic_friction={POLISHING_DYNAMIC_FRICTION}, "
        f"static_friction={POLISHING_STATIC_FRICTION}, "
        f"restitution={POLISHING_RESTITUTION}, "
        f"compliant_stiffness={POLISHING_COMPLIANT_STIFFNESS}, "
        f"compliant_damping={POLISHING_COMPLIANT_DAMPING}, "
        f"compliant_configured={compliant_material_configured}"
    )
    append_text_log(
        status_log_path,
        "[INFO] Polishing disk: "
        f"radius={POLISHING_DISK_RADIUS:.3f}m, "
        f"height={POLISHING_DISK_HEIGHT:.3f}m, "
        f"sides={POLISHING_DISK_SIDES}, "
        f"local_position={POLISHING_DISK_LOCAL_POSITION.tolist()}, "
        f"spin_velocity={PAD_SPIN_VELOCITY:.2f}rad/s, "
        f"visual_path={pad_visual_path}"
    )
    append_text_log(
        status_log_path,
        "[INFO] Contact collider mode: "
        f"lower_polishing_disk_only={USE_ONLY_POLISHING_DISK_COLLIDER}, "
        f"virtual_soft_pad={USE_VIRTUAL_SOFT_PAD}, "
        f"physical_disk_collider={ENABLE_POLISHING_DISK_COLLIDER}"
    )
    append_text_log(
        status_log_path,
        "[INFO] Virtual soft pad: "
        f"contact_distance={VIRTUAL_PAD_CONTACT_DISTANCE:.3f}m, "
        f"stiffness={VIRTUAL_PAD_STIFFNESS:.1f}N/m, "
        f"damping={VIRTUAL_PAD_DAMPING:.1f}N/(m/s), "
        f"visual_max_compression={POLISHING_VISUAL_MAX_COMPRESSION:.3f}m"
    )
    append_text_log(
        status_log_path,
        "[INFO] Stable contact advance: "
        f"{FORCE_ADVANCE_MIN_N:.1f}N <= filtered_force <= {FORCE_ADVANCE_MAX_N:.1f}N, "
        f"{PHYSICAL_FORCE_ADVANCE_MIN_N:.1f}N <= sensor_raw <= {PHYSICAL_FORCE_ADVANCE_MAX_N:.1f}N, "
        f"geometry_contact_max_offset={CONTACT_GEOMETRY_MAX_OFFSET:.3f}m, "
        f"settle_steps={CONTACT_SETTLE_STEPS}, "
        f"path_advance_per_step={PATH_ADVANCE_PER_STEP:.5f}"
    )
    append_text_log(
        status_log_path,
        "[INFO] Physical force guard: "
        f"soft_limit={PHYSICAL_FORCE_SOFT_LIMIT_N:.1f}N, "
        f"advance_max={PHYSICAL_FORCE_ADVANCE_MAX_N:.1f}N, "
        f"hard_limit={PHYSICAL_FORCE_HARD_LIMIT_N:.1f}N, "
        f"soft_retract_gain={PHYSICAL_FORCE_SOFT_RETRACT_GAIN:.5f}m/N, "
        f"soft_retract_max_step={PHYSICAL_FORCE_SOFT_RETRACT_MAX_STEP:.3f}m"
    )
    with open(log_file_path, "w") as f:
        f.write(
            "step,contact,physical_contact,moving,spin,geometry_contact_possible,"
            "sensor_raw_force,virtual_force,raw_force,filtered_force,"
            "pad_compression,z_offset,normal_tilt,path,pause,stable_steps,"
            "actual_clearance\n"
        )
        
    step_count = 0
    loop_count = 0
    previous_normal = None
    filtered_contact_force = 0.0
    high_force_pause_steps = 0
    stable_contact_steps = 0
    path_complete_reported = False
    
    green_sphere = None
    if len(points) > 0:
        green_sphere = VisualSphere(
            prim_path="/World/PathPointer",
            name="path_pointer",
            position=points[0],
            radius=0.0225, # 기존 0.015의 1.5배
            color=np.array([0.0, 1.0, 0.0])
        )

    from omni.isaac.core.utils.types import ArticulationAction

    run_state = STATE_HOME
    state_step_count = 0

    def apply_home_pose(teleport=False):
        dof_count = int(robot_articulation.num_dof)
        arm_dof_count = min(len(HOME_JOINT_POSITIONS), dof_count)
        if arm_dof_count <= 0:
            return False

        joint_indices = np.arange(arm_dof_count)
        joint_positions = HOME_JOINT_POSITIONS[:arm_dof_count]
        if teleport:
            try:
                robot_articulation.set_joint_positions(joint_positions, joint_indices=joint_indices)
                if hasattr(robot_articulation, "set_joint_velocities"):
                    robot_articulation.set_joint_velocities(np.zeros(arm_dof_count), joint_indices=joint_indices)
            except Exception as exc:
                print(f"[WARNING] Failed to set initial home joint positions: {exc}", flush=True)

        robot_articulation.apply_action(
            ArticulationAction(
                joint_positions=joint_positions,
                joint_indices=joint_indices,
            )
        )
        return True

    def apply_pad_spin_velocity(velocity):
        try:
            if "revolution" in robot_articulation.dof_names:
                pad_idx = robot_articulation.get_dof_index("revolution")
            elif robot_articulation.num_dof > 6:
                pad_idx = 6
            else:
                return False

            robot_articulation.apply_action(
                ArticulationAction(
                    joint_velocities=np.array([velocity]),
                    joint_indices=np.array([pad_idx]),
                )
            )
            return abs(float(velocity)) > 1e-6
        except Exception:
            return False

    def apply_safe_cartesian_target(target_pos, normal, clearance):
        controller._motion_policy.set_robot_base_pose(
            robot_position=ROBOT_BASE_POSITION,
            robot_orientation=ROBOT_BASE_ORIENTATION
        )
        target_orientation = np.array(z_align_quat(-normal))
        target_rot = R.from_quat([
            target_orientation[1],
            target_orientation[2],
            target_orientation[3],
            target_orientation[0],
        ])
        target_pad_pos = target_pos + normal * clearance
        link_6_target_pos = target_pad_pos - target_rot.apply(pad_contact_offset_local)
        actions = controller.forward(
            target_end_effector_position=link_6_target_pos,
            target_end_effector_orientation=target_orientation,
        )
        robot_articulation.apply_action(actions)
        return link_6_target_pos

    def set_run_state(new_state, message):
        nonlocal run_state, state_step_count, z_offset, z_vel
        nonlocal filtered_contact_force, stable_contact_steps, high_force_pause_steps

        run_state = new_state
        state_step_count = 0
        z_vel = 0.0
        filtered_contact_force = 0.0
        stable_contact_steps = 0
        high_force_pause_steps = 0
        if new_state in (STATE_APPROACH, STATE_RETRACT):
            z_offset = PRESS_OFFSET_MAX
        if message:
            log_status(message)

    apply_home_pose(teleport=True)
    apply_pad_spin_velocity(0.0)
    try:
        controller.reset()
    except Exception:
        pass
    log_status(
        "[상태] Initial bent home pose set. "
        f"home_joints={HOME_JOINT_POSITIONS.tolist()}"
    )
        
    while simulation_app.is_running():
        world.step(render=True)
        loop_count += 1

        if loop_count % 120 == 0 and not world.is_playing():
            print("[상태] Isaac timeline is not playing. Press Play or check timeline state.", flush=True)

        if not world.is_playing():
            continue

        if len(points) == 0:
            continue

        if run_state == STATE_HOME:
            apply_home_pose()
            apply_pad_spin_velocity(0.0)
            step_count += 1
            state_step_count += 1

            if step_count % STATUS_LOG_INTERVAL_STEPS == 0:
                log_status(
                    f"[상태] state={run_state} | holding bent home pose "
                    f"| step={state_step_count}/{HOME_INITIAL_SETTLE_STEPS}"
                )

            if state_step_count >= HOME_INITIAL_SETTLE_STEPS:
                set_run_state(
                    STATE_APPROACH,
                    "[상태] Home pose complete. Moving to first waypoint from above.",
                )
            continue

        if run_state == STATE_APPROACH:
            target_pos = points[0]
            normal, normal_tilt_deg = estimate_surface_normal(kdtree, raw_points, target_pos)
            normal = clamp_normal_tilt(normal)
            previous_normal = normal

            apply_safe_cartesian_target(target_pos, normal, SAFE_APPROACH_CLEARANCE)
            apply_pad_spin_velocity(0.0)
            step_count += 1
            state_step_count += 1

            if green_sphere:
                green_sphere.set_world_pose(position=points[0])

            if step_count % STATUS_LOG_INTERVAL_STEPS == 0:
                log_status(
                    f"[상태] state={run_state} | approaching above first waypoint "
                    f"| clearance={SAFE_APPROACH_CLEARANCE * 1000.0:.0f} mm "
                    f"| normal_tilt={normal_tilt_deg:4.1f} deg "
                    f"| step={state_step_count}/{APPROACH_SETTLE_STEPS}"
                )

            if state_step_count >= APPROACH_SETTLE_STEPS:
                z_offset = PRESS_OFFSET_MAX
                set_run_state(
                    STATE_POLISH,
                    "[상태] Approach complete. Starting polishing contact ramp.",
                )
            continue

        if current_target_idx >= len(points) and len(points) > 0:
            if run_state == STATE_POLISH and not path_complete_reported:
                current_target_idx = len(points)
                current_path_idx_float = float(len(points))
                print(f"[상태] Path complete: {current_target_idx}/{len(points)}", flush=True)
                path_complete_reported = True
                set_run_state(
                    STATE_RETRACT,
                    "[상태] Path complete. Retracting above the hood before returning home.",
                )

            if run_state == STATE_RETRACT:
                target_pos = points[-1]
                normal, normal_tilt_deg = estimate_surface_normal(kdtree, raw_points, target_pos)
                normal = clamp_normal_tilt(normal)
                apply_safe_cartesian_target(target_pos, normal, SAFE_RETRACT_CLEARANCE)
                apply_pad_spin_velocity(0.0)
                step_count += 1
                state_step_count += 1

                if step_count % STATUS_LOG_INTERVAL_STEPS == 0:
                    log_status(
                        f"[상태] state={run_state} | lifting from final waypoint "
                        f"| clearance={SAFE_RETRACT_CLEARANCE * 1000.0:.0f} mm "
                        f"| normal_tilt={normal_tilt_deg:4.1f} deg "
                        f"| step={state_step_count}/{RETRACT_SETTLE_STEPS}"
                    )

                if state_step_count >= RETRACT_SETTLE_STEPS:
                    set_run_state(
                        STATE_RETURN_HOME,
                        "[상태] Retract complete. Returning to bent home pose.",
                    )
                continue

            if run_state == STATE_RETURN_HOME:
                apply_home_pose()
                apply_pad_spin_velocity(0.0)
                step_count += 1
                state_step_count += 1

                if step_count % STATUS_LOG_INTERVAL_STEPS == 0:
                    log_status(
                        f"[상태] state={run_state} | returning to bent home pose "
                        f"| step={state_step_count}/{RETURN_HOME_SETTLE_STEPS}"
                    )

                if state_step_count >= RETURN_HOME_SETTLE_STEPS:
                    set_run_state(
                        STATE_DONE,
                        "[상태] Return home complete. Polishing sequence finished.",
                    )
                continue

            if run_state == STATE_DONE:
                apply_home_pose()
                apply_pad_spin_velocity(0.0)
                continue

            if not path_complete_reported:
                print(f"[상태] Path complete: {current_target_idx}/{len(points)}", flush=True)
                path_complete_reported = True
            continue
        
        if run_state == STATE_POLISH and current_target_idx < len(points):
            # [핵심] 점과 점 사이를 부드럽게 이어주는 선형 보간(Linear Interpolation)
            if current_target_idx < len(points) - 1:
                progress = current_path_idx_float - current_target_idx
                p0 = points[current_target_idx]
                p1 = points[current_target_idx + 1]
                target_pos = p0 * (1.0 - progress) + p1 * progress
            else:
                target_pos = points[current_target_idx]
                
            if green_sphere:
                # 다음 rmpflow 웨이포인트를 가리키도록 설정
                next_idx = min(current_target_idx + 1, len(points) - 1)
                green_sphere.set_world_pose(position=points[next_idx])
            
            # [RMPflow 동기화] 베이스 좌표계 동기화 (로봇 물리적 위치와 정확히 일치시켜야 IK 오류가 발생하지 않음)
            controller._motion_policy.set_robot_base_pose(
                robot_position=ROBOT_BASE_POSITION,
                robot_orientation=ROBOT_BASE_ORIENTATION
            )
            
            normal, normal_tilt_deg = estimate_surface_normal(kdtree, raw_points, target_pos)
            normal = clamp_normal_tilt(normal)
            if previous_normal is not None:
                if np.dot(normal, previous_normal) < 0:
                    normal = -normal
                normal = NORMAL_SMOOTHING * previous_normal + (1.0 - NORMAL_SMOOTHING) * normal
                normal = normal / np.linalg.norm(normal)
            previous_normal = normal
            # ---------------------------------------------------------
            
            # --- 동적 TCP 제어 ---
            # 1. 목표 회전: 샌딩면이 본네트 안쪽을 향하도록 link_6 +Z를 -normal에 맞춥니다.
            base_orientation = z_align_quat(-normal) # [w, x, y, z] 형태
            
            # 이전 답변에서 잘못 추가했던 90도 오프셋을 완전히 제거합니다! (이것 때문에 샌더가 옆을 보고 있었습니다)
            target_orientation = np.array(base_orientation) # [w, x, y, z]

            # 2. 어드미턴스 제어: 센서 힘 읽기
            contact_reading = contact_sensor.get_current_frame()
            sensor_raw_force = np.linalg.norm(contact_reading['force']) if contact_reading and 'force' in contact_reading else 0.0
            is_physical_contacting = sensor_raw_force >= CONTACT_FORCE_THRESHOLD
            geometry_contact_possible = z_offset <= CONTACT_GEOMETRY_MAX_OFFSET
            pad_compression = max(0.0, VIRTUAL_PAD_CONTACT_DISTANCE - z_offset)
            virtual_force = max(
                0.0,
                VIRTUAL_PAD_STIFFNESS * pad_compression - VIRTUAL_PAD_DAMPING * z_vel
            )
            if USE_VIRTUAL_SOFT_PAD:
                raw_force = virtual_force
                geometry_contact_possible = pad_compression > 0.0 or is_physical_contacting
            else:
                raw_force = sensor_raw_force if geometry_contact_possible else 0.0
            is_contacting = raw_force >= CONTACT_FORCE_THRESHOLD or is_physical_contacting
            filtered_contact_force = (
                FORCE_FILTER_ALPHA * raw_force +
                (1.0 - FORCE_FILTER_ALPHA) * filtered_contact_force
            )
            if step_count % SPONGE_VISUAL_UPDATE_INTERVAL_STEPS == 0:
                update_polishing_sponge_visual(stage, pad_visual_path, pad_compression)

            if sensor_raw_force > PHYSICAL_FORCE_SOFT_LIMIT_N:
                physical_overload = sensor_raw_force - PHYSICAL_FORCE_SOFT_LIMIT_N
                physical_retract_step = min(
                    PHYSICAL_FORCE_SOFT_RETRACT_MAX_STEP,
                    PHYSICAL_FORCE_SOFT_RETRACT_GAIN * physical_overload,
                )
                z_offset = min(PRESS_OFFSET_MAX, z_offset + physical_retract_step)
                z_vel = max(0.0, z_vel)

            if raw_force > FORCE_SPIKE_N or sensor_raw_force > PHYSICAL_FORCE_HARD_LIMIT_N:
                z_offset = min(PRESS_OFFSET_MAX, z_offset + SPIKE_RETRACT_STEP)
                z_vel = 0.0
                high_force_pause_steps = SPIKE_PAUSE_STEPS
            
            control_force = min(filtered_contact_force, FORCE_CONTROL_CLIP_N)
            force_error = control_force - target_normal_force
            accel = (force_error - admittance_damping * z_vel - admittance_stiffness * z_offset) / admittance_mass
            z_vel += accel * dt
            
            # 속도 제한: 물체에 쾅 부딪히거나 튕겨나가지 않도록 최대 속도를 낮게 제한
            z_vel = np.clip(z_vel, -MAX_PRESS_VELOCITY, MAX_PRESS_VELOCITY) 
            
            z_offset += z_vel * dt
            # 표면 아래로 과도하게 파고들면 팔꿈치와 패드 모서리 충돌이 커지므로 눌림 여유를 작게 제한합니다.
            z_offset = np.clip(z_offset, PRESS_OFFSET_MIN, PRESS_OFFSET_MAX)
            
            # 3. 목표 위치 계산
            # 패드가 가야 할 최종 월드 위치 (어드미턴스 오프셋 적용)
            target_pad_pos = target_pos + normal * z_offset
            
            target_rot = R.from_quat([target_orientation[1], target_orientation[2], target_orientation[3], target_orientation[0]])
            link_6_target_pos = target_pad_pos - target_rot.apply(pad_contact_offset_local)
            
            actions = controller.forward(
                target_end_effector_position=link_6_target_pos,
                target_end_effector_orientation=target_orientation
            )
            robot_articulation.apply_action(actions)
            
            # 새로 추가하신 RevoluteJoint가 기존 로봇(m0609)의 7번째 관절(DOF)로 병합되었으므로,
            # RMPFlow가 제어하는 6개 관절 외에 7번째 관절(인덱스 6)에 직접 각속도를 쏴주어야 모터가 돌아갑니다.
            if step_count == 1:
                print(f"[DEBUG] 로봇 관절 총 개수: {robot_articulation.num_dof}")
                print(f"[DEBUG] 관절 이름: {robot_articulation.dof_names}")
                
            pad_spin_applied_this_step = apply_pad_spin_velocity(PAD_SPIN_VELOCITY)
                
            step_count += 1
                    
            if ROS2_AVAILABLE:
                msg = Float64()
                msg.data = float(raw_force)
                force_pub.publish(msg)
                rclpy.spin_once(ros_node, timeout_sec=0.0)
            
            # 현재 로봇 끝단(link_6)의 실제 위치 가져오기
            link_6_path = "/World/M0609/m0609/m0609/link_6"
            path_advanced_this_step = False
            actual_surface_clearance = None
            if get_prim_at_path(link_6_path):
                tcp_pos, _ = get_world_pose(link_6_path)
                try:
                    pad_world_pos, pad_world_quat = get_world_pose(pad_path)
                    pad_world_rot = R.from_quat([
                        pad_world_quat[1],
                        pad_world_quat[2],
                        pad_world_quat[3],
                        pad_world_quat[0],
                    ])
                    actual_pad_contact_pos = (
                        pad_world_pos +
                        pad_world_rot.apply(np.array([0.0, -0.5 * POLISHING_DISK_HEIGHT, 0.0]))
                    )
                    actual_surface_clearance = float(np.dot(actual_pad_contact_pos - target_pos, normal))
                    if actual_surface_clearance < SURFACE_GUARD_MIN_CLEARANCE:
                        penetration_depth = SURFACE_GUARD_MIN_CLEARANCE - actual_surface_clearance
                        z_offset = min(
                            PRESS_OFFSET_MAX,
                            z_offset + penetration_depth + SURFACE_GUARD_RECOVER_CLEARANCE
                        )
                        z_vel = 0.0
                        high_force_pause_steps = max(high_force_pause_steps, SPIKE_PAUSE_STEPS)
                except Exception:
                    actual_surface_clearance = None
                
                # 앞으로 훑고 지나갈 미래의 경로(최대 500개 점)를 선으로 연결하여 시각화 (RViz Local Path 스타일)
                lookahead_pts = points[current_target_idx : current_target_idx + 500]
                if len(lookahead_pts) > 1:
                    # 표면에 겹치게 Z오프셋 제거
                    vec3f_pts = [Gf.Vec3f(float(p[0]), float(p[1]), float(p[2]) + 0.000) for p in lookahead_pts]
                    future_path_prim.GetPointsAttr().Set(Vt.Vec3fArray(vec3f_pts))
                    future_path_prim.GetCurveVertexCountsAttr().Set([len(lookahead_pts)])

                completed_count = min(max(current_target_idx + 1, 1), len(points))
                completed_pts = points[:completed_count]
                if len(completed_pts) > 1:
                    completed_vec3f_pts = [
                        Gf.Vec3f(float(p[0]), float(p[1]), float(p[2]) + 0.003)
                        for p in completed_pts
                    ]
                    completed_path_prim.GetPointsAttr().Set(Vt.Vec3fArray(completed_vec3f_pts))
                    completed_path_prim.GetCurveVertexCountsAttr().Set([len(completed_pts)])
                
                # 오차 벡터 계산 (목표 위치 - 실제 위치)
                error_vec = link_6_target_pos - tcp_pos
                
                # 어드미턴스 제어로 인해 표면을 누르는 방향(normal)으로는 의도적인 오차가 발생함
                # 따라서 경로 진행을 막지 않도록, 표면을 따라가는(수평) 오차만 계산합니다.
                error_normal_component = np.dot(error_vec, normal) * normal
                error_plane_vec = error_vec - error_normal_component
                pos_error_plane = np.linalg.norm(error_plane_vec)
                
                prev_path_idx_float = current_path_idx_float
                if high_force_pause_steps > 0:
                    high_force_pause_steps -= 1
                    stable_contact_steps = 0
                    current_path_idx_float += PATH_CREEP_ADVANCE_PER_STEP
                else:
                    stable_virtual_contact = (
                        raw_force >= CONTACT_FORCE_THRESHOLD and
                        FORCE_ADVANCE_MIN_N <= filtered_contact_force <= FORCE_ADVANCE_MAX_N
                    )
                    stable_physical_contact = (
                        PHYSICAL_FORCE_ADVANCE_MIN_N <= sensor_raw_force <= PHYSICAL_FORCE_ADVANCE_MAX_N
                    )
                    stable_contact = (
                        geometry_contact_possible and
                        (
                            actual_surface_clearance is None or
                            actual_surface_clearance >= SURFACE_GUARD_MIN_CLEARANCE
                        ) and
                        (stable_virtual_contact or stable_physical_contact)
                    )
                    if stable_contact:
                        stable_contact_steps += 1
                    else:
                        stable_contact_steps = 0

                    if stable_contact_steps >= CONTACT_SETTLE_STEPS:
                        current_path_idx_float += PATH_ADVANCE_PER_STEP
                    else:
                        current_path_idx_float += PATH_CREEP_ADVANCE_PER_STEP
                path_advanced_this_step = current_path_idx_float > prev_path_idx_float
                current_target_idx = int(current_path_idx_float)
                
                if current_target_idx > 0 and current_target_idx % 50 == 0 and step_count % 15 == 0:
                    print(f"Tracking point {current_target_idx}/{len(points)}")

            if step_count % STATUS_LOG_INTERVAL_STEPS == 0:
                stable_display_steps = min(stable_contact_steps, CONTACT_SETTLE_STEPS)
                clearance_text = (
                    "n/a" if actual_surface_clearance is None
                    else f"{actual_surface_clearance * 1000.0:+5.1f} mm"
                )
                status_msg = (
                    f"[물리 접촉력] : state={run_state} "
                    f"| contact={'YES' if is_contacting else 'NO '} "
                    f"| phys={'YES' if is_physical_contacting else 'NO '} "
                    f"| moving={'YES' if path_advanced_this_step else 'NO '} "
                    f"| spin={'YES' if pad_spin_applied_this_step else 'NO '} "
                    f"| geom={'YES' if geometry_contact_possible else 'NO '} "
                    f"| sensor_raw={sensor_raw_force:5.1f} N "
                    f"| virtual={virtual_force:5.1f} N | raw={raw_force:5.1f} N "
                    f"| filt={filtered_contact_force:5.1f} N "
                    f"| compress={pad_compression*1000.0:4.1f} mm "
                    f"| clearance={clearance_text} "
                    f"| z_offset={z_offset:+.3f} m | normal_tilt={normal_tilt_deg:4.1f} deg "
                    f"| path={current_path_idx_float:5.1f}/{len(points)} "
                    f"| stable={stable_display_steps:2d}/{CONTACT_SETTLE_STEPS} "
                    f"| pause={high_force_pause_steps:2d}"
                )
                log_status(status_msg)
                with open(log_file_path, "a") as f:
                    f.write(
                        f"{step_count},{int(is_contacting)},{int(is_physical_contacting)},"
                        f"{int(path_advanced_this_step)},"
                        f"{int(pad_spin_applied_this_step)},"
                        f"{int(geometry_contact_possible)},{sensor_raw_force},"
                        f"{virtual_force},{raw_force},{filtered_contact_force},"
                        f"{pad_compression},{z_offset},{normal_tilt_deg},"
                        f"{current_path_idx_float},{high_force_pause_steps},{stable_contact_steps},"
                        f"{'' if actual_surface_clearance is None else actual_surface_clearance}\n"
                    )

if __name__ == "__main__":
    main()
    if 'ROS2_AVAILABLE' in globals() and ROS2_AVAILABLE:
        rclpy.shutdown()
    simulation_app.close()
