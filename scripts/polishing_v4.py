"""
polishing_v4.py — 4대 로봇 동시 폴리싱 시뮬레이션

각 로봇의 베이스 위치는 path_generator.py의 ROBOT_BASE_POSITIONS와 반드시 동일해야 함.
경로 파일: scan_result/{obj}/path_robot_{i}.npy  (path_generator.py로 생성)
"""
import sys
import os
import numpy as np
from isaacsim import SimulationApp

if "PYTHONPATH" in os.environ:
    os.environ["PYTHONPATH"] = ":".join(
        [p for p in os.environ["PYTHONPATH"].split(":") if "/opt/ros" not in p]
    )
sys.path = [p for p in sys.path if "/opt/ros" not in p]

simulation_app = SimulationApp({"headless": False})

from omni.isaac.core.utils.extensions import enable_extension
enable_extension("omni.isaac.ros2_bridge")

try:
    import rclpy
    from std_msgs.msg import Float64
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False

from omni.isaac.core import World
from omni.isaac.core.objects import VisualSphere, VisualCylinder, VisualCuboid
from isaacsim.core.prims import SingleArticulation
from omni.isaac.core.utils.prims import create_prim
from omni.isaac.sensor import ContactSensor
from omni.isaac.core.utils.types import ArticulationAction
from scipy.spatial.transform import Rotation as R
from scipy.spatial import KDTree

import argparse
parser = argparse.ArgumentParser()
parser.add_argument("--obj_name", type=str, default="car")
args, unknown = parser.parse_known_args()
obj_name = args.obj_name

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
_SRC_DIR = os.path.dirname(_SCRIPT_DIR)

sys.path.append(os.path.join(_SRC_DIR, "rmpflow"))
from m0609_rmpflow_controller import RMPFlowController

# ─────────────────────────────────────────────
# 로봇 설정 (path_generator.py의 ROBOT_BASE_POSITIONS와 동일해야 함)
# base_yaw: 로봇이 차량 방향을 향하도록 설정
#   왼쪽 로봇(X < 0): -π/2 → +X 방향 바라봄
#   오른쪽 로봇(X > 0): +π/2 → -X 방향 바라봄
# ─────────────────────────────────────────────
ROBOT_CONFIGS = [
    {"base_position": np.array([-1.0, -0.65, 0.5]), "base_yaw": -0.5 * np.pi},  # Robot 0: 왼쪽 앞
    {"base_position": np.array([-1.0, -0.95, 0.5]), "base_yaw": -0.5 * np.pi},  # Robot 1: 왼쪽 뒤
    {"base_position": np.array([ 1.0, -0.65, 0.5]), "base_yaw":  0.5 * np.pi},  # Robot 2: 오른쪽 앞
    {"base_position": np.array([ 1.0, -0.95, 0.5]), "base_yaw":  0.5 * np.pi},  # Robot 3: 오른쪽 뒤
]

ROBOT_USD_PATH = os.path.join(_SRC_DIR, "usd", "env", "Collected_m0609_with_polisher", "m0609_with_polisher.usd")

# 로봇별 시각화 색상 (점군 + 경로 포인터)
ROBOT_COLORS = [
    (1.0, 0.0, 0.0),  # Robot 0: 빨강
    (0.0, 0.3, 1.0),  # Robot 1: 파랑
    (0.0, 0.8, 0.0),  # Robot 2: 초록
    (0.8, 0.0, 0.8),  # Robot 3: 보라
]
VIZ_UPDATE_INTERVAL_STEPS = 5  # 경로 시각화 업데이트 주기

# ─────────────────────────────────────────────
# 물리/제어 상수 (polishing_v1.py와 동일)
# ─────────────────────────────────────────────
HOOD_Y_MAX = 9999.0
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
TARGET_NORMAL_FORCE = 1.5
CONTACT_FORCE_THRESHOLD = 0.5
CONTACT_GEOMETRY_MAX_OFFSET = 0.045
USE_VIRTUAL_SOFT_PAD = True
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
# 왼쪽 로봇(yaw=-π/2): joint_1=0 → 팔이 world -90° (차량 방향) ✓
# 오른쪽 로봇(yaw=+π/2): joint_1=0 → 팔이 world +90° (차량 반대!) → joint_1=π 필요
HOME_JOINT_POSITIONS_LEFT  = np.array([0.0,      -1.05, 1.45, 0.0, 1.15, 0.0])
HOME_JOINT_POSITIONS_RIGHT = np.array([np.pi,    -1.05, 1.45, 0.0, 1.15, 0.0])
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


# ─────────────────────────────────────────────
# 유틸리티 함수 (polishing_v1.py와 동일)
# ─────────────────────────────────────────────

def load_ply_points(path):
    points = []
    with open(path, "r") as f:
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


def z_align_quat(z_vec, fwd=None):
    """EE Z축을 z_vec에 정렬. fwd는 EE X축이 향할 참조 방향 (로봇 접근 방향)."""
    z_vec = z_vec / np.linalg.norm(z_vec)
    if fwd is None:
        fwd = np.array([1.0, 0.0, 0.0])
    fwd = np.array(fwd, dtype=float)
    if abs(np.dot(fwd, z_vec)) > 0.99:
        fwd = np.array([0.0, -1.0, 0.0])
    y_vec = np.cross(z_vec, fwd)
    y_vec = y_vec / np.linalg.norm(y_vec)
    x_vec = np.cross(y_vec, z_vec)
    x_vec = x_vec / np.linalg.norm(x_vec)
    R_mat = np.column_stack((x_vec, y_vec, z_vec))
    q = R.from_matrix(R_mat).as_quat()
    return np.array([q[3], q[0], q[1], q[2]])


def estimate_surface_normal(kdtree, surface_points, target_pos, k=NORMAL_QUERY_K):
    sample_count = min(k, len(surface_points))
    _, indices = kdtree.query(target_pos, k=sample_count)
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
    h_norm = np.linalg.norm(horizontal)
    if h_norm < 1e-6:
        return vertical
    limited = vertical * cos_limit + (horizontal / h_norm) * np.sin(np.radians(max_tilt_deg))
    return limited / np.linalg.norm(limited)


def filter_safe_waypoints(points, surface_points, kdtree, base_position):
    """base_position 기준으로 각 로봇이 실제로 도달 가능한 웨이포인트만 필터링."""
    if len(points) == 0:
        return points

    surface_min = np.min(surface_points, axis=0)
    surface_max = np.max(surface_points, axis=0)
    safe_points = []

    for pt in points:
        dist = np.linalg.norm(pt - base_position)
        if dist < PATH_MIN_RADIUS or dist > PATH_MAX_RADIUS:
            continue
        inside_x = surface_min[0] + PATH_EDGE_MARGIN <= pt[0] <= surface_max[0] - PATH_EDGE_MARGIN
        inside_y = surface_min[1] + PATH_EDGE_MARGIN <= pt[1] <= surface_max[1] - PATH_EDGE_MARGIN
        if not (inside_x and inside_y):
            continue
        _, tilt_deg = estimate_surface_normal(kdtree, surface_points, pt)
        if tilt_deg > MAX_SURFACE_TILT_DEG:
            continue
        safe_points.append(pt)

    if len(safe_points) == 0:
        return np.array([])

    # 최장 연속 구간 선택 (점 간격이 MAX_PATH_JUMP 초과 시 끊김)
    safe_points = np.array(safe_points)
    segments = []
    seg = [safe_points[0]]
    for i in range(1, len(safe_points)):
        if np.linalg.norm(safe_points[i] - safe_points[i - 1]) > MAX_PATH_JUMP:
            segments.append(seg)
            seg = []
        seg.append(safe_points[i])
    segments.append(seg)
    longest = max(segments, key=len)
    return np.array(longest)


def append_text_log(path, message):
    with open(path, "a") as f:
        f.write(message + "\n")


def build_polishing_disk_mesh(radius, height, side_count, visual_compression=0.0):
    """polishing_v1.py와 동일 — Y축이 실린더 높이축, XZ 평면이 원형 단면."""
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
        s_attr = getattr(physx_material, "CreateCompliantContactStiffnessAttr", None)
        d_attr = getattr(physx_material, "CreateCompliantContactDampingAttr", None)
        if s_attr:
            s_attr().Set(float(POLISHING_COMPLIANT_STIFFNESS))
        else:
            material_prim.CreateAttribute(
                "physxMaterial:compliantContactStiffness", Sdf.ValueTypeNames.Float
            ).Set(float(POLISHING_COMPLIANT_STIFFNESS))
        if d_attr:
            d_attr().Set(float(POLISHING_COMPLIANT_DAMPING))
        else:
            material_prim.CreateAttribute(
                "physxMaterial:compliantContactDamping", Sdf.ValueTypeNames.Float
            ).Set(float(POLISHING_COMPLIANT_DAMPING))
        return True
    except Exception as exc:
        print(f"[WARNING] compliant material 설정 실패: {exc}")
        return False


def create_polishing_contact_disk_for_robot(stage, robot_root_path, old_pad_path, physics_material):
    """로봇별 고유 prim 경로를 사용하여 폴리싱 디스크를 생성. (polishing_v1.py와 동일 로직)"""
    from pxr import UsdGeom, UsdPhysics, PhysxSchema, UsdShade, Sdf, Gf, Vt

    sander_pad_path = robot_root_path + "/sander_pad"
    disk_path = robot_root_path + "/polishing_contact_pad"
    visual_path = disk_path + "/sponge_visual"
    joint_path = sander_pad_path + "/polishing_contact_pad_joint"

    for stale in (disk_path, visual_path, joint_path):
        if stage.GetPrimAtPath(stale).IsValid():
            stage.RemovePrim(Sdf.Path(stale))

    pts, fc, fi, ext = build_polishing_disk_mesh(POLISHING_DISK_RADIUS, POLISHING_DISK_HEIGHT, POLISHING_DISK_SIDES)

    # 충돌 전용 디스크 (투명)
    disk = UsdGeom.Mesh.Define(stage, disk_path)
    disk.GetPointsAttr().Set(pts)
    disk.GetFaceVertexCountsAttr().Set(fc)
    disk.GetExtentAttr().Set(ext)
    disk.GetFaceVertexIndicesAttr().Set(fi)
    disk.CreateDisplayColorAttr([Gf.Vec3f(1.0, 1.0, 1.0)])
    disk.CreateDisplayOpacityAttr([0.0])  # 투명 — 시각은 sponge_visual이 담당

    # 시각 전용 스펀지 메시
    sponge = UsdGeom.Mesh.Define(stage, visual_path)
    sponge.GetPointsAttr().Set(pts)
    sponge.GetFaceVertexCountsAttr().Set(fc)
    sponge.GetFaceVertexIndicesAttr().Set(fi)
    sponge.GetExtentAttr().Set(ext)
    sponge.CreateDisplayColorAttr([Gf.Vec3f(1.0, 1.0, 1.0)])
    sponge.CreateDisplayOpacityAttr([1.0])

    # ── 디스크 위치 계산: sander_pad 좌표계 기준으로 배치 (v1과 동일) ──
    sander_pad_prim = stage.GetPrimAtPath(sander_pad_path)
    robot_root_prim = stage.GetPrimAtPath(robot_root_path)
    cache = UsdGeom.XformCache()
    if sander_pad_prim.IsValid() and robot_root_prim.IsValid():
        sander_pad_to_world = cache.GetLocalToWorldTransform(sander_pad_prim)
        robot_root_to_world = cache.GetLocalToWorldTransform(robot_root_prim)
        local_offset = Gf.Matrix4d(1.0)
        local_offset.SetTranslate(Gf.Vec3d(*[float(v) for v in POLISHING_DISK_LOCAL_POSITION]))
        disk_to_world = local_offset * sander_pad_to_world
        disk_to_robot_root = disk_to_world * robot_root_to_world.GetInverse()
    else:
        print(f"[WARNING] Robot {robot_root_path}: sander_pad not found, using fallback offset")
        disk_to_robot_root = Gf.Matrix4d(1.0)
        disk_to_robot_root.SetTranslate(Gf.Vec3d(*[float(v) for v in POLISHING_DISK_LOCAL_POSITION]))

    xform = UsdGeom.Xformable(disk.GetPrim())
    xform.ClearXformOpOrder()
    xform.AddTransformOp().Set(disk_to_robot_root)

    # 물리 속성
    disk_prim = stage.GetPrimAtPath(disk_path)
    UsdPhysics.RigidBodyAPI.Apply(disk_prim)
    mass_api = UsdPhysics.MassAPI.Apply(disk_prim)
    mass_api.CreateMassAttr().Set(0.05)
    UsdPhysics.CollisionAPI.Apply(disk_prim).CreateCollisionEnabledAttr().Set(True)
    UsdPhysics.MeshCollisionAPI.Apply(disk_prim).CreateApproximationAttr().Set("convexHull")
    try:
        physx_col = PhysxSchema.PhysxCollisionAPI.Apply(disk_prim)
        physx_col.CreateContactOffsetAttr().Set(0.006)
        physx_col.CreateRestOffsetAttr().Set(0.0)
    except Exception:
        pass
    if physics_material:
        UsdShade.MaterialBindingAPI.Apply(disk_prim).Bind(
            physics_material, UsdShade.Tokens.weakerThanDescendants, "physics"
        )

    # 기존 하드 패드 충돌 비활성화
    if sander_pad_prim.IsValid():
        set_collision_enabled_recursive(stage, sander_pad_path, False)

    # FixedJoint: sander_pad(부모) ↔ 폴리싱 디스크(자식)
    fixed_joint = UsdPhysics.FixedJoint.Define(stage, joint_path)
    fixed_joint.CreateBody0Rel().SetTargets([Sdf.Path(sander_pad_path)])
    fixed_joint.CreateBody1Rel().SetTargets([Sdf.Path(disk_path)])
    fixed_joint.CreateLocalPos0Attr().Set(Gf.Vec3f(*[float(v) for v in POLISHING_DISK_LOCAL_POSITION]))
    fixed_joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
    fixed_joint.CreateLocalRot0Attr().Set(Gf.Quatf(1.0, Gf.Vec3f(0.0, 0.0, 0.0)))
    fixed_joint.CreateLocalRot1Attr().Set(Gf.Quatf(1.0, Gf.Vec3f(0.0, 0.0, 0.0)))
    fixed_joint.CreateCollisionEnabledAttr().Set(False)

    print(f"[INFO] Robot {robot_root_path}: 폴리싱 디스크 생성 완료 → {disk_path}")
    return disk_path


# ─────────────────────────────────────────────
# RobotAgent: 로봇 1대의 상태 및 시뮬레이션 로직 캡슐화
# ─────────────────────────────────────────────

class RobotAgent:
    def __init__(self, idx, config, raw_points, kdtree, scan_dir):
        self.idx = idx
        self.base_position = config["base_position"]
        self.base_yaw = config["base_yaw"]
        self.base_orientation = np.array([
            np.cos(config["base_yaw"] * 0.5), 0.0, 0.0, np.sin(config["base_yaw"] * 0.5)
        ])
        self.world_prim_path = f"/World/M0609_{idx}"
        self.robot_root_path = f"/World/M0609_{idx}/m0609/m0609"
        self.link_6_path = f"/World/M0609_{idx}/m0609/m0609/link_6"
        # EE X축 기준 방향: 로봇에서 차량 중심(X=0) 방향
        # 왼쪽 로봇(X<0) → +X, 오른쪽 로봇(X>0) → -X
        dx = -float(config["base_position"][0])
        dy = -float(config["base_position"][1])
        mag = np.sqrt(dx * dx + dy * dy)
        self.approach_dir = np.array([dx / mag, dy / mag, 0.0]) if mag > 1e-6 else np.array([1.0, 0.0, 0.0])

        # 좌우 로봇별 홈 자세: 오른쪽 로봇은 joint_1=π (차량 방향으로 팔 정렬)
        if float(config["base_position"][0]) > 0:
            self.home_joints = HOME_JOINT_POSITIONS_RIGHT
        else:
            self.home_joints = HOME_JOINT_POSITIONS_LEFT
        self.raw_points = raw_points
        self.kdtree = kdtree

        # 경로 로드
        path_file = os.path.join(scan_dir, f"path_robot_{idx}.npy")
        if os.path.exists(path_file):
            raw_path = np.load(path_file)
            self.path = filter_safe_waypoints(raw_path, raw_points, kdtree, self.base_position)
            print(f"[Robot {idx}] {len(self.path)}개 웨이포인트 로드 ({path_file})")
        else:
            self.path = np.array([])
            print(f"[WARNING] Robot {idx}: 경로 파일 없음 ({path_file})")

        # 제어기 및 센서 (setup()에서 초기화)
        self.articulation: SingleArticulation = None
        self.controller: RMPFlowController = None
        self.contact_sensor: ContactSensor = None
        self.pad_path = None
        self.pad_visual_path = None
        self.pad_contact_offset_local = np.array([-0.0025, -0.0200, -0.0040])

        # 상태 머신
        self.run_state = STATE_HOME
        self.state_step_count = 0
        self.step_count = 0
        self.current_target_idx = 0
        self.current_path_idx_float = 0.0
        self.z_offset = PRESS_OFFSET_MAX
        self.z_vel = 0.0
        self.filtered_contact_force = 0.0
        self.previous_normal = None
        self.high_force_pause_steps = 0
        self.stable_contact_steps = 0
        self.path_complete_reported = False
        self.done = False

        # 시각화 USD prims
        self.future_path_prim = None
        self.completed_path_prim = None
        self.path_pointer_path = None

        # 로깅
        log_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)))
        self.log_file = os.path.join(log_dir, f"force_log_robot_{idx}.csv")
        with open(self.log_file, "w") as f:
            f.write("step,sensor_raw,virtual_force,raw_force,filtered,z_offset,path_idx,state\n")

    def setup(self, world, stage, physics_material_path):
        import omni.usd
        from pxr import UsdShade, UsdPhysics, PhysxSchema

        # 로봇 USD 로드
        create_prim(
            prim_path=self.world_prim_path,
            prim_type="Xform",
            position=self.base_position,
            orientation=self.base_orientation,
            usd_path=ROBOT_USD_PATH,
        )

        self.articulation = SingleArticulation(
            prim_path=self.robot_root_path,
            name=f"m0609_robot_{self.idx}",
        )

        # 폴리싱 디스크 생성
        material_prim = stage.GetPrimAtPath(physics_material_path)
        physics_material = UsdShade.Material(material_prim) if material_prim.IsValid() else None

        pad_candidates = [
            self.robot_root_path + "/sander_pad/pad_visual/sander_ref/OnRobot_Sander_v2/tn__104327_",
            self.robot_root_path + "/link_6/quick_mount/sanding_kit/OnRobot_Sander_v2/tn__104327_",
            self.robot_root_path + "/sander_pad",
        ]
        old_pad_path = pad_candidates[0]
        try:
            from omni.isaac.core.utils.prims import get_prim_at_path
            for c in pad_candidates:
                if get_prim_at_path(c):
                    old_pad_path = c
                    break
        except Exception:
            pass

        self.pad_path = create_polishing_contact_disk_for_robot(
            stage, self.robot_root_path, old_pad_path, physics_material
        )
        self.pad_visual_path = self.pad_path + "/sponge_visual"

        # ContactSensor
        contact_report_path = self.pad_path
        pad_prim = stage.GetPrimAtPath(contact_report_path)
        if pad_prim.IsValid():
            parts = contact_report_path.split("/")
            for i in range(2, len(parts) + 1):
                p = stage.GetPrimAtPath("/".join(parts[:i]))
                if p.IsValid() and p.IsInstance():
                    p.SetInstanceable(False)
            pad_prim = stage.GetPrimAtPath(contact_report_path)
            if pad_prim.IsValid():
                report_api = PhysxSchema.PhysxContactReportAPI.Apply(pad_prim)
                report_api.CreateThresholdAttr().Set(0.0)

        self.contact_sensor = ContactSensor(
            prim_path=contact_report_path + "/contact_sensor",
            name=f"pad_contact_sensor_{self.idx}",
            frequency=60,
            translation=np.array([0, 0, 0]),
        )

        # 단상(받침대) 시각화
        world.scene.add(
            VisualCylinder(
                prim_path=f"/World/Pedestal_{self.idx}",
                name=f"robot_pedestal_{self.idx}",
                position=self.base_position - np.array([0.0, 0.0, 0.25]),
                radius=0.12,
                height=0.5,
                color=np.array([0.3, 0.3, 0.3]),
            )
        )

        print(f"[Robot {self.idx}] setup 완료: {self.world_prim_path}")

    def initialize(self):
        self.articulation.initialize()
        self.contact_sensor.initialize()
        self.controller = RMPFlowController(
            name=f"polishing_controller_{self.idx}",
            robot_articulation=self.articulation,
            urdf_path=os.path.join(
                _SRC_DIR, "rmpflow", "m0609_isaac_sim.urdf"
            ),
            end_effector_frame_name="link_6",
        )
        self._apply_home_pose(teleport=True)
        self._apply_pad_spin_velocity(0.0)
        print(f"[Robot {self.idx}] 초기화 완료")

    def setup_visualization(self, stage):
        """포인트 클라우드 + 경로 선분 + 목표 구체를 USD 씬에 생성."""
        from pxr import UsdGeom, Vt, Gf

        color = ROBOT_COLORS[self.idx % len(ROBOT_COLORS)]

        # 각 로봇 담당 점군 (경로 waypoints를 색깔 점으로)
        if len(self.path) > 0:
            pts_prim = UsdGeom.Points.Define(stage, f"/World/PointCloud_{self.idx}")
            pts_prim.CreatePointsAttr(Vt.Vec3fArray([
                Gf.Vec3f(float(p[0]), float(p[1]), float(p[2])) for p in self.path
            ]))
            pts_prim.CreateWidthsAttr(Vt.FloatArray([0.002] * len(self.path)))
            pts_prim.CreateDisplayColorAttr(Vt.Vec3fArray([Gf.Vec3f(*color)]))

        # 미래 경로 선분 (초록)
        self.future_path_prim = UsdGeom.BasisCurves.Define(stage, f"/World/FuturePath_{self.idx}")
        self.future_path_prim.CreateTypeAttr().Set(UsdGeom.Tokens.linear)
        self.future_path_prim.CreateWidthsAttr().Set([0.004])
        self.future_path_prim.CreateDisplayColorAttr().Set([Gf.Vec3f(0.0, 1.0, 0.0)])

        # 완료 경로 선분 (노란색)
        self.completed_path_prim = UsdGeom.BasisCurves.Define(stage, f"/World/CompletedPath_{self.idx}")
        self.completed_path_prim.CreateTypeAttr().Set(UsdGeom.Tokens.linear)
        self.completed_path_prim.CreateWidthsAttr().Set([0.007])
        self.completed_path_prim.CreateDisplayColorAttr().Set([Gf.Vec3f(1.0, 0.82, 0.05)])

        # 현재 목표 구체 (로봇 색상)
        if len(self.path) > 0:
            sphere_path = f"/World/PathPointer_{self.idx}"
            sphere = UsdGeom.Sphere.Define(stage, sphere_path)
            sphere.CreateRadiusAttr(0.0225)
            sphere.CreateDisplayColorAttr(Vt.Vec3fArray([Gf.Vec3f(*color)]))
            xf = UsdGeom.XformCommonAPI(sphere.GetPrim())
            xf.SetTranslate(Gf.Vec3d(
                float(self.path[0][0]), float(self.path[0][1]), float(self.path[0][2])
            ))
            self.path_pointer_path = sphere_path

        print(f"[Robot {self.idx}] 시각화 초기화 완료 (color={color})")

    def _update_path_visualization(self, stage):
        """POLISH 단계에서 매 VIZ_UPDATE_INTERVAL_STEPS 스텝마다 호출."""
        if self.future_path_prim is None:
            return
        from pxr import Vt, Gf, UsdGeom

        points = self.path
        idx = self.current_target_idx

        # 앞으로 500개 waypoint → 미래 경로
        lookahead = points[idx: idx + 500]
        if len(lookahead) > 1:
            self.future_path_prim.GetPointsAttr().Set(Vt.Vec3fArray([
                Gf.Vec3f(float(p[0]), float(p[1]), float(p[2])) for p in lookahead
            ]))
            self.future_path_prim.GetCurveVertexCountsAttr().Set([len(lookahead)])

        # 지나온 경로 → 완료 경로 (Z +3mm 띄워 가시성 확보)
        done_count = min(max(idx + 1, 1), len(points))
        done_pts = points[:done_count]
        if len(done_pts) > 1:
            self.completed_path_prim.GetPointsAttr().Set(Vt.Vec3fArray([
                Gf.Vec3f(float(p[0]), float(p[1]), float(p[2]) + 0.003) for p in done_pts
            ]))
            self.completed_path_prim.GetCurveVertexCountsAttr().Set([len(done_pts)])

        # 현재 목표 구체 이동
        if self.path_pointer_path and idx < len(points):
            prim = stage.GetPrimAtPath(self.path_pointer_path)
            if prim.IsValid():
                UsdGeom.XformCommonAPI(prim).SetTranslate(Gf.Vec3d(
                    float(points[idx][0]), float(points[idx][1]), float(points[idx][2])
                ))

    def _apply_home_pose(self, teleport=False):
        dof_count = int(self.articulation.num_dof)
        arm_dof = min(len(self.home_joints), dof_count)
        if arm_dof <= 0:
            return
        indices = np.arange(arm_dof)
        positions = self.home_joints[:arm_dof]
        if teleport:
            try:
                self.articulation.set_joint_positions(positions, joint_indices=indices)
                if hasattr(self.articulation, "set_joint_velocities"):
                    self.articulation.set_joint_velocities(np.zeros(arm_dof), joint_indices=indices)
            except Exception:
                pass
        self.articulation.apply_action(
            ArticulationAction(joint_positions=positions, joint_indices=indices)
        )

    def _apply_pad_spin_velocity(self, velocity):
        try:
            if "revolution" in self.articulation.dof_names:
                pad_idx = self.articulation.get_dof_index("revolution")
            elif self.articulation.num_dof > 6:
                pad_idx = 6
            else:
                return False
            self.articulation.apply_action(
                ArticulationAction(
                    joint_velocities=np.array([velocity]),
                    joint_indices=np.array([pad_idx]),
                )
            )
            return abs(float(velocity)) > 1e-6
        except Exception:
            return False

    def _apply_cartesian_target(self, target_pos, normal, clearance):
        self.controller._motion_policy.set_robot_base_pose(
            robot_position=self.base_position,
            robot_orientation=self.base_orientation,
        )
        target_orientation = np.array(z_align_quat(-normal, self.approach_dir))
        target_rot = R.from_quat([
            target_orientation[1], target_orientation[2],
            target_orientation[3], target_orientation[0],
        ])
        target_pad_pos = target_pos + normal * clearance
        link_6_target = target_pad_pos - target_rot.apply(self.pad_contact_offset_local)
        actions = self.controller.forward(
            target_end_effector_position=link_6_target,
            target_end_effector_orientation=target_orientation,
        )
        self.articulation.apply_action(actions)
        return link_6_target

    def _set_run_state(self, new_state, message=""):
        self.run_state = new_state
        self.state_step_count = 0
        self.z_vel = 0.0
        self.filtered_contact_force = 0.0
        self.stable_contact_steps = 0
        self.high_force_pause_steps = 0
        if new_state in (STATE_APPROACH, STATE_RETRACT):
            self.z_offset = PRESS_OFFSET_MAX
        if message:
            print(f"[Robot {self.idx}] {message}", flush=True)

    def step(self, stage):
        """1 physics step 실행. 완료 시 self.done = True."""
        if self.done or len(self.path) == 0:
            if not self.done:
                self.done = True
            return

        points = self.path
        dt = 1.0 / 60.0

        # ── HOME ──
        if self.run_state == STATE_HOME:
            self._apply_home_pose()
            self._apply_pad_spin_velocity(0.0)
            self.step_count += 1
            self.state_step_count += 1
            if self.state_step_count >= HOME_INITIAL_SETTLE_STEPS:
                self._set_run_state(STATE_APPROACH, "Home pose 완료. 첫 웨이포인트로 이동 중.")
            return

        # ── APPROACH ──
        if self.run_state == STATE_APPROACH:
            normal, tilt_deg = estimate_surface_normal(self.kdtree, self.raw_points, points[0])
            normal = clamp_normal_tilt(normal)
            self.previous_normal = normal
            self._apply_cartesian_target(points[0], normal, SAFE_APPROACH_CLEARANCE)
            self._apply_pad_spin_velocity(0.0)
            self.step_count += 1
            self.state_step_count += 1
            if self.state_step_count >= APPROACH_SETTLE_STEPS:
                self.z_offset = PRESS_OFFSET_MAX
                self._set_run_state(STATE_POLISH, "Approach 완료. 폴리싱 시작.")
            return

        # ── 경로 완료 후 RETRACT / RETURN_HOME / DONE ──
        if self.current_target_idx >= len(points) and len(points) > 0:
            if self.run_state == STATE_POLISH and not self.path_complete_reported:
                self.path_complete_reported = True
                self._set_run_state(STATE_RETRACT, "경로 완료. 리트랙트 중.")

            if self.run_state == STATE_RETRACT:
                normal, _ = estimate_surface_normal(self.kdtree, self.raw_points, points[-1])
                normal = clamp_normal_tilt(normal)
                self._apply_cartesian_target(points[-1], normal, SAFE_RETRACT_CLEARANCE)
                self._apply_pad_spin_velocity(0.0)
                self.step_count += 1
                self.state_step_count += 1
                if self.state_step_count >= RETRACT_SETTLE_STEPS:
                    self._set_run_state(STATE_RETURN_HOME, "리트랙트 완료. 홈으로 복귀.")
                return

            if self.run_state == STATE_RETURN_HOME:
                self._apply_home_pose()
                self._apply_pad_spin_velocity(0.0)
                self.step_count += 1
                self.state_step_count += 1
                if self.state_step_count >= RETURN_HOME_SETTLE_STEPS:
                    self._set_run_state(STATE_DONE, "폴리싱 완료.")
                    self.done = True
                return

            if self.run_state == STATE_DONE:
                self._apply_home_pose()
                self._apply_pad_spin_velocity(0.0)
                self.done = True
                return
            return

        # ── POLISH ──
        if self.run_state == STATE_POLISH and self.current_target_idx < len(points):
            # 선형 보간으로 목표 위치 계산
            if (self.current_target_idx + 1 < len(points) and
                    self.current_path_idx_float > self.current_target_idx):
                progress = self.current_path_idx_float - self.current_target_idx
                target_pos = (points[self.current_target_idx] * (1.0 - progress) +
                              points[self.current_target_idx + 1] * progress)
            else:
                target_pos = points[self.current_target_idx]

            # 법선 추정 및 스무딩
            normal, normal_tilt_deg = estimate_surface_normal(self.kdtree, self.raw_points, target_pos)
            normal = clamp_normal_tilt(normal)
            if self.previous_normal is not None:
                if np.dot(normal, self.previous_normal) < 0:
                    normal = -normal
                normal = NORMAL_SMOOTHING * self.previous_normal + (1.0 - NORMAL_SMOOTHING) * normal
                normal = normal / np.linalg.norm(normal)
            self.previous_normal = normal

            # 목표 자세 (EE X축을 로봇 접근 방향으로 맞춰 오른쪽 로봇 팔 꼬임 방지)
            target_orientation = np.array(z_align_quat(-normal, self.approach_dir))

            # 접촉력 읽기
            contact_reading = self.contact_sensor.get_current_frame()
            sensor_raw_force = (
                np.linalg.norm(contact_reading["force"])
                if contact_reading and "force" in contact_reading else 0.0
            )
            is_physical_contacting = sensor_raw_force >= CONTACT_FORCE_THRESHOLD
            pad_compression = max(0.0, VIRTUAL_PAD_CONTACT_DISTANCE - self.z_offset)
            virtual_force = max(0.0, VIRTUAL_PAD_STIFFNESS * pad_compression - VIRTUAL_PAD_DAMPING * self.z_vel)

            if USE_VIRTUAL_SOFT_PAD:
                raw_force = virtual_force
                geometry_contact_possible = pad_compression > 0.0 or is_physical_contacting
            else:
                geometry_contact_possible = self.z_offset <= CONTACT_GEOMETRY_MAX_OFFSET
                raw_force = sensor_raw_force if geometry_contact_possible else 0.0

            is_contacting = raw_force >= CONTACT_FORCE_THRESHOLD or is_physical_contacting
            self.filtered_contact_force = (
                FORCE_FILTER_ALPHA * raw_force +
                (1.0 - FORCE_FILTER_ALPHA) * self.filtered_contact_force
            )

            # 경로 시각화 업데이트
            if self.step_count % VIZ_UPDATE_INTERVAL_STEPS == 0:
                self._update_path_visualization(stage)

            # 스펀지 시각 업데이트
            if self.step_count % SPONGE_VISUAL_UPDATE_INTERVAL_STEPS == 0 and self.pad_visual_path:
                visual_prim = stage.GetPrimAtPath(self.pad_visual_path)
                if visual_prim.IsValid():
                    visual_compression = float(np.clip(pad_compression, 0.0, POLISHING_VISUAL_MAX_COMPRESSION))
                    pts, _, _, ext = build_polishing_disk_mesh(
                        POLISHING_DISK_RADIUS, POLISHING_DISK_HEIGHT, POLISHING_DISK_SIDES,
                        visual_compression=visual_compression,
                    )
                    from pxr import UsdGeom
                    mesh = UsdGeom.Mesh(visual_prim)
                    mesh.GetPointsAttr().Set(pts)
                    mesh.GetExtentAttr().Set(ext)

            # 물리력 초과 대응
            if sensor_raw_force > PHYSICAL_FORCE_SOFT_LIMIT_N:
                overload = sensor_raw_force - PHYSICAL_FORCE_SOFT_LIMIT_N
                retract_step = min(PHYSICAL_FORCE_SOFT_RETRACT_MAX_STEP,
                                   PHYSICAL_FORCE_SOFT_RETRACT_GAIN * overload)
                self.z_offset = min(PRESS_OFFSET_MAX, self.z_offset + retract_step)
                self.z_vel = max(0.0, self.z_vel)

            if raw_force > FORCE_SPIKE_N or sensor_raw_force > PHYSICAL_FORCE_HARD_LIMIT_N:
                self.z_offset = min(PRESS_OFFSET_MAX, self.z_offset + SPIKE_RETRACT_STEP)
                self.z_vel = 0.0
                self.high_force_pause_steps = SPIKE_PAUSE_STEPS

            # 어드미턴스 제어
            control_force = min(self.filtered_contact_force, FORCE_CONTROL_CLIP_N)
            force_error = control_force - TARGET_NORMAL_FORCE
            accel = force_error / 1.0  # admittance_mass=1.0
            self.z_vel += accel * dt
            self.z_vel = np.clip(self.z_vel, -MAX_PRESS_VELOCITY, MAX_PRESS_VELOCITY)
            self.z_offset += self.z_vel * dt
            self.z_offset = np.clip(self.z_offset, PRESS_OFFSET_MIN, PRESS_OFFSET_MAX)

            # RMPFlow 목표 전송
            target_pad_pos = target_pos + normal * self.z_offset
            target_rot = R.from_quat([
                target_orientation[1], target_orientation[2],
                target_orientation[3], target_orientation[0],
            ])
            link_6_target = target_pad_pos - target_rot.apply(self.pad_contact_offset_local)
            self.controller._motion_policy.set_robot_base_pose(
                robot_position=self.base_position,
                robot_orientation=self.base_orientation,
            )
            actions = self.controller.forward(
                target_end_effector_position=link_6_target,
                target_end_effector_orientation=target_orientation,
            )
            self.articulation.apply_action(actions)
            self._apply_pad_spin_velocity(PAD_SPIN_VELOCITY)

            # 경로 진행
            prev_idx = self.current_path_idx_float
            if self.high_force_pause_steps > 0:
                self.high_force_pause_steps -= 1
                self.current_path_idx_float += PATH_CREEP_ADVANCE_PER_STEP
            else:
                stable = (
                    geometry_contact_possible and
                    (USE_VIRTUAL_SOFT_PAD and FORCE_ADVANCE_MIN_N <= self.filtered_contact_force <= FORCE_ADVANCE_MAX_N) or
                    (PHYSICAL_FORCE_ADVANCE_MIN_N <= sensor_raw_force <= PHYSICAL_FORCE_ADVANCE_MAX_N)
                )
                if stable:
                    self.stable_contact_steps += 1
                else:
                    self.stable_contact_steps = 0
                if self.stable_contact_steps >= CONTACT_SETTLE_STEPS:
                    self.current_path_idx_float += PATH_ADVANCE_PER_STEP
                else:
                    self.current_path_idx_float += PATH_CREEP_ADVANCE_PER_STEP

            self.current_target_idx = int(self.current_path_idx_float)
            self.step_count += 1
            self.state_step_count += 1

            # CSV 로그
            if self.step_count % STATUS_LOG_INTERVAL_STEPS == 0:
                with open(self.log_file, "a") as f:
                    f.write(
                        f"{self.step_count},{sensor_raw_force:.3f},{virtual_force:.3f},"
                        f"{raw_force:.3f},{self.filtered_contact_force:.3f},"
                        f"{self.z_offset:.4f},{self.current_path_idx_float:.1f},{self.run_state}\n"
                    )
                print(
                    f"[Robot {self.idx}] path={self.current_target_idx}/{len(points)} "
                    f"| sensor={sensor_raw_force:.1f}N | z={self.z_offset:.3f}m "
                    f"| state={self.run_state}",
                    flush=True,
                )


# ─────────────────────────────────────────────
# 메인
# ─────────────────────────────────────────────

def main():
    import omni.usd
    from pxr import UsdPhysics, PhysxSchema, UsdShade, Sdf

    scan_dir = os.path.join(_SRC_DIR, "scan_result", obj_name)
    ply_path = os.path.join(scan_dir, "points", "real_camera_surface_points.ply")

    # 공유 씬 초기화
    world = World(stage_units_in_meters=1.0)
    world.scene.add_default_ground_plane()
    world.scene.add(
        VisualCuboid(
            prim_path="/World/DarkFloor",
            name="dark_floor",
            position=np.array([0.0, 0.0, -0.01]),
            scale=np.array([10.0, 10.0, 0.02]),
            color=np.array([0.15, 0.15, 0.15]),
        )
    )
    create_prim("/World/DomeLight", "DomeLight",
                attributes={"inputs:intensity": 1000.0, "inputs:color": (1.0, 1.0, 1.0)})

    # 포인트 클라우드 로드 (공유)
    raw_points = load_ply_points(ply_path)
    raw_points = raw_points[raw_points[:, 1] < HOOD_Y_MAX]
    print(f"[main] 본네트 점 {len(raw_points)}개 로드 완료")
    kdtree = KDTree(raw_points)

    # 자동차 USD 로드
    create_prim("/World/Car", "Xform",
                position=np.array([0.0, 0.0, 0.0]),
                usd_path=os.path.join(_SRC_DIR, "scan_obj", f"{obj_name}.usd"))
    create_prim("/World/Room", "Xform",
                position=np.array([0.0, 0.0, 0.0]),
                usd_path=os.path.join(_SRC_DIR, "usd", "env", "room.usd"))

    stage = omni.usd.get_context().get_stage()

    # ── 전체 포인트 클라우드 시각화 ──
    from pxr import UsdGeom, Vt, Gf
    pts_prim = UsdGeom.Points.Define(stage, "/World/RawPoints")
    pts_prim.CreatePointsAttr(Vt.Vec3fArray([
        Gf.Vec3f(float(p[0]), float(p[1]), float(p[2])) for p in raw_points
    ]))
    pts_prim.CreateWidthsAttr(Vt.FloatArray([0.002] * len(raw_points)))
    pts_prim.CreateDisplayColorAttr(Vt.Vec3fArray([Gf.Vec3f(1.0, 0.0, 0.0)]))  # 원본 점군은 빨간색으로 표시


    # Physics 씬 안정화 (공유 1회)
    physics_scene = stage.GetPrimAtPath("/physicsScene")
    if physics_scene.IsValid():
        PhysxSchema.PhysxSceneAPI(physics_scene).GetPrim().CreateAttribute(
            "physxScene:maxDepenetrationVelocity", Sdf.ValueTypeNames.Float
        ).Set(0.05)

    # 공유 물리 재질 (NoBounceMaterial)
    from omni.isaac.core.materials import PhysicsMaterial
    PhysicsMaterial(
        prim_path="/World/NoBounceMaterial",
        dynamic_friction=POLISHING_DYNAMIC_FRICTION,
        static_friction=POLISHING_STATIC_FRICTION,
        restitution=POLISHING_RESTITUTION,
    )
    configure_compliant_material(stage, "/World/NoBounceMaterial")

    # 자동차 콜라이더 설정 (공유 1회)
    from pxr import Usd, UsdGeom
    target_prim = stage.GetPrimAtPath("/World/Car")
    if target_prim.IsValid():
        for prim in Usd.PrimRange(target_prim):
            if prim.IsA(UsdGeom.Mesh):
                UsdPhysics.CollisionAPI.Apply(prim)
                mc = UsdPhysics.MeshCollisionAPI.Apply(prim)
                mc.CreateApproximationAttr().Set("none")
                UsdShade.MaterialBindingAPI.Apply(prim).Bind(
                    UsdShade.Material(stage.GetPrimAtPath("/World/NoBounceMaterial")),
                    UsdShade.Tokens.weakerThanDescendants, "physics",
                )

    # 로봇 에이전트 생성 및 setup
    agents = []
    for i, cfg in enumerate(ROBOT_CONFIGS):
        agent = RobotAgent(i, cfg, raw_points, kdtree, scan_dir)
        agent.setup(world, stage, "/World/NoBounceMaterial")
        agent.setup_visualization(stage)
        agents.append(agent)

    # 물리 초기화
    world.reset()
    for agent in agents:
        agent.initialize()

    import omni.timeline
    omni.timeline.get_timeline_interface().play()
    print("[main] 시뮬레이션 시작 (4대 로봇 동시 실행)")

    while simulation_app.is_running():
        world.step(render=True)
        if not world.is_playing():
            continue

        stage = omni.usd.get_context().get_stage()
        for agent in agents:
            agent.step(stage)

        if all(a.done for a in agents):
            print("[main] 모든 로봇 폴리싱 완료.")
            break

    simulation_app.close()


if __name__ == "__main__":
    try:
        main()
    except Exception:
        import traceback
        traceback.print_exc()
    finally:
        simulation_app.close()
