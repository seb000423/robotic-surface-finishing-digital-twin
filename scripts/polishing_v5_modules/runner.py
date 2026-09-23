"""Scene setup and main loop for polishing_v5."""
import json
import os
import time
import sys

import numpy as np
from scipy.spatial import KDTree

from isaacsim.core.api import World
from isaacsim.core.api.objects import VisualCuboid
from isaacsim.core.utils.prims import create_prim

from . import common
from .common import *
from .common import _SCRIPT_DIR, _SRC_DIR
from .agent import RailRobotAgent
from .visualization import CoverageMap, PolishViz


def _smoothstep01(value):
    q = float(np.clip(value, 0.0, 1.0))
    return q * q * (3.0 - 2.0 * q)


def _create_entry_scanner(stage, root_path="/World/EntryScanner"):
    """Create a visual-only half-donut scanner that rises from the floor before parking."""
    from pxr import Gf, Sdf, UsdGeom, UsdShade, Vt

    if stage.GetPrimAtPath(root_path).IsValid():
        stage.RemovePrim(Sdf.Path(root_path))

    root = UsdGeom.Xform.Define(stage, root_path)
    hidden_z = -ENTRY_SCANNER_RADIUS - ENTRY_SCANNER_TUBE_RADIUS - 0.10
    UsdGeom.XformCommonAPI(root.GetPrim()).SetTranslate(Gf.Vec3d(0.0, CAR_ENTRY_SCAN_Y, hidden_z))

    looks_path = Sdf.Path(root_path).AppendChild("Looks")
    UsdGeom.Scope.Define(stage, looks_path)

    def _material(name, color, opacity, emissive=0.0):
        mat_path = looks_path.AppendChild(name)
        mat = UsdShade.Material.Define(stage, mat_path)
        shader = UsdShade.Shader.Define(stage, mat_path.AppendChild("Surface"))
        shader.CreateIdAttr("UsdPreviewSurface")
        cr, cg, cb = float(color[0]), float(color[1]), float(color[2])
        c = Gf.Vec3f(cr, cg, cb)
        shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(c)
        shader.CreateInput("emissiveColor", Sdf.ValueTypeNames.Color3f).Set(
            Gf.Vec3f(cr * emissive, cg * emissive, cb * emissive)
        )
        shader.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(float(opacity))
        shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.28)
        mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
        return mat

    ring_mat = _material("ScannerRingCyan", (0.08, 0.85, 1.0), 0.82, emissive=0.45)
    sheet_mat = _material("ScanSheetCyan", (0.0, 0.75, 1.0), 0.22, emissive=0.65)

    arc_steps = 64
    tube_steps = 12
    radius = float(ENTRY_SCANNER_RADIUS)
    tube_r = float(ENTRY_SCANNER_TUBE_RADIUS)
    points = []
    for i in range(arc_steps + 1):
        theta = np.pi * i / float(arc_steps)
        radial = np.array([np.cos(theta), 0.0, np.sin(theta)], dtype=float)
        center = radial * radius
        for j in range(tube_steps):
            phi = 2.0 * np.pi * j / float(tube_steps)
            p = center + tube_r * (np.cos(phi) * radial + np.sin(phi) * np.array([0.0, 1.0, 0.0]))
            points.append(Gf.Vec3f(float(p[0]), float(p[1]), float(p[2])))

    counts = []
    indices = []
    for i in range(arc_steps):
        for j in range(tube_steps):
            a = i * tube_steps + j
            b = i * tube_steps + ((j + 1) % tube_steps)
            c = (i + 1) * tube_steps + ((j + 1) % tube_steps)
            d = (i + 1) * tube_steps + j
            counts.append(4)
            indices.extend([a, b, c, d])

    ring = UsdGeom.Mesh.Define(stage, f"{root_path}/HalfDonutRing")
    ring.CreatePointsAttr(Vt.Vec3fArray(points))
    ring.CreateFaceVertexCountsAttr(Vt.IntArray(counts))
    ring.CreateFaceVertexIndicesAttr(Vt.IntArray(indices))
    ring.CreateExtentAttr(Vt.Vec3fArray([
        Gf.Vec3f(-radius - tube_r, -tube_r, -tube_r),
        Gf.Vec3f(radius + tube_r, tube_r, radius + tube_r),
    ]))
    ring.CreateDisplayColorAttr(Vt.Vec3fArray([Gf.Vec3f(0.08, 0.85, 1.0)]))
    ring.CreateDisplayOpacityAttr(Vt.FloatArray([0.82]))
    ring.CreateDoubleSidedAttr(True)
    UsdShade.MaterialBindingAPI(ring.GetPrim()).Bind(ring_mat)

    sheet_radius = radius
    sheet_points = [Gf.Vec3f(0.0, 0.0, 0.0)]
    for i in range(arc_steps + 1):
        theta = np.pi * i / float(arc_steps)
        sheet_points.append(Gf.Vec3f(
            float(np.cos(theta) * sheet_radius),
            0.0,
            float(np.sin(theta) * sheet_radius),
        ))
    sheet_counts = []
    sheet_indices = []
    for i in range(1, arc_steps + 1):
        sheet_counts.append(3)
        sheet_indices.extend([0, i, i + 1])

    sheet = UsdGeom.Mesh.Define(stage, f"{root_path}/ScanSheet")
    sheet.CreatePointsAttr(Vt.Vec3fArray(sheet_points))
    sheet.CreateFaceVertexCountsAttr(Vt.IntArray(sheet_counts))
    sheet.CreateFaceVertexIndicesAttr(Vt.IntArray(sheet_indices))
    sheet.CreateExtentAttr(Vt.Vec3fArray([
        Gf.Vec3f(-sheet_radius, -0.002, 0.0),
        Gf.Vec3f(sheet_radius, 0.002, sheet_radius),
    ]))
    sheet.CreateDisplayColorAttr(Vt.Vec3fArray([Gf.Vec3f(0.0, 0.75, 1.0)]))
    sheet.CreateDisplayOpacityAttr(Vt.FloatArray([0.22]))
    sheet.CreateDoubleSidedAttr(True)
    UsdShade.MaterialBindingAPI(sheet.GetPrim()).Bind(sheet_mat)

    print(f"[main] 건물 진입 전 반도넛 스캐너 생성: Y={CAR_ENTRY_SCAN_Y:.2f}", flush=True)
    return {"root": root_path, "hidden_z": hidden_z}


def _set_entry_scanner_pose(xform_cls, scanner, y_pos, z_pos):
    if not scanner:
        return
    xform_cls(prim_path=scanner["root"]).set_world_pose(
        position=np.array([0.0, float(y_pos), float(z_pos)])
    )


def main(simulation_app, obj_name="car"):
    import omni.usd
    from pxr import UsdPhysics, PhysxSchema, UsdShade, Sdf
    import datetime

    scan_dir = os.path.join(_SRC_DIR, "scan_result", obj_name)
    ply_path = os.path.join(scan_dir, "points", "real_camera_surface_points.ply")

    # 실행마다 status_log.txt 초기화
    status_log_path = os.path.join(_SCRIPT_DIR, "status_log.txt")
    with open(status_log_path, "w") as f:
        f.write(f"=== polishing_v5.py 실행 시작: {datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')} ===\n")

    # 공유 씬 초기화
    world = World(stage_units_in_meters=1.0)
    world.scene.add_default_ground_plane()
    world.scene.add(
        VisualCuboid(
            prim_path="/World/DarkFloor",
            name="dark_floor",
            position=np.array([0.0, 0.0, -0.01]),
            scale=np.array([10.9, 11.8, 0.02]),
            color=np.array([0.15, 0.15, 0.15]),
        )
    )
    create_prim("/World/DomeLight", "DomeLight",
                attributes={"inputs:intensity": 1000.0, "inputs:color": (1.0, 1.0, 1.0)})

    # 포인트 클라우드 로드 (공유) — 리프트 좌표계로 z 상승
    all_scan_points = load_ply_points(ply_path).astype(float)
    all_scan_points[:, 2] += CAR_LIFT_Z
    if HOOD_Y_MAX < 999.0:
        raw_points = all_scan_points[all_scan_points[:, 1] < HOOD_Y_MAX]
    else:
        raw_points = all_scan_points   # 차량 전체 사용
    _smin = np.min(raw_points, axis=0)
    _smax = np.max(raw_points, axis=0)
    print(f"[main] 스캔 점: {len(all_scan_points)}개 → 제어 점: {len(raw_points)}개")
    print(f"[main] 차량 좌표 X:[{_smin[0]:.3f},{_smax[0]:.3f}] "
          f"Y:[{_smin[1]:.3f},{_smax[1]:.3f}] Z:[{_smin[2]:.3f},{_smax[2]:.3f}]")
    kdtree = KDTree(raw_points)

    # 자동차 USD 로드 (리프트 높이에 배치 — 시작 시 바닥에서 올라옴)
    create_prim("/World/Car", "Xform",
                position=np.array([0.0, 0.0, CAR_LIFT_Z]),
                usd_path=os.path.join(_SRC_DIR, "scan_obj", f"{obj_name}.usd"))
    create_prim("/World/Room", "Xform",
                position=np.array([0.0, 0.0, 0.0]),
                usd_path=os.path.join(_SRC_DIR, "usd", "env", "room.usd"))

    stage = omni.usd.get_context().get_stage()

    # 룸 크게 (엄청) + 바닥도 크게
    from pxr import UsdGeom as _UG, Gf as _GfR
    _room = stage.GetPrimAtPath("/World/Room")
    if _room.IsValid():
        _UG.XformCommonAPI(_room).SetScale(_GfR.Vec3f(7.0, 7.0, 3.5))   # 룸 확대(멀리 찍는 카메라 수용)
    _df = stage.GetPrimAtPath("/World/DarkFloor")
    if _df.IsValid():
        _UG.XformCommonAPI(_df).SetScale(_GfR.Vec3f(7.0, 7.0, 1.0))     # 바닥도 함께 확대

    # 창문/창틀 + 바닥 몰딩(floor_rim) 제거 (room.usd의 참조 prim이라 비활성화로 숨김)
    _n_hidden = 0
    for _p in stage.Traverse():
        _nm = _p.GetName().lower()
        if "window" in _nm or "floor_rim" in _nm:
            _p.SetActive(False)
            _n_hidden += 1
    if _n_hidden:
        print(f"[main] 창문/창틀/바닥몰딩 prim {_n_hidden}개 비활성화(숨김)")

    entry_scanner = _create_entry_scanner(stage) if CAR_ENTRY_SCANNER_ENABLED else None

    # 정비소 4바퀴 리프트 단상: 차 밑 바퀴 위치에 다리(원통) 4개 + 베이스 슬래브 (차와 함께 상승)
    # 바퀴보다 안쪽으로 좁게 (차량 바깥으로 안 삐져나오게)
    _sx, _sy = float(_smax[0]) - 0.30, abs(float(_smax[1])) - 0.62   # 바퀴 근사 X/Y 오프셋(좁게)
    wheel_xy = [(-_sx, -_sy), (-_sx, _sy), (_sx, -_sy), (_sx, _sy)]
    # 리프트 애니메이션 대상: (prim_path, x, y, z_full)  z(p)=z_full - CAR_LIFT_Z*(1-p)
    lift_anim = [("/World/Car", 0.0, 0.0, CAR_LIFT_Z),
                 ("/World/WheelLift_base", 0.0, 0.0, CAR_LIFT_Z - 0.06)]
    world.scene.add(VisualCuboid(
        prim_path="/World/WheelLift_base", name="wheel_lift_base",
        position=np.array([0.0, 0.0, CAR_LIFT_Z - 0.06]),
        scale=np.array([2.0 * _sx + 0.22, 2.0 * _sy + 0.30, 0.10]),   # 좁게
        color=np.array([0.30, 0.32, 0.36]),
    ))
    # 다리 기둥 4개를 Vention 텔레스코픽 리프트 USD(시각 프롭)로 교체.
    # 에셋 원점=베이스라 z_full=0(완전 상승 시 베이스가 바닥에 닿음). 차와 함께 통짜 상승.
    for _i, (_wx, _wy) in enumerate(wheel_xy):
        load_tele_lift_prop(
            stage, f"/World/WheelLift_pad_{_i}", f"tele_lift_wheel_{_i}.usd",
            (_wx, _wy, 0.0),
            extra_scale=WHEEL_TELE_LIFT_SCALE, freeze_ext=WHEEL_TELE_LIFT_EXT)
        lift_anim.append((f"/World/WheelLift_pad_{_i}", _wx, _wy, 0.0))

    # 스캔 포인트 클라우드 시각화 — 전체 차량, 최대 80,000점 서브샘플, 빨간색
    from pxr import UsdGeom, Vt, Gf as _Gf
    _MAX_DISPLAY_PTS = 80000
    if len(all_scan_points) > _MAX_DISPLAY_PTS:
        idx = np.linspace(0, len(all_scan_points) - 1, _MAX_DISPLAY_PTS, dtype=int)
        display_pts = all_scan_points[idx]
    else:
        display_pts = all_scan_points
    scan_cloud = UsdGeom.Points.Define(stage, "/World/ScanPointCloud")
    scan_cloud.CreatePointsAttr(Vt.Vec3fArray([
        _Gf.Vec3f(float(p[0]), float(p[1]), float(p[2])) for p in display_pts
    ]))
    scan_cloud.CreateWidthsAttr(Vt.FloatArray([0.003] * len(display_pts)))
    # 점마다 색상(vertex 보간) — 폴리싱되면 해당 점만 빨강→하늘색
    color_primvar = scan_cloud.CreateDisplayColorPrimvar(UsdGeom.Tokens.vertex)
    color_primvar.Set(Vt.Vec3fArray([_Gf.Vec3f(1.0, 0.0, 0.0)] * len(display_pts)))
    UsdGeom.Imageable(scan_cloud.GetPrim()).MakeInvisible()
    print(f"[main] 포인트 클라우드 준비: {len(display_pts)}점 (리프트 완료 후 표시)")

    # 폴리싱 진행 가시화: 패드가 지나간 점만 하늘색으로 (마킹 반경=닦는 면 크기)
    polish_viz = PolishViz(stage, "/World/ScanPointCloud", display_pts, POLISH_MARK_RADIUS)

    # 스캔 점군도 리프트 애니메이션 대상에 추가 (z_full=0: 기하가 이미 리프트 좌표)
    lift_anim.append(("/World/ScanPointCloud", 0.0, 0.0, 0.0))

    # Physics 씬 안정화 (공유 1회)
    physics_scene = stage.GetPrimAtPath("/physicsScene")
    if physics_scene.IsValid():
        PhysxSchema.PhysxSceneAPI(physics_scene).GetPrim().CreateAttribute(
            "physxScene:maxDepenetrationVelocity", Sdf.ValueTypeNames.Float
        ).Set(0.05)

    # 공유 물리 재질 (NoBounceMaterial)
    from isaacsim.core.api.materials import PhysicsMaterial
    PhysicsMaterial(
        prim_path="/World/NoBounceMaterial",
        dynamic_friction=POLISHING_DYNAMIC_FRICTION,
        static_friction=POLISHING_STATIC_FRICTION,
        restitution=POLISHING_RESTITUTION,
    )
    configure_compliant_material(stage, "/World/NoBounceMaterial")
    # ── 진단: compliant 속성이 실제로 기록됐는지 읽어서 출력 ──
    _cm = stage.GetPrimAtPath("/World/NoBounceMaterial")
    _cs = _cm.GetAttribute("physxMaterial:compliantContactStiffness")
    _cd = _cm.GetAttribute("physxMaterial:compliantContactDamping")
    print(f"[COMPLIANT-DIAG] stiffness_attr_valid={_cs.IsValid()} val={_cs.Get() if _cs.IsValid() else None} "
          f"damping_attr_valid={_cd.IsValid()} val={_cd.Get() if _cd.IsValid() else None}", flush=True)

    # 자동차 콜라이더 설정 (공유 1회)
    from pxr import Usd, UsdGeom
    _remove_names = {"bmw_z4_car_007_color_polySurface371"}
    target_prim = stage.GetPrimAtPath("/World/Car")
    if target_prim.IsValid():
        for prim in Usd.PrimRange(target_prim):
            if prim.GetName() in _remove_names:
                UsdGeom.Imageable(prim).MakeInvisible()
                UsdPhysics.CollisionAPI.Apply(prim).CreateCollisionEnabledAttr().Set(False)
                prim.SetActive(False)
                print(f"[main] 차량 메시 제거: {prim.GetPath()}")
        for prim in Usd.PrimRange(target_prim):
            if prim.IsValid() and prim.IsA(UsdGeom.Mesh):
                UsdPhysics.CollisionAPI.Apply(prim)
                mc = UsdPhysics.MeshCollisionAPI.Apply(prim)
                # "none"(정확 삼각망)은 compliant 접촉 미지원 → rigid 슬램(차체에선 1000N+ 검증됨).
                # convexDecomposition이 그나마 슬램이 작아 유지. (옛 v1이 none에서 OK였던 건 작은 마우스라서)
                # 실접촉 모드 기본 "none"(정확 삼각망): convexDecomposition 껍데기는 점군보다 바깥에
                # 부풀어 패드가 표면 위 수 cm 에서 먼저 닿는다(힘 게이트 탈락·슬램). 환경변수로 재정의.
                mc.CreateApproximationAttr().Set(os.environ.get(
                    "POLISH_CAR_COLLIDER", "none" if USE_PHYSICAL_CONTACT_SENSOR else "convexDecomposition"))
                _car_mat = "/World/NoBounceMaterial"
                if USE_PHYSICAL_CONTACT_SENSOR:
                    # 실접촉: 차체 쪽에도 순응 접촉 재질 (양쪽 순응이어야 PhysX compliant contact 유효)
                    _cm = UsdShade.Material.Define(stage, "/World/PhysicsMaterials/car_compliant")
                    _um = UsdPhysics.MaterialAPI.Apply(_cm.GetPrim())
                    _um.CreateStaticFrictionAttr().Set(0.0); _um.CreateDynamicFrictionAttr().Set(0.0)
                    _um.CreateRestitutionAttr().Set(0.0)
                    _pm = PhysxSchema.PhysxMaterialAPI.Apply(_cm.GetPrim())
                    _pm.CreateCompliantContactStiffnessAttr().Set(float(PAD_COMPLIANT_STIFFNESS_N_M))
                    _pm.CreateCompliantContactDampingAttr().Set(float(PAD_COMPLIANT_DAMPING_N_S_M))
                    _car_mat = "/World/PhysicsMaterials/car_compliant"
                UsdShade.MaterialBindingAPI.Apply(prim).Bind(
                    UsdShade.Material(stage.GetPrimAtPath(_car_mat)),
                    UsdShade.Tokens.weakerThanDescendants, "physics",
                )

    # rail_config.json 로드
    import json
    config_path = os.path.join(scan_dir, "rail_config.json")
    if not os.path.exists(config_path):
        print(f"[ERROR] rail_config.json 없음: {config_path}")
        print("[ERROR] 먼저 path_generator.py --mode rail 을 실행하세요.")
        return
    with open(config_path) as f:
        rail_config_data = json.load(f)

    # path_generator.py와 반경값 동기화 (rail_config.json 우선)
    first_cfg = next(iter(rail_config_data.values()))
    common.PATH_MIN_RADIUS = float(first_cfg.get("min_radius", common.PATH_MIN_RADIUS))
    common.PATH_MAX_RADIUS = float(first_cfg.get("max_radius", common.PATH_MAX_RADIUS))
    print(f"[main] 팔 도달 반경: {common.PATH_MIN_RADIUS:.2f}m ~ {common.PATH_MAX_RADIUS:.2f}m")

    # RAIL_CONFIGS 동적 구성 (L=왼쪽, R=오른쪽)
    RAIL_CONFIGS = [
        {
            "label":      label,
            "rail_x":    float(cfg["rail_x"]),
            "base_yaw":  float(cfg["base_yaw"]),
            "yz_stops":  cfg["yz_stops"],
            "mount_mode": cfg.get("mount_mode"),
            "outward_sign": cfg.get("outward_sign", -1),   # 측면 바깥방향 — 빠뜨리면 SR이 -1로 떨어져 관통/반대됨
        }
        for label, cfg in rail_config_data.items()
    ]
    # 웹 UI 대수 반영: POLISH_ROBOTS="C,SL,SR" 처럼 라벨을 주면 그 로봇만 만든다 (3대 = 천장 C + 측면 SL/SR)
    _want = [x.strip() for x in os.environ.get("POLISH_ROBOTS", "").split(",") if x.strip()]
    if _want:
        _have = [c["label"] for c in RAIL_CONFIGS]
        RAIL_CONFIGS = [c for c in RAIL_CONFIGS if c["label"] in _want]
        print(f"[main] POLISH_ROBOTS={_want} → 활성 {[c['label'] for c in RAIL_CONFIGS]} (구성 {_have})", flush=True)
        if not RAIL_CONFIGS:
            print("[main] POLISH_ROBOTS 에 맞는 로봇이 없어 전체를 사용합니다", flush=True)
            RAIL_CONFIGS = [{"label": label, "rail_x": float(cfg["rail_x"]), "base_yaw": float(cfg["base_yaw"]), "yz_stops": cfg["yz_stops"],
                             "mount_mode": cfg.get("mount_mode"), "outward_sign": cfg.get("outward_sign", -1)} for label, cfg in rail_config_data.items()]
    is_overhead_mode = any(c.get("mount_mode") == "overhead" for c in RAIL_CONFIGS)

    # 시각적 레일 — 레일 모드에서만 (오버헤드는 갠트리가 대체)
    if not is_overhead_mode:
        from pxr import UsdGeom, Gf, Vt
        all_y_stops   = [yz[0] for cfg in RAIL_CONFIGS for yz in cfg["yz_stops"]]
        rail_center_y = (max(all_y_stops) + min(all_y_stops)) / 2.0
        rail_length   = abs(max(all_y_stops) - min(all_y_stops)) + 0.30
        rail_x_vis    = RAIL_CONFIGS[0]["rail_x"]          # 왼쪽 X 좌표
        rail_z_vis    = 0.1
        rail_prim = UsdGeom.Cube.Define(stage, "/World/VisualRail_Left")
        rail_prim.CreateDisplayColorAttr(Vt.Vec3fArray([Gf.Vec3f(0.25, 0.25, 0.25)]))
        xf = UsdGeom.XformCommonAPI(rail_prim.GetPrim())
        xf.SetTranslate(Gf.Vec3d(rail_x_vis, rail_center_y, rail_z_vis))
        xf.SetScale(Gf.Vec3f(0.05, rail_length, 0.05))
        print(f"[main] 레일 시각화: Y 중심={rail_center_y:.2f}, 길이={rail_length:.2f}m")

    # 공유 커버리지맵 (두 로봇이 동일 인스턴스 참조)
    shared_coverage_map = CoverageMap(voxel_size=0.02)
    print("[main] 커버리지맵 초기화 완료 (복셀 크기: 2cm)")

    # 레일 로봇 에이전트 생성 및 setup
    agents = []
    for i, cfg in enumerate(RAIL_CONFIGS):
        agent = RailRobotAgent(i, cfg, raw_points, kdtree, scan_dir,
                               coverage_map=shared_coverage_map, polish_viz=polish_viz)
        agent.setup(world, stage, "/World/NoBounceMaterial")
        agent.setup_visualization(stage)
        agents.append(agent)
        print(f"[main] Robot {cfg['label']}: 베이스={[round(v,3) for v in agent.base_position]}, "
              f"YZ정지={[[round(y,2), round(z,2)] for y, z in agent.yz_stops]}")

    # 잔차 정책 + 차 전체 셀 격자 (POLISH_RL=1). v5 시뮬레이션 동작은 기본 그대로.
    rl_registry = None
    def _rl_flush(tag=""):
        if rl_registry is not None:
            from .rl_bridge import write_judgement
            out = os.environ.get("POLISH_RL_OUT", os.path.join(_SRC_DIR, "learning", "rl", "logs",
                                                                "v5_rl", "car_cells_judgement.csv"))
            summary = write_judgement(rl_registry, out)
            # 웹 UI 결과 요약 (learning/ui_bridge/out/last_run.json) — /api/sim/status 가 읽는다
            try:
                import json as _json, time as _time
                from .rl_bridge import judge_cells
                rows = judge_cells(rl_registry)
                done = [r for r in rows if r["visits"] > 0]
                gus = sorted(float(r["gu_proxy_after"]) for r in done)
                def _mean(xs): return float(sum(xs) / len(xs)) if xs else 0.0
                def _pct(xs, p):
                    if not xs: return 0.0
                    k = max(0, min(len(xs) - 1, int(round(p / 100.0 * (len(xs) - 1))))); return float(xs[k])
                sd = (sum((g - _mean(gus)) ** 2 for g in gus) / len(gus)) ** 0.5 if gus else 0.0
                robots_f = {a.label: float(a.filtered_contact_force) for a in agents}
                rec = {
                    "ts": _time.time(), "tag": tag, "sim_step": sim_step, "elapsed_s": sim_step / 60.0,
                    "recipe": {"force": float(rl_bridge.recipe_top.target_contact_force_n),
                               "feed_mm_s": float(rl_bridge.recipe_top.feed_speed_mm_s),
                               "rpm": float(rl_bridge.recipe_top.rpm),
                               "spacing_ratio": float(rl_bridge.recipe_top.step_over_spacing_ratio),
                               "n_passes": int(rl_bridge.recipe_top.n_passes)},
                    "cells": summary,
                    "quality": {
                        "cells": len(done), "ra": _mean([float(r["ra_after_um"]) for r in done]),
                        "rz": max([float(r["rz_after_um"]) for r in done], default=0.0),
                        "clearcoat": min([float(r["clearcoat_remaining_min_um"]) for r in done], default=0.0),
                        "scratch": _mean([float(r["scratch_after_um"]) for r in done]),
                        "scratchBefore": _mean([float(r["scratch_before_um"]) for r in done]),
                        "glossMean": _mean(gus), "glossP10": _pct(gus, 10), "glossStd": sd,
                        "glossMin": (gus[0] if gus else 0.0), "glossTiles": len(gus),
                        "glossPass": bool(gus and _mean(gus) >= 70.0 and _pct(gus, 10) >= 60.0 and sd <= 10.0 and gus[0] >= 45.0),
                        "glossBand": ("target_pass" if gus and _mean(gus) >= 70 else "partial" if gus and _mean(gus) >= 60 else "low"),
                    },
                    "rl": {"force": _mean([float(r["force_n"]) for r in done]),
                           "force_scale_mean": _mean([1.0 + float(r["policy_action_force"]) * 0.3 for r in done]),
                           "feed_scale_mean": _mean([1.0 + float(r["policy_action_feed"]) * 0.5 for r in done]),
                           "stiffness": 350.0 if os.environ.get("POLISH_PHYSICAL_CONTACT", "0") != "1" else 2000.0,
                           "damping": 35.0 if os.environ.get("POLISH_PHYSICAL_CONTACT", "0") != "1" else 200.0,
                           "robots": robots_f},
                }
                _out_dir = os.path.join(_SRC_DIR, "learning", "ui_bridge", "out")
                os.makedirs(_out_dir, exist_ok=True)
                _tmp = os.path.join(_out_dir, "last_run.json.tmp")
                with open(_tmp, "w", encoding="utf-8") as fh:
                    _json.dump(rec, fh, ensure_ascii=False, indent=1)
                os.replace(_tmp, os.path.join(_out_dir, "last_run.json"))
                if recorder is not None:
                    try: recorder.finish(_res)
                    except Exception as _exc: print(f"[main] 기록 종료 실패: {_exc}", flush=True)
            except Exception as exc:
                print(f"[main] last_run.json 기록 실패: {exc}")
    if os.environ.get("POLISH_RL", "0") == "1":
        from .rl_bridge import CellRegistry, ResidualPolicyBridge
        _ckpt = os.environ.get("POLISH_RL_CKPT") or None
        _rt = os.environ.get("POLISH_RL_RECIPE_TOP") or None
        _rs = os.environ.get("POLISH_RL_RECIPE_SIDE") or None
        rl_registry = CellRegistry(raw_points, profile=os.environ.get("POLISH_RL_PROFILE", "new_car"))
        _kw = {}
        if _ckpt: _kw["ckpt"] = _ckpt
        if _rt: _kw["recipe_json"] = _rt
        if _rs: _kw["recipe_json_side"] = _rs
        rl_bridge = ResidualPolicyBridge(rl_registry, **_kw)
        for agent in agents:
            agent.rl_bridge = rl_bridge
        print(f"[main] POLISH_RL=1 — 잔차 정책 브리지 연결 ({len(agents)}대), 셀 {len(rl_registry.cells)}개")

    # 모니터 피드 (POLISH_MONITOR_FEED=<json 경로>): 20 스텝마다 로봇 힘·상태·진행·셀 판정 기록
    monitor_feed = None
    _feed_path = os.environ.get("POLISH_MONITOR_FEED", "")
    if _feed_path:
        from learning.ui_bridge.monitor_feed import MonitorFeed
        monitor_feed = MonitorFeed(_feed_path)
        monitor_feed.event("C", f"공정 시작 — 로봇 {len(agents)}대, 정책 {'ON' if rl_registry is not None else 'OFF'}", "info", 0.0)
        print(f"[main] 모니터 피드 → {_feed_path}")
    _NAMES = {"C": "천장", "SL": "좌측", "SR": "우측"}
    # 웹 UI가 Isaac 월드(Z-up, 차 길이축 Y)를 자기 좌표계로 옮길 때 쓰는 기준: 차 점군 bbox
    _feed_scene = {"up": "z", "long": "y",
                   # 설비 수치(웹 UI 소품 배치 동기화): 갠트리 보 높이·반폭·반길이, 천장 베이스 z 범위, 측면 레일 x, 텔레리프트 접힘/펼침
                   "gantry_beam_z": float(common.GANTRY_BEAM_Z), "gantry_half_x": float(common.GANTRY_HALF_X), "gantry_half_y": float(common.GANTRY_HALF_Y),
                   "overhead_z": [float(common.OVERHEAD_Z_MIN), float(common.OVERHEAD_Z_MAX)],
                   "rail_x": [float(c["rail_x"]) for c in RAIL_CONFIGS if c.get("mount_mode") == "side"],
                   "lift_h": [float(common.TELE_LIFT_RETRACTED_H), float(common.TELE_LIFT_EXTENDED_H)],
                   "car_lift_z": float(common.CAR_LIFT_Z),
                   "car_min": [float(v) for v in np.min(raw_points, axis=0)] if len(raw_points) else [0, 0, 0],
                   "car_max": [float(v) for v in np.max(raw_points, axis=0)] if len(raw_points) else [0, 0, 0]}
    _FEED_EVERY = max(1, int(os.environ.get("POLISH_MONITOR_FEED_EVERY", "6")))   # 6 스텝 ≈ 10 Hz (팔 동기화용)
    # 유효 레시피 값(웹 UI 상태줄·조건 대조용): 잔차 브리지의 top 레시피
    _feed_recipe = None
    try:
        _rc = getattr(rl_bridge, "recipe_top", None) if rl_bridge is not None else None
        if _rc is not None:
            _feed_recipe = {"force_n": float(getattr(_rc, "target_contact_force_n", 0.0)), "feed_mm_s": float(getattr(_rc, "feed_speed_mm_s", 0.0)),
                            "rpm": float(getattr(_rc, "rpm", 0.0)), "step_over_ratio": float(getattr(_rc, "step_over_spacing_ratio", 0.0)),
                            "n_passes": int(getattr(_rc, "n_passes", 0)), "pad_radius_m": float(common.POLISHING_DISK_RADIUS),
                            "robots": [a.label for a in agents]}
    except Exception as _exc:
        print(f"[main] 레시피 값 수집 실패: {_exc}", flush=True)
    # 기록기(POLISH_RECORD=<sqlite 경로> 또는 1): UI 리플레이/지연 재생용 — 피드와 같은 주기로 프레임을 DB 에 쓴다
    recorder = None
    _rec_env = os.environ.get("POLISH_RECORD", "")
    if _rec_env:
        try:
            from learning.ui_bridge.sim_recorder import SimRecorder
            _rec_path = _rec_env if _rec_env != "1" else os.path.join(_SRC_DIR, "learning", "ui_bridge", "out",
                                                                       time.strftime("run_%Y%m%d_%H%M%S.sqlite"))
            recorder = SimRecorder(_rec_path, meta={"scene": _feed_scene, "hz": 60.0 / _FEED_EVERY, "recipe_values": _feed_recipe,
                                                     "robots": [{"id": a.label, "name": _NAMES.get(a.label, a.label)} for a in agents],
                                                     "recipe": {"top": os.environ.get("POLISH_RL_RECIPE_TOP", ""),
                                                                "side": os.environ.get("POLISH_RL_RECIPE_SIDE", "")},
                                                     "rl": os.environ.get("POLISH_RL", "0") == "1",
                                                     "physical_contact": os.environ.get("POLISH_PHYSICAL_CONTACT", "0") == "1"})
            print(f"[main] 기록기 → {_rec_path}", flush=True)
        except Exception as _exc:
            print(f"[main] 기록기 생성 실패(시뮬 계속): {_exc}", flush=True)
    def _feed_tick():
        if monitor_feed is None and recorder is None:
            return
        robots = []
        for a in agents:
            st_name = str(getattr(a, "run_state", "POLISH")).split(".")[-1]
            seg_prog = (a.current_path_idx_float / max(len(a.path) - 1, 1)) if len(getattr(a, "path", [])) else 0.0
            rob = {"id": a.label, "name": _NAMES.get(a.label, a.label),
                   "force": float(a.filtered_contact_force), "target": float(getattr(a, "_target_force", 0.0)),
                   "state": st_name, "progress": float(min(max(seg_prog, 0.0), 1.0)),
                   "rl_force_scale": float(getattr(a, "_rl_force_scale", 1.0)),
                   "rl_feed_scale": float(getattr(a, "_rl_feed_scale", 1.0))}
            # 웹 UI 3D 팔 동기화용: 팔 관절각 6개(rad, URDF 순서) + 베이스 자세(Isaac 월드, quat w,x,y,z)
            try:
                _q = np.asarray(a.articulation.get_joint_positions(), dtype=float)
                rob["q"] = [float(v) for v in _q[:6]]
            except Exception:
                pass
            try:
                rob["base"] = {"pos": [float(v) for v in a.base_position[:3]],
                               "quat": [float(v) for v in a.base_orientation[:4]]}
            except Exception:
                pass
            robots.append(rob)
        try:
            cov = polish_viz.covered_count() / max(polish_viz.total_count(), 1)
        except Exception:
            cov = 0.0
        cells = {}
        if rl_registry is not None:
            try:
                from .rl_bridge import judge_cells
                rows = judge_cells(rl_registry)
                cells = {"total": len(rows),
                         "pass": sum(1 for r in rows if r["overall_pass"]),
                         "rework": sum(1 for r in rows if r["disposition"] == "rework_candidate"),
                         "repaint": sum(1 for r in rows if r["disposition"] == "spot_repaint_review"),
                         "not_reached": sum(1 for r in rows if r["disposition"] == "not_reached"),
                         "items": [[float(r["center_x_m"]), float(r["center_y_m"]), float(r["center_z_m"]),
                                    r["disposition"], float(r["gu_proxy_after"])] for r in rows if r["visits"] > 0]}
            except Exception as exc:
                cells = {"error": str(exc)[:80]}
        overall_state = "DONE" if all(a.done for a in agents) else ("HOLD" if _ctl.get("pause") else (robots[0]["state"] if robots else "POLISH"))
        if recorder is not None:
            try:
                recorder.frame(sim_step / 60.0, overall_state, cov, sim_step / 60.0, robots)
                if cells and sim_step % (_FEED_EVERY * 100) == 0:     # 셀 판정 스냅샷은 ~10 s 마다
                    recorder.cells(sim_step / 60.0, cells)
            except Exception as _exc:
                print(f"[main] 기록 실패: {_exc}", flush=True)
        if monitor_feed is not None:
            monitor_feed.update(overall_state, cov, robots, elapsed_s=sim_step / 60.0, cells=cells, scene=_feed_scene, recipe=_feed_recipe)

    # 물리 초기화
    world.reset()
    for agent in agents:
        agent.initialize()

    import omni.timeline
    omni.timeline.get_timeline_interface().play()
    print("[main] 시뮬레이션 시작 — 차량 좌표 확인 후 위 로봇 베이스와 비교하세요")

    # 대시보드용 ROS2 퍼블리셔(계측 전용, 실패해도 시뮬 계속)
    from .ros_publisher import make_publisher
    _car_center = (np.mean(raw_points, axis=0) if len(raw_points) else np.array([0.0, 0.0, 1.0]))
    ros_pub = make_publisher(agents, polish_viz, _car_center)

    from isaacsim.core.prims import SingleXFormPrim as _XFP
    viz_flush_step = 0
    lift_step = 0
    scan_cloud_revealed = False
    # 차/점군이 +Y에서 바닥으로 진입한다. 진입 중 z는 리프트 p=0(바닥) 상태.
    park_items = [it for it in lift_anim if it[0] != "/World/WheelLift_base"]
    _park_set = {it[0] for it in park_items}
    scan_stop_y = float(np.clip(CAR_ENTRY_SCAN_Y, 0.1, CAR_PARK_START_Y - 0.1))
    scan_enabled = entry_scanner is not None
    pre_scan_steps = max(1, int(CAR_PARK_ANIM_STEPS * (CAR_PARK_START_Y - scan_stop_y) / CAR_PARK_START_Y))
    post_scan_steps = max(1, CAR_PARK_ANIM_STEPS - pre_scan_steps)
    park_phase = "to_scan" if scan_enabled else "to_lift"
    park_step = 0
    scan_step = 0

    def _set_parking_pose(y_off):
        for _path, _x, _y, _zf in lift_anim:
            _yo = y_off if _path in _park_set else 0.0
            _XFP(prim_path=_path).set_world_pose(
                position=np.array([_x, _y + _yo, _zf - CAR_LIFT_Z])
            )

    # 재폴리싱은 각 로봇이 독립적으로 관리 (agent._try_start_next_pass).
    # runner는 모든 로봇이 진짜 done 됐을 때 리프트 하강만 처리.
    lowering = False               # 리프트 하강 단계 진입 여부
    lowered_done = False           # 하강 1회 완료(재진입 방지)
    lower_step = 0
    # 렌더 스로틀(속도): N스텝마다 1번만 렌더. 물리는 매 스텝 그대로 → 결과 동일, 화면만 듬성듬성.
    # POLISH_RENDER_EVERY=1(기본)=매 스텝 렌더, 4~10이면 벽시계 속도 크게 상승.
    render_every = max(1, int(os.environ.get("POLISH_RENDER_EVERY", "1")))
    render_idx = 0
    # 헤드리스 종료 조건 (기본값은 기존과 동일: 창을 닫을 때까지 무한 루프).
    #   MAX_SIM_STEPS=N          → world.step() N회 후 루프 종료 (0=무제한)
    #   POLISH_EXIT_WHEN_DONE=1  → 모든 로봇 완료 + 리프트 하강 완료 시 루프 종료
    max_sim_steps = int(os.environ.get("MAX_SIM_STEPS", "0") or 0)
    exit_when_done = os.environ.get("POLISH_EXIT_WHEN_DONE", "0") == "1"
    _ctl_path = os.environ.get("POLISH_CONTROL", ""); _ctl_mtime = [None]; _ctl = {"pause": False}
    sim_step = 0
    while simulation_app.is_running():
        do_render = (render_idx % render_every == 0)
        render_idx += 1
        world.step(render=do_render)
        sim_step += 1
        if max_sim_steps > 0 and sim_step >= max_sim_steps:
            print(f"[main] MAX_SIM_STEPS={max_sim_steps} 도달 — 시뮬레이션 루프 종료", flush=True)
            _rl_flush("max_steps")
            break
        if not world.is_playing():
            continue

        stage = omni.usd.get_context().get_stage()

        # 주차 진입: 건물 진입 직전 정지 → 반도넛 스캐너 상승/스윕 → 리프트 원점까지 주차.
        if park_phase != "done":
            if park_phase == "to_scan":
                q = _smoothstep01(park_step / float(pre_scan_steps))
                y_off = CAR_PARK_START_Y + (scan_stop_y - CAR_PARK_START_Y) * q
                _set_parking_pose(y_off)
                _set_entry_scanner_pose(
                    _XFP, entry_scanner,
                    scan_stop_y + ENTRY_SCANNER_SWEEP_HALF_Y,
                    entry_scanner["hidden_z"],
                )
                park_step += 1
                if park_step > pre_scan_steps:
                    park_phase = "scan"
                    scan_step = 0
                    print(f"[main] 차량 진입 전 스캔 위치 도착(Y={scan_stop_y:.2f}) — 하부 스캐너 작동")
                continue

            if park_phase == "scan":
                _set_parking_pose(scan_stop_y)
                rise_n = max(1, ENTRY_SCANNER_RISE_STEPS)
                sweep_n = max(1, ENTRY_SCANNER_SWEEP_STEPS)
                lower_n = max(1, ENTRY_SCANNER_LOWER_STEPS)
                hidden_z = entry_scanner["hidden_z"]
                shown_z = 0.0
                start_y = scan_stop_y + ENTRY_SCANNER_SWEEP_HALF_Y
                end_y = scan_stop_y - ENTRY_SCANNER_SWEEP_HALF_Y
                if scan_step <= rise_n:
                    q = _smoothstep01(scan_step / float(rise_n))
                    scanner_y = start_y
                    scanner_z = hidden_z + (shown_z - hidden_z) * q
                elif scan_step <= rise_n + sweep_n:
                    q = _smoothstep01((scan_step - rise_n) / float(sweep_n))
                    scanner_y = start_y + (end_y - start_y) * q
                    scanner_z = shown_z
                else:
                    q = _smoothstep01((scan_step - rise_n - sweep_n) / float(lower_n))
                    scanner_y = end_y
                    scanner_z = shown_z + (hidden_z - shown_z) * q
                _set_entry_scanner_pose(_XFP, entry_scanner, scanner_y, scanner_z)
                scan_step += 1
                if scan_step > rise_n + sweep_n + lower_n:
                    _set_entry_scanner_pose(_XFP, entry_scanner, end_y, hidden_z)
                    park_phase = "to_lift"
                    park_step = 0
                    print("[main] 진입 전 하부 스캔 완료 — 차량 주차 재개")
                continue

            if park_phase == "to_lift":
                start_y = scan_stop_y if scan_enabled else CAR_PARK_START_Y
                step_n = post_scan_steps if scan_enabled else CAR_PARK_ANIM_STEPS
                q = _smoothstep01(park_step / float(step_n))
                _set_parking_pose(start_y * (1.0 - q))
                park_step += 1
                if park_step > step_n:
                    park_phase = "done"
                continue   # 주차 완료 전엔 로봇 스텝 보류

        # 정비소 리프트: 차+점군+단상이 바닥에서 리프트 높이까지 함께 상승 (폴리싱 시작 전)
        if lift_step <= CAR_LIFT_ANIM_STEPS:
            p = lift_step / float(CAR_LIFT_ANIM_STEPS)
            drop = CAR_LIFT_Z * (1.0 - p)   # 현재 내려가 있는 양
            for _path, _x, _y, _zf in lift_anim:
                _XFP(prim_path=_path).set_world_pose(position=np.array([_x, _y, _zf - drop]))
            lift_step += 1
            if lift_step > CAR_LIFT_ANIM_STEPS and not scan_cloud_revealed:
                _scan_prim = stage.GetPrimAtPath("/World/ScanPointCloud")
                if _scan_prim.IsValid():
                    UsdGeom.Imageable(_scan_prim).MakeVisible()
                    print("[main] 리프트 완료 — 포인트 클라우드 표시 시작")
                scan_cloud_revealed = True
            if lift_step <= CAR_LIFT_ANIM_STEPS:
                continue   # 상승 완료 전엔 로봇 스텝 보류(대기)

        # 리프트 하강: 모든 면 폴리싱 수렴 후, 차+리프트+점군이 함께 원위치(바닥)로 내려옴
        if lowering:
            p = lower_step / float(CAR_LIFT_ANIM_STEPS)
            drop = CAR_LIFT_Z * p   # 0 → CAR_LIFT_Z 까지 내려감
            for _path, _x, _y, _zf in lift_anim:
                _XFP(prim_path=_path).set_world_pose(position=np.array([_x, _y, _zf - drop]))
            polish_viz.flush(stage)
            lower_step += 1
            if lower_step > CAR_LIFT_ANIM_STEPS:
                print("[main] 리프트 하강 완료 — 차량 원위치 복귀. (시뮬레이션 계속 실행 중)")
                lowering = False  # 하강 루프 탈출, 이후 idle 루프
                lowered_done = True  # 재진입 방지
                if exit_when_done:
                    print("[main] POLISH_EXIT_WHEN_DONE=1 — 폴리싱·하강 완료, 시뮬레이션 루프 종료", flush=True)
                    _rl_flush("done")
                    break
            continue

        # 하강까지 모두 끝났으면 idle(시뮬 창만 유지, 아무 동작 안 함)
        if lowered_done:
            if exit_when_done:
                break
            continue

        # 웹 UI 실행 중 컨트롤(POLISH_CONTROL=<json>): {"pause": bool, "force_scale": x, "feed_scale": y} — 30 스텝마다 mtime 확인
        if _ctl_path and sim_step % 30 == 0:
            try:
                _m = os.path.getmtime(_ctl_path) if os.path.exists(_ctl_path) else None
                if _m is not None and _m != _ctl_mtime[0]:
                    _ctl_mtime[0] = _m
                    _c = json.load(open(_ctl_path)) or {}
                    _ctl["pause"] = bool(_c.get("pause", False))
                    _fs = max(0.3, min(2.0, float(_c.get("force_scale", 1.0)))); _fd = max(0.2, min(3.0, float(_c.get("feed_scale", 1.0))))
                    for agent in agents:
                        agent._ui_force_scale = _fs; agent._ui_feed_scale = _fd
                    print(f"[main] 컨트롤: pause={_ctl['pause']} force×{_fs:.2f} feed×{_fd:.2f}", flush=True)
                    if monitor_feed is not None:
                        monitor_feed.event("C", f"웹 UI 컨트롤: {'일시정지' if _ctl['pause'] else '진행'} · 힘×{_fs:.2f} · 이송×{_fd:.2f}", "info", sim_step / 60.0)
            except Exception as _exc:
                print(f"[main] 컨트롤 파일 읽기 실패: {_exc}", flush=True)
        if not _ctl["pause"]:
            for agent in agents:
                agent.step(stage)
        if (monitor_feed is not None or recorder is not None) and sim_step % _FEED_EVERY == 0:
            _feed_tick()
        if rl_registry is not None and sim_step % 3000 == 0:      # ~50 s 마다 중간 판정 저장
            _rl_flush("periodic")

        # 대시보드 실시간 데이터 퍼블리시(계측 전용)
        ros_pub.tick(polishing=True)

        # 폴리싱된 점 하늘색 재채색을 주기적으로 반영 (성능)
        viz_flush_step += 1
        if viz_flush_step % 30 == 0:
            polish_viz.flush(stage)

        if all(a.done for a in agents):
            polish_viz.flush(stage)
            cov = polish_viz.covered_count()
            total = polish_viz.total_count()
            print(f"[main] 모든 로봇 폴리싱 완료(누적 {cov}/{total}점) — 리프트 하강 시작")
            lowering = True
            lower_step = 0
