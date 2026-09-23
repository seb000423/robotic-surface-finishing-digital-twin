"""Rail robot state machine for polishing_v5."""
import os
import sys

import numpy as np
from scipy.spatial.transform import Rotation as R

from isaacsim.core.prims import SingleArticulation
from omni.isaac.core.objects import VisualCuboid, VisualCylinder
from omni.isaac.core.utils.prims import create_prim
from omni.isaac.core.utils.types import ArticulationAction
from omni.isaac.sensor import ContactSensor

from .common import *
from .common import _SCRIPT_DIR, _SRC_DIR, ROBOT_USD_PATH
from .visualization import CoverageMap, PolishViz
from m0609_rmpflow_controller import RMPFlowController

class RailRobotAgent:
    def __init__(self, rail_idx, config, raw_points, kdtree, scan_dir, coverage_map: CoverageMap = None,
                 polish_viz: "PolishViz" = None):
        self.idx = rail_idx
        self.coverage_map = coverage_map
        self.polish_viz = polish_viz
        self.rail_x   = float(config["rail_x"])
        self.nominal_rail_x = float(config["rail_x"])
        self.base_yaw = float(config["base_yaw"])
        self.label    = config["label"]
        # 리프트 좌표계로: 정지 위치 z 에 CAR_LIFT_Z 더함 (경로 점군도 동일하게 올림)
        self.yz_stops = [[float(y), float(z) + CAR_LIFT_Z] for (y, z) in config["yz_stops"]]
        self.is_overhead = config.get("mount_mode") == "overhead"
        self.is_side = config.get("mount_mode") == "side"
        self.outward_sign = int(config.get("outward_sign", -1))   # 측면 바깥 방향(좌-1/우+1)

        if self.is_overhead:
            # 거꾸로(상하 180° 반전) 매달림 + yaw → base 프레임 쿼터니언
            rot = R.from_euler("x", np.pi) * R.from_euler("z", self.base_yaw)
            q = rot.as_quat()  # [x, y, z, w]
            self.base_orientation = np.array([q[3], q[0], q[1], q[2]])  # [w, x, y, z]
        else:
            self.base_orientation = np.array([
                np.cos(self.base_yaw * 0.5), 0.0, 0.0, np.sin(self.base_yaw * 0.5)
            ])

        self.world_prim_path = f"/World/Rail_{self.label}"
        self.robot_root_path = f"/World/Rail_{self.label}/m0609/m0609"

        if self.is_overhead:
            # 위에서 내려다보며 폴리싱 — 손목 롤 기준은 레일(Y) 방향
            self.approach_dir = np.array([0.0, 1.0, 0.0])
            self.home_joints = HOME_JOINT_POSITIONS_OVERHEAD
            self.car_top_z = float(np.max(raw_points[:, 2]))
            self.transit_ee_z = self.car_top_z + OVERHEAD_TRANSIT_CLEAR
            self.press_start_offset = OVERHEAD_POLISH_START_OFFSET
        elif self.is_side:
            # 측면 EE 기준 방향(z_align의 fwd)은 '월드 위'로 — 수평 법선과 평행해지는
            # 퇴화(좌우 거울 뒤집힘) 방지. 이래야 좌/우 패드가 일관되게 표면을 향함.
            self.approach_dir = np.array([0.0, 0.0, 1.0])
            self.home_joints = HOME_JOINT_POSITIONS
            self.car_top_z = 0.0
            self.transit_ee_z = 0.0
            self.press_start_offset = SIDE_POLISH_START_OFFSET   # 표면 가까이서 시작 → 접촉
        else:
            # 왼쪽 레일(X=-1.0) → 차량 방향 +X 로 접근
            self.approach_dir = np.array([1.0, 0.0, 0.0])
            self.home_joints = HOME_JOINT_POSITIONS
            self.car_top_z = 0.0
            self.transit_ee_z = 0.0
            self.press_start_offset = PRESS_OFFSET_MAX

        self.raw_points = raw_points
        self.kdtree     = kdtree
        self.car_center_y = float((np.min(raw_points[:, 1]) + np.max(raw_points[:, 1])) * 0.5)

        # ── 레일 구간별 경로 로드 ──
        # segments: [(y_stop, z_stop, path_array), ...]  (빈 구간은 건너뜀)
        self.segments = []
        for stop_idx, (y_stop, z_stop) in enumerate(self.yz_stops):
            fname = os.path.join(scan_dir, f"path_{self.label}{stop_idx}.npy")
            if not os.path.exists(fname):
                print(f"[Rail {self.label}] 구간 {stop_idx}: 경로 파일 없음 ({fname})")
                continue
            raw_path = np.load(fname).astype(float)
            raw_path[:, 2] += CAR_LIFT_Z   # 경로도 리프트 좌표계로 (점군과 동일하게 상승)
            base_at_stop = np.array([self.rail_x, y_stop, z_stop])
            path = filter_safe_waypoints(
                raw_path, raw_points, kdtree, base_at_stop,
                is_side=self.is_side, is_overhead=self.is_overhead,
            )
            if len(path) == 0:
                print(f"[Rail {self.label}] 구간 {stop_idx} (Y={y_stop:.2f}, Z={z_stop:.2f}): 안전 웨이포인트 없음")
                continue
            self.segments.append((y_stop, z_stop, path))
            print(f"[Rail {self.label}] 구간 {stop_idx} (Y={y_stop:.2f}, Z={z_stop:.2f}): {len(path)}개 웨이포인트")

        # ── 발자국(footprint) 점군 커버리지 기준값 ──
        # 패드 발자국 아래 점 개수의 구간 중앙값을 기준으로, 그 비율 미만이면
        # 유리/구멍/가장자리로 보고 폴리싱을 건너뛴다(유리엔 점군이 없으므로 자동 회피).
        fp_counts = []
        for _, _, seg_path in self.segments:
            stride = max(1, len(seg_path) // 100)
            for q in seg_path[::stride]:
                fp_counts.append(len(kdtree.query_ball_point(np.asarray(q), SURFACE_FOOTPRINT_RADIUS)))
        self.footprint_ref = float(np.median(fp_counts)) if fp_counts else 0.0
        self.footprint_min = max(float(SURFACE_FOOTPRINT_MIN_ABS),
                                 SURFACE_FOOTPRINT_MIN_RATIO * self.footprint_ref)
        print(f"[Rail {self.label}] 발자국 기준: 중앙값={self.footprint_ref:.0f}점, "
              f"스킵 임계={self.footprint_min:.0f}점 (반경 {SURFACE_FOOTPRINT_RADIUS*100:.1f}cm)")

        # 현재 슬라이딩 위치 (초기: 첫 번째 정지 위치)
        if self.segments:
            first_y, first_z = self.segments[0][0], self.segments[0][1]
        else:
            first_y, first_z = self.yz_stops[0][0], self.yz_stops[0][1]
        self.base_position = np.array([self.rail_x, first_y, first_z])
        # 측면 단상은 짧은 중립 높이에서 시작 → 차 상승 후 SLIDE에서 working 높이까지 천천히 상승
        if self.is_side:
            self.base_position[2] = SIDE_COLUMN_NEUTRAL_Z
        # 오버헤드는 천장 가까이(슬라이더 짧게) 시작 → 어프로치에서 천천히 하강
        if self.is_overhead:
            self.base_position[2] = OVERHEAD_Z_MAX

        # 제어기 및 센서
        self.articulation: SingleArticulation = None
        self.controller: RMPFlowController = None
        self.contact_sensor: ContactSensor = None
        self.coverage_map = coverage_map
        self.pad_path = None
        self.pad_visual_path = None
        self.pad_contact_offset_local = np.array([-0.0025, -0.0200, -0.0040])
        self.pad_contact_axis_local = np.array([0.0, 0.0, 1.0])
        self._last_command_clearance = PRESS_OFFSET_MAX
        self._last_target_pad_pos = None
        self._elbow_collision_count = 0
        self.evade_attempts = 0
        self._consecutive_evades = 0   # 충돌 군집(예: 앞유리)을 빠르게 건너뛰기 위한 카운터
        self.elbow_floor = None
        self.elbow_floor_path = f"/World/ElbowFloor_{self.label}"

        # 상태 머신
        self.run_state = STATE_HOME
        self.state_step_count = 0
        self.step_count = 0
        self.current_seg_idx = 0         # 현재 처리 중인 구간 인덱스
        self.path = np.array([])         # 현재 구간의 경로
        self.slide_target_y = first_y    # 슬라이딩 목표 Y 위치
        self.slide_target_z = first_z    # 슬라이딩 목표 Z 위치
        self.current_target_idx = 0
        self.current_path_idx_float = 0.0
        self.z_offset = PRESS_OFFSET_MAX
        self.z_vel = 0.0
        self._lag_ff = 0.0   # 추종지연 피드포워드 추정값(법선 방향, m)
        self.filtered_contact_force = 0.0
        self.previous_normal = None
        self.high_force_pause_steps = 0
        self.stable_contact_steps = 0
        self.path_complete_reported = False
        self.done = False
        self._bad_contact_steps = 0
        self._bad_contact_last_idx = -1
        self._visual_pad_compression = 0.0
        self._arm_guard_strikes = 0

        # 진단/시각 변형 상태
        self._last_conform_max = 0.0     # 직전 스펀지 곡면 변형 최대치(m)
        self._glass_skip_count = 0       # 유리/구멍 스킵 누적
        self._side_debug_done = False    # 측면 디스크 접촉축 1회 측정 플래그
        self._repolish_pass = False      # 재폴리싱 패스 여부(이미 칠한 점 건너뜀)
        self._completed_segs: set = set()  # 이미 완료(RETRACT까지 진행)한 구간 인덱스
        self._polish_pass = 0            # 독립 재폴리싱 패스 카운터 (0=초기, ≥1=재폴리싱)
        self._max_passes = 2             # 최대 독립 재폴리싱 횟수
        self._pass_rearm_attempted = False  # STATE_DONE 진입 시 재폴리싱 시도 여부(1회만)

        # 시각화
        self.future_path_prim = None
        self.completed_path_prim = None
        self.path_pointer_path = None

        # Stuck / 팔꿈치 충돌 감지용
        # 오버헤드는 가상 패드가 압입에 시간이 더 필요 → 정체 판정 완화
        # 오버헤드는 표면까지 압입(0.045→음수)에 시간이 더 필요 → 정체 판정 더 완화(600)
        self._stuck_check_interval = 600 if self.is_overhead else (360 if self.is_side else 180)   # 스텝 (@60Hz)
        self._stuck_last_path_idx  = 0.0
        self._stuck_steps_since_check = 0
        self._elbow_collision_count = 0

        # 로깅
        log_dir = _SCRIPT_DIR
        self.log_file   = os.path.join(log_dir, f"force_log_rail_{self.label}.csv")
        self.status_log = os.path.join(log_dir, "status_log.txt")
        with open(self.log_file, "w") as f:
            f.write(
                "step,seg,sensor_valid_force,raw_sensor,virtual_force,raw_force,filtered,"
                "z_offset,cmd_clearance,actual_gap,actual_clearance,path_idx,state,event\n"
            )

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
            name=f"m0609_rail_{self.label}",
        )

        # 영상 기준 기존 흰 원형 패드가 커 보이고 실제 접촉패드와 겹쳐 보여 숨긴다.
        self._remove_sander_parts(stage, {"tn__114555_", "tn__104327_"})

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
        self._calibrate_pad_contact_offset(stage)

        if PAD_ONLY_ROBOT_COLLISIONS:
            # 팔/샌더 본체 충돌은 항상 끔(팔꿈치 슬램 방지). 패드 디스크 충돌만 플래그로 제어:
            #  - USE_PHYSICAL_CONTACT_SENSOR=True  → 패드만 차체와 실제 접촉(실센서 힘 발생)
            #  - False → 패드도 충돌 OFF, 실측위치 가상 스프링만 사용(슬램 완전 제거)
            disabled = set_collision_enabled_recursive(stage, self.robot_root_path, False)
            pad_prim_for_collision = stage.GetPrimAtPath(self.pad_path)
            if pad_prim_for_collision.IsValid():
                pad_collision_api = UsdPhysics.CollisionAPI.Apply(pad_prim_for_collision)
                attr = pad_collision_api.GetCollisionEnabledAttr()
                if not attr:
                    attr = pad_collision_api.CreateCollisionEnabledAttr()
                attr.Set(bool(USE_PHYSICAL_CONTACT_SENSOR))
            print(f"[Rail {self.label}] robot collision 비활성화 {disabled}개, "
                  f"pad collision={'ON(실센서)' if USE_PHYSICAL_CONTACT_SENSOR else 'OFF(가상)'}")

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
            name=f"pad_contact_sensor_rail_{self.label}",
            frequency=60,
            translation=np.array([0, 0, 0]),
        )

        if self.is_overhead:
            self._setup_gantry_visuals(world)
            if USE_TELE_LIFT_ASSET:
                self._setup_tele_lift(stage)
        else:
            # 측면 레일(바닥 가이드 바) — 차체 길이만큼 Y로 길게
            if self.is_side and self.segments:
                # 측면 레일을 갠트리 포스트(X=±GANTRY_HALF_X, Y=±GANTRY_HALF_Y)까지 늘려
                # 양 끝이 기둥에 닿게. 측면 레일 X(±1.36)=GANTRY_HALF_X라 같은 선상.
                rail_cy = 0.0
                rail_len = 2.0 * GANTRY_HALF_Y
                if RAIL_USE_ASSET:
                    # Vention Rail.usd 세그먼트를 Y축으로 타일링 (시각 전용)
                    load_rail_tiles(stage, self.label, self.rail_x, rail_cy, rail_len)
                else:
                    world.scene.add(
                        VisualCuboid(
                            prim_path=f"/World/SideRail_{self.label}",
                            name=f"side_rail_{self.label}",
                            position=np.array([self.rail_x, rail_cy, 0.05]),
                            scale=np.array([0.12, rail_len, 0.08]),
                            color=np.array([0.25, 0.25, 0.28]),
                        )
                    )
            # 레일 베이스(바닥)
            world.scene.add(
                VisualCuboid(
                    prim_path=f"/World/RailSlider_{self.label}",
                    name=f"rail_slider_{self.label}",
                    position=np.array([self.rail_x, self.base_position[1], 0.05]),
                    scale=np.array([0.4, 0.4, 0.1]),
                    color=np.array([0.2, 0.2, 0.2]),
                )
            )
            if self.is_side and USE_TELE_LIFT_ASSET:
                # 측면 단상: Vention 텔레스코픽 리프트 USD (2단 신축)
                self._setup_tele_lift(stage)
            elif self.is_side:
                # (구버전) 측면 단상: 바닥에서 베이스까지 '자라는' 단일 원통
                init_h = max(0.04, float(self.base_position[2]) - 0.10)
                col = VisualCylinder(
                    prim_path=f"/World/SideColumn_{self.label}",
                    name=f"side_column_{self.label}",
                    position=np.array([self.rail_x, self.base_position[1], 0.10 + init_h / 2.0]),
                    radius=0.10, height=1.0,
                    color=np.array([0.35, 0.55, 0.75]),
                )
                world.scene.add(col)
                col.set_local_scale(np.array([1.0, 1.0, init_h]))
            else:
                # 3단 텔레스코픽 기둥 (옛 레일 모드 전용)
                for s, (rad, col_c) in enumerate([(0.12, (0.15, 0.25, 0.35)),
                                                  (0.10, (0.25, 0.40, 0.55)),
                                                  (0.08, (0.35, 0.55, 0.75))], start=1):
                    world.scene.add(VisualCylinder(
                        prim_path=f"/World/TelescopicColumn_S{s}_{self.label}",
                        name=f"telescopic_column_s{s}_{self.label}",
                        position=np.array([self.rail_x, self.base_position[1], 0.275]),
                        radius=rad, height=0.35, color=np.array(col_c),
                    ))

        print(f"[Rail {self.label}] setup 완료: {self.world_prim_path}")

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

        # 팔꿈치 차체 관통 방지: 삭제됨 (사용자 요청)
        pass

        # 차체를 RMPFlow 장애물로 등록: 삭제됨 (사용자 요청)
        pass

        print(f"[Rail {self.label}] 초기화 완료")

    def _update_elbow_floor(self, target_z):
        pass

    def setup_visualization(self, stage):
        """포인트 클라우드 + 경로 선분 + 목표 구체를 USD 씬에 생성."""
        from pxr import UsdGeom, Vt, Gf

        color = RAIL_COLORS[self.idx % len(RAIL_COLORS)]

        # 모든 구간 점군을 한꺼번에 시각화
        all_pts = []
        for _, _, seg_path in self.segments:
            all_pts.extend(seg_path)
        if all_pts:
            all_pts_arr = np.array(all_pts)
            pts_prim = UsdGeom.Points.Define(stage, f"/World/PointCloud_Rail_{self.label}")
            pts_prim.CreatePointsAttr(Vt.Vec3fArray([
                Gf.Vec3f(float(p[0]), float(p[1]), float(p[2])) for p in all_pts_arr
            ]))
            pts_prim.CreateWidthsAttr(Vt.FloatArray([0.002] * len(all_pts_arr)))
            pts_prim.CreateDisplayColorAttr(Vt.Vec3fArray([Gf.Vec3f(*color)]))

        # 미래 경로 선분 (초록)
        self.future_path_prim = UsdGeom.BasisCurves.Define(stage, f"/World/FuturePath_Rail_{self.label}")
        self.future_path_prim.CreateTypeAttr().Set(UsdGeom.Tokens.linear)
        self.future_path_prim.CreateWidthsAttr().Set([0.004])
        self.future_path_prim.CreateDisplayColorAttr().Set([Gf.Vec3f(0.0, 1.0, 0.0)])

        # 완료 경로 선분 (노란색)
        self.completed_path_prim = UsdGeom.BasisCurves.Define(stage, f"/World/CompletedPath_Rail_{self.label}")
        self.completed_path_prim.CreateTypeAttr().Set(UsdGeom.Tokens.linear)
        self.completed_path_prim.CreateWidthsAttr().Set([0.007])
        self.completed_path_prim.CreateDisplayColorAttr().Set([Gf.Vec3f(1.0, 0.82, 0.05)])

        # 현재 목표 구체(큰 공) — 사용자 요청으로 제거(생성 안 함)
        self.path_pointer_path = None

        # 커버리지 완료 포인트 (녹색)
        self.coverage_viz_path = f"/World/CoverageCloud_Rail_{self.label}"
        cov_prim = UsdGeom.Points.Define(stage, self.coverage_viz_path)
        cov_prim.CreateWidthsAttr().Set([0.025])
        cov_prim.CreateDisplayColorAttr().Set([Gf.Vec3f(0.0, 1.0, 0.2)])

        print(f"[Rail {self.label}] 시각화 초기화 완료 (color={color})")

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

    def _update_coverage_visualization(self, stage):
        """(비활성) 큰 녹색 복셀 블롭 대신, 스캔 포인트를 패드 반경 내 하늘색으로 재채색
        (PolishViz)으로 대체했다."""
        return
        if self.coverage_map is None:
            return
        from pxr import Vt, Gf, UsdGeom
        cov_prim = stage.GetPrimAtPath(self.coverage_viz_path)
        if not cov_prim.IsValid():
            return
        pts = self.coverage_map.covered_positions()
        if len(pts) == 0:
            return
        mesh = UsdGeom.Points(cov_prim)
        mesh.GetPointsAttr().Set(Vt.Vec3fArray([
            Gf.Vec3f(float(p[0]), float(p[1]), float(p[2])) for p in pts
        ]))
        mesh.GetWidthsAttr().Set(Vt.FloatArray([0.025] * len(pts)))

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
        target_orientation = self._ee_orientation(normal)
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

    def _calibrate_pad_contact_offset(self, stage):
        """Find the real pad contact-face center in link_6 coordinates."""
        try:
            from pxr import UsdGeom, Gf
            link_prim = stage.GetPrimAtPath(self.robot_root_path + "/link_6")
            pad_prim = stage.GetPrimAtPath(self.pad_path)
            if not (link_prim.IsValid() and pad_prim.IsValid()):
                raise RuntimeError("link_6 or pad prim is missing")
            cache = UsdGeom.XformCache()
            link_to_world = cache.GetLocalToWorldTransform(link_prim)
            pad_to_world = cache.GetLocalToWorldTransform(pad_prim)
            contact_local = Gf.Vec3d(0.0, -0.5 * float(POLISHING_DISK_HEIGHT), 0.0)
            contact_world = pad_to_world.Transform(contact_local)
            contact_link = link_to_world.GetInverse().Transform(contact_world)
            axis_world = pad_to_world.TransformDir(Gf.Vec3d(0.0, -1.0, 0.0))
            axis_link = link_to_world.GetInverse().TransformDir(axis_world)
            self.pad_contact_offset_local = np.array([
                float(contact_link[0]),
                float(contact_link[1]),
                float(contact_link[2]),
            ])
            self.pad_contact_axis_local = np.array([
                float(axis_link[0]),
                float(axis_link[1]),
                float(axis_link[2]),
            ])
            self.pad_contact_axis_local /= (np.linalg.norm(self.pad_contact_axis_local) + 1e-9)
            print(
                f"[Rail {self.label}] pad contact offset(link_6)="
                f"({self.pad_contact_offset_local[0]:+.4f},"
                f"{self.pad_contact_offset_local[1]:+.4f},"
                f"{self.pad_contact_offset_local[2]:+.4f}) "
                f"axis=({self.pad_contact_axis_local[0]:+.3f},"
                f"{self.pad_contact_axis_local[1]:+.3f},"
                f"{self.pad_contact_axis_local[2]:+.3f})",
                flush=True,
            )
        except Exception as exc:
            print(
                f"[Rail {self.label}] pad contact offset 자동보정 실패, 기본값 사용: {exc}",
                flush=True,
            )

    def _virtual_contact_distance(self):
        return SIDE_VIRTUAL_PAD_CONTACT_DISTANCE if self.is_side else VIRTUAL_PAD_CONTACT_DISTANCE

    def _real_contact_tolerances(self):
        if self.is_side:
            return REAL_CONTACT_CLEARANCE_TOL_SIDE, REAL_CONTACT_GAP_TOL_SIDE
        return REAL_CONTACT_CLEARANCE_TOL_TOP, REAL_CONTACT_GAP_TOL_TOP

    def _pad_command_clearance(self, contact_distance=None):
        """Convert virtual spring z_offset to the commanded real pad-face clearance.

        z_offset is the compliant-pad state for force control. The RMPFlow target needs
        the physical pad face, so equilibrium should sit at the surface rather than
        several centimeters above it.
        """
        cdist = self._virtual_contact_distance() if contact_distance is None else float(contact_distance)
        clearance = float(self.z_offset) - cdist
        return float(np.clip(clearance, SURFACE_GUARD_MIN_CLEARANCE, PRESS_OFFSET_MAX))

    def _pad_contact_world_pos(self, stage):
        local_contact = np.array([0.0, -0.5 * float(POLISHING_DISK_HEIGHT), 0.0])
        try:
            from omni.isaac.core.utils.xforms import get_world_pose
            pad_world_pos, pad_world_quat = get_world_pose(self.pad_path)
            pad_world_rot = R.from_quat([
                pad_world_quat[1],
                pad_world_quat[2],
                pad_world_quat[3],
                pad_world_quat[0],
            ])
            return np.asarray(pad_world_pos, dtype=float) + pad_world_rot.apply(local_contact)
        except Exception:
            pass
        try:
            from pxr import UsdGeom, Gf
            prim = stage.GetPrimAtPath(self.pad_path)
            if not prim.IsValid():
                return None
            matrix = UsdGeom.XformCache().GetLocalToWorldTransform(prim)
            contact_local = Gf.Vec3d(*[float(v) for v in local_contact])
            p = matrix.Transform(contact_local)
            return np.array([float(p[0]), float(p[1]), float(p[2])])
        except Exception:
            return None

    def _pad_contact_face_world_points(self, stage):
        """Sample the actual circular pad face, not just its center."""
        half_height = 0.5 * float(POLISHING_DISK_HEIGHT)
        side_count = max(8, int(POLISHING_DISK_SIDES))
        local_points = [np.array([0.0, -half_height, 0.0], dtype=float)]
        for radius_scale in (0.55, 0.95):
            radius = float(POLISHING_DISK_RADIUS) * radius_scale
            for i in range(side_count):
                angle = 2.0 * np.pi * float(i) / float(side_count)
                local_points.append(np.array([
                    radius * np.cos(angle),
                    -half_height,
                    radius * np.sin(angle),
                ], dtype=float))

        try:
            from omni.isaac.core.utils.xforms import get_world_pose
            pad_world_pos, pad_world_quat = get_world_pose(self.pad_path)
            pad_world_rot = R.from_quat([
                pad_world_quat[1],
                pad_world_quat[2],
                pad_world_quat[3],
                pad_world_quat[0],
            ])
            origin = np.asarray(pad_world_pos, dtype=float)
            return np.asarray([origin + pad_world_rot.apply(p) for p in local_points], dtype=float)
        except Exception:
            pass

        try:
            from pxr import UsdGeom, Gf
            prim = stage.GetPrimAtPath(self.pad_path)
            if not prim.IsValid():
                return None
            matrix = UsdGeom.XformCache().GetLocalToWorldTransform(prim)
            world_points = []
            for p in local_points:
                wp = matrix.Transform(Gf.Vec3d(float(p[0]), float(p[1]), float(p[2])))
                world_points.append([float(wp[0]), float(wp[1]), float(wp[2])])
            return np.asarray(world_points, dtype=float)
        except Exception:
            return None

    def _pad_contact_footprint_metrics(self, stage, normal):
        face_points = self._pad_contact_face_world_points(stage)
        if face_points is None or len(face_points) == 0:
            return None, None, None

        n = np.asarray(normal, dtype=float)
        n /= (np.linalg.norm(n) + 1e-9)
        best_pos = None
        best_gap = None
        best_clearance = None
        best_score = None
        for point in face_points:
            gap, idx = self.kdtree.query(np.asarray(point))
            surface_point = np.asarray(self.raw_points[int(idx)], dtype=float)
            clearance = float(np.dot(np.asarray(point) - surface_point, n))
            score = float(gap) + 0.15 * abs(clearance)
            if best_score is None or score < best_score:
                best_pos = np.asarray(point, dtype=float)
                best_gap = float(gap)
                best_clearance = clearance
                best_score = score
        return best_pos, best_gap, best_clearance

    def _arm_surface_clearance(self, stage):
        """Approximate robot-link clearance to the scanned car shell.

        Robot body collisions stay disabled so the force sensor is not polluted by
        link impacts. This lightweight guard catches obvious visual penetration and
        retracts the polishing target before the arm keeps driving through the car.
        """
        try:
            from pxr import UsdGeom, Gf
        except Exception:
            return (
                None,
                "",
                ARM_SURFACE_GUARD_CLEARANCE,
                ARM_SURFACE_SKIP_CLEARANCE_SIDE if self.is_side else ARM_SURFACE_SKIP_CLEARANCE_TOP,
            )

        # Local samples are based on the M0609 RMPFlow collision spheres, expanded
        # slightly for visual safety. The pad itself is excluded because it is the
        # only part allowed to touch the car.
        guard_spheres = {
            "link_2": [
                ((0.10, 0.00, 0.04), 0.085),
                ((0.20, 0.00, 0.09), 0.075),
                ((0.31, 0.00, 0.12), 0.075),
                ((0.39, 0.00, 0.15), 0.065),
            ],
            "link_3": [
                ((0.00, 0.00, 0.04), 0.075),
                ((0.00, -0.01, 0.09), 0.065),
            ],
            "link_4": [
                ((0.00, 0.05, -0.05), 0.072),
                ((0.00, 0.10, -0.14), 0.072),
                ((0.00, 0.16, -0.23), 0.066),
                ((0.00, 0.24, -0.31), 0.064),
            ],
            "link_5": [
                ((0.00, 0.00, 0.02), 0.040),
                ((0.00, 0.00, 0.06), 0.036),
            ],
        }
        watched_links = ("link_2", "link_3", "link_4", "link_5")
        min_clearance = None
        min_link = ""
        min_guard_margin = ARM_SURFACE_GUARD_CLEARANCE
        min_skip_clearance = ARM_SURFACE_SKIP_CLEARANCE_SIDE if self.is_side else ARM_SURFACE_SKIP_CLEARANCE_TOP
        min_risk = None

        def thresholds_for(link_name):
            if link_name == "link_5":
                if self.is_side:
                    return -0.010, -0.024
                return -0.004, -0.018
            if link_name == "link_4":
                if self.is_side:
                    # 측면은 팔뚝이 옆면에 자연히 가까움. 얕은 접촉(>−12mm)은 폴리싱 지속,
                    # 깊은 관통일 때만 스킵 → SR이 waypoint마다 무한 스킵하던 루프 제거.
                    return 0.006, -0.012
                return 0.024, 0.004
            return (
                ARM_SURFACE_GUARD_CLEARANCE,
                ARM_SURFACE_SKIP_CLEARANCE_SIDE if self.is_side else ARM_SURFACE_SKIP_CLEARANCE_TOP,
            )

        def measure(point, radius, label, guard_margin, skip_clearance):
            nonlocal min_clearance, min_link, min_guard_margin, min_skip_clearance, min_risk
            dist, _ = self.kdtree.query(np.asarray(point, dtype=float))
            clearance = float(dist) - float(radius)
            risk = clearance - float(guard_margin)
            if min_risk is None or risk < min_risk:
                min_risk = risk
                min_clearance = clearance
                min_link = label
                min_guard_margin = float(guard_margin)
                min_skip_clearance = float(skip_clearance)

        cache = UsdGeom.XformCache()
        for link_name in watched_links:
            prim = stage.GetPrimAtPath(f"{self.robot_root_path}/{link_name}")
            if not prim.IsValid():
                continue
            try:
                link_to_world = cache.GetLocalToWorldTransform(prim)
            except Exception:
                continue
            guard_margin, skip_clearance = thresholds_for(link_name)
            samples = []
            for sample_idx, (center, radius) in enumerate(guard_spheres[link_name]):
                w = link_to_world.Transform(Gf.Vec3d(*[float(v) for v in center]))
                world_center = np.array([float(w[0]), float(w[1]), float(w[2])], dtype=float)
                samples.append((world_center, float(radius), f"{link_name}[{sample_idx}]"))
                measure(world_center, radius, f"{link_name}[{sample_idx}]", guard_margin, skip_clearance)
            for sample_idx in range(1, len(samples)):
                a, ar, _ = samples[sample_idx - 1]
                b, br, _ = samples[sample_idx]
                radius = max(ar, br) * 0.92
                for t in np.linspace(0.25, 0.75, 3):
                    p = a * (1.0 - t) + b * t
                    measure(p, radius, f"{link_name}[{sample_idx - 1}-{sample_idx}]",
                            guard_margin, skip_clearance)
        return min_clearance, min_link, min_guard_margin, min_skip_clearance

    def _try_recover_from_arm_guard(self, stage, arm_guard_link, arm_clearance):
        """Lift the gantry/side pedestal a little so IK can open the elbow."""
        if self._arm_guard_strikes > ARM_SURFACE_MAX_RECOVER_STRIKES:
            return False

        if self.is_overhead:
            lift = ARM_SURFACE_RECOVER_LIFT_TOP
            upper = OVERHEAD_Z_MAX
        elif self.is_side:
            outward = float(self.outward_sign)
            max_abs_x = abs(float(self.nominal_rail_x)) + SIDE_RECOVER_OUTWARD_MAX
            new_x = float(self.rail_x) + outward * SIDE_RECOVER_OUTWARD_STEP
            if abs(new_x) <= max_abs_x + 1e-6:
                self.rail_x = new_x
                self.base_position[0] = new_x
                self._update_column_visuals(stage, self.base_position[1], self.base_position[2])
                self.z_offset = self.press_start_offset
                self.z_vel = 0.0
                self.filtered_contact_force = 0.0
                self.stable_contact_steps = 0
                self.high_force_pause_steps = max(self.high_force_pause_steps, SPIKE_PAUSE_STEPS)
                self._log_diag(
                    "ARM_GUARD_RECOVER",
                    idx=self.current_target_idx,
                    link=arm_guard_link,
                    clearance=float(arm_clearance),
                    base_x=float(self.base_position[0]),
                    base_z=float(self.base_position[2]),
                    strikes=int(self._arm_guard_strikes),
                )
                return True
            lift = ARM_SURFACE_RECOVER_LIFT_SIDE
            upper = SIDE_BASE_Z_MAX
        else:
            return False

        old_z = float(self.base_position[2])
        new_z = min(float(upper), old_z + float(lift))
        if new_z <= old_z + 1e-4:
            return False

        self.base_position[2] = new_z
        self._update_column_visuals(stage, self.base_position[1], self.base_position[2])
        self.z_offset = self.press_start_offset
        self.z_vel = 0.0
        self.filtered_contact_force = 0.0
        self.stable_contact_steps = 0
        self.high_force_pause_steps = max(self.high_force_pause_steps, SPIKE_PAUSE_STEPS)
        self._log_diag(
            "ARM_GUARD_RECOVER",
            idx=self.current_target_idx,
            link=arm_guard_link,
            clearance=float(arm_clearance),
            base_z=float(self.base_position[2]),
            strikes=int(self._arm_guard_strikes),
        )
        return True

    def _set_run_state(self, new_state, message=""):
        self.run_state = new_state
        self.state_step_count = 0
        self.z_vel = 0.0
        self.filtered_contact_force = 0.0
        self.stable_contact_steps = 0
        self.high_force_pause_steps = 0
        self._bad_contact_steps = 0
        self._bad_contact_last_idx = -1
        self._arm_guard_strikes = 0
        if new_state in (STATE_APPROACH, STATE_RETRACT):
            self.z_offset = PRESS_OFFSET_MAX
        if message:
            print(f"[Rail {self.label}] {message}", flush=True)
        self._log_diag("STATE", to=new_state, msg=message.replace(" ", "_") if message else "")

    def _footprint_count(self, pos):
        """패드 발자국 반경 내 점군 점 개수 (유리/구멍 판정용)."""
        return len(self.kdtree.query_ball_point(np.asarray(pos), SURFACE_FOOTPRINT_RADIUS))

    def _orient_normal(self, normal, target_pos=None):
        """모드별 법선 보정: 측면=수평 바깥(±X), 오버헤드=윗면+앞/뒤면, 그 외=수직."""
        if self.is_side:
            return clamp_normal_horizontal(normal, self.outward_sign)
        if self.is_overhead:
            return clamp_normal_overhead(normal, target_pos, self.car_center_y)
        return clamp_normal_tilt(normal)

    def _is_overhead_front_rear_normal(self, normal):
        """천장 로봇이 앞/뒤 수직면을 따라 닦는 자세인지 판정."""
        if not self.is_overhead:
            return False
        n = np.asarray(normal, dtype=float)
        n /= (np.linalg.norm(n) + 1e-9)
        tilt_deg = np.degrees(np.arccos(np.clip(n[2], -1.0, 1.0)))
        return bool(
            tilt_deg >= FRONT_REAR_MIN_TILT and
            abs(float(n[1])) >= FRONT_REAR_NORMAL_MIN_DOT and
            abs(float(n[1])) >= abs(float(n[0])) + 0.12
        )

    @staticmethod
    def _rotation_between_vectors(src, dst):
        src = np.asarray(src, dtype=float)
        dst = np.asarray(dst, dtype=float)
        src /= (np.linalg.norm(src) + 1e-9)
        dst /= (np.linalg.norm(dst) + 1e-9)
        dot = float(np.clip(np.dot(src, dst), -1.0, 1.0))
        if dot > 0.9998:
            return R.identity()
        if dot < -0.9998:
            axis = np.cross(src, np.array([1.0, 0.0, 0.0]))
            if np.linalg.norm(axis) < 1e-6:
                axis = np.cross(src, np.array([0.0, 1.0, 0.0]))
            axis /= (np.linalg.norm(axis) + 1e-9)
            return R.from_rotvec(np.pi * axis)
        axis = np.cross(src, dst)
        axis /= (np.linalg.norm(axis) + 1e-9)
        return R.from_rotvec(np.arccos(dot) * axis)

    def _ee_orientation(self, normal):
        """EE 목표 쿼터니언 [w,x,y,z].

        기본 자세를 만든 뒤, 실제 패드 접촉면(local -Y로 보정한 축)이 표면 안쪽(-normal)을
        향하도록 한 번 더 보정한다. 한쪽 측면 로봇이 비스듬히 닿던 문제가 여기서 잡힌다.
        """
        q = np.array(z_align_quat(-normal, self.approach_dir))
        base = R.from_quat([q[1], q[2], q[3], q[0]])
        desired_axis = -np.asarray(normal, dtype=float)
        desired_axis /= (np.linalg.norm(desired_axis) + 1e-9)
        current_axis = base.apply(self.pad_contact_axis_local)
        base = self._rotation_between_vectors(current_axis, desired_axis) * base
        if self.is_side and self.outward_sign in SIDE_EE_FLIP_SIGNS:
            flip = R.from_rotvec(np.pi * base.apply([0.0, 1.0, 0.0]))  # EE-Y축(접선) 기준 180° → 손등↔패드면 앞뒤 반전
            base = flip * base
        nq = base.as_quat()  # [x,y,z,w]
        q = np.array([nq[3], nq[0], nq[1], nq[2]])
        return q

    def _log_diag(self, tag, **kv):
        """status_log.txt에 key=value 형태로 진단 한 줄 기록(내가 읽고 튜닝)."""
        if not DIAG_LOG_ENABLED:
            return
        parts = [f"t={self.step_count}", f"rail={self.label}", f"seg={self.current_seg_idx}",
                 f"state={self.run_state}", f"tag={tag}"]
        for k, v in kv.items():
            parts.append(f"{k}={v:.4f}" if isinstance(v, float) else f"{k}={v}")
        try:
            with open(self.status_log, "a") as f:
                f.write(" ".join(parts) + "\n")
        except Exception:
            pass

    def slide_to_stop(self, seg_idx):
        """다음 슬라이딩 목표를 설정. 실제 이동은 STATE_SLIDE에서 매 스텝 수행."""
        y_stop, z_stop, path = self.segments[seg_idx]
        if self.is_side:
            self.rail_x = float(self.nominal_rail_x)
            self.base_position[0] = self.rail_x
        self.current_seg_idx = seg_idx
        self.slide_target_y = y_stop
        self.slide_target_z = z_stop

        # 현재 베이스 위치에서 경로의 시작/끝 중 가까운 쪽부터 폴리싱 시작 (한붓그리기 연속성)
        if len(path) > 1:
            curr = self.base_position  # 현재 레일 (X, Y, Z)
            d_first = float(np.linalg.norm(path[0]  - curr))
            d_last  = float(np.linalg.norm(path[-1] - curr))
            if d_last < d_first:
                path = path[::-1].copy()
                print(f"[Rail {self.label}] 구간 {seg_idx} 경로 역방향 시작 (끝점이 {d_last:.2f}m로 더 가까움)")

        self.path = path
        self.current_target_idx = 0
        self.current_path_idx_float = 0.0
        self._stuck_last_path_idx = 0.0
        self._stuck_steps_since_check = 0
        self._bad_contact_steps = 0
        self._bad_contact_last_idx = -1
        self.path_complete_reported = False
        self.previous_normal = None
        print(f"[Rail {self.label}] 슬라이딩 목표 → 구간 {seg_idx} (Y={y_stop:.2f}, Z={z_stop:.2f}), {len(path)}개 웨이포인트")

    def _find_nearest_seg(self):
        """현재 레일 (Y,Z) 위치에서 가장 가까운 미완료 구간 인덱스 반환.
        재폴리싱 패스에서는 샘플 점이 모두 이미 폴리싱된 구간을 건너뜀."""
        cy, cz = float(self.base_position[1]), float(self.base_position[2])
        best_i, best_d = None, float('inf')
        for i, (y, z, path) in enumerate(self.segments):
            if i in self._completed_segs:
                continue
            if len(path) == 0:
                self._completed_segs.add(i)
                continue
            if self._repolish_pass and self.polish_viz is not None:
                stride = max(1, len(path) // 5)
                if all(self.polish_viz.is_polished(p) for p in path[::stride]):
                    self._completed_segs.add(i)
                    continue
            d = float(np.hypot(y - cy, z - cz))
            if d < best_d:
                best_d, best_i = d, i
        return best_i

    def rearm_next_pass(self, new_segments=None):
        """재폴리싱 패스 시작. new_segments가 주어지면 구간 경로를 교체(빨간 점 재생성분).
        가장 가까운 미완료 구간부터 시작한다."""
        if new_segments is not None:
            self.segments = new_segments
        if not self.segments:
            self.done = True
            return
        self._repolish_pass = True
        self._completed_segs.clear()
        self._pass_rearm_attempted = False  # 다음 DONE 진입 시 재시도 허용
        self._consecutive_evades = 0
        self._glass_skip_count = 0
        self.evade_attempts = 0
        self._elbow_collision_count = 0
        self.z_offset = PRESS_OFFSET_MAX
        self.z_vel = 0.0
        self.filtered_contact_force = 0.0
        self.stable_contact_steps = 0
        self.high_force_pause_steps = 0
        self._bad_contact_steps = 0
        self._bad_contact_last_idx = -1
        self.done = False
        first_seg = self._find_nearest_seg()
        if first_seg is None:
            self.done = True
            return
        self.slide_to_stop(first_seg)
        self._set_run_state(STATE_SLIDE, f"재폴리싱 패스 — 구간 {first_seg} 슬라이딩 시작.")

    def _regen_segs_from_red(self):
        """현재 polish_viz 빨간 점만으로 각 구간 경로를 재생성하여 반환."""
        if self.polish_viz is None:
            return []
        if _SCRIPT_DIR not in sys.path:
            sys.path.insert(0, _SCRIPT_DIR)
        from path_generator import generate_3d_raster_path, generate_side_raster_path

        red_pts = self.polish_viz.unpolished_positions()
        if len(red_pts) == 0:
            print(f"[Rail {self.label}] 빨간 점 없음 — 재폴리싱 스킵")
            return []

        # 측면 로봇은 세밀한 step 사용(참조코드 기준 0.04~0.05m)
        side_step, side_res = 0.05, 0.02
        top_step,  top_res  = 0.09, 0.025

        new_segments = []
        for y_stop, z_stop, _old_path in self.segments:
            base_at_stop = np.array([self.rail_x, y_stop, z_stop])
            dists = np.linalg.norm(red_pts - base_at_stop, axis=1)
            reachable = red_pts[(dists >= PATH_MIN_RADIUS) & (dists <= PATH_MAX_RADIUS)]
            if len(reachable) < 3:
                continue
            if self.is_side:
                new_raw = generate_side_raster_path(reachable, step=side_step, resolution=side_res)
            else:
                new_raw = generate_3d_raster_path(reachable, step=top_step, resolution=top_res)
            if len(new_raw) == 0:
                continue
            new_path = filter_safe_waypoints(
                new_raw, self.raw_points, self.kdtree, base_at_stop,
                is_side=self.is_side, is_overhead=self.is_overhead,
            )
            if len(new_path) == 0:
                continue
            new_segments.append((y_stop, z_stop, new_path))
            print(f"[Rail {self.label}] 구간 Y={y_stop:.2f}: "
                  f"{len(reachable)}개 빨간 점 → {len(new_path)}개 웨이포인트")
        total_wp = sum(len(p) for _, _, p in new_segments)
        print(f"[Rail {self.label}] 재생성 완료: {len(new_segments)}구간, 총 {total_wp}개 웨이포인트")
        return new_segments

    def _try_start_next_pass(self):
        """완료 시 다음 독립 재폴리싱 패스를 시도. 시작하면 True, 완전히 끝났으면 False."""
        if self._polish_pass >= self._max_passes:
            return False
        new_segs = self._regen_segs_from_red()
        total_wp = sum(len(p) for _, _, p in new_segs)
        if total_wp < 5:
            print(f"[Rail {self.label}] 재폴리싱 웨이포인트 {total_wp}개 미만 — 수렴으로 판단")
            return False
        self._polish_pass += 1
        print(f"[Rail {self.label}] ↻ 독립 재폴리싱 패스 {self._polish_pass} 시작 "
              f"({len(new_segs)}구간, {total_wp}개 웨이포인트)")
        self.rearm_next_pass(new_segments=new_segs)
        return True

    def _move_rail_step(self, stage):
        """현재 위치에서 목표(Y, Z)로 한 스텝 이동. 도착하면 True 반환."""
        from pxr import UsdGeom, Gf
        current_y = float(self.base_position[1])
        current_z = float(self.base_position[2])
        dy = self.slide_target_y - current_y
        dz = self.slide_target_z - current_z
        dist = np.hypot(dy, dz)
        
        if dist <= RAIL_SLIDE_ARRIVAL_TOL:
            new_y = self.slide_target_y
            new_z = self.slide_target_z
            arrived = True
        else:
            step = min(RAIL_SLIDE_SPEED, dist)
            new_y = current_y + (dy / dist) * step
            new_z = current_z + (dz / dist) * step
            arrived = False

        self.base_position = np.array([self.rail_x, new_y, new_z])
        
        self._update_column_visuals(stage, new_y, new_z)
        return arrived

    @property
    def tele_lift_path(self):
        return f"/World/TeleLift_{self.label}"

    def _tele_lift_anchor_z(self):
        """리프트 고정단 Z: 측면=바닥, 천장=갠트리 빔(여기 매달려 아래로 신축)."""
        return GANTRY_BEAM_Z if self.is_overhead else TELE_LIFT_BASE_Z

    def _setup_tele_lift(self, stage):
        """Vention 텔레스코픽 리프트 USD를 받침대로 로드.
        에셋이 Y-up·cm 단위라 X축 회전+0.01 스케일로 보정. 측면=바닥서 위로,
        천장=빔에 거꾸로 매달려 아래로 신축(회전 부호 반대)."""
        rot_deg = TELE_LIFT_OVERHEAD_ROT_DEG if self.is_overhead else TELE_LIFT_UP_ROT_DEG
        anchor_z = self._tele_lift_anchor_z()
        q = R.from_euler("x", np.radians(rot_deg)).as_quat()  # [x,y,z,w]
        orient = np.array([q[3], q[0], q[1], q[2]])                        # [w,x,y,z]
        # 같은 USD를 여러 로봇이 reference하면 USD 인스턴스 공유로 2번째가 누락되거나
        # 튜브 신축이 서로 간섭함 → 로봇별 사본 파일을 reference한다(원천 차단).
        import shutil
        per_label_usd = os.path.join(
            os.path.dirname(TELE_LIFT_USD_PATH), f"tele_lift_{self.label}.usd"
        )
        if not os.path.exists(per_label_usd):
            shutil.copy(TELE_LIFT_USD_PATH, per_label_usd)
            # 단위(cm)·업축(Y) 메타데이터를 스테이지(m, Z-up)에 맞춰 패치 → MetricsAssembler
            # 자동 단위보정 비활성화. (자동보정이 로봇마다 들쭉날쭉 적용돼 SR이 ×0.01 이중
            # 스케일로 100배 작아져 사라졌음. 변환은 아래 수동 scale/rotation으로만 처리.)
            try:
                from pxr import Usd, UsdGeom
                _st = Usd.Stage.Open(per_label_usd)
                UsdGeom.SetStageMetersPerUnit(_st, 1.0)
                UsdGeom.SetStageUpAxis(_st, UsdGeom.Tokens.z)
                _st.GetRootLayer().Save()
            except Exception as _e:
                print(f"[Rail {self.label}] tele_lift 메타 패치 실패: {_e}", flush=True)
        create_prim(
            prim_path=self.tele_lift_path,
            prim_type="Xform",
            position=np.array([self.rail_x, self.base_position[1], anchor_z]),
            orientation=orient,
            scale=np.array([TELE_LIFT_UNIT_SCALE] * 3),
            usd_path=per_label_usd,
        )
        # 생성 검증 (SR 등 2번째 reference가 누락되는지 진단)
        lift_prim = stage.GetPrimAtPath(self.tele_lift_path)
        if not lift_prim or not lift_prim.IsValid():
            print(f"[Rail {self.label}] ⚠ tele_lift prim 생성 실패: {self.tele_lift_path}", flush=True)
            return
        # 같은 USD를 여러 로봇이 reference → 인스턴스화되면 내부 튜브 prim을 못 움직이므로 해제
        try:
            from pxr import Usd
            for p in Usd.PrimRange(lift_prim):
                if p.IsInstance():
                    p.SetInstanceable(False)
        except Exception:
            pass
        # 시각 전용 — 혹시 들어있을 콜라이더 비활성화
        set_collision_enabled_recursive(stage, self.tele_lift_path, False)
        # 너무 검은 머티리얼만 회색으로 보정(원본 USD 불변, 런타임 오버라이드)
        if TELE_LIFT_RECOLOR:
            recolor_tele_lift_dark(stage, self.tele_lift_path)
        # 초기 신축 상태 적용
        self._update_tele_lift(stage, self.base_position[1], self.base_position[2])
        n26 = stage.GetPrimAtPath(f"{self.tele_lift_path}/{TELE_LIFT_STAGE1_PRIM}").IsValid()
        n27 = stage.GetPrimAtPath(f"{self.tele_lift_path}/{TELE_LIFT_STAGE2_PRIM}").IsValid()
        print(f"[Rail {self.label}] tele_lift 로드 OK: {self.tele_lift_path} "
              f"(base_z={self.base_position[2]:.2f}, 튜브 NAUO26={n26} NAUO27={n27})", flush=True)

    def _set_tube_lift(self, stage, rel_prim, lift_m):
        """신축 튜브 prim을 에셋 로컬 +Y(=세운 뒤 월드 +Z)로 lift_m 만큼 올린다(native cm)."""
        from pxr import Gf
        prim = stage.GetPrimAtPath(f"{self.tele_lift_path}/{rel_prim}")
        if not prim or not prim.IsValid():
            return
        lift_cm = float(lift_m) / TELE_LIFT_UNIT_SCALE     # m → native cm
        attr = prim.GetAttribute("xformOp:translate")
        if not attr:
            from pxr import UsdGeom
            UsdGeom.XformCommonAPI(prim).SetTranslate(Gf.Vec3d(0.0, lift_cm, 0.0))
            return
        if "float" in str(attr.GetTypeName()).lower():
            attr.Set(Gf.Vec3f(0.0, lift_cm, 0.0))
        else:
            attr.Set(Gf.Vec3d(0.0, lift_cm, 0.0))

    def _update_tele_lift(self, stage, new_y, new_z):
        """고정단(측면=바닥/천장=빔)은 그 높이에 고정(Y만 레일 추종),
        2단 튜브를 고정단↔로봇 베이스 거리에 맞춰 신축."""
        from omni.isaac.core.prims import XFormPrim
        anchor_z = self._tele_lift_anchor_z()
        XFormPrim(prim_path=self.tele_lift_path).set_world_pose(
            position=np.array([self.rail_x, new_y, anchor_z]))
        # 필요한 컬럼 길이(고정단↔로봇 베이스 거리)를 실제 신축 한계로 클램프
        col_h = float(np.clip(abs(new_z - anchor_z),
                              TELE_LIFT_RETRACTED_H, TELE_LIFT_EXTENDED_H))
        ext = col_h - TELE_LIFT_RETRACTED_H                 # 0 .. 0.87 m
        # 2단 균등 분배: 1단 ext/2, 2단(최상단) ext
        self._set_tube_lift(stage, TELE_LIFT_STAGE1_PRIM, 0.5 * ext)
        self._set_tube_lift(stage, TELE_LIFT_STAGE2_PRIM, ext)

    def _update_column_visuals(self, stage, new_y, new_z):
        from omni.isaac.core.prims import XFormPrim
        import numpy as np
        
        # 로봇 베이스 위치 업데이트 (공통)
        XFormPrim(prim_path=self.world_prim_path).set_world_pose(position=np.array([self.rail_x, new_y, new_z]))

        if self.is_overhead:
            self._update_gantry_visuals(new_y, new_z)
            if USE_TELE_LIFT_ASSET:
                self._update_tele_lift(stage, new_y, new_z)
            return

        # 하단 슬라이더 위치 업데이트 (Z=0.05)
        XFormPrim(prim_path=f"/World/RailSlider_{self.label}").set_world_pose(position=np.array([self.rail_x, new_y, 0.05]))

        if self.is_side and USE_TELE_LIFT_ASSET:
            self._update_tele_lift(stage, new_y, new_z)
            return
        if self.is_side:
            # (구버전) 측면 단상: 바닥(0.10)에서 베이스까지 자라는 단일 원통
            h = max(0.04, new_z - 0.10)
            col = XFormPrim(prim_path=f"/World/SideColumn_{self.label}")
            col.set_world_pose(position=np.array([self.rail_x, new_y, 0.10 + h / 2.0]))
            col.set_local_scale(np.array([1.0, 1.0, h]))
            return

        # 3단 텔레스코픽 기둥 동기식 업데이트 (옛 레일 모드)
        ext = max(0.0, new_z - 0.45)
        XFormPrim(prim_path=f"/World/TelescopicColumn_S1_{self.label}").set_world_pose(position=np.array([self.rail_x, new_y, 0.275]))
        XFormPrim(prim_path=f"/World/TelescopicColumn_S2_{self.label}").set_world_pose(position=np.array([self.rail_x, new_y, 0.275 + ext / 2.0]))
        XFormPrim(prim_path=f"/World/TelescopicColumn_S3_{self.label}").set_world_pose(position=np.array([self.rail_x, new_y, 0.275 + ext]))

    def _remove_sander_parts(self, stage, part_names):
        """지정 부품 제거. USD 인스턴스 내부 prim은 그대로는 못 지우므로,
        로봇 하위 모든 인스턴스를 먼저 non-instanceable로 풀고(중첩 반복) 제거한다."""
        from pxr import Usd, UsdGeom
        robot_root_prim = stage.GetPrimAtPath(self.robot_root_path)
        if not robot_root_prim.IsValid():
            return

        # 1) 로봇 하위 모든 인스턴스 de-instance (중첩 대응 반복)
        for _ in range(8):
            insts = [p for p in Usd.PrimRange(robot_root_prim) if p.IsInstance()]
            if not insts:
                break
            for p in insts:
                p.SetInstanceable(False)

        # 2) 대상 부품 비활성+숨김 (참조된 prim이라 RemovePrim은 무효 → override로 처리)
        #    ※ RemovePrim을 호출하면 방금 author한 active/visibility override를 지워버려
        #      prim이 다시 살아나므로 호출하지 않는다.
        targets = [p.GetPath() for p in Usd.PrimRange(robot_root_prim)
                   if p.GetName() in part_names]
        if not targets:
            print(f"[Rail {self.label}] ⚠ 부품 못 찾음: {part_names}", flush=True)
        for tpath in targets:
            prim = stage.GetPrimAtPath(tpath)
            if prim and prim.IsValid():
                UsdGeom.Imageable(prim).MakeInvisible()
                set_collision_enabled_recursive(stage, tpath.pathString, False)
                prim.SetActive(False)
                print(f"[Rail {self.label}] 부품 숨김+비활성: {tpath} (active={prim.IsActive()})", flush=True)

    def _setup_gantry_visuals(self, world):
        """오버헤드 갠트리 포털 + 캐리지 + 수직 Z슬라이더 생성 (1회)."""
        L = self.label
        beam_z = GANTRY_BEAM_Z
        # 기둥: 바닥(z=0)에 닿게 + 가로빔/레일 '밑면'까지만 → 빔이 기둥 위에 얹히고
        #       기둥이 레일 위로 솟지 않음 (사용자 요청). 빔/레일 z(beam_z)는 로봇 마운트라 불변.
        post_bottom = 0.0
        post_top = beam_z - 0.06          # 가로빔(두께 0.12) 밑면
        post_h = post_top - post_bottom
        post_cz = 0.5 * (post_bottom + post_top)

        # (베이스 테이블 큰 박스는 제거 — 사용자 요청)
        # 4개 수직 기둥 (모서리)
        for i, (sx, sy) in enumerate([(-1, -1), (-1, 1), (1, -1), (1, 1)]):
            world.scene.add(VisualCylinder(
                prim_path=f"/World/GantryPost_{L}_{i}", name=f"gantry_post_{L}_{i}",
                position=np.array([sx * GANTRY_HALF_X, sy * GANTRY_HALF_Y, post_cz]),
                radius=0.06, height=post_h,
                color=np.array([0.30, 0.45, 0.60]),
            ))
        # 상단 가로 빔 (양 끝, X방향) — 포털 연결
        for i, sy in enumerate([-1, 1]):
            world.scene.add(VisualCuboid(
                prim_path=f"/World/GantryCross_{L}_{i}", name=f"gantry_cross_{L}_{i}",
                position=np.array([0.0, sy * GANTRY_HALF_Y, beam_z]),
                scale=np.array([2.0 * GANTRY_HALF_X, 0.12, 0.12]),
                color=np.array([0.30, 0.45, 0.60]),
            ))
        # 길이방향 레일 빔 (Y방향, X=rail_x) — 캐리지가 탐
        world.scene.add(VisualCuboid(
            prim_path=f"/World/GantryRail_{L}", name=f"gantry_rail_{L}",
            position=np.array([self.rail_x, 0.0, beam_z]),
            scale=np.array([0.14, 2.0 * GANTRY_HALF_Y, 0.10]),
            color=np.array([0.22, 0.22, 0.25]),
        ))
        # 캐리지 (레일 위를 Y로 슬라이딩)
        world.scene.add(VisualCuboid(
            prim_path=f"/World/GantryCarriage_{L}", name=f"gantry_carriage_{L}",
            position=np.array([self.rail_x, self.base_position[1], beam_z - 0.06]),
            scale=np.array([0.22, 0.22, 0.10]),
            color=np.array([0.85, 0.55, 0.10]),
        ))
        # 수직 Z 슬라이더 (캐리지 → 로봇 베이스, '단상 Z 움직임'의 시각 대응)
        # tele_lift 사용 시엔 그 기둥이 대신하므로 큐브 슬라이더는 생성하지 않음.
        if not USE_TELE_LIFT_ASSET:
            base_z = float(self.base_position[2])
            col_h = max(0.02, beam_z - base_z)
            world.scene.add(VisualCuboid(
                prim_path=f"/World/GantryZSlider_{L}", name=f"gantry_zslider_{L}",
                position=np.array([self.rail_x, self.base_position[1], 0.5 * (beam_z + base_z)]),
                scale=np.array([0.12, 0.12, col_h]),
                color=np.array([0.45, 0.60, 0.75]),
            ))

    def _update_gantry_visuals(self, new_y, new_z):
        """캐리지 Y이동 + 수직 Z슬라이더 신축 (매 스텝)."""
        from omni.isaac.core.prims import XFormPrim
        L = self.label
        beam_z = GANTRY_BEAM_Z
        XFormPrim(prim_path=f"/World/GantryCarriage_{L}").set_world_pose(
            position=np.array([self.rail_x, new_y, beam_z - 0.06]))
        # tele_lift 사용 시엔 큐브 슬라이더가 없으므로 갱신 생략
        if not USE_TELE_LIFT_ASSET:
            col_h = max(0.02, beam_z - new_z)
            zsl = XFormPrim(prim_path=f"/World/GantryZSlider_{L}")
            zsl.set_world_pose(position=np.array([self.rail_x, new_y, 0.5 * (beam_z + new_z)]))
            zsl.set_local_scale(np.array([0.12, 0.12, col_h]))

    def _apply_transit_pose(self, stage):
        """오버헤드 이동/대기 중: 베이스를 Z_MAX로 올리고 EE를 차량 위 안전 높이로
        들어 아래로 향하게 → 위치 전환 시 팔이 차체로 파고드는 것 방지."""
        target_z = OVERHEAD_Z_MAX
        self.base_position[2] += float(np.clip(target_z - self.base_position[2], -0.012, 0.012))
        self._update_column_visuals(stage, self.base_position[1], self.base_position[2])
        safe_target = np.array([self.rail_x, self.base_position[1], self.transit_ee_z])
        self._apply_cartesian_target(safe_target, np.array([0.0, 0.0, 1.0]), 0.0)

    def _move_rail_step_overhead(self, stage):
        """오버헤드 슬라이딩: Y축만 이동(Z는 transit이 Z_MAX로 유지). 도착 시 True."""
        current_y = float(self.base_position[1])
        dy = self.slide_target_y - current_y
        if abs(dy) <= RAIL_SLIDE_ARRIVAL_TOL:
            self.base_position[1] = self.slide_target_y
            return True
        step = min(RAIL_SLIDE_SPEED, abs(dy))
        self.base_position[1] = current_y + np.sign(dy) * step
        return False

    def _track_surface_z(self, stage, target_pos):
        """목표 표면 높이에 맞춰 베이스를 Z축으로 승강(단상 Z 트래킹).
        오버헤드: 표면 위 standoff에 베이스 배치(위→아래). 레일: 표면 아래에서 승강."""
        if self.is_overhead:
            # Path generation already chooses a safe Z for each gantry Y stop. If we
            # chase every local roof height here, the base rises out of the M0609
            # reach sphere and the pad floats above the car even while the target is valid.
            target_z = float(np.clip(self.slide_target_z, OVERHEAD_Z_MIN, OVERHEAD_Z_MAX))
            zstep = 0.005   # 천천히 하강(짧은 슬라이더에서 working 높이까지)
            pass   # 가상 바닥을 표면 높이에 맞춰 이동 (비활성)
        elif self.is_side:
            # 단상 리프트: 표면보다 살짝 높은 자세로 팔을 펴서 링크 4/5가 차체 쪽으로 접히지 않게 한다.
            target_z = float(np.clip(target_pos[2] + SIDE_BASE_Z_OFFSET, SIDE_BASE_Z_MIN, SIDE_BASE_Z_MAX))
            zstep = 0.006   # 천천히 추적(RMPFlow가 충분히 계산하게)
        else:
            target_z = max(0.45, min(1.15, target_pos[2] - 0.15))
            zstep = 0.005
        dz = target_z - self.base_position[2]
        self.base_position[2] += np.clip(dz, -zstep, zstep)
        self._update_column_visuals(stage, self.base_position[1], self.base_position[2])

    def step(self, stage):
        """1 physics step 실행. 완료 시 self.done = True."""
        if self.done:
            return
        if not self.segments:
            self.done = True
            return

        dt = 1.0 / 60.0

        # ── HOME: 시작 자세 안정화 후 첫 구간 슬라이딩 목표 설정 ──
        if self.run_state == STATE_HOME:
            if self.is_overhead:
                self._apply_transit_pose(stage)
            else:
                self._apply_home_pose()
            self._apply_pad_spin_velocity(0.0)
            self.step_count += 1
            self.state_step_count += 1
            if self.state_step_count >= HOME_INITIAL_SETTLE_STEPS:
                self.slide_to_stop(0)
                self._set_run_state(STATE_SLIDE, f"구간 0 슬라이딩 시작 (Y={self.slide_target_y:.2f}).")
            return

        # ── SLIDE: 매 스텝 조금씩 이동 ──
        if self.run_state == STATE_SLIDE:
            self._apply_pad_spin_velocity(0.0)
            if self.is_overhead:
                # 팔을 차량 위로 든 채(transit) Y축만 이동 → 차체 충돌 방지
                arrived = self._move_rail_step_overhead(stage)
                self._apply_transit_pose(stage)
            else:
                self._apply_home_pose()
                arrived = self._move_rail_step(stage)
            self.step_count += 1
            self.state_step_count += 1
            if arrived:
                self._set_run_state(STATE_SETTLE, f"Y={self.slide_target_y:.2f} 도착. 물리 안정화 중.")
            return

        # ── SETTLE_SLIDE: 도착 후 물리 안정화 ──
        if self.run_state == STATE_SETTLE:
            if self.is_overhead:
                self._apply_transit_pose(stage)
            else:
                self._apply_home_pose()
            self._apply_pad_spin_velocity(0.0)
            self.step_count += 1
            self.state_step_count += 1
            if self.state_step_count >= SLIDE_SETTLE_STEPS:
                self._set_run_state(STATE_APPROACH, f"슬라이딩 안정화 완료. 어프로치 시작.")
            return

        # ── EVADE_COLLISION: 팔꿈치/툴 충돌 회피 — 베이스를 들어올려 빠져나온 뒤 해당 웨이포인트 스킵 ──
        if self.run_state == STATE_EVADE_COLLISION:
            if self.is_overhead:
                self.base_position[2] += float(np.clip(OVERHEAD_Z_MAX - self.base_position[2], -0.02, 0.02))
                self._update_column_visuals(stage, self.base_position[1], self.base_position[2])
                safe_target = np.array([self.rail_x, self.base_position[1], self.transit_ee_z])
                self._apply_cartesian_target(safe_target, np.array([0.0, 0.0, 1.0]), 0.0)
            else:
                self.base_position[2] = min(1.15, self.base_position[2] + 0.01)
                self._update_column_visuals(stage, self.base_position[1], self.base_position[2])
            self._apply_pad_spin_velocity(0.0)
            self.step_count += 1
            self.state_step_count += 1
            if self.state_step_count >= RETRACT_SETTLE_STEPS:
                pts = self.path
                # 충돌이 연속될수록(앞유리 같은 군집) 더 크게 건너뜀 → 무한 그라인딩 방지
                self._consecutive_evades += 1
                skip = min(12, self._consecutive_evades * 3)
                end_idx = min(len(pts), self.current_target_idx + skip)
                self.current_path_idx_float = float(end_idx)
                self.current_target_idx = end_idx
                self.evade_attempts = 0
                self.z_offset = self.press_start_offset
                self.z_vel = 0.0
                self._set_run_state(STATE_POLISH, f"회피 완료 → {skip}개 스킵 후 재개 (연속 {self._consecutive_evades}회)")
            return

        # ── TRANSIT: 다음 구간으로 홈 복귀 없이 레일+로봇 동시 연속 이동 ──
        if self.run_state == STATE_TRANSIT:
            if len(self.path) == 0:
                self.done = True
                return
            target_pos = self.path[0]
            normal, _ = estimate_surface_normal(self.kdtree, self.raw_points, target_pos)
            normal = self._orient_normal(normal, target_pos)
            if self.is_overhead:
                # 팔을 안전 높이로 유지하면서 Y축 슬라이딩
                arrived = self._move_rail_step_overhead(stage)
                self._apply_transit_pose(stage)
            else:
                # 측면/레일: _move_rail_step으로 (Y,Z) 동시 이동. _track_surface_z는 Z를
                # slide_target_z 반대로 당겨 발산시키므로 TRANSIT에서는 호출 안 함.
                arrived = self._move_rail_step(stage)
                self._apply_cartesian_target(target_pos, normal, SAFE_RETRACT_CLEARANCE)
            self._apply_pad_spin_velocity(0.0)
            self.step_count += 1
            self.state_step_count += 1
            if arrived:
                self._set_run_state(STATE_APPROACH, f"구간 {self.current_seg_idx} 이동 완료. 어프로치 시작.")
            return

        points = self.path
        if len(points) == 0:
            self.done = True
            return

        # ── APPROACH ──
        if self.run_state == STATE_APPROACH:
            target_pos = points[0]
            normal, _ = estimate_surface_normal(self.kdtree, self.raw_points, target_pos)
            normal = self._orient_normal(normal, target_pos)
            self.previous_normal = normal
            
            # 동적 Z축 추적 (목표물 높이에 맞춰 기둥 승강)
            self._track_surface_z(stage, target_pos)

            # 오버헤드: 목표점 '바로 위'에서 수직 하강(차체 관통 방지).
            # transit 높이(차 위)에서 시작해 SAFE_APPROACH_CLEARANCE까지 clearance를 램프 →
            # EE가 법선(≈수직) 방향으로만 내려와 팔이 차체를 가로질러 파고드는 일이 없음.
            if self.is_overhead:
                big_clr = max(SAFE_APPROACH_CLEARANCE, self.transit_ee_z - float(target_pos[2]))
                frac = min(1.0, self.state_step_count / max(1, APPROACH_SETTLE_STEPS))
                clearance = big_clr * (1.0 - frac) + SAFE_APPROACH_CLEARANCE * frac
            elif self.is_side:
                # 측면: 바깥 멀리서 수평으로 천천히 접근(법선 방향) → 차체 관통 방지
                frac = min(1.0, self.state_step_count / max(1, APPROACH_SETTLE_STEPS))
                clearance = (
                    SIDE_APPROACH_FAR_CLEARANCE * (1.0 - frac) +
                    SIDE_APPROACH_FINAL_CLEARANCE * frac
                )
            else:
                clearance = SAFE_APPROACH_CLEARANCE
            self._apply_cartesian_target(target_pos, normal, clearance)
            self._apply_pad_spin_velocity(0.0)
            self.step_count += 1
            self.state_step_count += 1
            if self.state_step_count >= APPROACH_SETTLE_STEPS:
                self.z_offset = self.press_start_offset
                self._set_run_state(STATE_POLISH, "Approach 완료. 폴리싱 시작.")
            return

        # ── 경로 완료 후 RETRACT / RETURN_HOME ──
        if self.current_target_idx >= len(points) and len(points) > 0:
            if self.run_state == STATE_POLISH and not self.path_complete_reported:
                self.path_complete_reported = True
                self._set_run_state(STATE_RETRACT, f"구간 {self.current_seg_idx} 경로 완료. 리트랙트 중.")

            if self.run_state == STATE_RETRACT:
                if self.is_overhead:
                    # 패드를 차량 위 안전 높이로 즉시 들어올림 (차체 충돌 방지)
                    self._apply_transit_pose(stage)
                else:
                    target_pos = points[-1]
                    normal, _ = estimate_surface_normal(self.kdtree, self.raw_points, target_pos)
                    normal = self._orient_normal(normal, target_pos)
                    # 동적 Z축 추적
                    self._track_surface_z(stage, target_pos)
                    self._apply_cartesian_target(target_pos, normal, SAFE_RETRACT_CLEARANCE)
                self._apply_pad_spin_velocity(0.0)
                self.step_count += 1
                self.state_step_count += 1
                if self.state_step_count >= RETRACT_SETTLE_STEPS:
                    self._completed_segs.add(self.current_seg_idx)
                    next_seg = self._find_nearest_seg()
                    if next_seg is not None:
                        self.slide_to_stop(next_seg)
                        self._set_run_state(STATE_TRANSIT, f"구간 {next_seg} 연속 이동 시작 (홈 복귀 없음).")
                    else:
                        self._set_run_state(STATE_DONE, "모든 구간 폴리싱 완료.")
                        # done 설정은 STATE_DONE 핸들러에서 _try_start_next_pass 호출 후 결정
                return

            if self.run_state == STATE_RETURN_HOME:
                if self.is_overhead:
                    self._apply_transit_pose(stage)
                else:
                    self._apply_home_pose()
                self._apply_pad_spin_velocity(0.0)
                self.step_count += 1
                self.state_step_count += 1
                if self.state_step_count >= RETURN_HOME_SETTLE_STEPS:
                    next_seg = self.current_seg_idx + 1
                    if next_seg < len(self.segments):
                        self.slide_to_stop(next_seg)
                        self._set_run_state(STATE_SLIDE, f"구간 {next_seg} 슬라이딩 시작 (Y={self.slide_target_y:.2f}).")
                    else:
                        self._set_run_state(STATE_DONE, f"모든 구간 폴리싱 완료.")
                return

            if self.run_state == STATE_DONE:
                self._apply_home_pose()
                self._apply_pad_spin_velocity(0.0)
                if not self._pass_rearm_attempted:
                    self._pass_rearm_attempted = True
                    if self._try_start_next_pass():
                        return  # 새 패스 시작 — state가 SLIDE로 바뀜
                self.done = True
                return
            return

        # ── POLISH ──
        if self.run_state == STATE_POLISH and self.current_target_idx < len(points):
            # 이미 완료된 웨이포인트 빠르게 건너뜀 (다른 로봇이 먼저 폴리싱한 영역)
            if self.coverage_map is not None:
                while (self.current_target_idx < len(points) and
                       self.coverage_map.is_covered(points[self.current_target_idx])):
                    self.current_target_idx += 1
                    self.current_path_idx_float = float(self.current_target_idx)
                if self.current_target_idx >= len(points):
                    return  # 이 구간 전부 커버됨 → 상단 완료 로직에서 처리

            # 재폴리싱 패스: 이미 색칠(폴리싱)된 점은 건너뛰고 빨간(미완료) 점만 누름
            if self._repolish_pass and self.polish_viz is not None:
                while (self.current_target_idx < len(points) and
                       self.polish_viz.is_polished(points[self.current_target_idx])):
                    self.current_target_idx += 1
                    self.current_path_idx_float = float(self.current_target_idx)
                if self.current_target_idx >= len(points):
                    return

            # 선형 보간으로 목표 위치 계산
            if (self.current_target_idx + 1 < len(points) and
                    self.current_path_idx_float > self.current_target_idx):
                progress = self.current_path_idx_float - self.current_target_idx
                target_pos = (points[self.current_target_idx] * (1.0 - progress) +
                              points[self.current_target_idx + 1] * progress)
            else:
                target_pos = points[self.current_target_idx]

            # ── 유리/구멍 회피: 패드 발자국 아래 점군이 부족하면(유리=점 없음) 스킵 ──
            fp_count = self._footprint_count(target_pos)
            if fp_count < self.footprint_min:
                self._glass_skip_count += 1
                self._log_diag("GLASS_HOLE_SKIP", fp=fp_count, ref=self.footprint_ref,
                               idx=self.current_target_idx, total=len(points),
                               x=float(target_pos[0]), y=float(target_pos[1]), z=float(target_pos[2]),
                               cum=self._glass_skip_count)
                self.current_path_idx_float = float(self.current_target_idx + 1)
                self.current_target_idx = int(self.current_path_idx_float)
                self.z_offset = self.press_start_offset
                self.z_vel = 0.0
                self.step_count += 1
                self.state_step_count += 1
                return

            # 법선 추정 및 스무딩
            normal, normal_tilt_deg = estimate_surface_normal(self.kdtree, self.raw_points, target_pos)
            normal = self._orient_normal(normal, target_pos)
            if self.previous_normal is not None:
                if np.dot(normal, self.previous_normal) < 0:
                    normal = -normal
                normal = NORMAL_SMOOTHING * self.previous_normal + (1.0 - NORMAL_SMOOTHING) * normal
                normal = normal / np.linalg.norm(normal)
            self.previous_normal = normal

            # 목표 자세 (측면 미러 뒤집힘 보정 포함)
            target_orientation = self._ee_orientation(normal)

            # ── 측면 디스크 접촉축 1회 측정 ── 실제 폴리싱 디스크 prim의 월드 축이
            # 표면 안쪽(-normal)과 얼마나 정렬되는지 출력 → 올바른 EE 자세 재설계용
            if self.is_side and not self._side_debug_done:
                try:
                    from pxr import UsdGeom
                    dprim = stage.GetPrimAtPath(self.pad_path)
                    M = UsdGeom.XformCache().GetLocalToWorldTransform(dprim)
                    def _ax(v):
                        d = M.TransformDir(v); a = np.array([d[0], d[1], d[2]], float)
                        return a / (np.linalg.norm(a) + 1e-9)
                    inn = -normal
                    dx, dy, dz = _ax((1, 0, 0)), _ax((0, 1, 0)), _ax((0, 0, 1))
                    trot = R.from_quat([
                        target_orientation[1], target_orientation[2],
                        target_orientation[3], target_orientation[0],
                    ])
                    pred_axis = trot.apply(self.pad_contact_axis_local)
                    msg = (f"[SIDE_DEBUG {self.label}] outward={self.outward_sign} "
                           f"diskX·(-n)={np.dot(dx, inn):+.2f} diskY·(-n)={np.dot(dy, inn):+.2f} "
                           f"diskZ·(-n)={np.dot(dz, inn):+.2f} predAxis·(-n)={np.dot(pred_axis, inn):+.2f} "
                           f"normal=({normal[0]:+.2f},{normal[1]:+.2f},{normal[2]:+.2f})")
                    print(msg, flush=True)
                    self._log_diag("SIDE_DEBUG", outw=self.outward_sign,
                                   dX=float(np.dot(dx, inn)), dY=float(np.dot(dy, inn)),
                                   dZ=float(np.dot(dz, inn)), pred=float(np.dot(pred_axis, inn)))
                except Exception as _e:
                    print(f"[SIDE_DEBUG {self.label}] 실패: {_e}", flush=True)
                self._side_debug_done = True

            # 동적 Z축 추적 (목표물 높이에 맞춰 기둥 승강)
            self._track_surface_z(stage, target_pos)

            # 앞/뒤 수직면은 오버헤드 로봇이 Y방향 법선을 유지해서 닦는다.
            overhead_front_rear = self._is_overhead_front_rear_normal(normal)

            # 가변 목표힘: 표면 곡률(tilt)에 따라 평탄=강/곡면=약
            self._target_force = adaptive_target_force(
                normal_tilt_deg,
                mode="side" if self.is_side else "top",
            )

            # 측면은 디스크가 더 일찍(zoff↑) 물리적으로 닿음 → 가상 접촉거리를 그에 맞춰
            # (안 맞추면 가상이 접촉을 못 보고 계속 밀어 N↑→원위치)
            _contact_dist = self._virtual_contact_distance()
            press_min = SIDE_PRESS_OFFSET_MIN if self.is_side else PRESS_OFFSET_MIN
            press_max = SIDE_PRESS_OFFSET_MAX if (self.is_side or overhead_front_rear) else PRESS_OFFSET_MAX
            cmd_clearance = self._pad_command_clearance(_contact_dist)
            target_pad_pos = target_pos + normal * cmd_clearance
            self._last_command_clearance = cmd_clearance
            self._last_target_pad_pos = target_pad_pos
            tip_gap_for_sensor, _ = self.kdtree.query(np.asarray(target_pad_pos))
            actual_pad_pos, actual_gap, actual_clearance = self._pad_contact_footprint_metrics(stage, normal)
            contact_clear_tol, contact_gap_tol = self._real_contact_tolerances()
            sensor_valid_gap = SIDE_SENSOR_VALID_GAP if self.is_side else TOP_SENSOR_VALID_GAP
            z_sensor_margin = SIDE_SENSOR_Z_MARGIN if self.is_side else TOP_SENSOR_Z_MARGIN
            soft_force_limit = PHYSICAL_FORCE_SOFT_LIMIT_SIDE_N if self.is_side else PHYSICAL_FORCE_SOFT_LIMIT_TOP_N
            hard_force_limit = PHYSICAL_FORCE_HARD_LIMIT_SIDE_N if self.is_side else PHYSICAL_FORCE_HARD_LIMIT_TOP_N
            bad_gap_mult = BAD_CONTACT_GAP_MULT_SIDE if self.is_side else BAD_CONTACT_GAP_MULT
            clear_mult = SIDE_CONTACT_CLEARANCE_MULT if self.is_side else BAD_CONTACT_GAP_MULT
            # ★실제 패드 위치 기반 가상 스프링 (명령값 z_offset이 아니라 실측 접촉면 거리 사용).
            # actual_clearance = 패드 접촉면이 표면 위로 떠 있는 실제 거리(법선방향, +위/−파묻힘).
            # 명령값 기준이면 안 닿아도 목표 N을 만들어 제어기를 속이지만, 실측 기준이면 정직함.
            if actual_clearance is not None:
                pad_compression = max(0.0, _contact_dist - float(actual_clearance))
            else:
                pad_compression = max(0.0, _contact_dist - self.z_offset)   # 폴백
            command_virtual_force = max(
                0.0,
                VIRTUAL_PAD_STIFFNESS * pad_compression - VIRTUAL_PAD_DAMPING * self.z_vel,
            )
            command_virtual_force = min(command_virtual_force, soft_force_limit)
            # 힘센서는 실제 패드-차체 접촉이 가능한 자세/거리에서만 인정한다.
            # 오버헤드도 색칠/완료 판정에는 반드시 유효한 센서 힘이 필요하다.
            sensor_model_gate = max(CONTACT_FORCE_THRESHOLD, 0.35 * self._target_force)

            # 접촉력 읽기
            contact_reading = self.contact_sensor.get_current_frame()
            sensor_measured_force = (
                np.linalg.norm(contact_reading["force"])
                if contact_reading and "force" in contact_reading else 0.0
            )
            if not USE_PHYSICAL_CONTACT_SENSOR:
                # 디스크 충돌 off 모드: 물리센서 reading은 잔여 아티팩트 → 무시(가상스프링만 사용)
                sensor_measured_force = 0.0
            raw_physical_contact = sensor_measured_force >= CONTACT_FORCE_THRESHOLD
            raw_contact_gap_ok = (
                actual_gap is None or
                actual_gap <= contact_gap_tol * bad_gap_mult
            )
            raw_contact_clear_ok = (
                actual_clearance is None or
                actual_clearance <= contact_clear_tol * clear_mult
            )
            # raw 센서는 손목/패드 주변 충돌에도 튀므로, 실제 패드 기준점이 표면
            # 근처일 때만 물리 접촉으로 인정한다.
            measured_physical_contact = (
                raw_physical_contact and raw_contact_gap_ok and raw_contact_clear_ok
            )
            side_gap_contact = (
                self.is_side and actual_gap is not None and actual_gap <= contact_gap_tol
            )
            actual_contact_close = (
                actual_clearance is None or
                actual_clearance <= contact_clear_tol or
                side_gap_contact or
                measured_physical_contact
            )
            actual_gap_close = (
                actual_gap is None or
                actual_gap <= contact_gap_tol or
                measured_physical_contact
            )
            virtual_force = command_virtual_force if (actual_contact_close and actual_gap_close) else 0.0
            sensor_force_is_valid = (
                float(tip_gap_for_sensor) <= sensor_valid_gap and
                (actual_gap is None or actual_gap <= sensor_valid_gap) and
                raw_contact_clear_ok and
                actual_contact_close and
                self.z_offset <= (_contact_dist + z_sensor_margin) and
                command_virtual_force >= sensor_model_gate
            )
            sensor_raw_force = (
                min(sensor_measured_force, hard_force_limit)
                if sensor_force_is_valid else 0.0
            )
            is_physical_contacting = sensor_raw_force >= CONTACT_FORCE_THRESHOLD
            force_sensor_verified = bool(
                sensor_force_is_valid and sensor_raw_force >= CONTACT_FORCE_THRESHOLD
            )
            sensor_spike_guard = (
                sensor_measured_force >= REAL_ARM_COLLISION_N and
                raw_contact_gap_ok and
                raw_contact_clear_ok
            )
            soft_retract_gain = (
                PHYSICAL_FORCE_SOFT_RETRACT_GAIN_SIDE if self.is_side
                else PHYSICAL_FORCE_SOFT_RETRACT_GAIN_TOP
            )
            soft_retract_max_step = (
                PHYSICAL_FORCE_SOFT_RETRACT_MAX_STEP_SIDE if self.is_side
                else PHYSICAL_FORCE_SOFT_RETRACT_MAX_STEP_TOP
            )
            if USE_VIRTUAL_SOFT_PAD:
                # 실제 센서는 물리적으로 말이 되는 거리에서만 받아들이고,
                # 그 외에는 말랑 패드 압축모델(Newton)을 사용해 안정적으로 목표 힘에 수렴시킨다.
                raw_force = virtual_force
                geometry_contact_possible = (
                    is_physical_contacting or
                    virtual_force >= CONTACT_FORCE_THRESHOLD or
                    (cmd_clearance <= 0.002 and actual_contact_close and actual_gap_close)
                )
            else:
                geometry_contact_possible = self.z_offset <= CONTACT_GEOMETRY_MAX_OFFSET
                raw_force = sensor_raw_force if geometry_contact_possible else 0.0

            is_contacting = (
                raw_force >= CONTACT_FORCE_THRESHOLD or
                is_physical_contacting or
                (cmd_clearance <= 0.002 and actual_contact_close and actual_gap_close)
            )
            # 포인트 클라우드 색 변경은 "실제 폴리싱 접촉"일 때만 허용한다.
            # raw sensor/virtual force가 튀거나 waypoint를 스킵하는 경우는 폴리싱으로 보지 않는다.
            cmd_mark_clearance = SIDE_PRESS_OFFSET_MAX if overhead_front_rear else 0.0
            geometry_contact_verified = (
                actual_gap is not None and
                actual_clearance is not None and
                actual_gap <= min(contact_gap_tol, sensor_valid_gap) and
                SURFACE_GUARD_MIN_CLEARANCE <= actual_clearance <= contact_clear_tol and
                # 명령이 표면 안쪽(cmd_clearance≤0)이거나, 실제 패드가 표면에 충분히 가까우면(≤8mm) 인정.
                # 앞/뒤 수직면은 normal이 수평에 가까워 command clearance가 약간 양수로 남을 수 있어 별도 허용.
                # 측면은 기존 동작을 그대로 유지한다.
                (cmd_clearance <= cmd_mark_clearance or actual_gap <= 0.008) and
                pad_compression >= POLISH_MARK_MIN_COMPRESSION
            )
            # 충돌 off 모드: 물리센서(force_sensor_verified) 대신 실측위치 가상힘으로도 폴리싱 인정.
            polish_contact_verified = bool(
                (force_sensor_verified or virtual_force >= sensor_model_gate)
                and geometry_contact_verified
            )
            self.filtered_contact_force = (
                FORCE_FILTER_ALPHA * raw_force +
                (1.0 - FORCE_FILTER_ALPHA) * self.filtered_contact_force
            )

            arm_clearance, arm_guard_link, arm_guard_margin, arm_skip_clearance = self._arm_surface_clearance(stage)
            arm_guard_active = (
                arm_clearance is not None and
                arm_clearance < arm_guard_margin
            )
            if arm_guard_active:
                self._arm_guard_strikes += 1
                self.z_offset = min(press_max, self.z_offset + ARM_SURFACE_RETRACT_STEP)
                self.z_vel = max(0.0, self.z_vel)
                self.high_force_pause_steps = max(self.high_force_pause_steps, SPIKE_PAUSE_STEPS)
                geometry_contact_verified = False
                polish_contact_verified = False
                is_contacting = False
                self._log_diag(
                    "ARM_GUARD",
                    link=arm_guard_link,
                    clearance=float(arm_clearance),
                    margin=float(arm_guard_margin),
                    zoff=float(self.z_offset),
                )
                if arm_clearance < arm_skip_clearance:
                    if self._try_recover_from_arm_guard(stage, arm_guard_link, arm_clearance):
                        self.step_count += 1
                        self.state_step_count += 1
                        return
                    msg = (f"[Rail {self.label}] ⚠ 팔 링크 차체 접근 "
                           f"→ waypoint {self.current_target_idx} 스킵 "
                           f"(link={arm_guard_link}, clearance={float(arm_clearance):+.3f}m)")
                    print(msg, flush=True)
                    with open(self.status_log, "a") as f:
                        f.write(msg + "\n")
                    self._log_diag(
                        "ARM_GUARD_SKIP",
                        idx=self.current_target_idx,
                        link=arm_guard_link,
                        clearance=float(arm_clearance),
                        margin=float(arm_guard_margin),
                    )
                    self.current_path_idx_float = float(min(len(points), self.current_target_idx + 1))
                    self.current_target_idx = int(self.current_path_idx_float)
                    self.z_offset = self.press_start_offset
                    self.z_vel = 0.0
                    self.filtered_contact_force = 0.0
                    self.stable_contact_steps = 0
                    self.high_force_pause_steps = SPIKE_PAUSE_STEPS
                    self._arm_guard_strikes = 0
                    self._stuck_last_path_idx = float(self.current_path_idx_float)
                    self._stuck_steps_since_check = 0
                    self.step_count += 1
                    self.state_step_count += 1
                    return
            else:
                self._arm_guard_strikes = 0

            # 경로 시각화 업데이트
            if self.step_count % VIZ_UPDATE_INTERVAL_STEPS == 0:
                self._update_path_visualization(stage)

            # 스펀지 시각 업데이트 (균일 압축 + 곡면 변형)
            if self.step_count % SPONGE_VISUAL_UPDATE_INTERVAL_STEPS == 0 and self.pad_visual_path:
                visual_prim = stage.GetPrimAtPath(self.pad_visual_path)
                if visual_prim.IsValid():
                    physical_visual_compression = sensor_raw_force / 1200.0 if polish_contact_verified else 0.0
                    conformal_command_compression = pad_compression if polish_contact_verified else 0.0
                    target_visual_compression = float(np.clip(
                        max(conformal_command_compression, physical_visual_compression),
                        0.0,
                        POLISHING_VISUAL_MAX_COMPRESSION,
                    ))
                    self._visual_pad_compression = (
                        0.75 * self._visual_pad_compression +
                        0.25 * target_visual_compression
                    )
                    visual_compression = self._visual_pad_compression
                    pts, _, _, ext = build_polishing_disk_mesh(
                        POLISHING_DISK_RADIUS, POLISHING_DISK_HEIGHT, POLISHING_DISK_SIDES,
                        visual_compression=visual_compression,
                    )
                    from pxr import UsdGeom
                    # 말랑 패드 곡면 변형: 바닥면 정점을 그 아래 표면 곡률에 맞춰 휘게
                    if CONFORMAL_PAD_ENABLED and polish_contact_verified:
                        world_M = UsdGeom.XformCache().GetLocalToWorldTransform(visual_prim)
                        pts, self._last_conform_max = deform_disk_points_to_surface(
                            pts, POLISHING_DISK_SIDES, world_M, self.kdtree, self.raw_points,
                            normal, CONFORMAL_MAX_DEFORM,
                        )
                    else:
                        self._last_conform_max = 0.0
                    mesh = UsdGeom.Mesh(visual_prim)
                    mesh.GetPointsAttr().Set(pts)
                    mesh.GetExtentAttr().Set(ext)

            # ── 과압 처리 ──
            # 이제 로봇 본체 collision을 끄고 패드만 collision을 남겼으므로 센서 고값은
            # waypoint 스킵 사유가 아니라 "패드를 너무 눌렀다"는 피드백이다.
            # 스킵 대신 즉시 후퇴시키고 제어 루프가 목표 N으로 수렴하게 둔다.
            if sensor_spike_guard:
                overload = sensor_measured_force - REAL_ARM_COLLISION_N
                retract_step = min(
                    soft_retract_max_step,
                    soft_retract_gain * overload,
                )
                self.z_offset = min(press_max, self.z_offset + retract_step)
                self.z_vel = max(0.0, self.z_vel)
            if sensor_spike_guard and sensor_measured_force > REAL_ARM_COLLISION_N * 1.5:
                self.z_offset = min(press_max, self.z_offset + SPIKE_RETRACT_STEP)
                self.z_vel = 0.0
                self.high_force_pause_steps = SPIKE_PAUSE_STEPS
            if (
                actual_clearance is not None and
                actual_clearance < SURFACE_GUARD_MIN_CLEARANCE and
                (actual_gap is None or actual_gap <= contact_gap_tol * 1.5)
            ):
                penetration_recover = min(
                    soft_retract_max_step,
                    SURFACE_GUARD_MIN_CLEARANCE - actual_clearance + SURFACE_GUARD_RECOVER_CLEARANCE,
                )
                self.z_offset = min(press_max, self.z_offset + penetration_recover)
                self.z_vel = 0.0
                self.high_force_pause_steps = max(self.high_force_pause_steps, SPIKE_PAUSE_STEPS)
            # 실제 패드 위치가 표면에서 계속 멀면 더 누르는 대신 해당 점을 버린다.
            # 유리/창문/가장자리/점군 구멍에서 가상힘만 커지며 팔이 떠오르던 현상 방지.
            at_press_floor = (
                self.z_offset <= press_min + BAD_CONTACT_PRESS_EPS or
                cmd_clearance <= SURFACE_GUARD_MIN_CLEARANCE + BAD_CONTACT_CLEARANCE_EPS
            )
            bad_gap = (
                actual_gap is not None and
                actual_gap > contact_gap_tol * bad_gap_mult
            )
            confirmed_contact = (
                actual_contact_close and actual_gap_close and
                (virtual_force >= CONTACT_FORCE_THRESHOLD or is_physical_contacting)
            )
            if at_press_floor and bad_gap and not confirmed_contact:
                if self._bad_contact_last_idx != self.current_target_idx:
                    self._bad_contact_last_idx = self.current_target_idx
                    self._bad_contact_steps = 0
                self._bad_contact_steps += 1
            else:
                if self._bad_contact_last_idx == self.current_target_idx:
                    self._bad_contact_steps = max(0, self._bad_contact_steps - 1)
                else:
                    self._bad_contact_steps = 0
                    self._bad_contact_last_idx = self.current_target_idx

            bad_contact_limit = (
                BAD_CONTACT_SKIP_STEPS_SIDE if self.is_side else BAD_CONTACT_SKIP_STEPS_TOP
            )
            if self._bad_contact_steps >= bad_contact_limit:
                msg = (f"[Rail {self.label}] ⚠ 실제 접촉 실패 "
                       f"→ waypoint {self.current_target_idx} 스킵 "
                       f"(actual_gap={float(actual_gap):.3f}m, "
                       f"cmd_gap={cmd_clearance:+.3f}m, virtual={virtual_force:.1f}N)")
                print(msg, flush=True)
                with open(self.status_log, "a") as f:
                    f.write(msg + "\n")
                self._log_diag(
                    "BAD_CONTACT_SKIP",
                    idx=self.current_target_idx,
                    gap=float(actual_gap),
                    cmd=float(cmd_clearance),
                    zoff=float(self.z_offset),
                    virtual=float(virtual_force),
                    fp=fp_count,
                )
                self.current_path_idx_float = float(min(len(points), self.current_target_idx + 1))
                self.current_target_idx = int(self.current_path_idx_float)
                self.z_offset = self.press_start_offset
                self.z_vel = 0.0
                self.filtered_contact_force = 0.0
                self.stable_contact_steps = 0
                self.high_force_pause_steps = SPIKE_PAUSE_STEPS
                self._bad_contact_steps = 0
                self._bad_contact_last_idx = -1
                self._stuck_last_path_idx = float(self.current_path_idx_float)
                self._stuck_steps_since_check = 0
                self.step_count += 1
                self.state_step_count += 1
                return

            # ── Stuck 타임아웃 감지 ──
            # 일정 스텝 이상 path_idx가 변하지 않으면 강제 스킵
            self._stuck_steps_since_check += 1
            if self._stuck_steps_since_check >= self._stuck_check_interval:
                cur_progress = float(self.current_path_idx_float)
                if cur_progress <= self._stuck_last_path_idx + 0.20:
                    msg = (f"[Rail {self.label}] ⚠ {self._stuck_check_interval}스텝 정체 "
                           f"→ waypoint {self.current_target_idx} 강제 스킵 "
                           f"(sensor={sensor_raw_force:.1f}N, virtual={virtual_force:.2f}N)")
                    print(msg, flush=True)
                    with open(self.status_log, "a") as f:
                        f.write(msg + "\n")
                    self.current_path_idx_float = float(self.current_target_idx + 1)
                    self.current_target_idx = int(self.current_path_idx_float)
                    self.z_offset = self.press_start_offset
                    self.z_vel = 0.0
                self._stuck_last_path_idx = float(self.current_path_idx_float)
                self._stuck_steps_since_check = 0

            # 물리력 초과 대응 — 실모드 전용 백업. 가상모드에서는 위 과압 처리에서 이미 후퇴한다.
            if not USE_VIRTUAL_SOFT_PAD:
                if sensor_raw_force > soft_force_limit:
                    overload = sensor_raw_force - soft_force_limit
                    retract_step = min(soft_retract_max_step,
                                       soft_retract_gain * overload)
                    self.z_offset = min(press_max, self.z_offset + retract_step)
                    self.z_vel = max(0.0, self.z_vel)

                if raw_force > FORCE_SPIKE_N or sensor_raw_force > hard_force_limit:
                    self.z_offset = min(press_max, self.z_offset + SPIKE_RETRACT_STEP)
                    self.z_vel = 0.0
                    self.high_force_pause_steps = SPIKE_PAUSE_STEPS

            # ── 어드미턴스 힘제어 (polishing_v1 레시피 + 가상스프링 속임 제거) ──
            #  accel = (F_err − D·v)/M.  댐핑 D로 진동/슬램 억제.
            #  ★핵심 수정: '가상 스프링'은 명령값 기준이라 실제 패드가 안 닿아도 목표 N을
            #    만들어 제어기를 속여 압입을 멈추게 했음(=보이지 않는 스프링에 패드가 뜸).
            #  → 진짜 물리 센서가 잡힐 때만 그 실측 N으로 힘제어. 아직 안 닿았으면 control_force=0
            #    으로 둬서 '닿을 때까지 계속 압입'(seek). 실제로 닿는 순간 실측 N으로 전환.
            if force_sensor_verified or is_physical_contacting:
                control_force = min(float(sensor_measured_force), FORCE_CONTROL_CLIP_N)
            else:
                # 충돌 off 모드: 물리센서가 없으므로 '실측위치 기반 가상힘'으로 힘제어.
                # 가상힘이 실제 패드 거리로 계산되므로 더 이상 제어기를 속이지 않음(안 닿으면 0→압입).
                control_force = min(self.filtered_contact_force, FORCE_CONTROL_CLIP_N)
            force_error = control_force - self._target_force
            accel = (force_error - ADMITTANCE_DAMPING * self.z_vel) / ADMITTANCE_MASS
            self.z_vel += accel * dt
            self.z_vel = np.clip(self.z_vel, -ADMITTANCE_MAX_VEL, ADMITTANCE_MAX_VEL)
            self.z_offset += self.z_vel * dt
            self.z_offset = np.clip(self.z_offset, press_min, press_max)

            # RMPFlow 목표 전송: 실제 패드 접촉면은 표면에 붙이고,
            # 힘은 위의 가상 스프링 z_offset으로 안정화한다.
            cmd_clearance = self._pad_command_clearance(_contact_dist)
            target_pad_pos = target_pos + normal * cmd_clearance
            self._last_command_clearance = cmd_clearance
            self._last_target_pad_pos = target_pad_pos
            target_rot = R.from_quat([
                target_orientation[1], target_orientation[2],
                target_orientation[3], target_orientation[0],
            ])
            # ── 추종지연 피드포워드 ──
            # RMPFlow는 명령보다 패드를 덜 내린다(정상상태 추종지연 ~2cm). 그 지연을 추정해
            # link_6 명령을 그만큼 더 깊게(−normal) 보내 실제 패드가 표면까지 닿게 한다.
            # 이미 닿았으면(센서 유효/물리접촉) 더 키우지 않고 천천히 감쇠(과압 방지).
            if actual_clearance is not None:
                lag = float(actual_clearance) - cmd_clearance   # >0: 명령보다 패드가 위(덜 내려옴)
                # ★패드가 실제로 표면에 닿았을 때(actual_clearance≤2mm)만 보정을 감쇠한다.
                #   - virtual_force는 떠 있어도 댐핑항(−D·z_vel)만으로 유령 접촉력을 만들고,
                #   - pad_compression>0도 두꺼운 가상패드(1.8cm) 탓에 1.7cm 떠 있어도 토큰 압축이 생겨
                #   둘 다 lag 보정을 꺼버렸음 → 패드가 표면 위에 영원히 떠 있던 원인.
                #   실측 거리(actual_clearance)로 판정해야 진짜 닿을 때까지 lag를 계속 메운다.
                really_touching = (
                    force_sensor_verified or is_physical_contacting or
                    (actual_clearance is not None and float(actual_clearance) <= 0.002)
                )
                if really_touching:
                    self._lag_ff *= 0.98
                else:
                    self._lag_ff += LAG_FEEDFORWARD_GAIN * (max(0.0, lag) - self._lag_ff)
                self._lag_ff = float(np.clip(self._lag_ff, 0.0, LAG_FEEDFORWARD_MAX))
            ff_pad_pos = target_pos + normal * (cmd_clearance - self._lag_ff)
            link_6_target = ff_pad_pos - target_rot.apply(self.pad_contact_offset_local)
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

            # 연속 마킹: 접촉 중이면 디스크 발자국을 주기적으로 색칠 → 지나간 자리가 띠(swath)로 보임
            # (웨이포인트 완료 순간에만 칠하면 간헐 접촉을 놓쳐 듬성듬성해짐)
            if (self.polish_viz is not None and polish_contact_verified
                    and self.step_count % 4 == 0):
                self.polish_viz.mark(target_pad_pos)

            # 경로 진행
            prev_int_idx = int(self.current_path_idx_float)
            if self.high_force_pause_steps > 0:
                self.high_force_pause_steps -= 1
                self.current_path_idx_float += PATH_CREEP_ADVANCE_PER_STEP
            else:
                advance_ratio = 0.65 if self.is_overhead else 0.45
                advance_min = max(FORCE_ADVANCE_MIN_N, advance_ratio * self._target_force)
                advance_max = min(FORCE_ADVANCE_MAX_N, 1.85 * self._target_force)
                stable = (
                    polish_contact_verified and
                    geometry_contact_possible and
                    (
                        (USE_VIRTUAL_SOFT_PAD and advance_min <= self.filtered_contact_force <= advance_max) or
                        (USE_VIRTUAL_SOFT_PAD and advance_min <= virtual_force <= advance_max) or
                        (advance_min <= sensor_raw_force <= advance_max)
                    )
                )
                if stable:
                    self.stable_contact_steps += 1
                else:
                    self.stable_contact_steps = 0
                if self.stable_contact_steps >= CONTACT_SETTLE_STEPS:
                    step_advance = (
                        PATH_ADVANCE_PER_CONTACT_STEP_SIDE if self.is_side
                        else PATH_ADVANCE_PER_CONTACT_STEP_TOP
                    )
                    self.current_path_idx_float += step_advance
                else:
                    self.current_path_idx_float += PATH_CREEP_ADVANCE_PER_STEP

            self.current_target_idx = int(self.current_path_idx_float)

            # 정수 인덱스가 증가했을 때 = 웨이포인트 완료 → 커버리지 마킹
            if self.current_target_idx > prev_int_idx:
                self._consecutive_evades = 0   # 정상 진행 → 충돌 군집 카운터 리셋
                # 패드가 '실제로 눌렸을 때만' 하늘색 표시 (떠서 지나가거나 회피 스킵 구간은 칠 안 함)
                if (self.polish_viz is not None and prev_int_idx < len(points)
                        and polish_contact_verified):
                    self.polish_viz.mark(points[prev_int_idx])
            if self.coverage_map is not None and self.current_target_idx > prev_int_idx:
                if prev_int_idx < len(points) and polish_contact_verified:
                    self.coverage_map.mark(points[prev_int_idx])
                # 커버리지 시각화 (30스텝마다)
                if self.step_count % 30 == 0:
                    self._update_coverage_visualization(stage)
            self.step_count += 1
            self.state_step_count += 1

            # CSV 로그 + status_log 진단
            if self.step_count % STATUS_LOG_INTERVAL_STEPS == 0:
                with open(self.log_file, "a") as f:
                    f.write(
                        f"{self.step_count},{self.current_seg_idx},{sensor_raw_force:.3f},"
                        f"{sensor_measured_force:.3f},{virtual_force:.3f},{raw_force:.3f},"
                        f"{self.filtered_contact_force:.3f},{self.z_offset:.4f},"
                        f"{cmd_clearance:.4f},"
                        f"{float(actual_gap) if actual_gap is not None else -1.0:.4f},"
                        f"{float(actual_clearance) if actual_clearance is not None else -9.0:.4f},"
                        f"{self.current_path_idx_float:.1f},{self.run_state}\n"
                    )
                # 패드 끝(접촉 기준점)과 가장 가까운 점군 점 사이 거리 = "착 붙음" 잔여 gap
                tip_gap, _ = self.kdtree.query(np.asarray(target_pad_pos))
                self._log_diag(
                    "POLISH",
                    idx=self.current_target_idx, total=len(points),
                    sensor=float(sensor_raw_force), virtual=float(virtual_force),
                    filt=float(self.filtered_contact_force),
                    zoff=float(self.z_offset), comp=float(pad_compression),
                    cmd_clear=float(cmd_clearance),
                    base_z=float(self.base_position[2]),
                    tip_gap=float(tip_gap),
                    actual_gap=float(actual_gap) if actual_gap is not None else -1.0,
                    actual_clear=float(actual_clearance) if actual_clearance is not None else -9.0,
                    tilt=float(normal_tilt_deg),
                    fp=fp_count, conform=float(self._last_conform_max),
                    contact=int(bool(is_contacting)),
                    polish_ok=int(bool(polish_contact_verified)),
                    real_close=int(bool(actual_contact_close and actual_gap_close)),
                    arm_clear=float(arm_clearance) if arm_clearance is not None else 9.0,
                    raw_sensor=float(sensor_measured_force),
                    sensor_gap=float(tip_gap_for_sensor),
                    sensor_valid=int(bool(sensor_force_is_valid)),
                    tgt=float(getattr(self, "_target_force", TARGET_NORMAL_FORCE)),
                    cov=(self.polish_viz.covered_count() if self.polish_viz is not None else 0),
                )
                print(
                    f"[Rail {self.label}] seg={self.current_seg_idx} "
                    f"path={self.current_target_idx}/{len(points)} "
                    f"| ctrl_force={raw_force:.1f}N "
                    f"virtual={virtual_force:.1f}N target={self._target_force:.1f}N "
                    f"| z={self.z_offset:.3f}m cmd_gap={cmd_clearance:+.3f}m "
                    f"actual_gap={(float(actual_gap) if actual_gap is not None else -1.0):.3f}m "
                    f"polish_ok={int(bool(polish_contact_verified))} "
                    f"| state={self.run_state}",
                    flush=True,
                )


# ─────────────────────────────────────────────
# 메인
# ─────────────────────────────────────────────
