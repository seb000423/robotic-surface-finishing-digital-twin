"""로봇 팔 시연 — 2대 정책 비교 또는 16대 병렬 정책 재생.

    ~/isaacsim/python.sh learning/rl/demo_arm.py            # GUI
    ~/isaacsim/python.sh learning/rl/demo_arm.py --num_envs 16 --all_policy \
        --checkpoint learning/rl/champion/model_terminal_ppo_14ch_it800.pt
    python learning/rl/demo_arm.py --headless # 검증 실행

기본 구성 (env 2개, 좌=baseline / 우=정책):
  · M0609 + 폴리셔 (usd/env/Collected_m0609_with_polisher — 상대경로 수집본. 원본 wrapper 는
    외부 절대경로 참조라 사용 불가)
  · 표면 patch 는 로봇 앞 바닥의 회색 판. 스크래치는 얇은 막대 마커 —
    잔여 깊이에 따라 빨강(깊음)→노랑→초록(제거됨)으로 변한다.
  · 팔은 DifferentialIK 로 패드 목표(경로점 + 어드미턴스 z_offset)를 추종한다.
    접촉력·품질은 검증된 해석 모델(contact.py + LiteraturePolishingModel)이 계산한다 —
    원 시뮬도 강체 충돌 OFF 였으므로(A안) 팔은 시각·기구 검증 역할이다.
"""
import argparse
import sys

import os as _os
_REPO_ROOT = _os.path.dirname(_os.path.dirname(_os.path.dirname(_os.path.abspath(__file__))))
sys.path.insert(0, _REPO_ROOT)

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
parser.add_argument("--checkpoint", type=str,
                    default=None,
                    help="기본: <repo>/learning/rl/champion/model_bc.pt")
parser.add_argument("--surface_seed", type=int, default=1000)
parser.add_argument("--num_envs", type=int, default=2,
                    help="시각화할 로봇/환경 수. 16이면 Isaac Lab 기본 격자로 4x4 배치")
parser.add_argument("--all_policy", action="store_true",
                    help="모든 환경에 checkpoint 정책 적용. 미지정 시 env0=baseline, 나머지=정책")
parser.add_argument("--same_surface", action="store_true",
                    help="모든 환경에 같은 surface_seed 사용 (정책 비교용)")
parser.add_argument("--render_every", type=int, default=3,
                    help="몇 physics substep마다 렌더할지 (16대 권장값 6)")
parser.add_argument("--record", type=str, default=None,
                    help="부감 카메라 영상 녹화 출력 디렉토리 (헤드리스 가능 — PNG 시퀀스, "
                         "종료 후 ffmpeg 로 mp4 병합). --enable_cameras 와 함께 사용")
parser.add_argument("--max_seconds", type=float, default=None,
                    help="시뮬 시간 상한 (기본: 에피소드 완주까지)")
parser.add_argument("--surface_kinds", type=str, default="flat",
                    help="env 별 순환 배정할 작업면 (쉼표): flat,cylinder,sphere,freeform")
parser.add_argument("--curvature_radius", type=float, default=0.3)
parser.add_argument("--env_spacing", type=float, default=2.5)
parser.add_argument("--record_dt", type=float, default=0.1,
                    help="녹화 프레임 간 시뮬 시간(s). 0.1=실시간 10fps, 0.6=6배속 타임랩스")
parser.add_argument("--record_res", type=str, default="1280x720")
parser.add_argument("--cam_preset", type=str, default="auto", choices=["auto", "close", "far"])
parser.add_argument("--cam_focal", type=float, default=18.0)
parser.add_argument("--cam_dist_scale", type=float, default=1.0)
parser.add_argument("--no_status", action="store_true", help="환경 상태등(구슬) 표시 안 함")
parser.add_argument("--cam_anim", type=str, default="none", choices=["none", "zoomout"],
                    help="zoomout: 근접(두 env) → 전체 격자로 카메라 이동 (녹화 전용)")
parser.add_argument("--cam_zoom_start", type=float, default=200.0, help="줌아웃 시작 시뮬 시각(s)")
parser.add_argument("--cam_zoom_end", type=float, default=350.0, help="줌아웃 종료 시뮬 시각(s)")
parser.add_argument("--cam_close_envs", type=str, default="0,1", help="근접 시점이 잡을 env 인덱스")
parser.add_argument("--cam_close_dist", type=float, default=2.6, help="근접 시점 카메라 거리(m)")
parser.add_argument("--return_lift", type=float, default=0.03,
                    help="패스 경계 복귀 스트로크: 패드를 들어올리는 높이(m). 0 이면 비활성")
parser.add_argument("--return_time", type=float, default=3.0, help="복귀 스트로크 시간(s)")
parser.add_argument("--debug_return_at", type=float, default=None,
                    help="(테스트) 지정 시각(s)에 복귀 스트로크를 강제로 1회 실행")
parser.add_argument("--n_passes", type=int, default=1,
                    help="래스터 패스 수 (판정 레시피=2; 기본 1은 짧은 GUI 시연용)")
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
if args.checkpoint is None:
    args.checkpoint = _os.path.join(_REPO_ROOT, "learning", "rl", "champion", "model_bc.pt")
app = AppLauncher(args).app

from importlib import metadata  # noqa: E402

import numpy as np  # noqa: E402
import torch  # noqa: E402

import isaaclab.sim as sim_utils  # noqa: E402
from isaaclab.actuators import ImplicitActuatorCfg  # noqa: E402
from isaaclab.assets import Articulation, ArticulationCfg  # noqa: E402
from isaaclab.controllers import DifferentialIKController, DifferentialIKControllerCfg  # noqa: E402
from isaaclab.markers import VisualizationMarkers, VisualizationMarkersCfg  # noqa: E402
from isaaclab.scene import InteractiveScene, InteractiveSceneCfg  # noqa: E402
from isaaclab.sim import SimulationContext  # noqa: E402
from isaaclab.utils.configclass import configclass  # noqa: E402
from isaaclab.utils.math import subtract_frame_transforms  # noqa: E402

from learning.digital_twin import config as PC  # noqa: E402
from learning.digital_twin.gloss_proxy import LiteratureGlossProxyModel  # noqa: E402
from learning.digital_twin.path_executor import load_calibrated_config, raster_waypoints  # noqa: E402
from learning.digital_twin.polishing_model import ContactState, LiteraturePolishingModel  # noqa: E402
from learning.digital_twin.surface_state import (  # noqa: E402
    curve_height_normal, make_curved_patch, make_flat_patch)
from learning.rl.env.contact import VirtualPadContact  # noqa: E402
from learning.rl.env.polish_env import _load_recipe  # noqa: E402
from learning.rl.env.polish_env_cfg import PolishEnvCfg  # noqa: E402

ROBOT_USD = _os.path.join(_REPO_ROOT, "usd", "env", "Collected_m0609_with_polisher",
                          "m0609_with_polisher.usd")
PATCH_SIZE = (0.20, 0.20)   # 시연용 확대 — 학습/판정은 0.12 (정책은 국소 관측이라 무관)
PATCH_CENTER = (0.45, 0.0)          # 로봇 베이스 기준 앞쪽 45cm (M0609 도달반경 내)
SCRATCH_LEVELS = 8                  # 자국 마커 밝기 단계 (은색→도장색 페이드)
E = args.num_envs
if E < 1:
    parser.error("--num_envs must be >= 1")
if args.render_every < 1:
    parser.error("--render_every must be >= 1")


@configclass
class DemoSceneCfg(InteractiveSceneCfg):
    robot: ArticulationCfg = ArticulationCfg(
        prim_path="{ENV_REGEX_NS}/Robot",
        spawn=sim_utils.UsdFileCfg(usd_path=ROBOT_USD),
        init_state=ArticulationCfg.InitialStateCfg(
            joint_pos={"joint_1": 0.0, "joint_2": -1.05, "joint_3": 1.45,
                       "joint_4": 0.0, "joint_5": 1.15, "joint_6": 0.0, "pad_joint": 0.0},
        ),
        actuators={
            "arm": ImplicitActuatorCfg(joint_names_expr=["joint_[1-6]"],
                                       stiffness=10000.0, damping=500.0),
            # pad_joint 는 위치제어로 고정한다. 강성 0(자유 관절)이면 tool0 의 회전을
            #   관절이 그대로 흡수해 원판 자세가 따라오지 않는다 — 실측: IK 는 명령을 0.3° 로
            #   달성하는데 패드면 법선은 홈값 그대로였다 (필요 회전축과 pad_joint 축이 일치).
            "pad": ImplicitActuatorCfg(joint_names_expr=["pad_joint"],
                                       stiffness=2000.0, damping=100.0),
        },
    )


def main():
    # 16대 GUI에서도 초기화가 빠른 Fabric을 사용한다. 뷰포트는 아래에서 별도 USD 카메라에
    # 직접 연결하므로 기본 Perspective 카메라 동기화 문제에 의존하지 않는다.
    sim = SimulationContext(sim_utils.SimulationCfg(dt=1 / 60, use_fabric=True))
    scene_cfg = DemoSceneCfg(num_envs=E, env_spacing=args.env_spacing)
    scene = InteractiveScene(scene_cfg)
    sim_utils.GroundPlaneCfg().func("/World/ground", sim_utils.GroundPlaneCfg())
    light = sim_utils.DomeLightCfg(intensity=5000.0, color=(0.9, 0.93, 1.0))
    light.func("/World/Light", light)

    # 16개 환경 전체에 확실한 명암을 주는 대각선 상부 보조광.
    origins_np = scene.env_origins.detach().cpu().numpy()
    grid_center = origins_np.mean(axis=0)
    grid_span = max(float(np.ptp(origins_np[:, 0])), float(np.ptp(origins_np[:, 1])), 2.5)
    key_light = sim_utils.SphereLightCfg(
        radius=max(2.0, 0.35 * grid_span), intensity=90000.0,
        color=(1.0, 0.88, 0.72),
    )
    key_light.func(
        "/World/OverviewKeyLight", key_light,
        translation=(float(grid_center[0] - 0.35 * grid_span),
                     float(grid_center[1] - 0.45 * grid_span),
                     max(6.0, 0.9 * grid_span)),
    )

    # 작업대(회색) 위에 차 도장 패널(네이비) — 팔이 자연스러운 자세로 작업하는 허리 높이
    WORK_TOP = 0.40
    pedestal = sim_utils.CuboidCfg(
        size=(PATCH_SIZE[0] + 0.06, PATCH_SIZE[1] + 0.06, WORK_TOP - 0.05),
        visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.35, 0.35, 0.38)))
    plate = sim_utils.CuboidCfg(
        size=(PATCH_SIZE[0] + 0.04, PATCH_SIZE[1] + 0.04, 0.05),
        visual_material=sim_utils.PreviewSurfaceCfg(
            diffuse_color=(0.05, 0.12, 0.35), roughness=0.15, metallic=0.6))
    KINDS = [k.strip() for k in args.surface_kinds.split(",") if k.strip()]
    kind_of = [KINDS[i % len(KINDS)] for i in range(E)]
    for i in range(E):
        pedestal.func(f"/World/envs/env_{i}/Pedestal", pedestal,
                      translation=(PATCH_CENTER[0], PATCH_CENTER[1], (WORK_TOP - 0.05) / 2))
        if kind_of[i] == "flat":
            plate.func(f"/World/envs/env_{i}/Workpiece", plate,
                       translation=(PATCH_CENTER[0], PATCH_CENTER[1], WORK_TOP - 0.025))
        else:
            _spawn_curved_plate(f"/World/envs/env_{i}/Workpiece", kind_of[i],
                                args.curvature_radius, WORK_TOP)
    print(f"[demo] 작업면 배정: " + ", ".join(f"{k}x{kind_of.count(k)}" for k in KINDS))

    _pad_geo = _measure_pad_geometry(E)
    # 패드는 링크 자식 프림이 아니라 마커로 그린다 — Fabric 물리 자세를 매 프레임 직접 반영
    # (자식 프림은 Fabric 계층을 따라오지 않아 화면에 나타나지 않는 것이 확인됨).
    pad_markers = VisualizationMarkers(VisualizationMarkersCfg(
        prim_path="/Visuals/pads", markers={"pad": sim_utils.CylinderCfg(
            radius=PAD_RADIUS_VIS, height=PAD_THICK_VIS, axis="Z",
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.80, 0.28, 0.02), roughness=0.9))}))   # 진한 주황 — 강한 조명에서도 흰색으로 날아가지 않게
    if _pad_geo is None:
        _pad_geo = np.array([0.0, -0.0202, -0.0902])
    # Isaac Lab 6.x: 쿼터니언 xyzw. 물리 body 프레임은 시각 prim 프레임과 y축 180° 관계
    # (도구축 prim −Z ↔ body +Z, y 부호 동일 — sander_pad body 실측으로 확인). 매핑: (−x, y, −z)
    PAD_OFFSET_BODY = np.array([-_pad_geo[0], _pad_geo[1], -(_pad_geo[2] - PAD_THICK_VIS)])      # 패드 바닥
    PAD_CENTER_BODY = np.array([-_pad_geo[0], _pad_geo[1], -(_pad_geo[2] - PAD_THICK_VIS / 2)])  # 패드 중심
    Q_FLAT = np.array([0.0, 1.0, 0.0, 0.0])      # (w,x,y,z) x축 180° — 평면에서 도구가 아래를 향하는 자세
    pad_center_world = np.zeros((E, 3))
    if args.record:
        _setup_record(scene.env_origins)   # 반드시 sim.reset() 이전 (이후 생성 시 빈 프레임)
    sim.reset()
    _force_environment_visibility(E)
    _set_overview_camera(sim, scene.env_origins)
    _install_overview_usd_camera(scene.env_origins)
    robot: Articulation = scene["robot"]

    # 수동 씬에서는 기본 관절상태(홈 자세)를 명시적으로 써야 한다 — 안 쓰면 0-자세(수직)로
    #   시작해 기준 방향 캡처가 오염되고 IK 가 불가능 명령을 받는다 (ik_diag 로 실측 확인).
    robot.write_joint_state_to_sim(robot.data.default_joint_pos.torch.clone(),
                                   robot.data.default_joint_vel.torch.clone())
    robot.reset()
    for _ in range(60):    # 1초 안정화
        robot.set_joint_position_target_index(
            target=robot.data.default_joint_pos.torch.clone(),
            joint_ids=list(range(robot.num_joints)))
        scene.write_data_to_sim(); sim.step(render=False); scene.update(1 / 60)
    _force_environment_visibility(E)
    # Kit visualizer가 시작 직후 viewport 카메라를 한 번 덮어쓸 수 있어 안정화 뒤 재설정한다.
    _set_overview_camera(sim, scene.env_origins)
    _install_overview_usd_camera(scene.env_origins)

    # ── 팔 IK 세팅 (튜토리얼 run_diff_ik 패턴) ──
    arm_joints = [robot.joint_names.index(f"joint_{k}") for k in range(1, 7)]
    pad_joint = robot.joint_names.index("pad_joint")
    pad_body = robot.body_names.index("sander_pad")   # (숨김 대상 중복본 — 로깅용 아님)
    # IK 대상은 tool0 — sander_pad 는 pad_joint 로 3000 rpm 회전하는 링크라 그 자세를
    #   IK 로 고정하려 하면 스핀과 싸운다 (실측: 명령해도 패드면 법선 ·down = −0.399).
    #   원판 법선은 스핀축과 같아 회전에 불변이므로, 비회전 링크 tool0 을 제어하면 된다.
    # IK 대상 = link_6. 화면에 보이는 샌더는 link_6/quick_mount/sanding_kit 에 달려 있다
    #   (자산에 샌더 메쉬가 두 벌: sander_pad 아래 복사본은 팔을 따라오지 않아 숨긴다).
    ee_body = robot.body_names.index("link_6")
    ik = DifferentialIKController(
        DifferentialIKControllerCfg(command_type="pose", use_relative_mode=False, ik_method="dls"),
        num_envs=E, device=sim.device)
    ee_jacobi_idx = ee_body - 1 if robot.is_fixed_base else ee_body

    PAD_HALF_THICKNESS = 0.0115   # 실측: sander_pad 링크프레임 extent Y = 0.023 m

    # ── 폴리싱 로직 (PolishEnv 와 동일 구성 요소) ──
    env_cfg = PolishEnvCfg()
    recipe = _load_recipe(env_cfg.recipe_json_path)
    from dataclasses import replace as _dc_replace
    recipe = _dc_replace(recipe, n_passes=args.n_passes)   # 판정 레시피는 2패스
    contact = VirtualPadContact(E, sim.device, dt=1 / 60, lag_offset=0.0)
    is_side = torch.zeros(E, dtype=torch.bool, device=sim.device)
    contact.reset(is_side=is_side)
    cal = load_calibrated_config()
    model = LiteraturePolishingModel(cal)
    gloss = LiteratureGlossProxyModel()
    # 기본 비교는 같은 표면, 병렬 정책 재생은 서로 다른 seed의 손상·clearcoat 맵을 사용한다.
    seeds = ([args.surface_seed] * E if args.same_surface or not args.all_policy
             else [args.surface_seed + 97 * i for i in range(E)])
    surfaces = []
    for i, seed in enumerate(seeds):
        if kind_of[i] == "flat":
            surfaces.append(make_flat_patch(PATCH_SIZE, 0.002, seed=seed, with_scratches=True))
        else:
            surfaces.append(make_curved_patch(
                kind=kind_of[i], curvature_radius_m=args.curvature_radius,
                patch_size_m=PATCH_SIZE, resolution_m=0.002, seed=seed, with_scratches=True))
    scratch_specs = [_extract_scratch_segments(surface) for surface in surfaces]

    spacing = recipe.step_over_spacing_ratio * PC.PAD_DIAMETER_M
    lines = raster_waypoints(PATCH_SIZE, spacing)
    line_len = np.array([float(np.hypot(p1[0] - p0[0], p1[1] - p0[1])) for p0, p1, _ in lines])
    path_len = float(line_len.sum()) * recipe.n_passes

    def pos_at_arc(a):
        one = float(line_len.sum())
        a = a % one if a < path_len else one - 1e-9
        for (p0, p1, _), L in zip(lines, line_len):
            if a <= L:
                t = a / L
                return (p0[0] + (p1[0] - p0[0]) * t, p0[1] + (p1[1] - p0[1]) * t)
            a -= L
        return lines[-1][1]

    # 패드 자세·오프셋 — 원 프로젝트 규약 + 실측
    #   agent.py `_ee_orientation`: EE 의 패드 접촉축을 −normal(표면 안쪽)에 정렬한다.
    #   실측 (BBoxCache.ComputeRelativeBound, link_6 프레임, 최하단 메쉬 57개):
    #     패드 중심 XY = (0.0000, −0.0156),  접촉면 Z = −0.0902,  지름 ≈ 0.070 m
    #   → 접촉축은 link_6 로컬 −Z. 수평 패널이면 −Z 가 세계 −Z 를 향하면 되므로 자세는 항등.
    PAD_OFFSET_LINK6 = np.array([0.0, -0.0156, -0.0902])
    # 녹화 프레임에서 패드가 작업면 위 3~4cm 에 떠 있는 것이 확인됨 → 위 상수는
    # 보이는 샌더 메쉬의 바닥이 아니었다. 런타임에 link_6 로컬 프레임에서 가시 메쉬 bbox 를
    # 실측해 접촉면 오프셋(XY 중심, 최저 Z)을 덮어쓴다.
    # 물리 기반 보정: sander_pad body(패드 물리 링크, pad_joint 로 팔을 따라감)의 원점을
    # link_6 body 프레임에서 실측 → 하우징/패드가 실제로 있는 곳.
    _ee0 = robot.data.body_pose_w.torch[0, ee_body].cpu().numpy()
    _pd0 = robot.data.body_pose_w.torch[0, pad_body].cpu().numpy()
    _x, _y, _z, _w = [float(v) for v in _ee0[3:7]]
    _lb = _quat_to_R_np(np.array([_w, _x, _y, _z])).T @ (_pd0[:3] - _ee0[:3])
    print(f"[demo] 패드 보정(물리): sander_pad−link_6 세계 {np.round(_pd0[:3] - _ee0[:3], 4)} "
          f"(|d|={np.linalg.norm(_pd0[:3] - _ee0[:3]):.4f}) → body 프레임 {np.round(_lb, 4)}")
    print(f"[demo] 패드 바닥 오프셋(body 프레임) {np.round(PAD_OFFSET_BODY, 4)}, Q_FLAT(wxyz) {Q_FLAT}")

    def _quat_to_R(q):                      # q = (w, x, y, z)
        w, x, y, z = [float(v) for v in q]
        return np.array([
            [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])

    quat_cmd_world = torch.zeros(E, 4, device=sim.device); quat_cmd_world[:, 0] = 1.0
    tool_offset_world = PAD_OFFSET_LINK6.copy()      # 자세가 항등이라 그대로 세계 오프셋
    _root0 = robot.data.root_pose_w.torch
    _, quat_ref_b = subtract_frame_transforms(
        _root0[:, 0:3], _root0[:, 3:7], _root0[:, 0:3], quat_cmd_world)
    quat_ref_b = quat_ref_b.clone()
    print(f"[demo] link_6 → 패드접촉면 오프셋 {np.round(tool_offset_world, 4)} m, 목표자세 항등")

    # ── 정책 로드 (legacy 11-D / thermal 14-D 자동 판별) ──
    policy = _load_policy(args.checkpoint, sim.device)

    # ── 스크래치 마커: 실제 자국처럼 — 은색 선이 잔여 깊이 비율만큼 가늘어지다 사라진다 ──
    #   밝기 8단계: 은색(온전) → 도장색(사라짐) 보간. 폭도 잔여 비율로 줄어든다.
    _silver, _paint = np.array([0.82, 0.85, 0.90]), np.array([0.05, 0.12, 0.35])
    proto = {}
    for k in range(SCRATCH_LEVELS):
        f = 1.0 - k / (SCRATCH_LEVELS - 1)              # 1=온전 … 0=거의 사라짐
        c = _paint + (_silver - _paint) * f
        proto[f"lv{k}"] = sim_utils.CuboidCfg(
            size=(1.0, 0.005, 0.002),
            visual_material=sim_utils.PreviewSurfaceCfg(
                diffuse_color=tuple(float(v) for v in c),
                emissive_color=tuple(float(v) for v in (0.3 * f * c)), roughness=0.4))
    markers = VisualizationMarkers(VisualizationMarkersCfg(prim_path="/Visuals/scratches", markers=proto))

    # 환경 상태등: 파랑=실행 중, 초록=전체 통과, 노랑=재폴리싱 필요, 빨강=안전 실패.
    status_proto = {
        "running": sim_utils.SphereCfg(radius=0.08, visual_material=sim_utils.PreviewSurfaceCfg(
            diffuse_color=(0.1, 0.45, 1.0), emissive_color=(0.05, 0.2, 0.7))),
        "pass": sim_utils.SphereCfg(radius=0.08, visual_material=sim_utils.PreviewSurfaceCfg(
            diffuse_color=(0.1, 0.9, 0.2), emissive_color=(0.05, 0.6, 0.1))),
        "retry": sim_utils.SphereCfg(radius=0.08, visual_material=sim_utils.PreviewSurfaceCfg(
            diffuse_color=(1.0, 0.75, 0.05), emissive_color=(0.7, 0.4, 0.0))),
        "unsafe": sim_utils.SphereCfg(radius=0.08, visual_material=sim_utils.PreviewSurfaceCfg(
            diffuse_color=(1.0, 0.08, 0.05), emissive_color=(0.7, 0.02, 0.01))),
    }
    status_markers = None if args.no_status else VisualizationMarkers(VisualizationMarkersCfg(
        prim_path="/Visuals/env_status", markers=status_proto))
    if not args.no_status:
        _update_status_markers(status_markers, scene.env_origins, [0] * E)

    arc = torch.zeros(E, device=sim.device)
    prev_force = torch.zeros(E, device=sim.device)
    prev_action = torch.zeros(E, 2, device=sim.device)
    sim_t = 0.0
    control_n = 0
    force_cmd = torch.full((E,), recipe.target_contact_force_n, device=sim.device)
    feed_cmd = torch.full((E,), recipe.feed_speed_mm_s / 1000.0, device=sim.device)

    policy_desc = "all envs=policy" if args.all_policy else "env0=baseline, env1..=policy"
    print(f"[demo] recipe {recipe} | path {path_len:.2f} m | {E} envs | {policy_desc}")
    print(f"[demo] surface seeds: {seeds}")
    force_accum = torch.zeros(E, device=sim.device)
    # 패스 경계 복귀 스트로크 상태: 끝점 → 시작점으로 패드를 들고 이동 (그동안 연마 없음)
    one_pass = float(line_len.sum())
    returning = torch.zeros(E, dtype=torch.bool, device=sim.device)
    ret_t = torch.zeros(E, device=sim.device)
    ret_from = np.array(lines[-1][1], dtype=float); ret_to = np.array(lines[0][0], dtype=float)
    _dbg_ret_done = False
    sub = 0
    while app.is_running():
        # 초기 viewport가 검게 남는 Isaac Lab 3 Kit 초기화 순서에 대비해 몇 차례 재적용.
        if sub in (0, 30, 120):
            _set_overview_camera(sim, scene.env_origins)
            _install_overview_usd_camera(scene.env_origins)
        # ── 20Hz control step (3 substep 마다) ──
        if sub % 3 == 0 and sub > 0:
            f_mean = force_accum / 3.0
            force_accum.zero_()
            # 품질 갱신 + 관측 구성 (PolishEnv 와 동일 의미)
            obs = _build_obs(surfaces, arc, f_mean, feed_cmd, prev_force, prev_action,
                             pos_at_arc, path_len, force_cmd, env_cfg, sim.device)
            for i in range(E):
                if bool(returning[i]):
                    continue                       # 복귀 스트로크 중 — 패드가 떠 있어 연마 없음
                uv = pos_at_arc(float(arc[i]))
                model.step(surfaces[i], ContactState(uv, float(f_mean[i]), recipe.rpm,
                                                     float(feed_cmd[i])), 0.05, sim_t)
            prev_force = f_mean.clone()
            # 행동: 전체 정책 재생 또는 env0 baseline/나머지 정책 비교.
            a = torch.zeros(E, 2, device=sim.device)
            if policy is not None:
                start = 0 if args.all_policy else 1
                if start < E:
                    a[start:] = policy(obs[start:]).clamp(-1, 1)
            prev_action = a.clone()
            force_cmd = recipe.target_contact_force_n * (1 + a[:, 0] * env_cfg.force_ratio_limit)
            feed_cmd = (recipe.feed_speed_mm_s / 1000.0) * (1 + a[:, 1] * env_cfg.feed_ratio_limit)
            control_n += 1
            if control_n % 4 == 0:    # 0.2초마다 스크래치 마커 갱신 (페이드가 부드럽게)
                _update_markers(markers, scratch_specs, surfaces, scene.env_origins,
                                kind_of, args.curvature_radius)

        # ── 60Hz physics substep ──
        f = contact.step(force_cmd, is_side)
        force_accum += f
        if args.return_lift > 0:
            ret_t[returning] -= 1 / 60
            returning &= ret_t > 0
            new_arc = arc + feed_cmd * (1 / 60) * (~returning).float()
            crossed = (torch.floor(new_arc / one_pass) > torch.floor(arc / one_pass)) & (new_arc < path_len)
            if args.debug_return_at is not None and not _dbg_ret_done and sim_t >= args.debug_return_at:
                crossed[:] = True; _dbg_ret_done = True
            if bool(crossed.any()):
                returning |= crossed
                ret_t[crossed] = args.return_time
                new_arc[crossed] = torch.floor(new_arc[crossed] / one_pass) * one_pass   # 경계에 고정
            arc = new_arc
        else:
            arc += feed_cmd * (1 / 60)
        sim_t += 1 / 60

        # 패드 목표: 경로점 + z = clearance (팔이 어드미턴스 압입을 그대로 재현)
        # 접촉점 = 경로점의 곡면 높이 + 어드미턴스 clearance, 자세 = 국소 법선 정렬
        # (평면이면 h=0, 법선=+z → 항등 자세: 기존 동작과 동일)
        targets = torch.zeros(E, 7, device=sim.device)
        quat_cmd_np = np.zeros((E, 4)); quat_cmd_np[:, 0] = 1.0
        for i in range(E):
            if bool(returning[i]):                 # 복귀 스트로크: 들어올린 직선 경로
                s_ret = 1.0 - float(ret_t[i]) / max(args.return_time, 1e-6)
                s_ret = min(max(s_ret, 0.0), 1.0)
                uv = tuple(ret_from + (ret_to - ret_from) * s_ret)
                clearance = args.return_lift * float(np.sin(np.pi * s_ret))
            else:
                uv = pos_at_arc(float(arc[i]))
                clearance = float(contact.actual_clearance[i].clamp(-0.003, 0.0))
            h, nrm = curve_height_normal(kind_of[i], args.curvature_radius, PATCH_SIZE,
                                         uv[0], uv[1])
            q = _quat_mul(_quat_from_z_to(nrm), Q_FLAT)    # (w,x,y,z): 법선 정렬 ∘ 평면 자세
            R = _quat_to_R_np(q)
            off = R @ PAD_OFFSET_BODY                      # 패드 바닥의 세계 오프셋
            pad_center_world[i] = R @ PAD_CENTER_BODY
            targets[i, 0] = uv[0] - PATCH_SIZE[0] / 2 + PATCH_CENTER[0] - off[0]
            targets[i, 1] = uv[1] - PATCH_SIZE[1] / 2 + PATCH_CENTER[1] - off[1]
            # 접촉 모델의 actual_clearance 는 원 시뮬의 추종지연 규약상 +2~4cm 로 떠 있다
            # (contact.py: mean(actual−cmd)=+0.019m) — 힘·품질은 그 모델이 계산하므로, 시각 목표는
            # 표면 위로 뜨지 않게 0 이하로 묶는다 (살짝 눌림 −3mm 까지 허용).
            targets[i, 2] = WORK_TOP + h + clearance - off[2]
            quat_cmd_np[i] = (q[1], q[2], q[3], q[0])       # → Isaac Lab xyzw
        quat_cmd = torch.tensor(quat_cmd_np, dtype=torch.float32, device=sim.device)
        # world → base frame
        root = robot.data.root_pose_w.torch
        tgt_b_pos, tgt_b_quat = subtract_frame_transforms(
            root[:, 0:3], root[:, 3:7], targets[:, 0:3] + scene.env_origins, quat_cmd)
        ik.set_command(torch.cat([tgt_b_pos, tgt_b_quat], dim=1))

        jac = robot.data.body_link_jacobian_w.torch[:, ee_jacobi_idx, :, :][:, :, arm_joints]
        ee_w = robot.data.body_pose_w.torch[:, ee_body]
        ee_b_pos, ee_b_quat = subtract_frame_transforms(
            root[:, 0:3], root[:, 3:7], ee_w[:, 0:3], ee_w[:, 3:7])
        q = robot.data.joint_pos.torch[:, arm_joints]
        q_des = ik.compute(ee_b_pos, ee_b_quat, jac, q)
        robot.set_joint_position_target_index(target=q_des, joint_ids=arm_joints)
        # 시연에서 패드 스핀은 끈다. 실측: pad_joint 의 회전축이 원판 법선과 수직이라
        #   3000 rpm 을 주면 원판이 레코드판이 아니라 바퀴처럼 굴러 순간순간 날이 아래를
        #   향한다 (패드면 법선이 XZ 평면에서 연속 회전, y성분 0 으로 관측).
        #   회전은 어차피 20 Hz 렌더에서 보이지 않고, 제거량·접촉력은 해석 모델(rpm 파라미터)이
        #   계산하므로 시각적으로 끄는 것이 물리 결과에 영향을 주지 않는다.
        robot.set_joint_position_target_index(
            target=torch.zeros((E, 1), device=sim.device), joint_ids=[pad_joint])
        scene.write_data_to_sim()
        _stride = max(1, int(round(args.record_dt * 60))) if args.record else args.render_every
        _rendered = (sub % _stride == 0)
        if _rendered:
            from isaaclab.utils.math import quat_apply as _qapply
            # 패드 위치 = link_6 실제 위치 + (IK 오프셋 규약과 동일하게) 명령 자세로 회전한 로컬 중심.
            # 물리 body 프레임의 quat 은 시각 프림 프레임과 축이 달라(실측: 로컬 −Z 가 위) 쓰지 않는다.
            # 실제 손목(link_6 body) 자세로 부착 — 명령 자세를 쓰면 팔이 명령을 못 따라가는 복귀
            # 스트로크 동안 패드가 하우징에서 떨어져 보인다. body 프레임 오프셋 × 실제 xyzw 쿼터니언.
            _ee = robot.data.body_pose_w.torch[:, ee_body]
            _pp = _ee[:, 0:3] + _qapply(_ee[:, 3:7], torch.tensor(
                PAD_CENTER_BODY, dtype=torch.float32, device=sim.device).expand(E, 3))
            if _os.environ.get("DEMO_PAD_DEBUG"):      # 진단: 패드 마커를 작업면 위 공중 고정 위치에
                _pp = torch.tensor([[PATCH_CENTER[0], PATCH_CENTER[1], 0.70]], device=sim.device).expand(E, 3) + scene.env_origins
            pad_markers.visualize(translations=_pp, orientations=_ee[:, 3:7])
            if sub == _stride * 3:
                print(f"[demo][pad] marker count={pad_markers.count} pos0={_pp[0].tolist()} ee0={_ee[0, :3].tolist()}", flush=True)
        sim.step(render=_rendered)
        scene.update(1 / 60)
        if _rendered and "_REC" in globals():
            _rec = globals()["_REC"]
            _cam = _rec["cam"]
            if args.cam_anim == "zoomout":
                _e, _t = _anim_eye_target(scene.env_origins, sim_t)
                _set_rec_cam_pose(_cam, _e, _t, sim.device)
            elif not _rec["posed"]:
                _set_rec_cam_pose(_cam, _rec["eye"], _rec["tgt"], sim.device)
                _rec["posed"] = True
                print(f"[demo][rec] cam pos_w={_cam.data.pos_w[0].tolist()}", flush=True)
            _cam.update(1 / 60)
            _img = _cam.data.output["rgb"][0].cpu().numpy()
            if _rec["n"] < 2:
                print(f"[demo][rec] shape={_img.shape} dtype={_img.dtype}", flush=True)
            if _img.size > 0:
                from PIL import Image as _Image
                _Image.fromarray(_img[..., :3].astype("uint8")).save(
                    f"{_rec['dir']}/frame_{_rec['n']:05d}.png")
                _rec["n"] += 1
        sub += 1
        if sub % 600 == 0:                        # 10초마다 생존 로그 + 패드 자세 검증
            _ny = _quat_to_R(robot.data.body_pose_w.torch[0, ee_body, 3:7].cpu().numpy()) @ \
                  np.array([0.0, 0.0, -1.0])
            _cmd_q = quat_cmd_world[0].cpu().numpy(); _ach_q = robot.data.body_pose_w.torch[0, ee_body, 3:7].cpu().numpy()
            _dot = abs(float(np.dot(_cmd_q, _ach_q)))
            print(f"[demo] pad quat {np.round(robot.data.body_pose_w.torch[0,pad_body,3:7].cpu().numpy(),3)} "
                  f"pad_joint각 {float(robot.data.joint_pos.torch[0, pad_joint]):.3f} rad", flush=True)
            print(f"[demo] tool0 자세 명령{np.round(_cmd_q,3)} 달성{np.round(_ach_q,3)} "
                  f"각오차={np.degrees(2*np.arccos(min(_dot,1.0))):.1f}°", flush=True)
            print(f"[demo] 패드면 법선(세계) {np.round(_ny,3)}  ·down={float(-_ny[2]):.3f} "
                  f"(1.000 이면 면접촉)", flush=True)
            print(f"[demo] actual_clearance mean={float(contact.actual_clearance.mean())*100:.2f}cm  "
                  f"link6 z−WORK_TOP={float(robot.data.body_pose_w.torch[0, ee_body, 2])-WORK_TOP:+.4f}m "
                  f"(목표 {float(targets[0, 2]) - WORK_TOP:+.4f})", flush=True)
            print(f"[demo] t={sim_t:.0f}s  진행 {min(arc.min().item()/path_len,1)*100:.0f}%  "
                  f"F={contact.filtered.max().item():.1f}N  "
                  f"추종오차={float((robot.data.body_pose_w.torch[:, ee_body, 0:3] - (targets[:, 0:3] + scene.env_origins)).norm(dim=1).max())*100:.1f}cm", flush=True)

        done = bool((arc >= path_len).all())
        over = args.max_seconds is not None and sim_t >= args.max_seconds
        if done or over:
            break

    # ── 결과 ──
    print("\n=== 16대 병렬 시연 결과 (SYNTHETIC / GU proxy) ===")
    final_status = []
    for i in range(E):
        name = "policy" if args.all_policy or i > 0 else "baseline"
        q = model.evaluate(surfaces[i])
        g = gloss.evaluate(surfaces[i])["summary"]
        gu = float(g["gu_mean"])
        scratch = float(q["max_residual_scratch_um"])
        ra = float(q["ra_um"])
        rz = float(q["rz_um"])
        cc = float(q["clearcoat_min_um"])
        peak_c = float(q["temperature_peak_c"])
        damage = float(q["thermal_damage_peak"])
        unsafe = cc < env_cfg.clearcoat_safety_limit_um or peak_c > env_cfg.thermal_hard_limit_c
        passed = (gu >= 70.0 and ra <= env_cfg.t_ra_pass_max_um
                  and rz <= env_cfg.t_rz_pass_max_um and not unsafe)
        status = "UNSAFE" if unsafe else "PASS" if passed else "RETRY"
        final_status.append(3 if unsafe else 1 if passed else 2)
        print(f"  env{i:02d} {name:8s} [{status:6s}] | GU {gu:5.2f} | "
              f"Ra {ra:.3f} | Rz {rz:.3f} μm | scratch {scratch:.3f} μm | "
              f"CC {cc:.2f} μm | peak {peak_c:.2f}°C | damage {damage:.4f}")
    if not args.no_status:
        _update_status_markers(status_markers, scene.env_origins, final_status)
    print("[demo] 상태등: 초록=전체 통과, 노랑=재폴리싱 필요, 빨강=과열/clearcoat 안전 실패")
    if sim.has_gui:
        print("[demo] GUI 유지 중 — 창을 닫으면 종료됩니다.")
        while app.is_running():
            sim.step()


# ── helpers ───────────────────────────────────────────────────────────────
def _set_overview_camera(sim, env_origins):
    """환경 실제 좌표 범위를 기준으로 대각선 상공 overview 카메라를 설정한다."""
    p = env_origins.detach().cpu().numpy()
    center = p.mean(axis=0)
    span_x = float(np.ptp(p[:, 0])) if len(p) > 1 else 0.0
    span_y = float(np.ptp(p[:, 1])) if len(p) > 1 else 0.0
    span = max(span_x, span_y, 2.5)
    target = (float(center[0] + PATCH_CENTER[0]), float(center[1]), 0.42)
    if len(p) <= 2:
        eye = (target[0] + 0.75, target[1] - 0.85, 1.15)
    else:
        # +X/-Y 대각선 방향에서 전체 격자를 약 40° 아래로 내려다본다.
        eye = (target[0] + 0.92 * span,
               target[1] - 1.08 * span,
               max(6.5, 0.95 * span))
    try:
        sim.set_camera_view(eye=eye, target=target)
        print(f"[demo] overview camera eye={tuple(round(v, 2) for v in eye)} "
              f"target={tuple(round(v, 2) for v in target)} "
              f"grid=({span_x:.2f} x {span_y:.2f})m")
    except Exception as exc:
        print(f"[demo] overview camera 설정 실패: {exc}")


def _overview_eye_target(env_origins):
    p = env_origins.detach().cpu().numpy()
    center = p.mean(axis=0)
    span_x = float(np.ptp(p[:, 0])) if len(p) > 1 else 0.0
    span_y = float(np.ptp(p[:, 1])) if len(p) > 1 else 0.0
    span = max(span_x, span_y, 2.5)
    target = (float(center[0] + PATCH_CENTER[0]), float(center[1]), 0.42)
    preset = args.cam_preset
    if preset == "auto":
        preset = "close" if len(p) <= 2 else "far"
    s = args.cam_dist_scale
    if preset == "close":
        # 로봇 정면(+x) 쪽 3/4 각도에서 작업면들을 내려다본다 — 팔·패드·자국이 모두 보이는 구도
        spread = max(span_x, span_y)
        D = s * (0.9 * spread + 1.3)
        if span_x > span_y:      # 환경이 x 로 퍼져 있으면 옆(-y)에서
            eye = (target[0] + 0.35 * D, target[1] - 0.94 * D, 0.42 + 0.6 * D)
        else:
            eye = (target[0] + 0.82 * D, target[1] - 0.57 * D, 0.42 + 0.6 * D)
    else:
        eye = (target[0] + 0.92 * span * s,
               target[1] - 1.08 * span * s,
               max(6.5 * s, 0.95 * span * s))
    return eye, target


def _define_overview_camera(env_origins):
    """/World/OverviewCamera USD 카메라 프림 정의 (Z-up look-at, 18mm). 경로 반환."""
    import omni.usd
    from pxr import Gf, Sdf, UsdGeom
    eye, target = _overview_eye_target(env_origins)
    stage = omni.usd.get_context().get_stage()
    path = "/World/OverviewCamera"
    camera = UsdGeom.Camera.Define(stage, path)
    camera.GetProjectionAttr().Set(UsdGeom.Tokens.perspective)
    camera.GetFocalLengthAttr().Set(18.0)
    camera.GetClippingRangeAttr().Set(Gf.Vec2f(0.05, 1000.0))
    # translate + orient xformOp 로 look-at 설정 (transform 행렬 op 는 이 환경(Fabric)에서
    # 회전이 렌더에 반영되지 않는 것이 녹화 프레임으로 확인됨). USD 카메라: -Z 전방, +Y 위.
    e, t = np.asarray(eye, float), np.asarray(target, float)
    fwd = t - e; fwd /= np.linalg.norm(fwd)
    right = np.cross(fwd, np.array([0.0, 0.0, 1.0])); right /= np.linalg.norm(right)
    up = np.cross(right, fwd)
    rot = Gf.Matrix3d(*[float(v) for v in (*right, *up, *(-fwd))])   # 행우선 9개 = 기저벡터 행
    quat = rot.ExtractRotation().GetQuat()
    xform = UsdGeom.Xformable(camera.GetPrim())
    xform.ClearXformOpOrder()
    xform.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*e))
    xform.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(quat)
    camera.GetPrim().CreateAttribute(
        "omni:kit:centerOfInterest", Sdf.ValueTypeNames.Double3).Set(Gf.Vec3d(*target))
    print(f"[demo] overview cam eye={tuple(round(float(v),2) for v in e)} target={tuple(round(float(v),2) for v in t)}")
    return path, eye, target


def _close_eye_target(env_origins, env_ids, D):
    """지정 env 들의 작업면 중심을 로봇 정면(+x) 3/4 각도에서 내려다보는 근접 시점."""
    p = env_origins.detach().cpu().numpy()[env_ids]
    c = p.mean(axis=0)
    target = np.array([c[0] + PATCH_CENTER[0], c[1], 0.42])
    # 두 env 가 퍼진 축(d)에 수직인 로봇 정면(+x) 쪽에서 보되, d 의 반대쪽으로 살짝 비켜
    # 팔이 작업면을 가리지 않게 한다 → 두 대가 나란히 보이는 구도.
    if len(p) >= 2:
        d = p[-1, :2] - p[0, :2]; d = d / max(np.linalg.norm(d), 1e-6)
        horiz = np.array([1.0, 0.0]) - 0.12 * d      # 거의 정면, 살짝만 비켜 깊이감
    else:
        # 1대: 옆(−y)·앞(+x) 사이 높은 각도에서 내려다봐 팔에 가리지 않고 패드·자국이 보이게
        horiz = np.array([0.5, -0.85]); horiz = horiz / np.linalg.norm(horiz)
        eye = target + np.array([horiz[0] * D * 0.75, horiz[1] * D * 0.75, 0.72 * D])
        return eye, target
    horiz = horiz / np.linalg.norm(horiz)
    eye = target + np.array([horiz[0] * D * 0.88, horiz[1] * D * 0.88, 0.48 * D])
    return eye, target


def _anim_eye_target(env_origins, sim_t):
    """zoomout: 근접 시점 유지 → [zoom_start, zoom_end] 동안 smoothstep 으로 전체 시점까지 이동."""
    ids = [int(v) for v in args.cam_close_envs.split(",")]
    eye_c, tgt_c = _close_eye_target(env_origins, ids, args.cam_close_dist)
    _saved = args.cam_preset; args.cam_preset = "far"
    eye_f, tgt_f = _overview_eye_target(env_origins); args.cam_preset = _saved
    eye_f, tgt_f = np.asarray(eye_f, float), np.asarray(tgt_f, float)
    u = (sim_t - args.cam_zoom_start) / max(args.cam_zoom_end - args.cam_zoom_start, 1e-6)
    u = float(np.clip(u, 0.0, 1.0)); s = u * u * (3.0 - 2.0 * u)
    return eye_c + (eye_f - eye_c) * s, tgt_c + (tgt_f - tgt_c) * s


def _set_rec_cam_pose(cam, eye, tgt, device):
    """world 규약(전방 +X, 상단 +Z)의 look-at 회전을 구성해 Camera 센서 자세 설정."""
    from isaaclab.utils.math import quat_from_matrix as _qfm
    e = np.asarray(eye, float); t = np.asarray(tgt, float)
    f = t - e; f /= np.linalg.norm(f)
    l = np.cross(np.array([0.0, 0.0, 1.0]), f); l /= np.linalg.norm(l)
    u = np.cross(f, l)
    R = torch.tensor(np.stack([f, l, u], axis=1), dtype=torch.float32, device=device)
    cam.set_world_poses(torch.tensor([e], dtype=torch.float32, device=device),
                        _qfm(R).unsqueeze(0), convention="world")


def _setup_record(env_origins):
    """헤드리스 녹화: Isaac Lab Camera 센서(공식 헤드리스 캡처 경로)로 부감 rgb.
    반드시 sim.reset() 이전에 생성 (센서는 reset 시 초기화). 조준은 reset 후 첫 렌더 스텝에서
    set_world_poses_from_view 로 설정 — replicator 직접 부착은 이 환경에서 첫 프레임 이후
    갱신되지 않는 것이 확인되어 폐기."""
    import os as _os2
    from isaaclab.sensors import Camera, CameraCfg
    _os2.makedirs(args.record, exist_ok=True)
    eye, tgt = (_anim_eye_target(env_origins, 0.0) if args.cam_anim == "zoomout"
                else _overview_eye_target(env_origins))
    _w, _h = [int(v) for v in args.record_res.lower().split("x")]
    cam = Camera(CameraCfg(
        prim_path="/World/RecCam", update_period=0.0, height=_h, width=_w,
        data_types=["rgb"],
        spawn=sim_utils.PinholeCameraCfg(focal_length=args.cam_focal,
                                         clipping_range=(0.05, 1000.0)),
        offset=CameraCfg.OffsetCfg(pos=tuple(float(v) for v in eye), rot=(1.0, 0.0, 0.0, 0.0),
                                   convention="world")))
    globals()["_REC"] = {"cam": cam, "eye": eye, "tgt": tgt, "dir": args.record,
                         "n": 0, "posed": False}
    print(f"[demo] 녹화 시작 → {args.record} (Isaac Lab Camera, rgb PNG)")


def _install_overview_usd_camera(env_origins):
    """실제 USD 카메라를 만들고 Kit 활성 viewport에 직접 연결한다."""
    try:
        import omni.usd
        from omni.kit.viewport.utility import get_active_viewport
        from pxr import Gf, Sdf, UsdGeom

        path, eye, target = _define_overview_camera(env_origins)
        viewport = get_active_viewport()
        if viewport is None:
            raise RuntimeError("active Kit viewport 없음")
        viewport.set_active_camera(path)

        if args.record:
            return   # 헤드리스 녹화: 뷰포트 바인딩은 불필요하고 render product 를 비운다 (검증)
        # Isaac Sim 6 renderer에도 같은 카메라를 명시적으로 연결한다.
        from isaacsim.core.rendering_manager import ViewportManager
        ViewportManager.set_camera_view(path, eye=list(eye), target=list(target))
        print(f"[demo] active viewport camera={viewport.get_active_camera()} ({path})")
    except Exception as exc:
        print(f"[demo] USD overview camera 연결 실패: {exc}")


def _force_environment_visibility(num_envs):
    """Kit partial-visualization 잔여 상태와 상관없이 16개 환경을 USD에서 표시한다."""
    try:
        import omni.usd
        from pxr import UsdGeom
        stage = omni.usd.get_context().get_stage()
        visible = 0
        for i in range(num_envs):
            prim = stage.GetPrimAtPath(f"/World/envs/env_{i}")
            if prim and prim.IsValid():
                UsdGeom.Imageable(prim).MakeVisible()
                visible += 1
        print(f"[demo] USD 환경 visibility 강제 표시: {visible}/{num_envs}")
    except Exception as exc:
        print(f"[demo] 환경 visibility 설정 실패: {exc}")


def _extract_scratch_segments(surface):
    """스크래치 맵에서 마커용 선분(중심·방향·길이) 근사 추출 — 연결성분 PCA."""
    from scipy import ndimage
    labels, n = ndimage.label(surface.initial_scratch_depth_um > 0.05)
    segs = []
    for k in range(1, n + 1):
        i_idx, j_idx = np.nonzero(labels == k)     # 배열 축 = (x, y) 순서 — 스왑 금지
        if len(i_idx) < 5:
            continue
        pts = np.stack([i_idx, j_idx], 1).astype(float) * surface.resolution_m
        c = pts.mean(0)
        d = pts - c
        w, v = np.linalg.eigh(d.T @ d / len(d))
        axis = v[:, -1]
        length = float((d @ axis).max() - (d @ axis).min())
        cells = (labels == k)
        segs.append({"center": c, "angle": float(np.arctan2(axis[1], axis[0])),
                     "length": max(length, 0.01), "cells": cells})
    return segs


def _update_markers(markers, segs_by_env, surfaces, env_origins, kind_of, R, WORK_TOP=0.40):
    """자국 = 은색 선. 잔여 깊이 비율(rem/init)만큼 폭·높이가 줄어 6% 미만이면 사라진다.
    곡면에서는 선 중심의 곡면 높이에 놓고 국소 법선에 맞춰 기울인다."""
    pos, quat, scale, idx = [], [], [], []
    for e, surf in enumerate(surfaces):
        remaining_map = np.clip(surf.initial_scratch_depth_um - surf.cumulative_removal_um, 0, None)
        ox, oy = float(env_origins[e][0]), float(env_origins[e][1])
        for s in segs_by_env[e]:
            init_cells = surf.initial_scratch_depth_um[s["cells"]]
            rem_cells = remaining_map[s["cells"]]
            rem = float(rem_cells.max())
            # 평균 잔여 비율: 패드가 선을 지나가는 동안 점진적으로 내려간다 (최대값 기준은 '휙' 사라짐)
            ratio = float(np.clip(rem_cells.mean() / max(float(init_cells.mean()), 1e-6), 0.0, 1.0))
            cu, cv = float(s["center"][0]), float(s["center"][1])
            h, nrm = curve_height_normal(kind_of[e], R, PATCH_SIZE, cu, cv)
            pos.append([cu - PATCH_SIZE[0] / 2 + PATCH_CENTER[0] + ox,
                        cv - PATCH_SIZE[1] / 2 + PATCH_CENTER[1] + oy, WORK_TOP + h + 0.0015])
            half = s["angle"] / 2.0
            q_yaw = np.array([np.cos(half), 0.0, 0.0, np.sin(half)])
            _qw = _quat_mul(_quat_from_z_to(nrm), q_yaw)
            quat.append([_qw[1], _qw[2], _qw[3], _qw[0]])   # Isaac Lab 6.x 마커 자세 = xyzw
            if ratio < 0.08 or rem < 0.15:
                scale.append([1e-4, 1e-4, 1e-4]); idx.append(SCRATCH_LEVELS - 1)   # 가시 한계 미만 → 소멸
            else:
                scale.append([s["length"], 0.35 + 0.65 * ratio, 0.35 + 0.65 * ratio])
                idx.append(min(SCRATCH_LEVELS - 1, int((1.0 - ratio) * SCRATCH_LEVELS)))
    import torch as th
    markers.visualize(translations=th.tensor(pos), orientations=th.tensor(quat),
                      scales=th.tensor(scale), marker_indices=th.tensor(idx, dtype=th.int32))


def _quat_from_z_to(n):
    """세계 +z 를 단위벡터 n 으로 보내는 최소 회전 (w,x,y,z)."""
    n = np.asarray(n, float); n = n / np.linalg.norm(n)
    axis = np.cross(np.array([0.0, 0.0, 1.0]), n)
    s = np.linalg.norm(axis); c = float(n[2])
    if s < 1e-9:
        return np.array([1.0, 0.0, 0.0, 0.0]) if c > 0 else np.array([0.0, 1.0, 0.0, 0.0])
    ang = np.arctan2(s, c); axis /= s
    return np.array([np.cos(ang / 2), *(axis * np.sin(ang / 2))])


def _quat_mul(a, b):
    w1, x1, y1, z1 = a; w2, x2, y2, z2 = b
    return np.array([w1*w2 - x1*x2 - y1*y2 - z1*z2, w1*x2 + x1*w2 + y1*z2 - z1*y2,
                     w1*y2 - x1*z2 + y1*w2 + z1*x2, w1*z2 + x1*y2 - y1*x2 + z1*w2])


def _quat_to_R_np(q):
    w, x, y, z = [float(v) for v in q]
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def _measure_pad_geometry(num_envs):
    """env_0 의 link_6 에서 가시 메쉬 중 최저 z(백킹 플레이트)의 바닥 z 와 XY 중심을 측정."""
    import omni.usd
    from pxr import Usd, UsdGeom
    stage = omni.usd.get_context().get_stage()
    root = stage.GetPrimAtPath("/World/envs/env_0/Robot")
    link6 = None
    for prim in Usd.PrimRange(root):
        if prim.GetName() == "link_6":
            link6 = prim; break
    if link6 is None:
        print("[demo] link_6 프림을 찾지 못함"); return None
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                              [UsdGeom.Tokens.default_, UsdGeom.Tokens.render],
                              useExtentsHint=False, ignoreVisibility=False)
    best = None
    for prim in Usd.PrimRange(link6, Usd.TraverseInstanceProxies()):
        if prim.IsA(UsdGeom.Gprim) and UsdGeom.Imageable(prim).ComputeVisibility() != "invisible":
            r = cache.ComputeRelativeBound(prim, link6).ComputeAlignedRange()
            if r.IsEmpty():
                continue
            if best is None or r.GetMin()[2] < best[0]:
                best = (float(r.GetMin()[2]),
                        0.5 * (r.GetMin()[0] + r.GetMax()[0]), 0.5 * (r.GetMin()[1] + r.GetMax()[1]))
    if best is None:
        print("[demo] 샌더 메쉬 측정 실패"); return None
    z_plate, cx, cy = best
    print(f"[demo] 샌더 백킹 플레이트 바닥 z={z_plate:+.4f}, 중심=({cx:+.4f},{cy:+.4f}) (link_6 프레임)")
    return np.array([cx, cy, z_plate])


PAD_RADIUS_VIS, PAD_THICK_VIS = 0.042, 0.025   # 시각 폴리싱 패드 (폼 디스크)


def _calibrate_pad_offset_body(pad_geo_prim, ee_pos_w, ee_quat_xyzw):
    """하우징 바닥 중심(prim 프레임 측정값) → Fabric 세계행렬로 세계 위치 → body 프레임 오프셋.
    물리 body 원점·축이 시각 prim 과 다를 수 있어(실측: 8cm 어긋남) 추정 대신 실측한다."""
    try:
        import omni.usd, usdrt
        rt_stage = usdrt.Usd.Stage.Attach(omni.usd.get_context().get_stage_id())
        prim = rt_stage.GetPrimAtPath("/World/envs/env_0/Robot")
        link6_path = None
        for child in usdrt.Usd.PrimRange(prim):
            if child.GetName() == "link_6":
                link6_path = child.GetPath(); break
        if link6_path is None:
            print("[demo] (usdrt) link_6 없음"); return None
        xf = usdrt.Rt.Xformable(rt_stage.GetPrimAtPath(link6_path))
        wp_attr, wo_attr = xf.GetWorldPositionAttr(), xf.GetWorldOrientationAttr()
        if not (wp_attr.IsValid() and wo_attr.IsValid()):
            print("[demo] (usdrt) link_6 world pose 속성 없음"); return None
        wp = np.array([float(v) for v in wp_attr.Get()])
        wo = wo_attr.Get()                                   # usdrt Quat: real + imaginary
        wq = np.array([float(wo.GetReal()), *[float(v) for v in wo.GetImaginary()]])   # (w,x,y,z)
        p_w = wp + _quat_to_R_np(wq) @ np.asarray(pad_geo_prim, float)
        x, y, z, w = [float(v) for v in ee_quat_xyzw]
        Rb = _quat_to_R_np(np.array([w, x, y, z]))
        local_body = Rb.T @ (p_w - np.asarray(ee_pos_w, float))
        print(f"[demo] 패드 보정: 하우징 바닥 세계 {np.round(p_w, 4)}, link_6 {np.round(ee_pos_w, 4)} "
              f"→ body 오프셋 {np.round(local_body, 4)}")
        return local_body
    except Exception as exc:
        print(f"[demo] 패드 오프셋 보정 실패: {exc}"); return None


def _spawn_curved_plate(prim_path, kind, R, work_top, seed=0, thickness=0.05):
    """곡면 도장 패널 시각 메쉬 (61x61 상면 + 측면 스커트). 상면 중심 z = work_top."""
    import omni.usd
    from pxr import Gf, UsdGeom
    stage = omni.usd.get_context().get_stage()
    n, m = 61, 0.02
    us = np.linspace(-m, PATCH_SIZE[0] + m, n); vs = np.linspace(-m, PATCH_SIZE[1] + m, n)
    x0 = PATCH_CENTER[0] - PATCH_SIZE[0] / 2; y0 = PATCH_CENTER[1] - PATCH_SIZE[1] / 2
    pts, nrm = [], []
    for u in us:
        for v in vs:
            h, nv = curve_height_normal(kind, R, PATCH_SIZE, float(u), float(v), freeform_seed=seed)
            pts.append(Gf.Vec3f(x0 + float(u), y0 + float(v), work_top + float(h)))
            nrm.append(Gf.Vec3f(*[float(c) for c in nv]))
    idx = []
    for i in range(n - 1):
        for j in range(n - 1):
            a = i * n + j; b = a + 1; c = a + n; d = c + 1
            idx += [a, d, b, a, c, d]
    top = UsdGeom.Mesh.Define(stage, prim_path)
    top.CreatePointsAttr(pts)
    top.CreateFaceVertexCountsAttr([3] * (len(idx) // 3))
    top.CreateFaceVertexIndicesAttr(idx)
    top.CreateNormalsAttr(nrm); top.SetNormalsInterpolation("vertex")
    top.CreateSubdivisionSchemeAttr("none")
    top.CreateDoubleSidedAttr(True)
    # 스커트: 테두리 → 바닥(work_top - thickness)
    border = ([i * n + 0 for i in range(n)] + [(n - 1) * n + j for j in range(1, n)]
              + [i * n + (n - 1) for i in range(n - 2, -1, -1)] + [0 * n + j for j in range(n - 2, 0, -1)])
    spts, sidx = [], []
    for k, bi in enumerate(border):
        p = pts[bi]
        spts += [Gf.Vec3f(p[0], p[1], p[2]), Gf.Vec3f(p[0], p[1], work_top - thickness)]
    nb = len(border)
    for k in range(nb):
        a = 2 * k; b = a + 1; c = 2 * ((k + 1) % nb); d = c + 1
        sidx += [a, c, d, a, d, b]
    skirt = UsdGeom.Mesh.Define(stage, prim_path + "_skirt")
    skirt.CreatePointsAttr(spts)
    skirt.CreateFaceVertexCountsAttr([3] * (len(sidx) // 3))
    skirt.CreateFaceVertexIndicesAttr(sidx)
    skirt.CreateSubdivisionSchemeAttr("none")
    skirt.CreateDoubleSidedAttr(True)
    mat = sim_utils.PreviewSurfaceCfg(diffuse_color=(0.05, 0.12, 0.35), roughness=0.15, metallic=0.6)
    mat.func(prim_path + "_mat", mat)
    sim_utils.bind_visual_material(prim_path, prim_path + "_mat")
    sim_utils.bind_visual_material(prim_path + "_skirt", prim_path + "_mat")


def _update_status_markers(markers, env_origins, status_indices):
    """각 환경 위 상태등을 표시한다: 0 running / 1 pass / 2 retry / 3 unsafe."""
    import torch as th
    positions = env_origins.detach().cpu().clone()
    positions[:, 2] = 1.65
    orientations = th.zeros((len(status_indices), 4), dtype=th.float32)
    orientations[:, 0] = 1.0
    markers.visualize(
        translations=positions,
        orientations=orientations,
        marker_indices=th.tensor(status_indices, dtype=th.int32),
    )


def _build_obs(surfaces, arc, f_mean, feed_cmd, prev_force, prev_action,
               pos_at_arc, path_len, force_cmd, env_cfg, device):
    """PolishEnv._get_observations 와 같은 14차원 열 관측."""
    E = len(surfaces)
    stats = np.zeros((E, 7), dtype=np.float32)
    for i in range(E):
        s = surfaces[i]
        uv = pos_at_arc(float(arc[i]))
        res, R = s.resolution_m, 0.5 * PC.PAD_RADIUS_M
        i0 = max(int((uv[0] - R) / res), 0); i1 = min(int((uv[0] + R) / res) + 1, s.shape[0])
        j0 = max(int((uv[1] - R) / res), 0); j1 = min(int((uv[1] + R) / res) + 1, s.shape[1])
        sl = (slice(i0, i1), slice(j0, j1))
        remaining = np.clip(s.initial_scratch_depth_um[sl] - s.cumulative_removal_um[sl], 0, None)
        stats[i] = (
            remaining.mean(), remaining.max(), s.cumulative_removal_um[sl].mean(),
            s.clearcoat_remaining_um[sl].min() - env_cfg.clearcoat_safety_limit_um,
            s.temperature_c[sl].mean(), s.peak_temperature_c[sl].max(),
            s.thermal_damage_proxy[sl].mean(),
        )
    st = torch.as_tensor(stats, device=device)
    f = f_mean.unsqueeze(1)
    progress = (arc / path_len).clamp(0, 1).unsqueeze(1)
    return torch.cat([
        f / 10.0, (f - force_cmd.unsqueeze(1)) / 5.0, (f - prev_force.unsqueeze(1)) / 5.0,
        feed_cmd.unsqueeze(1) * 100.0, progress,
        st[:, 0:1] / 2.0, st[:, 1:2] / 2.0, st[:, 2:3] / 5.0, st[:, 3:4] / 20.0,
        prev_action,
        (st[:, 4:5] - PC.AMBIENT_TEMPERATURE_C) / 40.0,
        (st[:, 5:6] - PC.AMBIENT_TEMPERATURE_C) / 60.0,
        st[:, 6:7] / PC.THERMAL_DAMAGE_MAX,
    ], dim=1)


def _load_policy(ckpt_path, device):
    """legacy 11-D 또는 thermal 14-D actor를 checkpoint에서 직접 재구성."""
    try:
        from rsl_rl.models.mlp_model import MLPModel
        from tensordict import TensorDict

        ck = torch.load(ckpt_path, map_location=device, weights_only=False)
        state = ck["actor_state_dict"]
        obs_dim = int(state["mlp.0.weight"].shape[1])
        dummy = TensorDict({"policy": torch.zeros(1, obs_dim, device=device)}, batch_size=[1])
        actor = MLPModel(dummy, {"actor": ["policy"]}, "actor", 2,
                         hidden_dims=[128, 128], activation="elu", obs_normalization=True,
                         distribution_cfg={"class_name": "GaussianDistribution",
                                           "init_std": 0.3, "std_type": "scalar"}).to(device)
        actor.load_state_dict(state)
        actor.eval()

        def _policy(obs_tensor):
            td = TensorDict({"policy": obs_tensor[:, :obs_dim]}, batch_size=[len(obs_tensor)])
            with torch.no_grad():
                return actor(td)          # deterministic mean
        print(f"[demo] policy loaded: {ckpt_path} (obs_dim={obs_dim})")
        return _policy
    except Exception as exc:
        print(f"[demo] 정책 로드 실패 ({exc}) — env1 도 baseline 으로 동작")
        return None


if __name__ == "__main__":
    main()
    app.close()
