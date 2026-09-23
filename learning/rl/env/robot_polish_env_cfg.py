"""M0609+v5 polishing-pad configuration for the coupled RL environment."""
from __future__ import annotations

import os

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import ArticulationCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.utils.configclass import configclass

from .polish_env_cfg import PolishEnvCfg


_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
_ROBOT_USD = os.path.join(
    _REPO_ROOT, "usd", "env", "Collected_m0609_with_polisher", "m0609_with_polisher.usd"
)


@configclass
class RobotPolishEnvCfg(PolishEnvCfg):
    """The analytical quality model driven by the real M0609/pad pose."""

    scene: InteractiveSceneCfg = InteractiveSceneCfg(
        num_envs=4, env_spacing=1.6, replicate_physics=True
    )

    robot_cfg: ArticulationCfg = ArticulationCfg(
        prim_path="/World/envs/env_.*/Robot",
        spawn=sim_utils.UsdFileCfg(usd_path=_ROBOT_USD, activate_contact_sensors=True),
        init_state=ArticulationCfg.InitialStateCfg(
            joint_pos={
                "joint_1": 0.0,
                "joint_2": -1.05,
                "joint_3": 1.45,
                "joint_4": 0.0,
                "joint_5": 1.15,
                "joint_6": 0.0,
                "pad_joint": 0.0,
            },
        ),
        actuators={
            "arm": ImplicitActuatorCfg(
                joint_names_expr=["joint_[1-6]"], stiffness=10000.0, damping=500.0
            ),
            "pad": ImplicitActuatorCfg(
                joint_names_expr=["pad_joint"], stiffness=2000.0, damping=100.0
            ),
        },
    )

    work_top_m: float = 0.40
    # ── 차 셀 모드 (surface_kind="quad", car_cells_robot.py) — env 별 2차곡면 계수·초기 상태·자세 ──
    carcell_quads: list | None = None      # [[c0..c5], ...] env 개수만큼 (셀 로컬, 중심 기준)
    carcell_init: list | None = None       # [{"ra","scratch","n_scr","clearcoat","seed"}, ...]
    carcell_is_side: list | None = None    # [bool, ...] tilt > 45°
    patch_center_xy_m: tuple[float, float] = (0.45, 0.0)
    pad_contact_gate_m: float = 0.020
    pad_outside_margin_m: float = 0.055
    use_physical_force_when_valid: bool = False

    # 기본 OFF: 접촉력 검증(자유공간 0N → 정적 3·5·8·10N → 이동 → 병렬) 통과 전에는
    # 기존 force_model_n 거동을 바꾸지 않는다. 검증 스크립트가 ON으로 켠다.
    # ON이면 __init__에서 dt·decimation·solver iteration을 아래 값으로 교체하고,
    # _setup_scene에서 패드 collider 활성화 + compliant 재질 바인딩을 수행한다.
    enable_pad_physical_contact: bool = False
    # 접촉 충격·관통 완화용 물리 dt. 제어 주기는 dt×decimation = 1/20 s 로 유지.
    physical_sim_dt: float = 1.0 / 120.0
    physical_decimation: int = 6
    # PT-DESIGN — Phase B(정적 3·5·8·10 N) 실측으로 튜닝됨:
    #   초기값 5000/100, α=0.25, D=50, vmax=0.02 → 실효 접촉강성 ~1e4 N/m 에서
    #   지연-포화 한계순환 (std 2.2~2.8 N, overshoot ~5 N, 10 N 은 hard-limit 리셋 루프).
    #   → 접촉 연화 + 피드백 지연 축소 + 어드미턴스 댐핑 상향으로 안정화.
    pad_compliant_stiffness_n_m: float = 2000.0
    pad_compliant_damping_n_s_m: float = 200.0
    pad_contact_offset_m: float = 0.020           # v5와 동일 — 접촉 2cm 전 생성(슬램 완화)
    pad_rest_offset_m: float = 0.0
    pad_max_depenetration_vel_m_s: float = 0.5
    solver_position_iterations: int = 8
    solver_velocity_iterations: int = 2
    # 물리 모드 어드미턴스 오버라이드 (contact.py 모듈 상수는 불변 — replay 시험 보호)
    physical_admittance_damping: float = 150.0
    physical_admittance_max_vel_m_s: float = 0.012
    # 센서 후처리 — raw는 그대로 두고 filtered 를 별도 유지한다 (로그 5종 분리 원칙).
    sensor_filter_alpha: float = 0.5       # 1차 저역통과 (physics rate 기준 EMA 계수)
    force_hard_debounce_substeps: int = 6   # 하드리밋 초과가 이만큼 연속(120 Hz → 50 ms)일 때만 위반
    sensor_valid_min_n: float = 0.05       # 이 이하는 비접촉(자유공간 ≈ 0 N) 판정

    # ── 줄바꿈(레스터 line 전환) 램프 리미터 (PT-DESIGN) ────────────────────
    # 실측 진단: _pos_at_arc 의 줄 끝→다음 줄 시작 순간이동을 IK 목표에 그대로
    # 먹이면 접촉력이 5.77N→18N+ 로 튄다(action=0 에서도 재현 — 정책과 무관).
    # 목표 (u,v) 이동을 이 속도로 제한해 매 control step 최대 이동량을 억제한다.
    line_transition_speed_m_s: float = 0.05

    # 기존 dense/종말 보상(polish_env_cfg)은 그대로 두고 RobotPolishEnv 가 추가로 뺀다.
    # w_force(0.4)는 대칭 오차 항이라 overshoot 를 따로 벌하지 않는다 — 실접촉에서
    # 순간 과대힘(라인 전환 스파이크 실측 12.8 N)은 비대칭으로 더 벌해야 한다.
    w_force_overshoot: float = 0.4         # max(0, force_mean − cmd − tol) [N]당, clamp 2
    force_overshoot_tol_n: float = 0.5
    w_unstable_contact: float = 0.4        # control step 내 substep 힘 std 초과분 [N]당, clamp 2
    unstable_std_tol_n: float = 0.5

    # repolish_mode=False(기본)면 RobotPolishEnv._get_dones()는 부모(PolishEnv)와
    # 완전히 동일하게 동작한다 — 기존 학습/평가 스크립트는 영향 없음.
    repolish_target_gu: float = 70.0             # /기존 all_pass 판정과 동일 앵커
    repolish_scratch_improve_eps_um: float = 0.02  # 이 미만 개선이면 "더 이상 개선 없음"
    repolish_gu_improve_eps: float = 0.3
    repolish_ra_improve_eps_um: float = 0.005
    repolish_rz_improve_eps_um: float = 0.02
    repolish_max_passes: int = 6
    repolish_cooldown_s: float = 20.0            # pass 사이 무가공 냉각 시간
    # 접촉 불안정 하드컷 — 과부하(force_hard_limit_n)와 별개로, 순간 힘이 크게
    # 흔들리는 상태가 여러 control step 연속되면 실패 처리한다.
    repolish_unstable_std_hard_n: float = 2.0
    repolish_unstable_streak_limit: int = 15     # 연속 control step 수 (20Hz 기준 0.75s)

    # ── 미달분·안전예산 기반 다음 pass 목표힘 (PT-DESIGN) ─────────────────
    # "정해진 스텝만큼 무조건 힘을 올리는" 방식은 clearcoat 안전선을 넘길 수 있어 채택하지
    # 않는다. 대신 직전 pass의 실측 (힘당 clearcoat 감소율)로 다음 힘을 역산하고, 안전
    # 예산(clearcoat_safety_limit_um 까지 남은 여유) 안에서만 올린다.
    repolish_cc_safety_margin_um: float = 2.0     # 안전선 바로 위까지 밀어붙이지 않는 여유
    repolish_force_gain_um: float = 3.0           # 미달비율 1.0(100%)당 노리는 추가제거량 [μm]
    # 이 비율 미만의 미달은 안전예산이 빠듯해도 fail_infeasible 로 단정하지 않는다
    # (거의 다 왔는데 조기 실패 처리하는 것을 막는 완충값).
    repolish_infeasible_shortfall: float = 0.05
    repolish_force_cap_ratio: float = 0.85        # force_hard_limit_n 대비 상한 (과부하 재트립 방지)
    # 상한은 정책 잔차 최대치까지 감안해야 한다 (9.25 실측: 기본힘 11.9N + 잔차 +30%
    #   = 15.5N 명령 → 14N 위반 2건). 적용식: cap = ratio×hard/(1+force_ratio_limit) ≈ 9.15N
