"""가상 스프링 접촉 + 어드미턴스 힘제어 — Isaac Lab 병렬 환경용 벡터화 이식.

출처: scripts/polishing_v5_modules/{common,agent}.py (읽기 전용 원칙에 따라 복사했다.
      원본이 바뀌면 tests/test_contact_replay.py 가 대조해 준다.)

왜 이식인가:
  원 시뮬은 이미 강체 충돌을 끈 상태다 (USE_PHYSICAL_CONTACT_SENSOR = False).
  즉 접촉력은 물리엔진이 아니라 해석식이 만든다 → PhysX 접촉 페어 없이 그대로 옮길 수 있고,
  옮기면 BC 학습 분포(0.27~6.92N)가 구성상 보존된다.

한 스텝의 계산 순서 (agent.py 원본 순서를 그대로 지킨다):
  1) z_offset → 명령 clearance                     agent.py:568 _pad_command_clearance
  2) 명령 → 실제 clearance (추종지연 1차 모델)      아래 LAG 주석 참고
  3) 패드 압축 → 원시 접촉력 (가상 스프링)          agent.py:1685-1692
  4) 1차 저역통과 필터                              agent.py:1801-1804
  5) 어드미턴스 적분 → 다음 z_offset               agent.py:2029-2037
"""
from __future__ import annotations

import torch

# ── 원본 상수 (common.py) ──────────────────────────────────────────────────
# 값을 고칠 일이 생기면 원본이 아니라 여기를 고치고, 원본과 달라진 이유를 주석에 남길 것.
VIRTUAL_PAD_STIFFNESS = 350.0       # common.py:83  N/m — 강성↓ 부드러운 접촉
VIRTUAL_PAD_DAMPING = 35.0          # common.py:84  N·s/m — 접촉 시 튕김 억제
FORCE_FILTER_ALPHA = 0.55           # common.py:108
FORCE_CONTROL_CLIP_N = 80.0         # common.py:110

ADMITTANCE_MASS = 1.0               # common.py:102
ADMITTANCE_DAMPING = 50.0           # common.py:101
ADMITTANCE_MAX_VEL = 0.02           # common.py:103  m/s — seek 압입 속도 상한

SURFACE_GUARD_MIN_CLEARANCE = -0.018  # common.py:135  명령 clearance 하한(표면 안쪽 허용치)
# 명령 clearance 상한. 원본(agent.py:577)은 side 여부와 무관하게 PRESS_OFFSET_MAX 를 쓴다.
CMD_CLEARANCE_MAX = 0.080

# 작업면(top/side)별로 갈리는 값 — is_side 로 선택한다.
CONTACT_DIST_TOP = 0.018            # common.py:80  VIRTUAL_PAD_CONTACT_DISTANCE
CONTACT_DIST_SIDE = 0.010           # common.py:81  SIDE_VIRTUAL_PAD_CONTACT_DISTANCE
PRESS_MIN_TOP = 0.012               # common.py:52  PRESS_OFFSET_MIN  (작을수록 깊게 압입)
PRESS_MAX_TOP = 0.080               # common.py:53  PRESS_OFFSET_MAX
PRESS_MIN_SIDE = 0.012              # common.py:235 SIDE_PRESS_OFFSET_MIN
PRESS_MAX_SIDE = 0.068              # common.py:231 SIDE_PRESS_OFFSET_MAX
SOFT_LIMIT_TOP = 14.0               # common.py:122 PHYSICAL_FORCE_SOFT_LIMIT_TOP_N
SOFT_LIMIT_SIDE = 6.0               # common.py:123 PHYSICAL_FORCE_SOFT_LIMIT_SIDE_N

# 규칙 컨트롤러의 목표힘 공식 (common.py:57-68) — M3 baseline 재측정용.
# 학습에는 쓰지 않는다. 이 값은 tilt 의 결정적 함수라 BC 라벨로는 정보가 0이었다.
TARGET_FORCE_TOP_FLAT = 8.0
TARGET_FORCE_TOP_STEEP = 5.0
TARGET_FORCE_SIDE_FLAT = 6.0
TARGET_FORCE_SIDE_STEEP = 3.5
ADAPTIVE_FORCE_TILT_SPAN = 45.0

PHYSICS_HZ = 60.0                   # learning/config.py:PHYSICS_HZ 와 일치시킬 것

# ── 추종지연 (LAG) ─────────────────────────────────────────────────────────
# 원 시뮬에서 RMPFlow 는 명령보다 패드를 덜 내렸다. 로그 실측:
#     mean(actual_clearance − cmd_clearance) = +0.0191 m  (C 레일 1264행)
# 이것이 "명령 5.82N vs 실측 3.12N" 과 "압입 포화 65.7%" 의 직접 원인이다.
# 이 지연은 "과도 응답"이 아니라 지속되는 정상상태 오차다. 1차 지연(τ)으로 모델링하면
#   정상상태에서 0으로 사라져 원본의 포화를 재현하지 못한다. 반드시 상수 바이어스로 넣는다.
#   (lag_tau 는 그 바이어스로 수렴하는 속도일 뿐이다.)
# 새 환경에는 RMPFlow 가 없으므로 이 지연을 그대로 옮길 이유가 없다.
#   · lag_offset = 0.0     → 완전 추종. 폐루프가 목표힘에 수렴하고 포화가 사라진다 (M2 기대 결과)
#   · lag_offset = 0.0191  → 원 시뮬 재현. 포화·힘부족 병리가 그대로 나온다 (이식 검증용)
DEFAULT_LAG_OFFSET = 0.0     # m
MEASURED_LAG_OFFSET = 0.0191  # m — C 레일 로그 1264행 실측. 이식 검증 기준값.
DEFAULT_LAG_TAU = 0.0        # s — 바이어스로 수렴하는 시간상수. 0 이면 즉시.


def adaptive_target_force(tilt_deg: torch.Tensor, is_side: torch.Tensor) -> torch.Tensor:
    """규칙 컨트롤러의 목표힘 공식 (common.py:62-68). M3 baseline 전용."""
    t = (tilt_deg / ADAPTIVE_FORCE_TILT_SPAN).clamp(0.0, 1.0)
    flat = torch.where(is_side, torch.full_like(t, TARGET_FORCE_SIDE_FLAT),
                       torch.full_like(t, TARGET_FORCE_TOP_FLAT))
    steep = torch.where(is_side, torch.full_like(t, TARGET_FORCE_SIDE_STEEP),
                        torch.full_like(t, TARGET_FORCE_TOP_STEEP))
    return flat + (steep - flat) * t


class VirtualPadContact:
    """(num_envs,) 병렬 가상 스프링 접촉 + 어드미턴스 힘제어.

    env 당 상태는 3개뿐이다: z_offset(압입 명령), z_vel(압입 속도), filtered(필터된 접촉력).
    PhysX 접촉 페어를 쓰지 않으므로 env 수를 늘려도 메모리가 거의 늘지 않는다.
    """

    def __init__(self, num_envs: int, device, dt: float = 1.0 / PHYSICS_HZ,
                 lag_offset: float = DEFAULT_LAG_OFFSET, lag_tau: float = DEFAULT_LAG_TAU):
        self.num_envs = num_envs
        self.device = torch.device(device)
        self.dt = float(dt)
        self.lag_offset = float(lag_offset)   # 지속 바이어스 [m] — 위 LAG 주석 참고
        self.lag_tau = float(lag_tau)         # 그 바이어스로 수렴하는 시간상수 [s]
        # 어드미턴스 게인 — 기본값은 원본 상수 그대로 (replay 시험 기준). PhysX 물리
        # 접촉 모드는 접촉 강성(~1e4 N/m)이 가상 스프링(350)보다 훨씬 높아 한계순환이
        # 생기므로 RobotPolishEnv 가 인스턴스 값만 올린다 (모듈 상수는 불변).
        self.admittance_mass = ADMITTANCE_MASS
        self.admittance_damping = ADMITTANCE_DAMPING
        self.admittance_max_vel = ADMITTANCE_MAX_VEL

        z = lambda: torch.zeros(num_envs, device=self.device, dtype=torch.float32)
        self.z_offset = z()
        self.z_vel = z()
        self.filtered = z()
        self.actual_clearance = z()
        self.command_clearance = z()
        self.reset()

    # ── 작업면별 상수 선택 ────────────────────────────────────────────────
    def _consts(self, is_side: torch.Tensor):
        pick = lambda a, b: torch.where(is_side, torch.full_like(self.z_offset, a),
                                        torch.full_like(self.z_offset, b))
        return (pick(CONTACT_DIST_SIDE, CONTACT_DIST_TOP),
                pick(PRESS_MIN_SIDE, PRESS_MIN_TOP),
                pick(PRESS_MAX_SIDE, PRESS_MAX_TOP),
                pick(SOFT_LIMIT_SIDE, SOFT_LIMIT_TOP))

    def reset(self, env_ids: torch.Tensor | None = None, is_side: torch.Tensor | None = None):
        """접촉 상태 초기화. z_offset 은 press_max(가장 떠 있는 상태)에서 출발해 seek 로 내려온다."""
        if env_ids is None:
            env_ids = torch.arange(self.num_envs, device=self.device)
        if is_side is None:
            press_max = torch.full_like(self.z_offset, PRESS_MAX_TOP)
        else:
            _, _, press_max, _ = self._consts(is_side)
        self.z_offset[env_ids] = press_max[env_ids]
        self.z_vel[env_ids] = 0.0
        self.filtered[env_ids] = 0.0
        self.actual_clearance[env_ids] = press_max[env_ids]
        self.command_clearance[env_ids] = press_max[env_ids]

    # ── 한 스텝 ───────────────────────────────────────────────────────────
    def step(self, target_force: torch.Tensor, is_side: torch.Tensor,
             surface_offset: torch.Tensor | None = None,
             measured_clearance: torch.Tensor | None = None,
             control_force_override: torch.Tensor | None = None) -> torch.Tensor:
        """목표힘을 받아 한 스텝 진행하고, 필터된 실측 접촉력 [N] 을 돌려준다.

        target_force   : (E,) 어드미턴스 제어기의 목표힘 setpoint.
                         배포 계약 — 여기에 BC 출력 contact_force_n 을 넣는다.
        is_side: (E) bool. 로봇 1대 환경에서는 (tilt_deg > 45) 로 정의한다.
        surface_offset : (E,) 표면이 명목 경로보다 얼마나 나와 있는지 [m]. 굴곡/오차 주입용.
                         None 이면 0 (경로가 표면에 정확히 놓인 경우).
        control_force_override : (E,) 어드미턴스 피드백 힘을 외부 값으로 교체 [N].
                         PhysX 물리 접촉 모드에서 센서 필터힘을 넣어 폐루프 force
                         동작(가상 스프링 filtered 피드백)과 완전히 동일하다 —
                         기존 replay/baseline 시험에 영향 없음.
        """
        cdist, press_min, press_max, soft_limit = self._consts(is_side)

        # 1) z_offset → 명령 clearance  (agent.py:575-577)
        cmd_clearance = (self.z_offset - cdist).clamp(SURFACE_GUARD_MIN_CLEARANCE, CMD_CLEARANCE_MAX)
        self.command_clearance = cmd_clearance

        # 2) 명령 → 실제 clearance.
        #    목표는 cmd + lag_offset (패드가 명령보다 lag_offset 만큼 덜 내려온다).
        #    lag_offset=0 이면 완전 추종이 된다.
        if surface_offset is not None:
            cmd_clearance = cmd_clearance - surface_offset
        target_clearance = cmd_clearance + self.lag_offset
        if measured_clearance is not None:
            # M0609 결합 환경: 실제 패드 접촉면 자세에서 측정한 gap을 사용.
            # 패드가 경로를 추종하지 못하면 힘이 발생하지 않는다.
            self.actual_clearance = measured_clearance.to(self.device).clamp(min=-0.01, max=0.20)
        elif self.lag_tau > 0.0:
            alpha = self.dt / (self.lag_tau + self.dt)
            self.actual_clearance = self.actual_clearance + alpha * (target_clearance - self.actual_clearance)
        else:
            self.actual_clearance = target_clearance

        # 3) 가상 스프링 (agent.py:1685-1692)
        #    로그 대조: 정적항만으로 virtual_force 와 상관계수 0.9964 (잔차 = 아래 댐핑항)
        pad_compression = (cdist - self.actual_clearance).clamp(min=0.0)
        raw = (VIRTUAL_PAD_STIFFNESS * pad_compression
               - VIRTUAL_PAD_DAMPING * self.z_vel).clamp(min=0.0)
        raw = torch.minimum(raw, soft_limit)

        # 4) 1차 저역통과 (agent.py:1801-1804)
        self.filtered = FORCE_FILTER_ALPHA * raw + (1.0 - FORCE_FILTER_ALPHA) * self.filtered

        # 5) 어드미턴스 적분 (agent.py:2029-2037)
        #    충돌 off 모드라 control_force 는 물리센서가 아니라 filtered 를 쓴다 (agent.py:2031)
        #    물리 접촉 모드에서는 override(센서 필터힘)가 피드백이 된다.
        if control_force_override is not None:
            control_force = control_force_override.to(self.device).clamp(
                min=0.0, max=FORCE_CONTROL_CLIP_N
            )
        else:
            control_force = self.filtered.clamp(max=FORCE_CONTROL_CLIP_N)
        accel = (control_force - target_force
                 - self.admittance_damping * self.z_vel) / self.admittance_mass
        self.z_vel = (self.z_vel + accel * self.dt).clamp(
            -self.admittance_max_vel, self.admittance_max_vel)
        self.z_offset = self.z_offset + self.z_vel * self.dt
        self.z_offset = torch.maximum(torch.minimum(self.z_offset, press_max), press_min)

        return self.filtered

    # ── 진단 ──────────────────────────────────────────────────────────────
    def saturation(self, is_side: torch.Tensor, eps: float = 1e-4) -> torch.Tensor:
        """압입 깊이 한계에 붙어 있는 env 마스크. M2 판정 기준(시연 65.7%) 비교용."""
        _, press_min, _, _ = self._consts(is_side)
        return self.z_offset <= (press_min + eps)
