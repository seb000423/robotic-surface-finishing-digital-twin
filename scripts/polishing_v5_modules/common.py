"""Shared constants and USD/geometry helpers for polishing_v5."""
import os
import sys

import numpy as np
from scipy.spatial.transform import Rotation as R

_MODULE_DIR = os.path.dirname(os.path.abspath(__file__))
_SCRIPT_DIR = os.path.dirname(_MODULE_DIR)
_SRC_DIR = os.path.dirname(_SCRIPT_DIR)

if os.path.join(_SRC_DIR, "rmpflow") not in sys.path:
    sys.path.append(os.path.join(_SRC_DIR, "rmpflow"))
# ─────────────────────────────────────────────
# 레일 설정: path_generator.py가 생성한 rail_config.json에서 로드
# (두 로봇 모두 왼쪽 X=-1.0, 자동차 길이에 맞춰 Y 정지 위치 자동 계산됨)
# ─────────────────────────────────────────────
# RAIL_CONFIGS / RAIL_Y_STOPS 는 main()에서 rail_config.json 로드 후 동적으로 구성

ROBOT_USD_PATH = os.path.join(_SRC_DIR, "usd", "env", "Collected_m0609_with_polisher", "m0609_with_polisher.usd")

RAIL_COLORS = [
    (1.0, 0.4, 0.0),  # Robot A (앞끝): 주황
    (0.2, 0.6, 1.0),  # Robot B (뒤끝): 파랑
]
VIZ_UPDATE_INTERVAL_STEPS = 5

# 레일 슬라이딩 속도 및 안정화
# 비접촉(자유공간) 이동이라 접촉력과 무관 → 안전하게 더 빠르게. 도착 후 SLIDE_SETTLE_STEPS 로
# 위치 안정화하므로 슬라이딩 자체는 3배(0.0018→0.0054)로 올림. POLISH_TRAVEL_SCALE 로 조절(기본 3.0).
POLISH_TRAVEL_SCALE = max(0.1, float(os.environ.get("POLISH_TRAVEL_SCALE", "3.0")))
RAIL_SLIDE_SPEED = 0.0018 * POLISH_TRAVEL_SCALE   # m/step — 구간 간 비접촉 이동 속도
RAIL_SLIDE_ARRIVAL_TOL = 0.002  # 도착 판정 임계값 (m)
SLIDE_SETTLE_STEPS = 90    # 슬라이딩 후 물리 안정화 대기 스텝

# ─────────────────────────────────────────────
# 물리/제어 상수 (polishing_v1.py와 동일)
# ─────────────────────────────────────────────
HOOD_Y_MAX = 9999.0   # 레일 모드: 차량 전체 Y 범위 사용
# 반경 기본값 (rail_config.json이 없을 때 fallback)
PATH_MIN_RADIUS = 0.35   # M0609 특이점 회피 최소 반경
PATH_MAX_RADIUS = 0.85   # M0609 최대 도달 거리 기준
PATH_EDGE_MARGIN = 0.03
MAX_SURFACE_TILT_DEG = 15.0   # 유리/급경사 제외 강화 (윗면 평탄부만)
MAX_NORMAL_TILT_DEG = 15.0
NORMAL_QUERY_K = 35
NORMAL_SMOOTHING = 0.92
MAX_PATH_JUMP = 0.08
FRONT_REAR_MIN_TILT = 50.0      # 오버헤드 로봇이 앞/뒤 범퍼 수직면까지 담당할 최소 법선 기울기
FRONT_REAR_NORMAL_MIN_DOT = 0.58 # 앞/뒤면 판정: 법선 Y 성분 최소값
PRESS_OFFSET_MIN = 0.012  # run9 안정값 복원
PRESS_OFFSET_MAX = 0.080
TARGET_NORMAL_FORCE = 10.0  # 목표 접촉력 기본값(평탄면). 아래 가변 제어가 곡률에 따라 조정
# ── 가변 접촉력 제어(사용자 요청): 재질·곡률·패드에 따라 목표 N을 자동 조정 ──
# 상단(본네트/천장)은 넓고 비교적 평탄하므로 5~9N대, 측면은 수직·곡률·중력 영향으로 3~6N대.
TARGET_FORCE_TOP_FLAT = 8.0
TARGET_FORCE_TOP_STEEP = 5.0
TARGET_FORCE_SIDE_FLAT = 6.0
TARGET_FORCE_SIDE_STEEP = 3.5
ADAPTIVE_FORCE_TILT_SPAN = 45.0   # 이 표면 기울기(도)에서 STEEP 값에 도달
def adaptive_target_force(tilt_deg, mode="top"):
    """표면 기울기(곡률 대용)와 작업면에 따라 목표 접촉력을 가변."""
    t = min(max(float(tilt_deg) / ADAPTIVE_FORCE_TILT_SPAN, 0.0), 1.0)
    if mode == "side":
        flat, steep = TARGET_FORCE_SIDE_FLAT, TARGET_FORCE_SIDE_STEEP
    else:
        flat, steep = TARGET_FORCE_TOP_FLAT, TARGET_FORCE_TOP_STEEP
    return flat * (1.0 - t) + steep * t
CONTACT_FORCE_THRESHOLD = 0.5
CONTACT_GEOMETRY_MAX_OFFSET = 0.08   # 실제 센서를 (멀지 않으면) 항상 사용 — 측면도 닿는 zoff에서 인식
USE_VIRTUAL_SOFT_PAD = True   # 가상모델로 안정 제어/진행/마킹. 장애물OFF+convexDecomp라 평형 압입 시 디스크가 실제 표면 접촉→실센서 N도 읽힘(터미널 표시)
PAD_ONLY_ROBOT_COLLISIONS = True  # 팔/샌더 본체 collision은 끄고 폴리싱 패드만 차체와 접촉시킴
USE_PHYSICAL_CONTACT_SENSOR = False  # 디스크 충돌 off → 물리센서값은 잔여 아티팩트라 무시. 힘=실측위치 가상스프링만
TOP_SENSOR_VALID_GAP = 0.025   # 좁힘: 패드가 실제로 닿았을 때만 센서 유효(안 닿으면 정직하게 스킵)
SIDE_SENSOR_VALID_GAP = 0.035  # 좁힘: 측면도 실제 접촉 근처에서만 인정
TOP_SENSOR_Z_MARGIN = -0.005
SIDE_SENSOR_Z_MARGIN = -0.006
CAR_OBSTACLE_ENABLED = False  # 큰 RMPFlow 차체 장애물은 패드 접촉까지 밀어내므로 끔. 링크 가드로 관통 방지
VIRTUAL_PAD_CONTACT_DISTANCE = 0.018   # 충돌 off+실측위치 가상스프링: 평형 clearance=거리−목표/강성≈0(표면에 닿아 멈춤). 두꺼운 패드 안에 포함
SIDE_VIRTUAL_PAD_CONTACT_DISTANCE = 0.010   # 0.015→0.010: 힘 평형 actual_clearance=contact_dist−목표/강성. 0.015면 0.015−3.5/350=+0.005(표면 위 5~7mm 부유). 0.010이면 평형이 표면 근처에서 잡힘
POLISH_MARK_RADIUS = 0.038   # 하늘색 마킹 반경(줄인 패드에 맞춤)
VIRTUAL_PAD_STIFFNESS = 350.0   # 강성↓: 부드러운 접촉
VIRTUAL_PAD_DAMPING = 35.0   # 댐핑↑ → 접촉 시 튕김(floating) 억제, 착 달라붙게
POLISHING_COMPLIANT_STIFFNESS = 200.0   # ※실측 결과 PhysX 순응접촉이 이 셋업에선 미작동(200/2500 모두 슬램 동일) — 값 무의미
POLISHING_COMPLIANT_DAMPING = 130.0
POLISHING_DISK_RADIUS = 0.055   # 영상 기준 150mm가 과해 보여 110mm급으로 축소
POLISHING_DISK_HEIGHT = 0.070   # 가상 스프링(최장 측면 0.065)을 패드 두께 안에 넣도록 키움. 접촉면은 캘리브로 표면에 붙임
POLISHING_DISK_SIDES = 16
POLISHING_VISUAL_MAX_COMPRESSION = 0.014   # 말랑 재질: 접촉하면 납작해짐(과도한 높이 튐 억제)
POLISH_MARK_MIN_COMPRESSION = 0.002   # 패드가 이만큼 실제로 눌려야 하늘색(폴리싱됨) 표시
SPONGE_VISUAL_UPDATE_INTERVAL_STEPS = 10
POLISHING_DISK_LOCAL_POSITION = np.array([0.0, -0.002, 0.0])
POLISHING_DYNAMIC_FRICTION = 0.0
POLISHING_STATIC_FRICTION = 0.0
POLISHING_RESTITUTION = 0.0
PAD_SPIN_VELOCITY = 314.16   # 실제 사양: 3000 RPM = 314 rad/s
MAX_PRESS_VELOCITY = 0.025  # 0.008 -> 0.025 (표면 굴곡 추종성 확보)
# ── 어드미턴스 힘제어 (polishing_v1 검증 레시피 이식) ──
# accel = (F_err − D·v)/M. 댐핑 D로 진동/슬램 억제하며 목표 N에 수렴. 닿을 때까지 압입.
ADMITTANCE_DAMPING = 50.0
ADMITTANCE_MASS = 1.0
ADMITTANCE_MAX_VEL = 0.02   # m/s — seek 압입 속도 상한 (v1과 동일)
# 추종지연 피드포워드: RMPFlow가 명령보다 덜 내려오는 정상상태 지연을 추정해 명령을 더 깊게 보냄
LAG_FEEDFORWARD_GAIN = 0.12   # 0.08→0.12: bad-contact/stuck 스킵이 터지기 전에 지연을 더 빨리 메움
LAG_FEEDFORWARD_MAX = 0.08    # 0.05→0.08: RMPFlow 정상상태 지연(~2cm)+여유를 충분히 보정해 패드가 표면에 닿게
REAL_ARM_COLLISION_N = 100.0  # 과압/슬램 후퇴 임계 (run9의 300은 너무 높았고 50은 진동 유발 → 중간)
FORCE_FILTER_ALPHA = 0.55
# 접촉력 범위 5~15N 기준
FORCE_CONTROL_CLIP_N = 80.0
FORCE_SPIKE_N = 60.0
FORCE_ADVANCE_MIN_N = 2.5
FORCE_ADVANCE_MAX_N = 14.0
BAD_CONTACT_GAP_MULT = 1.15
BAD_CONTACT_GAP_MULT_SIDE = 1.55
SIDE_CONTACT_CLEARANCE_MULT = 2.5
BAD_CONTACT_PRESS_EPS = 0.003
BAD_CONTACT_CLEARANCE_EPS = 0.003
BAD_CONTACT_SKIP_STEPS_TOP = 35
BAD_CONTACT_SKIP_STEPS_SIDE = 150
PHYSICAL_FORCE_SOFT_LIMIT_N = 14.0
PHYSICAL_FORCE_SOFT_LIMIT_TOP_N = 14.0
PHYSICAL_FORCE_SOFT_LIMIT_SIDE_N = 6.0
PHYSICAL_FORCE_ADVANCE_MIN_N = 2.0
PHYSICAL_FORCE_ADVANCE_MAX_N = 14.0
PHYSICAL_FORCE_HARD_LIMIT_N = 14.0
PHYSICAL_FORCE_HARD_LIMIT_TOP_N = 14.0
PHYSICAL_FORCE_HARD_LIMIT_SIDE_N = 6.0
PHYSICAL_FORCE_SOFT_RETRACT_GAIN = 0.0012
PHYSICAL_FORCE_SOFT_RETRACT_MAX_STEP = 0.014
PHYSICAL_FORCE_SOFT_RETRACT_GAIN_TOP = 0.00035
PHYSICAL_FORCE_SOFT_RETRACT_GAIN_SIDE = 0.00035
PHYSICAL_FORCE_SOFT_RETRACT_MAX_STEP_TOP = 0.005
PHYSICAL_FORCE_SOFT_RETRACT_MAX_STEP_SIDE = 0.004
SURFACE_GUARD_MIN_CLEARANCE = -0.018   # 가장 안정(run9). 더 깊이 줘도 중앙값 gap 안 줄고 파묻힘/진동만 늘어 한계 확인됨
SURFACE_GUARD_RECOVER_CLEARANCE = 0.010
REAL_CONTACT_CLEARANCE_TOL_TOP = 0.045
REAL_CONTACT_CLEARANCE_TOL_SIDE = 0.060
REAL_CONTACT_GAP_TOL_TOP = 0.050
REAL_CONTACT_GAP_TOL_SIDE = 0.075
ARM_SURFACE_GUARD_CLEARANCE = 0.028
ARM_SURFACE_SKIP_CLEARANCE_TOP = 0.008
ARM_SURFACE_SKIP_CLEARANCE_SIDE = 0.005
ARM_SURFACE_RETRACT_STEP = 0.005
ARM_SURFACE_RECOVER_LIFT_TOP = 0.025
ARM_SURFACE_RECOVER_LIFT_SIDE = 0.012
ARM_SURFACE_MAX_RECOVER_STRIKES = 18
SPIKE_RETRACT_STEP = 0.003
SPIKE_PAUSE_STEPS = 8
CONTACT_SETTLE_STEPS = 2
PATH_ADVANCE_PER_STEP = 1.0 / 60.0
# 폴리싱 경로 추종 속도(접촉 중 스텝당 경로 인덱스 전진량). 로봇팔이 표면을 훑는 속도 = 폴리싱 속도.
# 기본값 3.0 = 접촉력이 유지되는 안전 상한(검증 기준). RMPFlow 추종 지연으로 패드가 뜨기 시작하는
# 한계가 ~3배 부근이라 그 직전으로 잡음. force_log 에서 접촉이 깨지면 POLISH_SPEED_SCALE=2.0 등으로
# 코드 수정 없이 낮출 수 있음. 더 키우려면(>3) 접촉력/완성률 저하를 반드시 확인할 것.
POLISH_SPEED_SCALE = max(0.1, float(os.environ.get("POLISH_SPEED_SCALE", "3.0")))
PATH_ADVANCE_PER_CONTACT_STEP_TOP = 0.10 * POLISH_SPEED_SCALE   # 천장(C, overhead)
PATH_ADVANCE_PER_CONTACT_STEP_SIDE = 0.08 * POLISH_SPEED_SCALE  # 측면(SL/SR)
PATH_CREEP_ADVANCE_PER_STEP = 0.8 / 60.0
STATUS_LOG_INTERVAL_STEPS = 15
# 두 로봇 모두 왼쪽(yaw=-π/2): joint_1=0 → 팔이 차량 방향
HOME_JOINT_POSITIONS = np.array([0.0, -1.05, 1.45, 0.0, 1.15, 0.0])
HOME_INITIAL_SETTLE_STEPS = 90
APPROACH_SETTLE_STEPS = 240   # 어프로치를 더 천천히(RMPFlow가 충분히 계산 → 차체/단상 관통 방지)
RETRACT_SETTLE_STEPS = 90
RETURN_HOME_SETTLE_STEPS = 180
SAFE_APPROACH_CLEARANCE = 0.22
SAFE_RETRACT_CLEARANCE = 0.22

# ─────────────────────────────────────────────
# 유리/구멍 회피 + 말랑 패드 곡면 변형 + 진단 로그 (사용자 요청)
#  - 유리는 깊이카메라에 안 잡혀 점군이 비어 있음 → 패드 발자국(footprint) 아래
#    점군 점이 충분할 때만 누른다. 부족하면(유리/구멍/가장자리) 스킵.
#  - 말랑 패드: 스펀지 바닥면 정점을 그 아래 표면 점에 맞춰 휘게(굽은 면이면 굽게).
# ─────────────────────────────────────────────
SURFACE_FOOTPRINT_RADIUS = POLISHING_DISK_RADIUS   # 발자국 판정 반경 = 패드 반경
SURFACE_FOOTPRINT_MIN_RATIO = 0.30   # 구간 중앙값 대비 이 비율 미만이면 유리/구멍으로 보고 스킵
SURFACE_FOOTPRINT_MIN_ABS = 5        # 발자국 최소 점 개수(절대 하한)
CONFORMAL_PAD_ENABLED = True         # 말랑 패드 곡면 변형 on/off
CONFORMAL_MAX_DEFORM = 0.012         # 정점 변형 최대치(m): 시각적으로 갑자기 길어지는 현상 억제
DIAG_LOG_ENABLED = True              # status_log.txt 진단 로그 on/off

# ─────────────────────────────────────────────
# 차량 리프트: 정비소 바퀴 리프트처럼 차+점군+경로를 통째로 이만큼 들어올림.
# 폴리싱 시작 전 차가 바닥에서 이 높이까지 상승(애니메이션). 측면이 바닥에 안 닿게.
# 모든 점군/경로/베이스 z 가 +CAR_LIFT_Z 된 '리프트 좌표계'에서 동작.
# ─────────────────────────────────────────────
CAR_LIFT_Z = 0.90
CAR_LIFT_ANIM_STEPS = 90   # 이 스텝 동안 바닥→리프트높이까지 상승

# 주차 진입 애니메이션: 차가 +Y(창문 없앤 벽) 쪽에서 바닥으로 들어와 리프트(원점)에 주차
CAR_PARK_START_Y   = 10.0   # 진입 시작 Y(원점에서 +Y 방향으로 떨어진 위치)
CAR_PARK_ANIM_STEPS = 480   # 이 스텝 동안 천천히 주차(클수록 느림)

# 건물 진입 전 하부 스캐너: 차가 잠깐 멈추고, 바닥 아래에서 반도넛형 스캐너가 올라와 Y방향으로 훑는다.
CAR_ENTRY_SCANNER_ENABLED = True
CAR_ENTRY_SCAN_Y = 4.20
ENTRY_SCANNER_RISE_STEPS = 70
ENTRY_SCANNER_SWEEP_STEPS = 160
ENTRY_SCANNER_LOWER_STEPS = 70
ENTRY_SCANNER_RADIUS = 1.25
ENTRY_SCANNER_TUBE_RADIUS = 0.045
ENTRY_SCANNER_SWEEP_HALF_Y = 1.75

# ─────────────────────────────────────────────
# 오버헤드 갠트리 모드 (rail_config.json의 mount_mode=="overhead"일 때 활성)
# ※ Z 한계는 리프트만큼 같이 올림(리프트 좌표계)
# ─────────────────────────────────────────────
OVERHEAD_STANDOFF = 0.70   # 경로 생성과 맞춘 기본 standoff. 실제 런타임 Z는 rail_config stop Z를 따른다.
OVERHEAD_Z_MIN    = 1.15 + CAR_LIFT_Z   # 베이스 Z 하한
OVERHEAD_Z_MAX    = 1.95 + CAR_LIFT_Z   # 베이스 Z 상한 (갠트리 승강 상한)
GANTRY_BEAM_Z     = 2.75 + CAR_LIFT_Z   # 상단 가로 빔(거꾸로 매달린 로봇 마운트)
GANTRY_HALF_X     = 1.36   # 갠트리 X 반폭 = 측면 레일 X(±1.36)에 맞춤 → 포스트가 레일 중심선에 옴
                           # (GantryCross 스케일 2*HALF_X도 함께 늘어 끝이 포스트에 맞음. 시각 전용)
GANTRY_HALF_Y     = 3.20   # 갠트리 Y 반길이 (차 ±1.53m + 경로여백 1.0m + 안전여유 0.67m)
OVERHEAD_USE_ELBOW_FLOOR = False  # 가상바닥 장애물 실험: 팔뚝-손목이 강결합이라 패드 접촉을 막아 비활성
OVERHEAD_TRANSIT_CLEAR = 0.40   # 이동/대기 중 패드를 차량 최상단보다 이만큼 위로 들어 충돌 방지
OVERHEAD_POLISH_START_OFFSET = 0.050   # 폴리싱 진입/스킵 후 시작 z_offset(접촉 직전) → 빠른 첫 접촉
# 오버헤드 홈 포즈 (거꾸로 매달린 베이스 기준 — 팔이 아래로, 초기 추정값·시뮬 튜닝 대상)
HOME_JOINT_POSITIONS_OVERHEAD = np.array([0.0, -1.05, 1.45, 0.0, 1.15, 0.0])

# 측면 단상 리프트: 베이스가 점 높이를 추적할 때의 Z 한계 (리프트 좌표계)
SIDE_BASE_Z_MIN = 0.15 + CAR_LIFT_Z
SIDE_BASE_Z_MAX = 1.55 + CAR_LIFT_Z
SIDE_BASE_Z_OFFSET = 0.06  # 측면 베이스를 표면보다 살짝 위로 둬 팔꿈치/손목이 차체 쪽으로 접히는 자세를 줄임
SIDE_WAYPOINT_MIN_TILT = 40.0  # 50→40: 휠 아치 위 곡면(법선 ~45°)까지 포함
SIDE_NORMAL_MIN_DOT = 0.58  # 측면 경로/런타임 필터: 진짜 좌우 옆면만 허용
SIDE_RECOVER_OUTWARD_STEP = 0.012
SIDE_RECOVER_OUTWARD_MAX = 0.060
SIDE_POLISH_START_OFFSET = 0.068   # 측면은 패드 접촉거리 근처에서 시작해 슬램을 줄임
SIDE_PRESS_OFFSET_MAX = 0.068      # 측면은 스파이크 후퇴 후에도 패드가 너무 멀리 뜨지 않게 제한
SIDE_COLUMN_NEUTRAL_Z = 0.85       # 시작 높이: 텔레스코픽 리프트 접힘 높이(0.83m) 위에 로봇이 얹히게(파묻힘 방지). 차 상승 후 SLIDE에서 working 높이까지 상승
SIDE_APPROACH_FAR_CLEARANCE = 0.24 # 측면 어프로치 시작 거리(이만큼 바깥에서 수평 접근 시작 → 차체 관통 방지)
SIDE_APPROACH_FINAL_CLEARANCE = 0.035  # POLISH 진입 전 패드가 표면 근처까지 오도록 함
SIDE_PRESS_OFFSET_MIN = 0.012      # 0.045→0.012: 0.045면 cmd_clearance=0.045-0.015=+0.030(표면 위 3cm)라 영원히 못 닿음. 표면 안쪽까지 명령 가능하게 낮춤
# 측면 물리접촉 추종: 곡면/수직 옆면은 가상모델로 떠/박혀서, 실제 센서로 닿을 때까지 압입
SIDE_SEEK_SPEED = 0.015            # m/s — 안 닿으면 이 속도로 더 압입(닿으면 멈춤)
SIDE_CONTACT_TARGET_N = 20.0       # 목표 20N 미만이면 더 압입
SIDE_CONTACT_MAX_N = 26.0          # 이 힘 초과면 후퇴
SIDE_MARK_FORCE_N = 1.5            # 측면: 물리 센서가 이 이상일 때만 하늘색 마킹(실제 닿음)
# 측면 좌우 미러로 한쪽 패드가 반대로 향하는 것 보정: 이 outward_sign을 가진 측면 로봇의
# EE를 자기 Y축 기준 180° 뒤집어 패드 면을 차체로 돌림. (좌=-1, 우=+1) — 보고 한 비트만 바꾸면 됨
SIDE_EE_FLIP_SIGNS = ()   # 뒤집기 끔(관통 방지) — 디스크 접촉축 측정 후 올바른 자세로 재설계

# ─────────────────────────────────────────────
# 측면 받침대: Vention 518823 텔레스코픽 리프트 USD 사용 (기존 파란 원통 대체)
# 실측 스펙: 접힘 830mm / 펼침 1700mm / 스트로크 870mm / 풋프린트 315mm. 2단 신축.
# USD는 '접힌 상태(83cm)'로 모델링됨. Y-up·cm 단위라 회전+스케일 보정 필요.
# ─────────────────────────────────────────────
USE_TELE_LIFT_ASSET = True   # True=tele_lift.usd 받침대, False=기존 파란 원통(문제 시 즉시 복귀)
TELE_LIFT_USD_PATH = os.path.join(_SRC_DIR, "usd", "env", "tele_lift.usd")
TELE_LIFT_BASE_Z = 0.0          # 리프트 바닥판이 놓이는 바닥 Z (튜닝: 위/아래 정렬)
TELE_LIFT_RETRACTED_H = 0.83    # 접힘 높이(m) — USD 모델 상태
TELE_LIFT_EXTENDED_H = 1.70     # 펼침 높이(m)
TELE_LIFT_UNIT_SCALE = 0.01     # cm→m
TELE_LIFT_UP_ROT_DEG = 90.0     # 측면(바닥→위): Y-up→Z-up 보정 회전(X축). 누우면 -90/180 등으로 조정
TELE_LIFT_OVERHEAD_ROT_DEG = -90.0  # 천장(빔→아래): 거꾸로 매달려 아래로 신축
# 신축 튜브 prim(고정 그룹은 그대로, 이 둘만 위로 슬라이드). 매핑은 STEP 부품명으로 확정됨:
#   NAUO26 = ST-EXT-012(645mm) 1단, NAUO27 = ST-EXT-013(635mm) 2단 최상단 캐리지
TELE_LIFT_STAGE1_PRIM = "MO_LM_050_0830__2_0/NAUO26"
TELE_LIFT_STAGE2_PRIM = "MO_LM_050_0830__2_0/NAUO27"
# 정비소 4주식 리프트 다리(WheelLift_pad_*)를 같은 텔레스코픽 에셋(시각 프롭)으로 교체.
# 접힘 0.83m가 기존 기둥 높이(~0.79m)와 거의 같아 추가 스케일/신장 없이 사용.
# 풋프린트(0.315m)가 기존 실린더 지름(0.26m)보다 넓으면 SCALE을 낮춰 줄인다.
WHEEL_TELE_LIFT_SCALE = 1.0     # 다리 추가 스케일(1.0=접힘 0.83m)
WHEEL_TELE_LIFT_EXT   = 0.0     # 고정 신장량(m). 0=완전 접힘
# 에셋 머티리얼이 거의 검정(메인 기둥 mat_0=(0.01,0.04,0.09))이고 전부 metallic=1.0이라
# 너무 검게 보임 → 런타임에 "어두운 머티리얼만" 회색으로 오버라이드(원본 USD 불변).
TELE_LIFT_RECOLOR      = True
TELE_LIFT_GRAY         = (0.42, 0.42, 0.45)  # 어두운 부품 대체 회색(약간 푸른 중간 회색)
TELE_LIFT_DARK_LUM_MAX = 0.18                # 휘도 < 이 값인 머티리얼만 회색으로 교체
TELE_LIFT_METALLIC     = 0.25                # 회색 부품 메탈릭(낮춰 무광 느낌, None이면 유지)

# 측면 레일(SideRail SL/SR)을 Vention Rail.obj→USD(시각 프롭)로 교체.
# 변환 원본은 mm 수치인데 cm 라벨(metersPerUnit=0.01)·Y-up·머티리얼 없음·원점 비중심.
# → 마스터 1개를 m·Z-up·중심정렬·회색 머티리얼로 패치하고 8개 타일(SL/SR×4)이 공유 reference.
RAIL_USE_ASSET   = True      # False면 기존 VisualCuboid 레일로 복귀
RAIL_USD_PATH    = os.path.join(_SRC_DIR, "usd", "env", "Rail.usd")        # 변환 원본
RAIL_MASTER_USD  = os.path.join(_SRC_DIR, "usd", "env", "Rail_master.usd") # 런타임 패치본
RAIL_UNIT_SCALE  = 0.001     # mm→m (수치 900 = 0.9m)
RAIL_SEG_LEN     = 0.9       # 세그먼트 길이(m) 기본값(마스터에서 실측해 덮어씀)
RAIL_WIDTH       = 0.4       # 레일 폭(m) — RailSlider 폭(0.4)에 맞춤. 단면(폭·두께)을 여기에 비례 확대
RAIL_ROT_DEG     = -90.0     # 네이티브 장축 Z→월드 Y 정렬(X축 회전). 뒤집히면 +90
RAIL_FLOOR_Z     = 0.05      # 레일 중심 높이(m)
RAIL_GRAY        = (0.45, 0.45, 0.48)  # 레일 색(원본에 머티리얼 없어 직접 바인드)

STATE_HOME    = "HOME"
STATE_SLIDE   = "SLIDE"         # 레일 슬라이딩 중
STATE_SETTLE  = "SETTLE_SLIDE"  # 슬라이딩 후 물리 안정화
STATE_APPROACH = "APPROACH"
STATE_POLISH  = "POLISH"
STATE_EVADE_COLLISION = "EVADE_COLLISION"
STATE_RETRACT = "RETRACT"
STATE_TRANSIT = "TRANSIT"       # 구간 간 연속 이동 (홈 복귀 없이 레일+로봇 동시 이동)
STATE_RETURN_HOME = "RETURN_HOME"
STATE_DONE    = "DONE"


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


def clamp_normal_horizontal(normal, outward_sign, max_tilt_deg=35.0):
    """측면용 법선 보정: x를 바깥(outward_sign: 좌=-1,우=+1)으로 향하게 하고,
    바깥 수평축(±X)에서 max_tilt_deg 이내로 제한 → EE가 수평으로 옆면에 접근."""
    n = np.array(normal, dtype=float)
    n = n / (np.linalg.norm(n) + 1e-9)
    if n[0] * outward_sign < 0:
        n = -n
    axis = np.array([float(outward_sign), 0.0, 0.0])
    cos_limit = np.cos(np.radians(max_tilt_deg))
    dot = float(np.dot(n, axis))
    if dot >= cos_limit:
        return n
    perp = n - axis * dot
    pn = np.linalg.norm(perp)
    if pn < 1e-6:
        return axis
    limited = axis * cos_limit + (perp / pn) * np.sin(np.radians(max_tilt_deg))
    return limited / np.linalg.norm(limited)


def clamp_normal_overhead(normal, target_pos=None, center_y=0.0):
    """천장 로봇용 법선 보정. 윗면은 수직으로 제한하고, 앞/뒤면은 Y 방향 법선을 유지."""
    n = np.array(normal, dtype=float)
    n = n / (np.linalg.norm(n) + 1e-9)
    tilt_deg = np.degrees(np.arccos(np.clip(n[2], -1.0, 1.0)))
    front_back = (
        tilt_deg >= FRONT_REAR_MIN_TILT and
        abs(float(n[1])) >= FRONT_REAR_NORMAL_MIN_DOT and
        abs(float(n[1])) >= abs(float(n[0])) + 0.12
    )
    if not front_back:
        return clamp_normal_tilt(n)

    # 앞/뒤 수직면은 차량 중심에서 바깥쪽으로 향하도록 법선 부호를 정렬한다.
    if target_pos is not None:
        outward = 1.0 if float(target_pos[1]) >= float(center_y) else -1.0
        if n[1] * outward < 0.0:
            n = -n
    # 좌우 성분은 줄이고, 완전 수평 법선이 되어 IK가 뒤집히는 것을 막기 위해
    # 약간의 위쪽 성분을 남긴다.
    n[0] *= 0.25
    if n[2] < 0.08:
        n[2] = 0.08
    return n / (np.linalg.norm(n) + 1e-9)


def filter_safe_waypoints(points, surface_points, kdtree, base_position, is_side=False, is_overhead=False):
    """base_position 기준으로 각 로봇이 실제로 도달 가능한 웨이포인트만 필터링.
    is_side=True(측면)면 수직면이므로 윗면용 tilt(≤15°)·X경계 필터를 적용하지 않음."""
    if len(points) == 0:
        return points

    surface_min = np.min(surface_points, axis=0)
    surface_max = np.max(surface_points, axis=0)
    safe_points = []

    for pt in points:
        dist = np.linalg.norm(pt - base_position)
        if dist < PATH_MIN_RADIUS or dist > PATH_MAX_RADIUS:
            continue
        # Y 경계는 기본적으로 공통이지만, 오버헤드 앞/뒤 수직면은 차량 끝단 근처가
        # 작업 대상이므로 front_back 판정 뒤에 예외 허용한다.
        inside_y = surface_min[1] + PATH_EDGE_MARGIN <= pt[1] <= surface_max[1] - PATH_EDGE_MARGIN
        normal, tilt_deg = estimate_surface_normal(kdtree, surface_points, pt)
        if (not inside_y) and (not is_overhead):
            continue
        if is_side:
            outward = 1.0 if float(base_position[0]) >= 0.0 else -1.0
            if normal[0] * outward < 0:
                normal = -normal
            side_axis = np.array([outward, 0.0, 0.0])
            if tilt_deg < SIDE_WAYPOINT_MIN_TILT:
                continue
            if float(np.dot(normal, side_axis)) < SIDE_NORMAL_MIN_DOT:
                continue
        elif is_overhead:
            inside_x = surface_min[0] + PATH_EDGE_MARGIN <= pt[0] <= surface_max[0] - PATH_EDGE_MARGIN
            if not inside_x:
                continue
            front_back = (
                tilt_deg >= FRONT_REAR_MIN_TILT and
                abs(float(normal[1])) >= FRONT_REAR_NORMAL_MIN_DOT and
                abs(float(normal[1])) >= abs(float(normal[0])) + 0.12
            )
            if (not inside_y) and not front_back:
                continue
            if tilt_deg > MAX_SURFACE_TILT_DEG and not front_back:
                continue
        else:
            inside_x = surface_min[0] + PATH_EDGE_MARGIN <= pt[0] <= surface_max[0] - PATH_EDGE_MARGIN
            if not inside_x:
                continue
            if tilt_deg > MAX_SURFACE_TILT_DEG:   # 윗면: 급경사(유리/측면) 제외
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


def deform_disk_points_to_surface(base_pts, side_count, world_matrix, kdtree, raw_points,
                                  surface_normal, max_deform):
    """말랑 패드 곡면 변형: 디스크 바닥 링 정점(앞 side_count개)을 그 아래 표면 점군에
    맞춰 국소적으로 휘게 한다(굽은 면이면 굽은 모양처럼).

    각 바닥 정점의 월드 위치에서 가장 가까운 스캔 점을 찾아, 패드 접촉축(local -Y의
    월드 방향)을 따라 표면에 닿을 만큼 정점 Y를 이동(±max_deform clamp).
    반환: (변형된 Vt.Vec3fArray, 적용된 최대 변형량[m])
    """
    from pxr import Gf, Vt
    pts = list(base_pts)
    n = np.asarray(surface_normal, dtype=float)
    n = n / (np.linalg.norm(n) + 1e-9)
    a4 = world_matrix.TransformDir(Gf.Vec3d(0.0, -1.0, 0.0))   # local -Y = 실제 접촉면 방향
    a = np.array([a4[0], a4[1], a4[2]], dtype=float)
    a = a / (np.linalg.norm(a) + 1e-9)
    denom = float(np.dot(a, n))
    if abs(denom) < 0.2:   # 접촉축이 표면과 거의 평행 → 변형 스킵(수치 불안정)
        return base_pts, 0.0

    # 1st pass: 각 바닥 정점이 표면에 닿기까지의 변위 t_i 계산
    raw_t = []
    for i in range(side_count):
        p = pts[i]
        w = world_matrix.Transform(Gf.Vec3d(float(p[0]), float(p[1]), float(p[2])))
        wp = np.array([w[0], w[1], w[2]], dtype=float)
        _, idx = kdtree.query(wp)
        s = raw_points[idx]
        raw_t.append(float(np.dot(s - wp, n) / denom))
    raw_t = np.asarray(raw_t)

    # 평균 변위를 빼서 '곡률(상대 차이)'만 반영 → 패드 전체 높이는 유지되고
    # 굽은 면에서만 바닥면이 휘어진다(균일하게 쭉 늘어나는 현상 제거).
    residual = raw_t - float(np.mean(raw_t))
    residual = np.clip(residual, -max_deform, max_deform)

    for i in range(side_count):
        p = pts[i]
        pts[i] = Gf.Vec3f(float(p[0]), float(p[1]) + float(residual[i]), float(p[2]))
    return Vt.Vec3fArray(pts), float(np.max(np.abs(residual)))


def set_collision_enabled_recursive(stage, root_path, enabled):
    from pxr import Usd, UsdGeom, UsdPhysics
    root_prim = stage.GetPrimAtPath(root_path)
    if not root_prim.IsValid():
        return 0
    changed = 0
    for prim in Usd.PrimRange(root_prim):
        # disabling할 때는 기존 CollisionAPI만 만진다. 시각 mesh에 새 CollisionAPI를
        # 붙이면 PhysX가 visual까지 dynamic collision으로 파싱해서 가짜 힘이 생긴다.
        if not enabled and not prim.HasAPI(UsdPhysics.CollisionAPI):
            continue
        if prim.IsA(UsdGeom.Mesh) or prim.HasAPI(UsdPhysics.CollisionAPI):
            collision_api = UsdPhysics.CollisionAPI.Apply(prim)
            attr = collision_api.GetCollisionEnabledAttr()
            if not attr:
                attr = collision_api.CreateCollisionEnabledAttr()
            attr.Set(bool(enabled))
            changed += 1
    return changed


def recolor_tele_lift_dark(stage, prim_path, gray=None, lum_max=None, metallic=None):
    """리프트 prim 아래의 '너무 어두운' 머티리얼만 회색으로 보정(런타임 오버라이드).

    원본 USD는 안 건드리고 스테이지에 override 오피니언만 얹으므로 사본 재생성 불필요.
    각 셰이더의 base_color_factor 휘도가 lum_max 미만이면 gray로 교체하고,
    metallic_factor를 낮춰(있으면) 검은 금속 대신 무광 회색으로 읽히게 한다.
    반환: 변경한 셰이더 수.
    """
    from pxr import Usd, UsdShade, Gf
    gray = TELE_LIFT_GRAY if gray is None else gray
    lum_max = TELE_LIFT_DARK_LUM_MAX if lum_max is None else lum_max
    metallic = TELE_LIFT_METALLIC if metallic is None else metallic
    root = stage.GetPrimAtPath(prim_path)
    if not root or not root.IsValid():
        return 0
    n = 0
    for prim in Usd.PrimRange(root):
        if not prim.IsA(UsdShade.Shader):
            continue
        sh = UsdShade.Shader(prim)
        cin = sh.GetInput("base_color_factor")
        if not cin:
            continue
        c = cin.Get()
        if c is None or (c[0] + c[1] + c[2]) / 3.0 > lum_max:
            continue
        cin.Set(Gf.Vec3f(float(gray[0]), float(gray[1]), float(gray[2])))
        if metallic is not None:
            min_ = sh.GetInput("metallic_factor")
            if min_:
                min_.Set(float(metallic))
        n += 1
    return n


def prepare_rail_master():
    """Rail.usd(변환 원본)를 m·Z-up·기하중심정렬·회색 머티리얼로 패치한 마스터를 1회 생성.

    8개 타일(SL/SR×4)이 이 마스터 1개를 공유 reference 한다(이미 보정돼 있어
    MetricsAssembler 이중스케일 없음). 회전·스케일은 reference 측 create_prim에서만 처리.
    상수 변경 시 Rail_master.usd 를 지우면 재생성된다. 반환: 마스터 경로(or 실패 시 원본).
    """
    import shutil
    from pxr import Usd, UsdGeom, UsdShade, Gf, Sdf
    if os.path.exists(RAIL_MASTER_USD):
        return RAIL_MASTER_USD
    if not os.path.exists(RAIL_USD_PATH):
        print(f"[Rail] ⚠ 변환 원본 없음: {RAIL_USD_PATH} (obj_to_usd.py로 먼저 생성)", flush=True)
        return RAIL_USD_PATH
    try:
        shutil.copy(RAIL_USD_PATH, RAIL_MASTER_USD)
        st = Usd.Stage.Open(RAIL_MASTER_USD)
        UsdGeom.SetStageMetersPerUnit(st, 1.0)
        UsdGeom.SetStageUpAxis(st, UsdGeom.Tokens.z)
        dp = st.GetDefaultPrim()
        # 기하 중심을 원점으로 (참조 후 위치지정이 단순해지도록 자식 Xform에 역translate).
        # 컨버터가 op를 xformOp:orient(쿼터니언)로 저장해 XformCommonAPI는 호환 안 됨
        # → 기존 xformOp:translate 어트리뷰트를 직접 설정(없으면 추가).
        cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
        c = cache.ComputeWorldBound(dp).ComputeAlignedRange().GetMidpoint()
        neg = Gf.Vec3d(-c[0], -c[1], -c[2])
        for child in dp.GetChildren():
            if not child.IsA(UsdGeom.Xformable):
                continue
            tattr = child.GetAttribute("xformOp:translate")
            if tattr and tattr.IsValid():
                tattr.Set(neg)
            else:
                UsdGeom.Xformable(child).AddTranslateOp().Set(neg)
            break
        # 회색 머티리얼 생성+바인드 (원본에 머티리얼 없음)
        mat_path = dp.GetPath().AppendChild("Looks").AppendChild("RailGray")
        mat = UsdShade.Material.Define(st, mat_path)
        sh = UsdShade.Shader.Define(st, mat_path.AppendChild("Surface"))
        sh.CreateIdAttr("UsdPreviewSurface")
        sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*RAIL_GRAY))
        sh.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.2)
        sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.5)
        mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
        for p in st.Traverse():
            if p.IsA(UsdGeom.Mesh):
                UsdShade.MaterialBindingAPI(p).Bind(mat)
        st.GetRootLayer().Save()
    except Exception as _e:
        print(f"[Rail] 마스터 패치 실패: {_e}", flush=True)
        return RAIL_USD_PATH
    return RAIL_MASTER_USD


def load_rail_tiles(stage, label, rail_x, rail_cy, rail_len, floor_z=None):
    """0.9m 레일 세그먼트를 Y축으로 N개(=ceil(rail_len/0.9)) 이어붙여 측면 레일 구성(시각 전용).

    네이티브 장축 Z→월드 Y(X축 RAIL_ROT_DEG 회전), mm→m(RAIL_UNIT_SCALE) 스케일.
    마스터는 중심정렬돼 있어 타일 위치=세그먼트 중심으로 바로 배치된다. 반환: 생성 타일 수.
    """
    import math
    from omni.isaac.core.utils.prims import create_prim
    from pxr import Usd, UsdGeom
    if floor_z is None:
        floor_z = RAIL_FLOOR_Z
    master = prepare_rail_master()
    # 마스터에서 네이티브 단면/길이 실측 → 폭을 RAIL_WIDTH(=RailSlider 폭)에 맞춰 단면 비례 확대,
    # 길이축만 mm→m(RAIL_UNIT_SCALE)로 둬 세그먼트 길이 유지.
    nat_w, seg_len = 35.0, RAIL_SEG_LEN
    try:
        mst = Usd.Stage.Open(master)
        rng = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_]) \
            .ComputeWorldBound(mst.GetDefaultPrim()).ComputeAlignedRange()
        nat_w = (rng.GetMax()[0] - rng.GetMin()[0])                       # 네이티브 폭(=월드 X)
        seg_len = (rng.GetMax()[2] - rng.GetMin()[2]) * RAIL_UNIT_SCALE   # 길이(=월드 Y)
    except Exception as _e:
        print(f"[Rail {label}] 마스터 치수 측정 실패, 기본값 사용: {_e}", flush=True)
    sc = RAIL_WIDTH / nat_w if nat_w > 1e-9 else RAIL_UNIT_SCALE          # 단면 스케일(폭·두께 동일)
    scale_vec = np.array([sc, sc, RAIL_UNIT_SCALE])                       # native (X폭, Y두께, Z길이)
    q = R.from_euler("x", np.radians(RAIL_ROT_DEG)).as_quat()             # [x,y,z,w]
    orient = np.array([q[3], q[0], q[1], q[2]])                           # [w,x,y,z]
    # 내림(floor): 타일 총길이 n*seg_len ≤ rail_len → 레일이 요청 길이(=포스트 Y)를 넘지 않음.
    # (올림이면 포스트를 최대 seg_len 만큼 초과해 깔림)
    n = max(1, int(math.floor(float(rail_len) / seg_len + 1e-6)))
    half_span = float(rail_len) / 2.0
    made = 0
    for i in range(n):
        y_i = rail_cy + (i - (n - 1) / 2.0) * seg_len
        # 안전장치: 타일 바깥 끝이 요청 길이를 벗어나면(=포스트 초과) 건너뜀
        if abs(y_i - rail_cy) + seg_len / 2.0 > half_span + 1e-6:
            continue
        path = f"/World/SideRail_{label}_{i}"
        create_prim(prim_path=path, prim_type="Xform",
                    position=np.array([rail_x, y_i, floor_z]),
                    orientation=orient, scale=scale_vec,
                    usd_path=master)
        prim = stage.GetPrimAtPath(path)
        if not prim or not prim.IsValid():
            continue
        for p in Usd.PrimRange(prim):
            if p.IsInstance():
                p.SetInstanceable(False)
        set_collision_enabled_recursive(stage, path, False)
        made += 1
    print(f"[Rail {label}] SideRail USD {made}/{n} 타일 (w={RAIL_WIDTH:.2f}m, len~{n*seg_len:.1f}m, "
          f"cy={rail_cy:.2f})", flush=True)
    return made


def load_tele_lift_prop(stage, prim_path, copy_name, pos_xyz,
                        rot_deg=TELE_LIFT_UP_ROT_DEG,
                        extra_scale=1.0, freeze_ext=0.0):
    """Vention 텔레스코픽 리프트 USD를 받침 프롭으로 로드(시각 전용).

    레일 로봇용 RailRobotAgent._setup_tele_lift와 동일한 절차를 모듈 함수로 추출한 것:
    - 인스턴스별 USD 사본을 만들고 단위(cm→m)·업축(Y→Z) 메타를 패치해 MetricsAssembler
      자동 단위보정을 끈다(같은 USD를 여러 번 reference할 때 이중 스케일/누락 방지).
    - cm 에셋이라 scale=0.01*extra_scale, X축 rot_deg로 세운다(에셋 원점=베이스).
    - 인스턴스화 해제(내부 튜브 prim 이동 가능) + 콜라이더 비활성화(시각 전용).
    - freeze_ext>0이면 1·2단 튜브(NAUO26/27)를 그 신장량(m)으로 고정.

    pos_xyz: 에셋 베이스 월드 좌표. 회전·스케일 상수를 바꾸면 사본을 지워야 메타 재패치됨.
    반환: 생성 성공 여부(bool).
    """
    import shutil
    from omni.isaac.core.utils.prims import create_prim
    from pxr import Usd, UsdGeom, Gf

    per_inst_usd = os.path.join(os.path.dirname(TELE_LIFT_USD_PATH), copy_name)
    if not os.path.exists(per_inst_usd):
        shutil.copy(TELE_LIFT_USD_PATH, per_inst_usd)
        try:
            _st = Usd.Stage.Open(per_inst_usd)
            UsdGeom.SetStageMetersPerUnit(_st, 1.0)
            UsdGeom.SetStageUpAxis(_st, UsdGeom.Tokens.z)
            _st.GetRootLayer().Save()
        except Exception as _e:
            print(f"[TeleLiftProp] 메타 패치 실패({copy_name}): {_e}", flush=True)

    q = R.from_euler("x", np.radians(rot_deg)).as_quat()   # [x,y,z,w]
    orient = np.array([q[3], q[0], q[1], q[2]])             # [w,x,y,z]
    scale = TELE_LIFT_UNIT_SCALE * float(extra_scale)
    create_prim(prim_path=prim_path, prim_type="Xform",
                position=np.asarray(pos_xyz, dtype=float),
                orientation=orient,
                scale=np.array([scale, scale, scale]),
                usd_path=per_inst_usd)

    lift_prim = stage.GetPrimAtPath(prim_path)
    if not lift_prim or not lift_prim.IsValid():
        print(f"[TeleLiftProp] ⚠ prim 생성 실패: {prim_path}", flush=True)
        return False
    try:
        for p in Usd.PrimRange(lift_prim):
            if p.IsInstance():
                p.SetInstanceable(False)
    except Exception:
        pass
    set_collision_enabled_recursive(stage, prim_path, False)

    # 1·2단 튜브를 고정 신장량으로 세팅(native cm). 로컬 +Y가 세운 뒤 월드 +Z.
    if freeze_ext and freeze_ext > 0.0:
        for _rel, _ext_m in ((TELE_LIFT_STAGE1_PRIM, 0.5 * freeze_ext),
                             (TELE_LIFT_STAGE2_PRIM, freeze_ext)):
            _p = stage.GetPrimAtPath(f"{prim_path}/{_rel}")
            if not _p or not _p.IsValid():
                continue
            _cm = float(_ext_m) / TELE_LIFT_UNIT_SCALE
            _a = _p.GetAttribute("xformOp:translate")
            if _a:
                if "float" in str(_a.GetTypeName()).lower():
                    _a.Set(Gf.Vec3f(0.0, _cm, 0.0))
                else:
                    _a.Set(Gf.Vec3d(0.0, _cm, 0.0))
            else:
                UsdGeom.XformCommonAPI(_p).SetTranslate(Gf.Vec3d(0.0, _cm, 0.0))

    if TELE_LIFT_RECOLOR:
        recolor_tele_lift_dark(stage, prim_path)
    return True


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
    disk.CreateDoubleSidedAttr(True)

    # 시각 전용 스펀지 메시
    sponge = UsdGeom.Mesh.Define(stage, visual_path)
    sponge.GetPointsAttr().Set(pts)
    sponge.GetFaceVertexCountsAttr().Set(fc)
    sponge.GetFaceVertexIndicesAttr().Set(fi)
    sponge.GetExtentAttr().Set(ext)
    sponge.CreateDisplayColorAttr([Gf.Vec3f(0.72, 0.74, 0.76)])
    sponge.CreateDisplayOpacityAttr([0.98])
    sponge.CreateDoubleSidedAttr(True)

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
    UsdPhysics.CollisionAPI.Apply(disk_prim).CreateCollisionEnabledAttr().Set(bool(USE_PHYSICAL_CONTACT_SENSOR))  # 디스크 물리충돌: 플래그 ON이면 실제 접촉/힘, OFF면 가상스프링만(슬램 제거)
    UsdPhysics.MeshCollisionAPI.Apply(disk_prim).CreateApproximationAttr().Set("convexHull")
    try:
        physx_col = PhysxSchema.PhysxCollisionAPI.Apply(disk_prim)
        physx_col.CreateContactOffsetAttr().Set(0.020)   # 0.006→0.020: 접촉 2cm 전부터 생성 → 점진적 힘 램프(슬램 완화)
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
# RailRobotAgent: 레일 로봇 1대 (Y축 슬라이딩 + 3구간 순서 폴리싱)
# ─────────────────────────────────────────────

# ─────────────────────────────────────────────
# 커버리지 공유맵 — 두 로봇이 동일 인스턴스 참조
# ─────────────────────────────────────────────
