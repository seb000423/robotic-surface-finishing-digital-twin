"""학습 파이프라인 공통 설정 — 기존 시뮬레이션 코드와 분리된 learning/ 전용."""
import os

# ── 경로 ──
LEARNING_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.dirname(LEARNING_DIR)
SCRIPTS_DIR = os.path.join(PROJECT_ROOT, "scripts")          # 기존 v5 코드 (읽기 전용으로만 사용)
SCAN_DIR = os.path.join(PROJECT_ROOT, "scan_result", "car")  # 경로/점군
PLY_PATH = os.path.join(SCAN_DIR, "points", "real_camera_surface_points.ply")
RAIL_CONFIG_PATH = os.path.join(SCAN_DIR, "rail_config.json")

DATA_DIR = os.path.join(LEARNING_DIR, "data", "processed")
CHECKPOINT_DIR = os.path.join(LEARNING_DIR, "checkpoints")
OUTPUT_DIR = os.path.join(LEARNING_DIR, "outputs")

DATASET_PATH = os.path.join(DATA_DIR, "bc_dataset.npz")
CHECKPOINT_PATH = os.path.join(CHECKPOINT_DIR, "bc_mlp.pt")

# 추출 대상 로그 (기존 워크스페이스 파일 — 읽기만 함)
RAIL_LOGS = {
    "C": os.path.join(SCRIPTS_DIR, "force_log_rail_C.csv"),
    "SL": os.path.join(SCRIPTS_DIR, "force_log_rail_SL.csv"),
    "SR": os.path.join(SCRIPTS_DIR, "force_log_rail_SR.csv"),
}
# 시뮬 재실행 로그 누적 위치 — learning/data/raw/<날짜>/force_log_rail_*.csv 로 복사해두면
# extract_dataset.py가 scripts/ 최신본과 함께 전부 읽는다 (내용 동일 파일은 자동 중복 제거).
# status_log.txt도 같이 복사해두면 CSV가 유실된 레일의 대체 소스로 쓰인다 (아래 참고).
RAW_LOG_DIR = os.path.join(LEARNING_DIR, "data", "raw")

# 상태 로그 — CSV 유실 시 대체 소스.
# agent.py에서 CSV 기록과 status_log 진단(tag=POLISH)은 같은 if 블록에서 연달아 실행되므로
# 두 로그의 POLISH 행은 1:1 대응한다 (C 레일 457건 전수 대조로 filt 값 완전 일치 확인).
# 단 status_log의 idx는 정수(current_target_idx)라 CSV의 path_idx(float)보다 해상도가 낮다.
# 그대로 쓰면 속도가 74% 0으로 양자화되지만, 라벨 스무딩(w=21) 후 상관계수 0.9944로 회복된다.
STATUS_LOG_NAME = "status_log.txt"
STATUS_LOG_PATH = os.path.join(SCRIPTS_DIR, STATUS_LOG_NAME)

# ── 데이터 정의 ──
# 상태(입력): 표면 위치, 법선, 곡률, 경로 진행률, 피니싱 단계
# ── v2: 절대 위치(pos_x/y/z) 제거 ────────────────────────────
# 이유: permutation importance에서 pos_z가 의존도 1위(+0.129N)였다. 정작 tilt(+0.017)와
#   curvature(+0.003)는 거의 쓰지 않았다 = "82° 경사면이면 3N"이 아니라
#   "높이 1.44m 지점이면 3N"을 외우고 있었다는 뜻. 차량 특정 좌표라 다른 오브젝트로 전이 불가.
#   RL은 Isaac Lab의 "로봇 1대 + 굴곡 오브젝트" 환경에서 진행하므로 전이가 필수.
# 실측: 위치를 빼니 오히려 좋아졌다 (힘 0.330→0.285N, 속도 69.7→63.3mm/s).
#   절대 좌표가 목발 역할을 하며 과적합을 유발하고 있었다.
# 주의: pos 자체는 extract에서 여전히 계산한다 (법선·곡률·이송속도 산출에 필요).
#   학습 입력에서만 빼고, 디버깅용으로 aux에 보존한다.
# 열린 질문: normal_x/y도 월드 축 기준이라 오브젝트 회전 시 달라진다 (tilt=normal_z만 회전 불변).
#   다만 importance가 ~0이라 이번엔 유지. 전이가 문제되면 6개 입력안으로 축소 검토.
STATE_COLUMNS = [
    "normal_x", "normal_y", "normal_z",  # 표면 법선
    "tilt_deg",                     # 표면 기울기 (법선-수직 각도)
    "curvature",                    # 국소 곡률 (PCA surface variation)
    "progress",                     # 구간 내 경로 진행률 0~1
    "is_side",                      # 측면(1)/천장(0) 작업면
    "phase",                        # 피니싱 단계 (러핑/중간/피니싱) — 현재 로그엔 없어 0 고정
    "prev_force",                   # 직전 POLISH 스텝의 실측 힘 filtered(t-1) [N] — 누수 방지 1스텝 지연
]
# ── 행동(출력) 정의와 배포 계약 ───────────────────────────────────────────────
# 회전속도는 고정이라 제외 — (접촉력, 이송속도) 2개.
# 이름 주의: 예전 이름은 `target_force_n`이었지만 값은 "목표힘"이 아니라
#   "실제 달성된 힘"이라 혼동을 낳았다. 시뮬에는 서로 다른 값이 셋 있다:
#     ① 목표힘 setpoint  = adaptive_target_force(tilt)  ← 규칙 공식, agent._target_force
#     ② 압입 깊이 z_offset = 어드미턴스 제어기가 실제로 움직이는 액추에이터 명령
#     ③ 실측 접촉력 filtered = 물리엔진이 낸 결과
#   우리가 라벨로 쓰는 건 ③이다. ①은 tilt의 결정적 함수라 학습 정보가 0이기 때문.
# ── 배포 계약 (deployment contract) ──
#   BC 출력 contact_force_n 은 배포 시 어드미턴스 제어기의 목표힘(agent._target_force)
#   자리에 투입한다. 즉 "이 표면에서는 이만큼의 힘이 실제로 걸리더라"를 그대로 목표로 삼는다.
#   근거 — 규칙의 목표힘은 물리적으로 도달 불가능한 값이다 (실측):
#     · 명령 평균 5.82N vs 실제 평균 3.12N, 98%의 스텝에서 명령 > 실제
#     · C 레일은 전체 스텝의 73.5%가 press_min(최대 압입 깊이)에 포화된 채로도
#       평균 3.27N밖에 못 냈다. 더 누를 여유가 없는 상태로 계속 밀고 있었다는 뜻.
#   따라서 ①을 그대로 복제하면 제어기가 계속 깊이 한계에 붙어 있게 되고,
#   2단계 RL 잔차가 Δ를 줘도 한쪽 방향으로는 아무 효과가 없다.
#   ③을 목표로 주면 제어기가 포화되지 않는 지점에 안착해 양방향 보정 여유가 생긴다.
#   ※ 이 계약은 아직 시뮬에 연결해 검증한 것이 아니다. 2단계 RL에서 정책을 실제로
#     투입할 때 z_offset 포화율과 힘 추종 오차로 반드시 확인할 것.
ACTION_COLUMNS = [
    "contact_force_n",  # 실측 접촉력 filtered(t) [N] — 배포 시 목표힘 setpoint로 사용
    "feed_speed_mps",   # 이번 스텝 경로 이송속도 [m/s]
]

PHYSICS_HZ = 60.0  # world.step 주기 (PATH_ADVANCE_PER_STEP = 1/60 기준)

# 라벨 스무딩 창 (홀수 스텝, 궤적별 centered rolling mean; 21스텝 ≈ 0.35초)
# 스텝 단위 힘/속도 요동은 물리 노이즈라 상태로부터 예측 불가 —
# BC가 배울 대상은 "그 지점에서 유지되는 부드러운 프로파일"(a_base)이고
# 순간 보정은 RL 잔차 담당. 실험 근거: raw 라벨은 naive(직전 힘 복사)보다 못하지만
# w=21이면 naive 0.63N vs 모델 0.44N으로 역전, 속도 MAE 131→94mm/s.
# 입력 prev_force는 배포 시 실제 센서값이므로 스무딩하지 않는다.
LABEL_SMOOTH_WINDOW = 21

# ── 학습 하이퍼파라미터 ──
HIDDEN_SIZES = [128, 128]
LEARNING_RATE = 1e-3
BATCH_SIZE = 256
MAX_EPOCHS = 500
EARLY_STOP_PATIENCE = 40
VAL_FRACTION = 0.2   # (rail, seg) 그룹 단위 분할
SEED = 42
