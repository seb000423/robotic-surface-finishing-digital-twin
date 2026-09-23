#!/usr/bin/env bash
# 시뮬레이션 1회 실행 → 로그를 learning/data/raw/<타임스탬프>/ 로 수집
# ./learning/collect_run.sh [오브젝트] [실행분]
# 예)
# ./learning/collect_run.sh                 # car, 30분, headless
# ./learning/collect_run.sh car 45          # car, 45분
# ./learning/collect_run.sh car_small 20    # 다른 차체
# HEADLESS=0 ./learning/collect_run.sh      # GUI 창 띄우고 보면서 실행
# 왜 시간 제한을 두나:
# runner의 메인 루프는 `while simulation_app.is_running()` 이라 폴리싱이 끝나도
# 저절로 종료되지 않는다 (완료 후 재폴리싱 패스를 반복한다). 그래서 정해진 시간만
# 돌리고 끊는다. 오래 돌릴수록 커버 구간이 늘고 재폴리싱 데이터도 쌓인다.
# 왜 실행 후 바로 복사하나:
# scripts/*.csv 는 git에 커밋돼 있어서 git checkout/stash 로 덮어써진다.
set -uo pipefail

PY="${ISAAC_PYTHON:-$HOME/isaacsim_venv/bin/python}"
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
SCRIPTS_DIR="$ROOT/scripts"
OBJ="${1:-car}"
MINUTES="${2:-30}"
STAMP="$(date +%Y%m%d_%H%M%S)"
DEST="$ROOT/learning/data/raw/${STAMP}_${OBJ}"

# 렌더링을 줄이면 같은 시간에 더 많은 시뮬 스텝을 돈다 = 더 많은 구간 커버.
export POLISH_RENDER_EVERY="${POLISH_RENDER_EVERY:-10}"
# ROS 퍼블리시는 학습 데이터와 무관 — 끄면 의존성 문제와 오버헤드를 피한다.
export POLISH_ROS_PUBLISH="${POLISH_ROS_PUBLISH:-0}"
export POLISH_ROS_CAMERAS="${POLISH_ROS_CAMERAS:-0}"
# POLISH_SPEED_SCALE / POLISH_TRAVEL_SCALE 은 건드리지 말 것.
# 이송속도 라벨에 직접 곱해지는 값이라, 실행마다 바꾸면 같은 상태에 다른 정답이
# 붙어서(멀티모달) 학습이 나빠진다. 다양성은 오브젝트/구간에서 얻는다.

if [ ! -x "$PY" ]; then
  echo "[ERROR] Isaac Sim 파이썬을 찾을 수 없음: $PY"
  echo "        ISAAC_PYTHON=/경로/python 로 지정하세요."
  exit 1
fi

HEADLESS_FLAG="--headless"
[ "${HEADLESS:-1}" = "0" ] && HEADLESS_FLAG=""

echo "=========================================================="
echo " 오브젝트     : $OBJ"
echo " 실행 시간    : ${MINUTES}분"
echo " 모드         : ${HEADLESS_FLAG:-GUI}  (RENDER_EVERY=$POLISH_RENDER_EVERY)"
echo " 수집 위치    : $DEST"
echo "=========================================================="

cd "$SCRIPTS_DIR"
# shellcheck disable=SC2086
"$PY" polishing_v5.py --obj_name "$OBJ" $HEADLESS_FLAG &
SIM_PID=$!

collect() {
  echo ""
  echo "[collect] 시뮬 종료 중 (pid $SIM_PID)..."
  kill "$SIM_PID" 2>/dev/null || true
  for _ in $(seq 20); do
    kill -0 "$SIM_PID" 2>/dev/null || break
    sleep 1
  done
  kill -9 "$SIM_PID" 2>/dev/null || true

  mkdir -p "$DEST"
  cp "$SCRIPTS_DIR"/force_log_rail_*.csv "$DEST"/ 2>/dev/null || true
  # status_log.txt 는 CSV가 유실됐을 때의 대체 소스 — 반드시 같이 보관
  cp "$SCRIPTS_DIR/status_log.txt" "$DEST"/ 2>/dev/null || true

  echo "[collect] 수집 완료: $DEST"
  for f in "$DEST"/*; do
    printf '  %-28s %s행\n' "$(basename "$f")" "$(wc -l < "$f")"
  done
  echo ""
  echo "다음 단계 — 데이터셋 재생성 후 재학습:"
  echo "  $PY learning/bc/extract_dataset.py"
  echo "  $PY learning/bc/train.py"
  echo "  $PY learning/bc/evaluate.py"
}
trap collect INT TERM

echo "[run] ${MINUTES}분 동안 실행합니다. 중간에 멈추려면 Ctrl+C (그래도 로그는 수집됨)."
sleep $(( MINUTES * 60 )) &
WAIT_PID=$!
wait "$WAIT_PID"

trap - INT TERM
collect
