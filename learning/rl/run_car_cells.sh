#!/usr/bin/env bash
# 차 전체 셀 순회 드라이버 — 한 배치(num_envs 셀) = Isaac 프로세스 하나. 순차 실행.
# bash learning/rl/run_car_cells.sh [start] [end] [batch] [checkpoint] [out_csv]
# 예: 전체 483셀  → bash learning/rl/run_car_cells.sh 0 482 8
# detached    → setsid nohup bash learning/rl/run_car_cells.sh 0 490 8 > learning/rl/robot/results/car_cells.log 2>&1 &
set -u
cd "$(dirname "$0")/../.."
START=${1:-0}; END=${2:-490}; BATCH=${3:-8}
CKPT=${4:-learning/rl/robot/champion/model_ppo_curved.pt}
OUT=${5:-learning/rl/robot/results/car_cells.csv}
PY=${ISAAC_PY:-$HOME/isaacsim/python.sh}
mkdir -p "$(dirname "$OUT")"
i=$START
while [ "$i" -le "$END" ]; do
  j=$(( i + BATCH - 1 )); [ "$j" -gt "$END" ] && j=$END
  echo "[run_car_cells] $(date +%H:%M:%S) 배치 $i-$j 시작"
  timeout 3600 "$PY" learning/rl/car_cells_robot.py --headless --checkpoint "$CKPT" \
      --cells "$i-$j" --num_envs "$BATCH" --out "$OUT" 2>&1 | grep -E "\[car_cells\]|Traceback|Error\]" | grep -v "omni.kit.app"
  echo "[run_car_cells] $(date +%H:%M:%S) 배치 $i-$j 종료 (누적 행: $(( $(wc -l < "$OUT" 2>/dev/null || echo 1) - 1 )))"
  i=$(( j + 1 ))
done
echo "[run_car_cells] 완료 → $OUT"
