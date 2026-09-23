#!/usr/bin/env bash
# 레시피 미니 격자: 이송 ×1.3/1.5/1.7 × 힘 ×1.0/1.15 — 신차 150셀 해석식 판정(CPU, 4 workers)
cd "$(dirname "$0")/../.."
PY=$HOME/isaacsim/python.sh
G=learning/rl/logs/recipe_time/grid
for feed in 1.3 1.5 1.7; do for force in 1.0 1.15; do
  tag="feed${feed}_f${force}"
  [ -f "$G/result_${tag}_summary.json" ] && { echo "[grid] $tag 있음, 건너뜀"; continue; }
  echo "[grid] $(date +%H:%M:%S) $tag 시작"
  $PY learning/vehicle_export/export_vehicle_results.py --checkpoint learning/rl/champion/model_terminal_ppo_14ch_it800.pt \
     --input learning/vehicle_export/vehicle_150_cells_newcar.csv --output "$G/result_${tag}.csv" \
     --recipe_json "$G/${tag}_top.json" --recipe_json_side "$G/${tag}_side.json" --workers 4 2>&1 | grep -vE "^\[|omni\." | tail -3
  echo "[grid] $(date +%H:%M:%S) $tag 종료"
done; done
echo "[grid] 완료"
