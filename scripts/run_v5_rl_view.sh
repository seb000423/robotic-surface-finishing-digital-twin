#!/usr/bin/env bash
# 차 전체 폴리싱 시뮬 + 강화학습 잔차 정책 — 창(GUI)으로 직접 보기.
# bash scripts/run_v5_rl_view.sh              # Isaac Sim 창이 뜨고 3대(천장 C·측면 SL/SR)가 차체를 닦는다
# bash scripts/run_v5_rl_view.sh --headless   # 창 없이(검증용)
# 주의: Isaac 프로세스는 한 번에 하나만(셀 순회/재실행이 돌고 있으면 끝난 뒤 실행).
set -u
cd "$(dirname "$0")/.."
ROOT="$PWD"      # 아래 경로는 전부 절대경로로 — 실행은 scripts/ 에서 하므로 상대경로면 못 찾는다
if pgrep -f "car_cells_rob[o]t.py|demo_ar[m].py|polishing_v[5].py" >/dev/null; then
  echo "다른 Isaac 프로세스가 실행 중입니다 — 끝난 뒤 다시 실행하세요:"; pgrep -fa "car_cells_rob[o]t.py|demo_ar[m].py|polishing_v[5].py" | cut -c1-120; exit 1
fi
mkdir -p learning/ui_bridge/out
export POLISH_RL=1                                   # 잔차 정책(힘·이송 보정) 켜기
export POLISH_RL_RECIPE_TOP=${POLISH_RL_RECIPE_TOP:-$ROOT/learning/digital_twin/outputs/bo_best_recipe_top.json}
export POLISH_RL_RECIPE_SIDE=${POLISH_RL_RECIPE_SIDE:-$ROOT/learning/digital_twin/outputs/bo_best_recipe_side.json}
export POLISH_RL_OUT=${POLISH_RL_OUT:-$ROOT/learning/ui_bridge/out/view_cells.csv}   # 셀 판정 CSV(종료 시)
export POLISH_MONITOR_FEED=${POLISH_MONITOR_FEED:-$ROOT/learning/ui_bridge/out/monitor_feed.json}  # 모니터 LIVE 피드
export POLISH_PHYSICAL_CONTACT=${POLISH_PHYSICAL_CONTACT:-0}   # 0 = 가상 접촉(안정), 1 = PhysX 패드 접촉(실험)
export POLISH_ROS_PUBLISH=0 POLISH_ROS_CAMERAS=0     # ROS 불필요
export POLISH_RENDER_EVERY=${POLISH_RENDER_EVERY:-1} # 1 = 매 스텝 렌더(보기용). 빠르게 돌리려면 4~10
export POLISH_RENDERER=${POLISH_RENDERER:-RaytracedLighting}  # RTX Real-Time(가벼움). 6.0 기본 RealTimePathTracing 은 무거움
export POLISH_SPEED_SCALE=${POLISH_SPEED_SCALE:-3.0} # 접촉 중 이송 배속(검증된 상한 3.0)
export POLISH_EXIT_WHEN_DONE=${POLISH_EXIT_WHEN_DONE:-0}       # 1 이면 완료 시 자동 종료
export POLISH_RECORD=${POLISH_RECORD:-$ROOT/learning/ui_bridge/out/run_view_$(date +%Y%m%d_%H%M%S).sqlite}   # 기록(웹 UI 재생용). 0 이면 끔
[ "$POLISH_RECORD" = "0" ] && unset POLISH_RECORD
for v in POLISH_RL_RECIPE_TOP POLISH_RL_RECIPE_SIDE POLISH_RL_OUT POLISH_MONITOR_FEED POLISH_RECORD; do
  val="${!v:-}"; [ -n "$val" ] && [ "${val#/}" = "$val" ] && export "$v=$ROOT/$val"
done
PY=${ISAAC_PY:-$HOME/isaacsim/python.sh}
echo "[v5-rl-view] 레시피 top=$POLISH_RL_RECIPE_TOP side=$POLISH_RL_RECIPE_SIDE, 접촉=${POLISH_PHYSICAL_CONTACT}, 피드=$POLISH_MONITOR_FEED"
if [ -n "${POLISH_RECORD:-}" ]; then
  echo "[v5-rl-view] 기록 → $POLISH_RECORD"
fi
cd scripts && exec "$PY" polishing_v5.py --obj_name car "$@"
