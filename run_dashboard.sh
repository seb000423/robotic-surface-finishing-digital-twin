#!/usr/bin/env bash
# ===========================================================================
# 웹 대시보드(UI) + ROS2 브릿지 + Isaac Sim 런처를 한 번에 실행
#
#   ./run_dashboard.sh
#
# 동작:
#   1) rosbridge_server 실행 (포트 9090)
#      → 브라우저 UI가 ROS2 토픽(/polishing/*)을 WebSocket으로 구독.
#   2) dashboard_launcher.py 실행 (포트 8765)
#      → UI '시작' 버튼이 이 서버로 요청 → Isaac Sim(polishing_v5.py) 실행.
#   3) 웹 UI(Vite dev server) 실행 (포트 5173)
#      → 브라우저에서 http://localhost:5173 접속.
#
# 다른 컴퓨터에서 Isaac Sim 경로가 다르면:
#   export ISAAC_PYTHON=/내/경로/isaacsim/python.sh
#   ./run_dashboard.sh
# ===========================================================================
set -e
HERE="$(cd "$(dirname "$0")" && pwd)"
PIDS=()
cleanup() { for p in "${PIDS[@]}"; do kill "$p" 2>/dev/null || true; done; }
trap cleanup EXIT

# --- 1) ROS2 rosbridge_server (UI ↔ ROS2 통신) ---
ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
if [ -f "$ROS_SETUP" ]; then
  # shellcheck disable=SC1090
  source "$ROS_SETUP"
  if ros2 pkg prefix rosbridge_server >/dev/null 2>&1; then
    echo "[run_dashboard] rosbridge_server 실행 (ws://localhost:9090)"
    ros2 launch rosbridge_server rosbridge_websocket_launch.xml >/tmp/rosbridge.log 2>&1 &
    PIDS+=($!)
  else
    echo "[run_dashboard] ⚠ rosbridge_server 미설치 → ROS2 통신 불가 (UI는 데모 모드)"
    echo "    설치: sudo apt install ros-humble-rosbridge-suite"
  fi
else
  echo "[run_dashboard] ⚠ ROS2($ROS_SETUP) 없음 → ROS2 통신 생략 (UI는 데모 모드)"
fi

# --- 2) Isaac Sim 런처 서버 ('시작' 버튼 처리) ---
echo "[run_dashboard] Isaac Sim 런처 실행 (http://localhost:8765)"
python3 "$HERE/scripts/dashboard_launcher.py" &
PIDS+=($!)

# --- 3) 웹 UI ---
cd "$HERE/web_dashboard"
if [ ! -d node_modules ]; then
  echo "[run_dashboard] node_modules 없음 → npm install 실행"
  npm install
fi
echo "[run_dashboard] UI: http://localhost:5173"
npm run dev
