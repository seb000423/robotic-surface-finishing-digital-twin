#!/usr/bin/env python3
"""웹 대시보드 '시작' 버튼용 런처 서버 (시스템 python3 로 실행, 의존성 없음).

브라우저는 프로세스를 직접 띄울 수 없으므로, 이 작은 HTTP 서버가
'시작' 버튼 요청을 받아 Isaac Sim 폴리싱 시뮬을 실행한다.

    POST /start   → isaac_python polishing_v5.py --obj_name <car>  실행
    POST /stop    → 실행 중인 시뮬 종료
    GET  /status  → 실행 여부/PID

실행:
    python3 dashboard_launcher.py
    (기본 http://127.0.0.1:8765, 환경변수로 변경 가능)

환경변수:
    LAUNCHER_HOST       (기본 127.0.0.1)
    LAUNCHER_PORT       (기본 8765)
    ISAAC_PYTHON        (기본 ~/isaacsim/python.sh)
    POLISH_SCRIPTS_DIR  (기본 이 파일이 있는 디렉터리)
"""
import json
import os
import subprocess
import sys
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

HOST = os.environ.get("LAUNCHER_HOST", "127.0.0.1")
PORT = int(os.environ.get("LAUNCHER_PORT", "8765"))
ISAAC_PYTHON = os.path.expanduser(os.environ.get(
    "ISAAC_PYTHON",
    "~/isaacsim/python.sh",
))
SCRIPTS_DIR = os.environ.get(
    "POLISH_SCRIPTS_DIR", os.path.dirname(os.path.abspath(__file__)))
POLISH_SCRIPT = "polishing_v5.py"
# 실제 Isaac 폴리싱은 bmw 한 대뿐 → 대시보드에서 어떤 차종을 골라도 폴리싱은 bmw(car) 에셋으로 실행.
POLISH_OBJ = os.environ.get("POLISH_OBJ", "car")

_lock = threading.Lock()
_proc = None  # 현재 실행 중인 시뮬 subprocess
_scan_thread = None  # 스캔(scan.py→path_generator.py) 시퀀스 스레드


def _safe_obj(name):
    name = (name or "car").strip().lower()
    return name if name.replace("_", "").isalnum() else "car"


def _scan_running():
    return _scan_thread is not None and _scan_thread.is_alive()


def _run_scan_seq(obj_name):
    """scan.py(isaac_python) → path_generator.py(python3) 순차 실행."""
    try:
        print(f"[launcher] 스캔 시작: scan.py --obj_name {obj_name}", flush=True)
        p1 = subprocess.Popen([ISAAC_PYTHON, "scan.py", "--obj_name", obj_name], cwd=SCRIPTS_DIR)
        p1.wait()
        print(f"[launcher] 경로 생성: path_generator.py --obj_name {obj_name}", flush=True)
        p2 = subprocess.Popen(["python3", "path_generator.py", "--obj_name", obj_name], cwd=SCRIPTS_DIR)
        p2.wait()
        print("[launcher] 스캔/경로 생성 완료", flush=True)
    except Exception as e:  # noqa: BLE001
        print(f"[launcher] 스캔 오류: {e}", flush=True)


def _scan(obj_name="car"):
    global _scan_thread
    with _lock:
        if _scan_running():
            return {"status": "already_running"}
        if not os.path.exists(ISAAC_PYTHON):
            return {"status": "error", "message": f"isaac_python 없음: {ISAAC_PYTHON}"}
        obj = _safe_obj(obj_name)
        _scan_thread = threading.Thread(target=_run_scan_seq, args=(obj,), daemon=True)
        _scan_thread.start()
        return {"status": "started", "obj_name": obj}


def _is_running():
    return _proc is not None and _proc.poll() is None


def _start(obj_name="car"):
    """시뮬을 실행. 이미 돌고 있으면 그대로 둔다."""
    global _proc
    with _lock:
        if _is_running():
            return {"status": "already_running", "pid": _proc.pid}
        if not os.path.exists(ISAAC_PYTHON):
            return {"status": "error",
                    "message": f"isaac_python 없음: {ISAAC_PYTHON}"}
        script_path = os.path.join(SCRIPTS_DIR, POLISH_SCRIPT)
        if not os.path.exists(script_path):
            return {"status": "error",
                    "message": f"스크립트 없음: {script_path}"}
        cmd = [ISAAC_PYTHON, POLISH_SCRIPT, "--obj_name", obj_name]
        print(f"[launcher] 실행: (cwd={SCRIPTS_DIR}) {' '.join(cmd)}", flush=True)
        _proc = subprocess.Popen(cmd, cwd=SCRIPTS_DIR)
        return {"status": "started", "pid": _proc.pid, "obj_name": obj_name}


def _stop():
    global _proc
    with _lock:
        if not _is_running():
            return {"status": "not_running"}
        pid = _proc.pid
        _proc.terminate()
        try:
            _proc.wait(timeout=10)
        except subprocess.TimeoutExpired:
            _proc.kill()
        print(f"[launcher] 종료: pid={pid}", flush=True)
        return {"status": "stopped", "pid": pid}


def _status():
    with _lock:
        if _is_running():
            return {"running": True, "pid": _proc.pid}
        rc = _proc.returncode if _proc is not None else None
        return {"running": False, "returncode": rc}


class Handler(BaseHTTPRequestHandler):
    def _cors(self):
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")

    def _json(self, payload, code=200):
        body = json.dumps(payload).encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self._cors()
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_OPTIONS(self):
        self.send_response(204)
        self._cors()
        self.end_headers()

    def _read_obj_name(self):
        # body(JSON) 또는 쿼리스트링에서 obj_name 추출 (기본 car)
        obj = "car"
        try:
            length = int(self.headers.get("Content-Length", 0))
            if length:
                data = json.loads(self.rfile.read(length).decode("utf-8") or "{}")
                obj = str(data.get("obj_name", obj)).strip().lower() or "car"
        except Exception:
            pass
        return obj

    def do_POST(self):
        path = self.path.split("?")[0]
        if path == "/start":
            # 폴리싱은 bmw 한 대만 → 대시보드 선택과 무관하게 POLISH_OBJ(car)로 실행
            self._json(_start(POLISH_OBJ))
        elif path == "/scan":
            self._json(_scan(self._read_obj_name()))
        elif path == "/stop":
            self._json(_stop())
        else:
            self._json({"status": "error", "message": "unknown endpoint"}, 404)

    def do_GET(self):
        path = self.path.split("?")[0]
        if path == "/status":
            self._json(_status())
        elif path == "/scan_status":
            self._json({"running": _scan_running()})
        elif path in ("/", "/health"):
            self._json({"ok": True, "isaac_python": ISAAC_PYTHON,
                        "scripts_dir": SCRIPTS_DIR})
        else:
            self._json({"status": "error", "message": "unknown endpoint"}, 404)

    def log_message(self, *args):
        pass  # 콘솔 과다 출력 억제


def main():
    print(f"[launcher] 대시보드 런처 서버 시작: http://{HOST}:{PORT}", flush=True)
    print(f"[launcher] isaac_python = {ISAAC_PYTHON}", flush=True)
    print(f"[launcher] scripts_dir  = {SCRIPTS_DIR}", flush=True)
    if not os.path.exists(ISAAC_PYTHON):
        print(f"[launcher] isaac_python 경로가 존재하지 않습니다. "
              f"ISAAC_PYTHON 환경변수로 지정하세요.", flush=True)
    server = ThreadingHTTPServer((HOST, PORT), Handler)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\n[launcher] 종료", flush=True)
        _stop()
        server.shutdown()


if __name__ == "__main__":
    sys.exit(main())
