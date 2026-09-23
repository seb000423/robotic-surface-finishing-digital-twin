import argparse
import os
import subprocess
import tkinter as tk
from tkinter import simpledialog, messagebox

SCRIPTS_DIR = os.path.dirname(os.path.abspath(__file__))
BASE_DIR = os.path.dirname(SCRIPTS_DIR)
PYTHON_SH = os.environ.get("ISAAC_PYTHON", os.path.expanduser("~/isaacsim/python.sh"))
VALID_MODES = {"single", "multi", "rail", "full"}
MIN_SCAN_POINTS = 100


def parse_args(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("positional", nargs="*", help="Compatibility form: obj_name [single|multi|rail|full]")
    parser.add_argument("--obj_name", type=str, default=None)
    parser.add_argument("--mode", type=str, choices=sorted(VALID_MODES), default=None)
    args, _ = parser.parse_known_args(argv)

    positional = list(args.positional)
    obj_name = args.obj_name
    mode = args.mode

    if obj_name is None and positional:
        obj_name = positional.pop(0)
    if mode is None and positional:
        mode = positional.pop(0)

    obj_name = (obj_name or "car").strip().lower()
    mode = (mode or "full").strip().lower()
    if mode not in VALID_MODES:
        parser.error(f"mode must be one of: {', '.join(sorted(VALID_MODES))}")
    return obj_name, mode


def count_ply_vertices(ply_path):
    if not os.path.exists(ply_path):
        return 0
    with open(ply_path, "r", encoding="utf-8", errors="ignore") as f:
        for line in f:
            if line.startswith("element vertex"):
                parts = line.split()
                if len(parts) >= 3:
                    return int(parts[2])
            if line.strip() == "end_header":
                break
    return 0


def main(argv=None):
    # 1. 커맨드 라인 인자 파싱
    # 2. 모드 선택: single(1대) / multi(4대 고정) / rail(2대 레일) / full(상단 1대 + 측면 2대)
    obj_name, mode = parse_args(argv)

    print(f"[INFO] 선택된 물체: {obj_name}, 모드: {mode}")

    # 3. USD 파일 존재 여부 확인
    usd_path = os.path.join(BASE_DIR, "scan_obj", f"{obj_name}.usd")
    if not os.path.exists(usd_path):
        print(f"[ERROR] 물체 파일이 존재하지 않습니다: {usd_path}")
        return

    print(f"[INFO] '{obj_name}'에 대한 스캔 및 경로 생성을 시작합니다...")

    # 스캔 실행 (scan.py)
    print("--------------------------------------------------")
    print(f">>> Running scan.py --obj_name {obj_name}")
    scan_result = subprocess.run([PYTHON_SH, os.path.join(SCRIPTS_DIR, "scan.py"), "--obj_name", obj_name, "--headless"])
    if scan_result.returncode != 0:
        print("[ERROR] 스캔 과정에서 오류가 발생했습니다.")
        return
    ply_path = os.path.join(BASE_DIR, "scan_result", obj_name, "points", "real_camera_surface_points.ply")
    scan_point_count = count_ply_vertices(ply_path)
    if scan_point_count < MIN_SCAN_POINTS:
        print(f"[ERROR] 스캔 포인트가 너무 적습니다: {scan_point_count}개 ({ply_path})")
        return
    print(f"[INFO] 스캔 포인트 확인 완료: {scan_point_count}개")

    # 경로 생성
    print("--------------------------------------------------")
    path_mode = mode if mode in {"rail", "full"} else "multi"
    print(f">>> Running path_generator.py --obj_name {obj_name} --mode {path_mode}")
    path_result = subprocess.run([
        "python3", os.path.join(SCRIPTS_DIR, "path_generator.py"),
        "--obj_name", obj_name, "--mode", path_mode,
    ])
    if path_result.returncode != 0:
        print("[ERROR] 3D 경로 생성 과정에서 오류가 발생했습니다.")
        return

    # 폴리싱 시뮬레이션 실행
    print("--------------------------------------------------")
    if mode == "single":
        print(f">>> Running polishing_v1.py --obj_name {obj_name}  (1대 모드)")
        sim_script = os.path.join(SCRIPTS_DIR, "polishing_v1.py")
    elif mode == "rail":
        print(f">>> Running polishing_v5.py --obj_name {obj_name}  (2대 레일 모드)")
        sim_script = os.path.join(SCRIPTS_DIR, "polishing_v5.py")
    elif mode == "full":
        print(f">>> Running polishing_v5.py --obj_name {obj_name}  (3대 full 모드: 오버헤드 1대 + 측면 2대)")
        sim_script = os.path.join(SCRIPTS_DIR, "polishing_v5.py")
    else:
        print(f">>> Running polishing_v4.py --obj_name {obj_name}  (4대 동시 모드)")
        sim_script = os.path.join(SCRIPTS_DIR, "polishing_v4.py")

    sim_result = subprocess.run([PYTHON_SH, sim_script, "--obj_name", obj_name])
    if sim_result.returncode != 0:
        print("[ERROR] 폴리싱 시뮬레이션 과정에서 오류가 발생했습니다.")
        return
    print("[INFO] 파이프라인 종료.")

if __name__ == "__main__":
    main()
