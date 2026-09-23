"""차 전체 셀 순회 — PhysX 로봇 환경(PhysX 실접촉) env 로 스캔 차체의 셀을 배치 단위로 폴리싱·판정.

    ~/isaacsim/python.sh learning/rl/car_cells_robot.py --headless \
        --checkpoint learning/rl/robot/champion/model_ppo_curved.pt \
        --cells 0-15 --num_envs 8 --out learning/rl/robot/results/car_cells.csv

원리:
  · 스캔 점군 → rl_bridge.CellRegistry 로 12 cm 셀 격자(윗면/측면/전후면) + 셀별 트윈 초기 상태.
  · 셀마다 점군을 셀 로컬 프레임(u, v, 법선)으로 옮겨 2차곡면 h(u,v) 최소제곱 → 로봇 env 의
    "quad" 작업면(kinematic trimesh)으로 스폰. env 하나 = 셀 하나 (배치 num_envs 개).
  PhysX 로봇 환경 챔피언(14ch, PhysX 접촉 학습)이 재폴리싱 상태기계(evaluate→cooldown→retry)로
    닦고, 결과를 150셀 판정과 같은 5종 기준 + 보증 플래그 + 처분으로 기록한다.
  · 한 프로세스 = 한 배치 (Isaac 스테이지 재생성 회피). 전체 순회는 --cells 범위를 바꿔
    여러 번 호출 (run_car_cells.sh).
"""
import argparse
import csv
import os
import sys

_REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, _REPO)

from isaaclab.app import AppLauncher  # noqa: E402

parser = argparse.ArgumentParser()
parser.add_argument("--checkpoint", type=str, required=True)
parser.add_argument("--ply", type=str,
                    default=os.path.join(_REPO, "scan_result", "car", "points",
                                         "real_camera_surface_points.ply"))
parser.add_argument("--profile", type=str, default="new_car", choices=["new_car", "correction"])
parser.add_argument("--cells", type=str, default="0-7",
                    help="셀 id 범위 'a-b' 또는 콤마 목록. 개수는 --num_envs 이하")
parser.add_argument("--num_envs", type=int, default=8)
parser.add_argument("--max_passes", type=int, default=4)
parser.add_argument("--cooldown_s", type=float, default=20.0)
parser.add_argument("--pass_time_factor", type=float, default=1.7)
parser.add_argument("--max_control_steps", type=int, default=120000)
parser.add_argument("--registry_seed", type=int, default=7000)
parser.add_argument("--force_scale", type=float, default=1.0,
                    help="강곡률 재실행: 레시피 목표 힘 배율 (감압, 예 0.7)")
parser.add_argument("--pad_radius", type=float, default=None,
                    help="강곡률 재실행: 물리 패드 반경 m (예 0.035 = Ø70 소형 패드)")
parser.add_argument("--tag", type=str, default="", help="결과 checkpoint 열에 붙일 태그")
parser.add_argument("--out", type=str,
                    default=os.path.join(_REPO, "learning", "rl", "robot", "results", "car_cells.csv"))
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
app = AppLauncher(args).app

import numpy as np  # noqa: E402
import torch  # noqa: E402
from rsl_rl.models.mlp_model import MLPModel  # noqa: E402
from tensordict import TensorDict  # noqa: E402

from learning.rl.env.robot_polish_env import RobotPolishEnv  # noqa: E402
from learning.rl.env.robot_polish_env_cfg import RobotPolishEnvCfg  # noqa: E402
from learning.vehicle_export.export_vehicle_results import (  # noqa: E402
    CLEARCOAT_SAFE_MIN_UM, GU_PASS_MIN, RA_PASS_MAX_UM, RZ_PASS_MAX_UM)
from scripts.polishing_v5_modules.common import load_ply_points  # noqa: E402
from scripts.polishing_v5_modules.rl_bridge import CellRegistry  # noqa: E402

# repolish_eval.load_policy 와 동일 (그 모듈은 임포트 시 argparse 를 실행하므로 인라인)
def load_policy(checkpoint, device):
    ck = torch.load(checkpoint, map_location=device, weights_only=False)
    state = ck["actor_state_dict"]
    obs_dim = int(state["mlp.0.weight"].shape[1])
    dummy = TensorDict({"policy": torch.zeros(1, obs_dim, device=device)}, batch_size=[1])
    actor = MLPModel(
        dummy, {"actor": ["policy"]}, "actor", 2,
        hidden_dims=[128, 128], activation="elu", obs_normalization=True,
        distribution_cfg={"class_name": "GaussianDistribution", "init_std": 0.3,
                          "std_type": "scalar"}).to(device)
    actor.load_state_dict(state)
    actor.eval()

    def policy(obs):
        x = obs["policy"][:, :obs_dim]
        td = TensorDict({"policy": x}, batch_size=[len(x)])
        return actor(td).clamp(-1.0, 1.0)

    print(f"[repolish] loaded {checkpoint} (obs_dim={obs_dim})")
    return policy


COLUMNS = [
    "cell_id", "region", "center_x_m", "center_y_m", "center_z_m", "tilt_deg", "is_side",
    "quad_c3", "quad_c5", "fit_rms_mm", "outcome", "passes",
    "gu_before", "gu_final", "ra_final_um", "rz_final_um", "scratch_before_um", "scratch_final_um",
    "clearcoat_initial_um", "clearcoat_min_um", "temperature_peak_c",
    "gu_target_pass", "ra_target_pass", "rz_target_pass", "scratch_improved", "clearcoat_safe",
    "warranty_removal_ok", "overall_pass", "disposition", "checkpoint",
]


def parse_cells(spec: str) -> list[int]:
    if "-" in spec and "," not in spec:
        a, b = spec.split("-"); return list(range(int(a), int(b) + 1))
    return [int(v) for v in spec.split(",") if v.strip()]


def fit_quad(cell, points: np.ndarray, patch_size: float):
    """셀 점군을 셀 로컬 프레임(u,v 원점 = 셀 origin, h = 법선 방향)으로 옮겨
    h = c0 + c1 cu + c2 cv + c3 cu² + c4 cu cv + c5 cv² (cu,cv = 중심 기준) 최소제곱."""
    d = points - cell.origin
    u = d @ cell.u_axis; v = d @ cell.v_axis; h = d @ cell.normal
    cu = u - patch_size / 2.0; cv = v - patch_size / 2.0
    A = np.stack([np.ones_like(cu), cu, cv, cu * cu, cu * cv, cv * cv], axis=1)
    c, *_ = np.linalg.lstsq(A, h, rcond=None)
    c[0] = 0.0                           # 중심 높이 0 = work_top (env 규약)
    rms = float(np.sqrt(np.mean((A @ c - h) ** 2))) if len(h) else 0.0
    return [float(x) for x in c], rms


def main():
    cell_ids = parse_cells(args.cells)
    pts = np.asarray(load_ply_points(args.ply), float)
    reg = CellRegistry(pts, profile=args.profile, seed=args.registry_seed)
    cells = [reg.cells[i] for i in cell_ids if i < len(reg.cells)]
    E = len(cells)
    assert 1 <= E <= args.num_envs, f"셀 {E}개 > num_envs {args.num_envs}"
    patch = 0.12
    quads, inits, sides, fits = [], [], [], []
    # 셀 점군: 등록 시 점 인덱스를 보관하지 않으므로 셀 중심 반경 내 점으로 재수집
    kd = reg._kd
    for cell in cells:
        idx = kd.query_ball_point(cell.center, r=patch * 0.75)
        c, rms = fit_quad(cell, pts[idx], patch)
        quads.append(c); fits.append(rms)
        inits.append({"ra": cell.init["ra"], "scratch": cell.init["scratch"],
                      "n_scr": cell.init["n_scr"], "clearcoat": cell.init["clearcoat"],
                      "seed": cell.seed})
        sides.append(bool(cell.is_side))
        print(f"[car_cells] cell {cell.cell_id} {cell.region} tilt {cell.tilt_deg:.1f}° "
              f"quad c3={c[3]:+.3f} c5={c[5]:+.3f} rms={rms*1000:.2f}mm init={cell.init}")

    env_cfg = RobotPolishEnvCfg()
    # ── 강곡률 재실행 옵션: 감압 / 소형 패드 ──
    if args.force_scale != 1.0:
        import json, tempfile
        from learning.rl.env.polish_env import _load_recipe
        base = _load_recipe(env_cfg.recipe_json_path)
        d = json.load(open(env_cfg.recipe_json_path))
        key = "target_contact_force_n"
        d[key] = float(base.target_contact_force_n * args.force_scale)
        tmp = tempfile.NamedTemporaryFile("w", suffix="_recipe.json", delete=False)
        json.dump(d, tmp); tmp.close()
        env_cfg.recipe_json_path = tmp.name
        print(f"[car_cells] 감압: 목표 힘 {base.target_contact_force_n:.2f} → {d[key]:.2f} N ({args.force_scale}×)")
    if args.pad_radius is not None:
        import scripts.polishing_v5_modules.common as _v5c
        import learning.rl.env.robot_polish_env as _rpe
        from learning.digital_twin import config as _PC
        _v5c.POLISHING_DISK_RADIUS = float(args.pad_radius)
        _rpe.POLISHING_DISK_RADIUS = float(args.pad_radius)
        _PC.PAD_RADIUS_M = float(args.pad_radius); _PC.PAD_DIAMETER_M = 2.0 * float(args.pad_radius)
        print(f"[car_cells] 소형 패드: 물리 반경 {args.pad_radius} m (footprint/raster 도 동일; "
              f"제거 모델 pad_radius 는 보정 설정값 유지 — 주의)")
    env_cfg.surface_kind = "quad"
    env_cfg.carcell_quads = quads
    env_cfg.carcell_init = inits
    env_cfg.carcell_is_side = sides
    env_cfg.scene.num_envs = E
    env_cfg.enable_pad_physical_contact = True
    env_cfg.repolish_max_passes = args.max_passes
    env_cfg.repolish_cooldown_s = args.cooldown_s
    nominal_pass_s = 260.0
    env_cfg.episode_length_s = args.max_passes * (nominal_pass_s * args.pass_time_factor
                                                   + args.cooldown_s) + 60.0
    env = RobotPolishEnv(env_cfg, render_mode=None)
    env._repolish_mode = True
    policy = load_policy(args.checkpoint, env.device)
    obs, _ = env.reset()

    done_rows: dict[int, dict] = {}
    step = 0
    while len(done_rows) < E and step < args.max_control_steps:
        with torch.no_grad():
            actions = policy(obs)
        obs, _, terminated, truncated, _ = env.step(actions)
        step += 1
        done_ids = (terminated | truncated).nonzero(as_tuple=False).squeeze(-1).cpu().tolist()
        for i in done_ids:
            if i in done_rows:
                continue
            log = env._repolish_log.pop(i, None)
            if log is None:
                continue
            cell = cells[i]; b, f = log["before"], log["final"]
            gu_pass = f["gu"] >= GU_PASS_MIN
            ra_pass = f["ra"] <= RA_PASS_MAX_UM
            rz_pass = f["rz"] <= RZ_PASS_MAX_UM
            scr_ok = (b["scratch"] < 0.05) or (f["scratch"] < b["scratch"])
            cc_safe = f["cc_min"] >= CLEARCOAT_SAFE_MIN_UM
            warranty = (cell.init["clearcoat"] - f["cc_min"]) <= 7.5
            overall = gu_pass and ra_pass and rz_pass and scr_ok and cc_safe and bool(log["safety_ok"])
            if overall:
                disp = "pass"
            elif (f["cc_min"] - CLEARCOAT_SAFE_MIN_UM) < 1.0 or not log["safety_ok"]:
                disp = "spot_repaint_review"
            else:
                disp = "rework_candidate"
            done_rows[i] = {
                "cell_id": cell.cell_id, "region": cell.region,
                "center_x_m": f"{cell.center[0]:.3f}", "center_y_m": f"{cell.center[1]:.3f}",
                "center_z_m": f"{cell.center[2]:.3f}", "tilt_deg": f"{cell.tilt_deg:.1f}",
                "is_side": cell.is_side, "quad_c3": f"{quads[i][3]:+.4f}", "quad_c5": f"{quads[i][5]:+.4f}",
                "fit_rms_mm": f"{fits[i]*1000:.2f}", "outcome": log["outcome"], "passes": log["passes"],
                "gu_before": f"{b['gu']:.2f}", "gu_final": f"{f['gu']:.2f}",
                "ra_final_um": f"{f['ra']:.4f}", "rz_final_um": f"{f['rz']:.3f}",
                "scratch_before_um": f"{b['scratch']:.3f}", "scratch_final_um": f"{f['scratch']:.3f}",
                "clearcoat_initial_um": f"{cell.init['clearcoat']:.2f}",
                "clearcoat_min_um": f"{f['cc_min']:.2f}",
                "temperature_peak_c": f"{f['temperature_peak_c']:.2f}",
                "gu_target_pass": gu_pass, "ra_target_pass": ra_pass, "rz_target_pass": rz_pass,
                "scratch_improved": scr_ok, "clearcoat_safe": cc_safe,
                "warranty_removal_ok": warranty, "overall_pass": overall, "disposition": disp,
                "checkpoint": os.path.basename(args.checkpoint) + (f"|{args.tag}" if args.tag else ""),
            }
            print(f"[car_cells] cell {cell.cell_id} {cell.region}: {log['outcome']} passes={log['passes']} "
                  f"GU {b['gu']:.1f}→{f['gu']:.1f} scr {b['scratch']:.2f}→{f['scratch']:.2f} "
                  f"cc_min {f['cc_min']:.1f} → {disp}", flush=True)
        if step % 2000 == 0:
            print(f"[car_cells] step={step} done={len(done_rows)}/{E}", flush=True)
    if len(done_rows) < E:
        print(f"[car_cells] 상한 도달 — {len(done_rows)}/{E} 셀만 완료")
    os.makedirs(os.path.dirname(args.out), exist_ok=True)
    new = not os.path.exists(args.out)
    with open(args.out, "a", newline="", encoding="utf-8") as fh:
        w = csv.DictWriter(fh, fieldnames=COLUMNS)
        if new:
            w.writeheader()
        for i in sorted(done_rows):
            w.writerow(done_rows[i])
    n_pass = sum(1 for r in done_rows.values() if r["overall_pass"])
    print(f"[car_cells] 배치 완료: {len(done_rows)}셀, 합격 {n_pass} → {args.out}")
    env.close()


if __name__ == "__main__":
    main()
    app.close()
