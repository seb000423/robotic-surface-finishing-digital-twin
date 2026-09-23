"""
Isaac Lab 실행형 시험 — pytest 아님. RobotPolishEnv 를 enable_pad_physical_contact=True
로 띄워 실제 PhysX ContactSensor normal force 를 검증한다.

    ~/isaacsim/python.sh learning/rl/tests/test_pad_contact_force.py \
        --phase free --headless

phases:
  free     자유공간 0 N — 비접촉 정지·이동 중 허위 힘 없음 + 무가공·무발열 확인
  static   정적 목표힘 3·5·8·10 N — 정상상태 오차/overshoot/안정화시간/관통량
  track    이동 경로(BO recipe) force tracking
  safety   과도한 힘 명령(20 N) / reset 직후 spike / NaN 가드 로직
  parallel --num_envs N 병렬 안정성 (1→4→8→16 은 별도 프로세스로 순차 실행)

결과: learning/rl/outputs/contact_validation/{phase}[_envN].json + env0 스텝 CSV.
모든 판정 임계값은 PT-DESIGN 이며 summary JSON 에 기록된다.
"""
from __future__ import annotations

import argparse
import csv
import json
import os
import sys

_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
sys.path.insert(0, _REPO_ROOT)

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
parser.add_argument("--phase", required=True,
                    choices=["free", "static", "track", "safety", "parallel", "quality"])
parser.add_argument("--num_envs", type=int, default=1)
parser.add_argument("--out_dir", type=str,
                    default=os.path.join(_REPO_ROOT, "learning", "rl", "outputs",
                                         "contact_validation"))
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
app = AppLauncher(args).app

import numpy as np  # noqa: E402
import torch  # noqa: E402

from learning.rl.env.robot_polish_env import RobotPolishEnv  # noqa: E402
from learning.rl.env.robot_polish_env_cfg import RobotPolishEnvCfg  # noqa: E402

FREE_SPACE_MAX_N = 0.05          # 자유공간 허위 힘 상한
STATIC_SS_ERR_MAX_N = 0.5        # 정상상태 |오차| 상한
STATIC_SS_STD_MAX_N = 0.3        # 정상상태 진동(std) 상한
STATIC_OVERSHOOT_MAX_RATIO = 0.5  # 목표 대비 overshoot 상한
STATIC_PENETRATION_MAX_M = 0.006  # 패드 관통량 상한
STATIC_BAND_RATIO = 0.10         # 안정화 판정 밴드 (목표 ±10%)
TRACK_ERR_MEAN_MAX_N = 1.0       # 이동 중 정상구간 평균 추종오차
TRACK_ERR_P95_MAX_N = 2.5
SETTLE_SKIP_S = 10.0             # 이동/병렬 시험의 정착 대기
RESET_SPIKE_MAX_N = 8.0          # reset 직후 10 control step 내 허용 상한
SAFETY_FORCE_CEIL_N = 30.0       # 과도명령 시험에서도 절대 넘으면 안 되는 값
MATRIX_MISMATCH_MAX_N = 0.5      # net vs 패드↔작업면 분리힘 괴리(허위 접촉 진단)

CONTROL_HZ = 20.0


def make_env(num_envs: int) -> RobotPolishEnv:
    cfg = RobotPolishEnvCfg()
    cfg.scene.num_envs = num_envs
    cfg.enable_pad_physical_contact = True
    env = RobotPolishEnv(cfg, render_mode=None)
    return env


def run_segment(env: RobotPolishEnv, steps: int, tag: str) -> dict:
    """reset 없이 control step 을 돌리며 전 env 의 힘/상태 시계열을 모은다."""
    E = env.num_envs
    zero = torch.zeros((E, env.cfg.action_space), device=env.device)
    keys = ["force_cmd", "sensor_raw", "sensor_filt", "force_model", "force_used",
            "sensor_matrix", "gap", "in_patch", "fault", "terminated", "force_mean"]
    rows = {k: np.zeros((steps, E), dtype=np.float64) for k in keys}
    finite = True
    for s in range(steps):
        obs, reward, terminated, truncated, _ = env.step(zero)
        finite = (finite and bool(torch.isfinite(obs["policy"]).all())
                  and bool(torch.isfinite(reward).all()))
        rows["force_cmd"][s] = env._force_cmd.cpu().numpy()
        rows["sensor_raw"][s] = env._force_sensor_n.cpu().numpy()
        rows["sensor_filt"][s] = env._force_sensor_filt_n.cpu().numpy()
        rows["force_model"][s] = env._force_model_n.cpu().numpy()
        rows["force_used"][s] = env._force_used_n.cpu().numpy()
        rows["sensor_matrix"][s] = env._sensor_matrix_n.cpu().numpy()
        rows["gap"][s] = env._pad_gap_m.cpu().numpy()
        rows["in_patch"][s] = env._pad_in_patch.cpu().numpy()
        rows["fault"][s] = env._sensor_fault.cpu().numpy()
        rows["terminated"][s] = (terminated | truncated).cpu().numpy()
        rows["force_mean"][s] = env._force_mean.cpu().numpy()
        if s % 100 == 0 or s == steps - 1:
            print(f"  [{tag}] step={s:04d} cmd={rows['force_cmd'][s][0]:.2f}N "
                  f"raw={rows['sensor_raw'][s][0]:.3f}N filt={rows['sensor_filt'][s][0]:.3f}N "
                  f"model={rows['force_model'][s][0]:.3f}N gap={rows['gap'][s][0]:+.4f}m",
                  flush=True)
    rows["finite"] = finite
    return rows


def save_csv(rows: dict, steps: int, path: str):
    """env0 시계열 CSV — force_cmd/sensor_raw/sensor_filtered/model/used 분리 저장."""
    cols = ["t_s", "force_cmd_n", "force_sensor_raw_n", "force_sensor_filtered_n",
            "force_model_n", "force_used_n", "sensor_matrix_n", "pad_gap_m",
            "in_patch", "sensor_fault", "terminated"]
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(cols)
        for s in range(steps):
            w.writerow([f"{s / CONTROL_HZ:.3f}",
                        f"{rows['force_cmd'][s][0]:.4f}", f"{rows['sensor_raw'][s][0]:.4f}",
                        f"{rows['sensor_filt'][s][0]:.4f}", f"{rows['force_model'][s][0]:.4f}",
                        f"{rows['force_used'][s][0]:.4f}", f"{rows['sensor_matrix'][s][0]:.4f}",
                        f"{rows['gap'][s][0]:.5f}", int(rows["in_patch"][s][0]),
                        int(rows["fault"][s][0]), int(rows["terminated"][s][0])])


def settle_metrics(filt: np.ndarray, target: float) -> dict:
    """단일 env 시계열의 정착/정상상태 지표."""
    n = len(filt)
    ss = filt[int(n * 0.75):]                      # 마지막 25% = 정상상태 구간
    band = STATIC_BAND_RATIO * target
    settle_step = None
    for s in range(n):
        if np.all(np.abs(filt[s:] - target) <= band):
            settle_step = s
            break
    return {
        "steady_state_mean_n": float(ss.mean()),
        "steady_state_err_n": float(ss.mean() - target),
        "steady_state_std_n": float(ss.std()),
        "overshoot_n": float(filt.max() - target),
        "settle_time_s": (settle_step / CONTROL_HZ) if settle_step is not None else None,
    }


def check(results: list, name: str, ok: bool, detail: str = ""):
    results.append({"name": name, "pass": bool(ok), "detail": detail})
    print(f"  [{'PASS' if ok else 'FAIL'}] {name}" + (f" — {detail}" if detail else ""),
          flush=True)


def main():
    os.makedirs(args.out_dir, exist_ok=True)
    results: list = []
    summary: dict = {
        "phase": args.phase, "num_envs": args.num_envs,
        "contact_mode": "physical",
        "thresholds": {
            "free_space_max_n": FREE_SPACE_MAX_N,
            "static_ss_err_max_n": STATIC_SS_ERR_MAX_N,
            "static_ss_std_max_n": STATIC_SS_STD_MAX_N,
            "static_overshoot_max_ratio": STATIC_OVERSHOOT_MAX_RATIO,
            "static_penetration_max_m": STATIC_PENETRATION_MAX_M,
            "track_err_mean_max_n": TRACK_ERR_MEAN_MAX_N,
            "track_err_p95_max_n": TRACK_ERR_P95_MAX_N,
            "reset_spike_max_n": RESET_SPIKE_MAX_N,
        },
        "segments": {},
    }

    env = make_env(args.num_envs)
    base_force = float(env.recipe.target_contact_force_n)
    base_feed = float(env.recipe.feed_speed_mm_s)
    print(f"[contact-gate] phase={args.phase} envs={args.num_envs} "
          f"recipe force={base_force:.3f}N feed={base_feed:.3f}mm/s "
          f"stiffness={env.cfg.pad_compliant_stiffness_n_m}N/m "
          f"dt={env.cfg.sim.dt:.5f}s decimation={env.cfg.decimation}", flush=True)

    if args.phase == "free":
        # 목표힘 0 → 어드미턴스가 누르지 않아 패드가 표면 위(≈6cm)에서 경로만 따라간다.
        env.recipe.target_contact_force_n = 0.0
        env.reset()
        steps = 400                                  # 20 s
        rows = run_segment(env, steps, "free")
        save_csv(rows, steps, os.path.join(args.out_dir, "free_env0.csv"))
        max_raw = float(rows["sensor_raw"].max())
        check(results, "finite", rows["finite"])
        check(results, "free-space sensor ~0N", max_raw < FREE_SPACE_MAX_N,
              f"max raw={max_raw:.4f}N (< {FREE_SPACE_MAX_N})")
        check(results, "no sensor fault", float(rows["fault"].sum()) == 0,
              f"faults={int(rows['fault'].sum())}")
        check(results, "force_used ~0 (비접촉 무가공 게이트)",
              float(rows["force_used"].max()) < FREE_SPACE_MAX_N,
              f"max used={float(rows['force_used'].max()):.4f}N")
        s0 = env._surfaces[0]
        removal = float(s0.cumulative_removal_um.max())
        t_rise = float(s0.temperature_c.max()) - float(s0.temperature_c.min())
        import learning.digital_twin.config as PC
        t_over_ambient = float(s0.temperature_c.max()) - PC.AMBIENT_TEMPERATURE_C
        check(results, "비접촉 제거량 0", removal == 0.0, f"removal max={removal:.6f}um")
        check(results, "비접촉 발열 0", t_over_ambient <= 0.1,
              f"temp-ambient={t_over_ambient:.3f}C (rise span={t_rise:.3f}C)")
        summary["segments"]["free"] = {
            "steps": steps, "max_sensor_raw_n": max_raw,
            "max_force_used_n": float(rows["force_used"].max()),
            "removal_max_um": removal, "temp_over_ambient_c": t_over_ambient,
        }

    elif args.phase == "static":
        env.recipe.feed_speed_mm_s = 0.0             # 정적: 경로 전진 없음
        for target in (3.0, 5.0, 8.0, 10.0):
            env.recipe.target_contact_force_n = target
            env.reset()
            steps = 600                              # 30 s
            rows = run_segment(env, steps, f"static-{target:g}N")
            save_csv(rows, steps,
                     os.path.join(args.out_dir, f"static_{target:g}N_env0.csv"))
            filt = rows["sensor_filt"][:, 0]
            m = settle_metrics(filt, target)
            penetration = max(0.0, -float(rows["gap"].min()))
            m["penetration_m"] = penetration
            m["max_raw_n"] = float(rows["sensor_raw"].max())
            m["model_ss_mean_n"] = float(rows["force_model"][int(steps * 0.75):, 0].mean())
            mismatch = float(np.abs(rows["sensor_raw"] - rows["sensor_matrix"]).max())
            m["net_vs_matrix_max_n"] = mismatch
            m["died_resets"] = int(rows["terminated"].sum())
            summary["segments"][f"static_{target:g}N"] = {"steps": steps, **m}
            ok = (rows["finite"]
                  and abs(m["steady_state_err_n"]) <= STATIC_SS_ERR_MAX_N
                  and m["steady_state_std_n"] <= STATIC_SS_STD_MAX_N
                  and m["overshoot_n"] <= STATIC_OVERSHOOT_MAX_RATIO * target
                  and penetration <= STATIC_PENETRATION_MAX_M
                  and m["settle_time_s"] is not None
                  and float(rows["fault"].sum()) == 0)
            check(results, f"static {target:g}N", ok,
                  f"ss={m['steady_state_mean_n']:.3f}N err={m['steady_state_err_n']:+.3f}N "
                  f"std={m['steady_state_std_n']:.3f}N ovs={m['overshoot_n']:.3f}N "
                  f"settle={m['settle_time_s']}s pen={penetration * 1000:.2f}mm "
                  f"resets={m['died_resets']}")
            check(results, f"static {target:g}N net≈pair힘(허위접촉 없음)",
                  mismatch <= MATRIX_MISMATCH_MAX_N, f"max|net-matrix|={mismatch:.3f}N")

    elif args.phase == "track":
        env.reset()                                  # BO recipe 그대로 (5.78 N, 5.95 mm/s)
        steps = 1200                                 # 60 s — 라인 전환 2~3회 포함
        rows = run_segment(env, steps, "track")
        save_csv(rows, steps, os.path.join(args.out_dir, "track_env0.csv"))
        skip = int(SETTLE_SKIP_S * CONTROL_HZ)
        err = np.abs(rows["sensor_filt"][skip:, 0] - rows["force_cmd"][skip:, 0])
        out_patch = rows["in_patch"][skip:, 0] < 0.5
        used_out = rows["force_used"][skip:, 0][out_patch]
        m = {"err_mean_n": float(err.mean()), "err_p95_n": float(np.percentile(err, 95)),
             "err_max_n": float(err.max()),
             "in_patch_ratio": float(rows["in_patch"][skip:, 0].mean()),
             "penetration_m": max(0.0, -float(rows["gap"].min())),
             "out_patch_steps": int(out_patch.sum()),
             "out_patch_used_max_n": float(used_out.max()) if len(used_out) else 0.0,
             "died_resets": int(rows["terminated"].sum())}
        summary["segments"]["track"] = {"steps": steps, **m}
        check(results, "finite", rows["finite"])
        check(results, "이동 중 force tracking",
              m["err_mean_n"] <= TRACK_ERR_MEAN_MAX_N and m["err_p95_n"] <= TRACK_ERR_P95_MAX_N,
              f"mean={m['err_mean_n']:.3f}N p95={m['err_p95_n']:.3f}N max={m['err_max_n']:.3f}N")
        check(results, "관통량 한계", m["penetration_m"] <= STATIC_PENETRATION_MAX_M,
              f"pen={m['penetration_m'] * 1000:.2f}mm")
        check(results, "작업면 밖 무가공 (해당 스텝 존재 시)",
              m["out_patch_used_max_n"] < FREE_SPACE_MAX_N,
              f"out-patch steps={m['out_patch_steps']} used_max={m['out_patch_used_max_n']:.4f}N")
        check(results, "no fault/reset", float(rows["fault"].sum()) == 0
              and m["died_resets"] == 0,
              f"faults={int(rows['fault'].sum())} resets={m['died_resets']}")

    elif args.phase == "safety":
        # 1) 과도한 힘 명령 20 N — hard limit(14 N) 종료가 작동하고 발산하지 않는가
        env.recipe.target_contact_force_n = 20.0
        env.recipe.feed_speed_mm_s = 0.0
        env.reset()
        steps = 400
        rows = run_segment(env, steps, "over-force")
        save_csv(rows, steps, os.path.join(args.out_dir, "safety_overforce_env0.csv"))
        max_f = float(rows["sensor_filt"].max())
        n_resets = int(rows["terminated"].sum())
        summary["segments"]["overforce_20N"] = {
            "steps": steps, "max_sensor_filtered_n": max_f, "died_resets": n_resets}
        check(results, "overforce finite+bounded", rows["finite"] and max_f < SAFETY_FORCE_CEIL_N,
              f"max filt={max_f:.2f}N (<{SAFETY_FORCE_CEIL_N})")
        check(results, "overforce hard-limit 종료 작동", n_resets >= 1,
              f"resets={n_resets}")

        # 2) reset 직후 spike — 접촉 상태에서 강제 reset 후 초기 10 step 힘 확인
        env.recipe.target_contact_force_n = base_force
        env.reset()
        run_segment(env, 300, "pre-reset(contact)")    # 15 s: 접촉 정착
        env.reset()                                     # 접촉 중 강제 reset
        rows2 = run_segment(env, 100, "post-reset")
        first10 = float(rows2["sensor_filt"][:10].max())
        summary["segments"]["reset_spike"] = {
            "post_reset_first10_max_n": first10,
            "post_reset_fault_steps": int(rows2["fault"].sum())}
        check(results, "reset 직후 spike 없음", rows2["finite"] and first10 < RESET_SPIKE_MAX_N,
              f"first-10-step max={first10:.3f}N (<{RESET_SPIKE_MAX_N})")

        # 3) 작업면 밖 즉시 0 N·무가공 — 접촉 유지 중 patch 기준만 이동시켜
        #    패드를 '패치 밖' 상태로 만들고, 센서가 접촉을 읽어도 force_used=0 확인.
        env.recipe.feed_speed_mm_s = 0.0
        env.reset()
        run_segment(env, 300, "pre-outside(contact)")
        orig_center = env.cfg.patch_center_xy_m
        env.cfg.patch_center_xy_m = (orig_center[0] + 0.5, orig_center[1])
        rows3 = run_segment(env, 5, "outside-patch")
        env.cfg.patch_center_xy_m = orig_center
        raw_during = float(rows3["sensor_raw"].max())
        used_during = float(rows3["force_used"].max())
        in_patch_any = bool(rows3["in_patch"].max() > 0)
        summary["segments"]["outside_patch_gate"] = {
            "sensor_raw_max_n": raw_during, "force_used_max_n": used_during,
            "in_patch_any": in_patch_any}
        check(results, "작업면 밖 즉시 무가공 (접촉 중 게이트)",
              (not in_patch_any) and used_during == 0.0 and raw_during > 1.0,
              f"raw={raw_during:.2f}N(접촉 유지) used={used_during:.4f}N in_patch={in_patch_any}")

        # 4) NaN 가드 로직 — _update_measured_pad_state 와 동일한 수식의 단위 검증
        net = torch.tensor([[float("nan"), 0.0, 3.0], [0.0, 0.0, 4.0]])
        fault = ~torch.isfinite(net).all(dim=-1)
        washed = torch.where(fault.unsqueeze(-1), torch.zeros_like(net), net)
        raw = washed[:, 2].abs()
        ok = (bool(fault[0]) and not bool(fault[1])
              and float(raw[0]) == 0.0 and float(raw[1]) == 4.0)
        summary["segments"]["nan_guard_logic"] = {"pass": ok}
        check(results, "NaN 가드 로직 (fault 분리, 조용한 성공처리 없음)", ok)

    elif args.phase == "quality":
        # 품질 모델이 소비한 힘(force_mean = force_used 평균)이 모델힘이 아니라
        # 센서힘과 일치해야 하고, 접촉 중에만 제거·발열이 생겨야 한다.
        import learning.digital_twin.config as PC
        env.reset()
        steps = 1200                                 # 60 s, BO recipe
        rows = run_segment(env, steps, "quality")
        save_csv(rows, steps, os.path.join(args.out_dir, "quality_env0.csv"))
        skip = int(SETTLE_SKIP_S * CONTROL_HZ)
        fm = rows["force_mean"][skip:, 0]            # 품질 모델이 소비한 힘
        sf = rows["sensor_filt"][skip:, 0]
        md = rows["force_model"][skip:, 0]
        s0 = env._surfaces[0]
        removal_max = float(s0.cumulative_removal_um.max())
        untouched = float((s0.cumulative_removal_um == 0.0).mean())
        temp_peak = float(s0.peak_temperature_c.max())
        m = {"consumed_force_mean_n": float(fm.mean()),
             "sensor_filtered_mean_n": float(sf.mean()),
             "model_force_mean_n": float(md.mean()),
             "consumed_vs_sensor_n": float(np.abs(fm.mean() - sf.mean())),
             "consumed_vs_model_n": float(np.abs(fm.mean() - md.mean())),
             "removal_max_um": removal_max, "untouched_cell_ratio": untouched,
             "temp_peak_c": temp_peak,
             "ambient_c": float(PC.AMBIENT_TEMPERATURE_C)}
        summary["segments"]["quality_coupling"] = {"steps": steps, **m}
        check(results, "finite", rows["finite"])
        check(results, "제거량 모델이 센서힘을 소비 (모델힘 아님)",
              m["consumed_vs_sensor_n"] < 0.05 and m["consumed_vs_model_n"] > 0.5,
              f"consumed={m['consumed_force_mean_n']:.3f}N ≈ sensor={m['sensor_filtered_mean_n']:.3f}N"
              f" ≠ model={m['model_force_mean_n']:.3f}N")
        check(results, "접촉 구간 제거 발생", removal_max > 0.0,
              f"removal max={removal_max:.4f}um")
        check(results, "미접촉 영역 무가공", untouched > 0.0,
              f"untouched ratio={untouched:.3f}")
        check(results, "온도 모델 구동 (상승 & 안전상한 미만)",
              temp_peak > m["ambient_c"] + 0.5 and temp_peak < 80.0,
              f"peak={temp_peak:.2f}C (ambient {m['ambient_c']}C, limit 80C)")

    elif args.phase == "parallel":
        env.reset()
        steps = 600                                  # 30 s, BO recipe
        rows = run_segment(env, steps, f"parallel-{args.num_envs}")
        save_csv(rows, steps,
                 os.path.join(args.out_dir, f"parallel_env{args.num_envs}_env0.csv"))
        skip = int(SETTLE_SKIP_S * CONTROL_HZ)
        per_env = []
        all_ok = rows["finite"]
        for e in range(args.num_envs):
            err = np.abs(rows["sensor_filt"][skip:, e] - rows["force_cmd"][skip:, e])
            st = {"env": e,
                  "ss_mean_n": float(rows["sensor_filt"][skip:, e].mean()),
                  "err_mean_n": float(err.mean()),
                  "err_p95_n": float(np.percentile(err, 95)),
                  "max_n": float(rows["sensor_filt"][:, e].max()),
                  "penetration_m": max(0.0, -float(rows["gap"][:, e].min())),
                  "faults": int(rows["fault"][:, e].sum())}
            per_env.append(st)
            all_ok = (all_ok and st["err_mean_n"] <= TRACK_ERR_MEAN_MAX_N
                      and st["err_p95_n"] <= TRACK_ERR_P95_MAX_N
                      and st["penetration_m"] <= STATIC_PENETRATION_MAX_M
                      and st["faults"] == 0)
        summary["segments"][f"parallel_{args.num_envs}"] = {
            "steps": steps, "per_env": per_env}
        worst = max(per_env, key=lambda s: s["err_mean_n"])
        check(results, f"parallel x{args.num_envs} 전 env 추종+무NaN", all_ok,
              f"worst env{worst['env']}: err_mean={worst['err_mean_n']:.3f}N "
              f"p95={worst['err_p95_n']:.3f}N pen={worst['penetration_m'] * 1000:.2f}mm")

    passed = all(r["pass"] for r in results)
    summary["checks"] = results
    summary["result"] = "PASS" if passed else "FAIL"
    out_name = (f"{args.phase}.json" if args.phase != "parallel"
                else f"{args.phase}_env{args.num_envs}.json")
    out_path = os.path.join(args.out_dir, out_name)
    with open(out_path, "w") as f:
        json.dump(summary, f, indent=2, ensure_ascii=False)
    print(f"[contact-gate] RESULT={summary['result']} "
          f"({sum(r['pass'] for r in results)}/{len(results)}) → {out_path}", flush=True)
    env.close()
    if not passed:
        raise RuntimeError(f"contact gate {args.phase} failed")


if __name__ == "__main__":
    try:
        main()
    except BaseException as exc:
        import traceback
        print(f"[contact-gate] EXCEPTION: {type(exc).__name__}: {exc}", flush=True)
        traceback.print_exc()
    finally:
        app.close()
