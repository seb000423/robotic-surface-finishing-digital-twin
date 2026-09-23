"""BO outer loop —: 챔피언 정책을 고정하고 recipe 를 재탐색.

    ~/isaacsim/python.sh learning/rl/bo_outer_loop.py \
        [--checkpoint learning/rl/champion/model_terminal_ppo_it400.pt] \
        [--n_initial 10] [--n_iter 18] [--workers 6]

구버전 BO(digital_twin/bo_runner)와의 차이 — 평가자가 실제 판정 조건이다:
  구: 완벽추종 kinematic 실행기, 재폴리싱 없음, "명령힘 = 달성힘" 가정
  신: 챔피언 정책 + 어드미턴스 접촉(달성힘) + 재폴리싱 루프(예산 가드 포함)
비용은 문헌 기반 판정과 정렬: GU 부족(지배) + 잔존 scratch + Ra 초과 + 공정시간.
결과는 outputs/bo_outer_dataset.json — bo_best_recipe.json 은 150셀 검증 통과 전까지
덮어쓰지 않는다 (recipe 는 학습 env 의 process context 라 부작용이 크다).
"""
from __future__ import annotations

import argparse
import json
import os
import sys

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
_REPO = os.path.dirname(os.path.dirname(_HERE))
sys.path.insert(0, _REPO)

from learning.digital_twin.bo_runner import GP, SPACE, _expected_improvement   # noqa: E402
from learning.digital_twin.path_executor import Recipe, load_calibrated_config  # noqa: E402
from learning.vehicle_export.export_vehicle_results import (               # noqa: E402
    load_bc_policy, run_cell_episode)

OUT_DIR = os.path.join(_REPO, "learning", "digital_twin", "outputs")
DEFAULT_CKPT = os.path.join(_REPO, "learning", "rl", "champion",
                            "model_terminal_ppo_it400.pt")

# ── 평가 셀 세트 (고정 seed — 짝지은 비교). --posture 로 선택 ──
# 9.11 교훈: 평가셀 자세 구성이 실배포와 다르면 BO 가 편향된 최적을 낸다.
#   자세별 recipe 체계에서는 자세별로 따로 탐색하는 게 정합적.
EVAL_SETS = {
    "mixed": [   # (seed, scratch_um, cc_um, normal)  top 4 + side 2 — 9.11 에서 사용
        (9101, 0.7, 44.0, (0.0, 0.0, 1.0)),
        (9102, 1.1, 47.0, (0.0, 0.0, 1.0)),
        (9103, 1.5, 41.0, (-0.10, 0.0, 0.995)),
        (9104, 1.9, 43.0, (0.0, 0.0, 1.0)),
        (9105, 0.9, 45.0, (0.0, 1.0, 0.0)),
        (9106, 1.5, 42.0, (0.0, -1.0, 0.0)),
    ],
    "newcar": [  # 신차 PDI 프로파일: Ra 0.12, 자국 0.05~1.0 μm, 코트 40~50 — top 5 + side 3
        (9301, 0.10, 48.0, (0.0, 0.0, 1.0), 0.12),
        (9302, 0.25, 44.0, (0.0, 0.0, 1.0), 0.12),
        (9303, 0.40, 41.0, (-0.10, 0.0, 0.995), 0.12),
        (9304, 0.60, 46.0, (0.0, 0.0, 1.0), 0.12),
        (9305, 0.90, 43.0, (0.0, 0.0, 1.0), 0.12),
        (9306, 0.30, 45.0, (0.0, 1.0, 0.0), 0.12),
        (9307, 0.50, 42.0, (0.0, -1.0, 0.0), 0.12),
        (9308, 0.80, 47.0, (0.15, -0.97, 0.19), 0.12),
    ],
    "side": [    # 전부 수직면 — side 전용 recipe 탐색 (달성힘 ~2.8N 포화 영역)
        (9201, 0.6, 44.0, (0.0, 1.0, 0.0)),
        (9202, 0.9, 47.0, (0.0, -1.0, 0.0)),
        (9203, 1.2, 41.0, (0.0, 1.0, 0.0)),
        (9204, 1.5, 43.0, (0.0, -1.0, 0.0)),
        (9205, 1.8, 45.0, (0.0, 1.0, 0.0)),
        (9206, 1.4, 42.0, (0.15, -0.97, 0.19)),
    ],
}

def _row(seed, scr, cc, n, ra=0.08):
    return {"region_id": "EV", "region_name": "bo_eval", "cell_id": seed,
            "position_x_m": 0, "position_y_m": 0, "position_z_m": 0,
            "normal_x": n[0], "normal_y": n[1], "normal_z": n[2],
            "init_ra_um": ra, "init_scratch_um": scr, "init_clearcoat_um": cc,
            "surface_seed": seed}

EVAL_CELLS = EVAL_SETS["mixed"]

_G = {}

def _init_worker(ckpt):
    _G["policy"] = load_bc_policy(ckpt)
    _G["cal"] = load_calibrated_config()

def _work(args_):
    row, recipe_x = args_
    recipe = Recipe(target_contact_force_n=float(recipe_x[0]),
                    feed_speed_mm_s=float(recipe_x[1]), rpm=float(recipe_x[2]),
                    step_over_spacing_ratio=float(recipe_x[3]),
                    n_passes=int(round(recipe_x[4])))
    return run_cell_episode(row, recipe, _G["policy"], _G["cal"], max_repolish=2)


def make_evaluator(pool, time_weight=0.15, time_ref_s=1800.0, require_all_pass=False):
    def evaluate(x: np.ndarray) -> dict:
        rows = [_row(*c) for c in EVAL_CELLS]
        outs = pool.map(_work, [(r, x) for r in rows])
        costs, feas = [], True
        for o in outs:
            gu = float(o["gu_proxy_after"]); scr = float(o["scratch_after_um"])
            ra = float(o["ra_after_um"]); t = float(o["process_time_s"])
            costs.append(1.0 * float(np.clip((70.0 - gu) / 10.0, 0, 1))
                         + 0.3 * float(np.clip(scr / 2.0, 0, 1))
                         + 0.3 * float(np.clip((ra - 0.20) / 0.20, 0, 1))
                         + time_weight * float(np.clip(t / time_ref_s, 0, 2)))
            if (o["episode_completed"] != True or "force_hard" in o["failure_reason"]
                    or float(o["clearcoat_remaining_min_um"]) < 35.0):
                feas = False
            if require_all_pass and o["overall_pass"] != True:
                feas = False
        return {"x": [float(v) for v in x],
                "cost": 0.7 * float(np.mean(costs)) + 0.3 * float(np.max(costs)),
                "feasible": feas,
                "n_pass": sum(1 for o in outs if o["overall_pass"] == True),
                "gu_mean": float(np.mean([float(o["gu_proxy_after"]) for o in outs])),
                "time_mean": float(np.mean([float(o["process_time_s"]) for o in outs]))}
    return evaluate


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--checkpoint", default=DEFAULT_CKPT)
    ap.add_argument("--n_initial", type=int, default=10)
    ap.add_argument("--n_iter", type=int, default=18)
    ap.add_argument("--workers", type=int, default=6)
    ap.add_argument("--seed", type=int, default=11)
    ap.add_argument("--posture", choices=list(EVAL_SETS), default="mixed")
    ap.add_argument("--time_weight", type=float, default=0.15,
                    help="비용의 시간 항 가중치 (9.35: 사이클 타임 최적화 시 0.6 권장)")
    ap.add_argument("--time_ref_s", type=float, default=1800.0, help="시간 항 정규화 기준(s)")
    ap.add_argument("--require_all_pass", action="store_true",
                    help="평가 셀 전부 5종 통과해야 feasible (품질을 제약으로, 시간을 목적으로)")
    ap.add_argument("--out_tag", type=str, default="", help="결과 파일명 태그")
    ap.add_argument("--baseline_json", default=os.path.join(OUT_DIR, "bo_best_recipe.json"),
                    help="비교 기준 recipe (side 탐색 시 side 현행)")
    args = ap.parse_args()
    global EVAL_CELLS
    EVAL_CELLS = EVAL_SETS[args.posture]

    import multiprocessing as mp
    from scipy.stats import qmc
    rng = np.random.default_rng(args.seed)
    pool = mp.get_context("fork").Pool(args.workers, _init_worker, (args.checkpoint,))
    ev = make_evaluator(pool, args.time_weight, args.time_ref_s, args.require_all_pass)

    # 현행 recipe 를 기준점으로 반드시 포함 (개선 여부를 같은 평가자로 직접 비교)
    with open(args.baseline_json, encoding="utf-8") as f:
        cur = json.load(f)
    x_cur = np.array([cur["target_contact_force_n"], cur["feed_speed_mm_s"],
                      cur["rpm"], cur["step_over_spacing_ratio"], cur["n_passes"]])
    dataset = []
    r = ev(x_cur); r["iteration"], r["phase"] = -1, "current"
    dataset.append(r)
    print(f"  현행 recipe: cost {r['cost']:.3f} GU {r['gu_mean']:.1f} "
          f"pass {r['n_pass']}/6 t {r['time_mean']:.0f}s")

    sobol = qmc.Sobol(d=5, scramble=True, seed=args.seed)
    for i, u in enumerate(sobol.random(args.n_initial)):
        r = ev(SPACE.from_unit(np.asarray(u)))
        r["iteration"], r["phase"] = i, "initial"
        dataset.append(r)
        print(f"  init {i:2d}: cost {r['cost']:.3f} GU {r['gu_mean']:.1f} "
              f"pass {r['n_pass']}/6 feas={r['feasible']}")

    for it in range(args.n_iter):
        X = np.array([SPACE.to_unit(np.array(d["x"])) for d in dataset])
        y = np.array([d["cost"] + (0.0 if d["feasible"] else 0.5) for d in dataset])
        gp = GP(X, y, rng)
        feas_costs = [d["cost"] for d in dataset if d["feasible"]]
        best = min(feas_costs) if feas_costs else y.min()
        cand = rng.uniform(0, 1, size=(4096, 5))
        mu, sd = gp.predict(cand)
        ei = _expected_improvement(mu, sd, best)
        r = ev(SPACE.from_unit(cand[int(np.argmax(ei))]))
        r["iteration"], r["phase"] = args.n_initial + it, "bo"
        dataset.append(r)
        print(f"  bo {it:2d}: cost {r['cost']:.3f} GU {r['gu_mean']:.1f} "
              f"pass {r['n_pass']}/6 feas={r['feasible']}")

    pool.close(); pool.join()
    feas = [d for d in dataset if d["feasible"]]
    best = min(feas, key=lambda d: d["cost"]) if feas else None
    out = {"policy_checkpoint": args.checkpoint, "eval_cells": EVAL_CELLS,
           "current_recipe_result": dataset[0], "best": best, "dataset": dataset,
           "note": "평가자 = 챔피언 정책 + 어드미턴스 + 재폴리싱. "
                   "SYNTHETIC — 논문 기반 트윈 출력."}
    path = os.path.join(OUT_DIR, f"bo_outer_dataset_{args.posture}{('_' + args.out_tag) if args.out_tag else ''}.json")
    with open(path, "w", encoding="utf-8") as f:
        json.dump(out, f, ensure_ascii=False, indent=2, default=str)
    print(f"\n현행 recipe cost {dataset[0]['cost']:.3f} (pass {dataset[0]['n_pass']}/6)")
    if best:
        b = best["x"]
        print(f"최적 후보: force {b[0]:.2f}N feed {b[1]:.2f}mm/s rpm {b[2]:.0f} "
              f"spacing {b[3]:.3f} passes {int(round(b[4]))}")
        print(f"  cost {best['cost']:.3f} pass {best['n_pass']}/6 GU {best['gu_mean']:.1f}")
        verdict = "개선 후보 — 150셀 검증 필요" if best["cost"] < dataset[0]["cost"] else "현행 유지"
        print(f"판정: {verdict}")
    print(f"→ {path}")


if __name__ == "__main__":
    main()
