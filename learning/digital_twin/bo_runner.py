"""Constrained BO.

디지털 트윈(LiteraturePolishingModel + GU proxy) 안에서 공정 레시피를 탐색한다.
외부 라이브러리 없이 scipy 만으로 GP + constrained Expected Improvement 를 구현.

이 단계의 평가자는 완벽 추종 kinematic 실행기다 — 여기서 나온 recipe 는 잠정치이며
  PPO 연결 후 outer loop 로 재탐색한다. recipe JSON 의 source 필드에 명시.
"""
from __future__ import annotations

import json
import os
from dataclasses import dataclass, field

import numpy as np
from scipy.stats import norm, qmc

from . import config as C
from .gloss_proxy import TARGET_GU, GlossProxyConfig, LiteratureGlossProxyModel
from .path_executor import Recipe, load_calibrated_config, run_episode
from .polishing_model import LiteraturePolishingModel
from .surface_state import make_flat_patch

OUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "outputs")


# ── Search space ────────────────────────────────────────────
@dataclass(frozen=True)
class SearchSpace:
    # force: intersection(논문 후보 5~20N, 도달가능 영역).
    # 실측 (test_lab_env_baseline): 어드미턴스 정적 상한 = K·(2·cdist−press_min)
    #   = 350·0.024 = 8.4 N. 그 위 명령은 포화 limit cycle 로 달성 불가 — 이전 상한 12N 은
    #   kinematic 평가자의 "명령=달성" 가정이 만든 오류였다.
    force_n: tuple = (5.0, 8.4)             # L-DERIVED(하한) + 실측(상한)
    feed_mm_s: tuple = (1.0, 8.0)           # L-DERIVED  Denkena 50~500 mm/min 환산
    rpm: tuple = (3000.0, 6000.0)           # PT-DESIGN anchor 5000 주변 탐색확장
    spacing_ratio: tuple = (0.15, 0.80)     # PT-DESIGN
    n_passes: tuple = (1, 3)                # PT-DESIGN: pass 는 고정 recipe 제공 허용
    # tool_path_type: raster 고정 (contour 미구현 — config 에 기록)

    @property
    def names(self):
        return ["force_n", "feed_mm_s", "rpm", "spacing_ratio", "n_passes"]

    def to_unit(self, x: np.ndarray) -> np.ndarray:
        lo, hi = self._bounds()
        return (x - lo) / (hi - lo)

    def from_unit(self, u: np.ndarray) -> np.ndarray:
        lo, hi = self._bounds()
        x = lo + u * (hi - lo)
        x[..., 4] = np.round(x[..., 4])          # n_passes 는 정수
        return x

    def _bounds(self):
        b = [self.force_n, self.feed_mm_s, self.rpm, self.spacing_ratio, self.n_passes]
        return np.array([v[0] for v in b], float), np.array([v[1] for v in b], float)


SPACE = SearchSpace()

# ── 목적함수 가중치·정규화 스케일 — 전부 PT-DESIGN, 로그에 기록 ──
OBJECTIVE = {
    "w_gu": 1.0, "w_scratch": 0.5, "w_map": 0.3,
    "w_healthy": 0.3, "w_time": 0.2, "w_ra": 0.2,
    "scratch_scale_um": C.SCRATCH_DEPTH_MAX_UM,
    "overremoval_scale_um": 2.0,
    "time_scale_s": 1800.0,        # reference 30분을 1.0 으로
    "ra_scale_um": 0.20,
}

# ── Hard constraints ────────────────────────────────────────
CONSTRAINTS = {
    "force_hard_limit_n": 14.0,                       # v5 상단 hard limit
    "clearcoat_safety_limit_um": C.CLEARCOAT_SAFETY_LIMIT_UM,
    "healthy_overremoval_limit_um": 3.0,              # PT-DESIGN
    "heat_proxy_limit": C.HEAT_PROXY_MAX * 0.9,       # PT-DESIGN
    "coverage_min": 0.90,                             # PT-DESIGN
}


# ── 평가자: 여러 seed 표면의 평균 + worst 를 함께 본다 ──────
class RecipeEvaluator:
    def __init__(self, surface_seeds=(101, 102, 103), patch=(0.20, 0.20),
                 resolution_m=0.002):
        self.cal = load_calibrated_config()
        self.gloss = LiteratureGlossProxyModel(GlossProxyConfig())
        self.seeds = tuple(surface_seeds)
        self.patch = patch
        self.resolution_m = resolution_m

    def _episode(self, recipe: Recipe, seed: int) -> dict:
        state = make_flat_patch(self.patch, self.resolution_m, seed=seed,
                                with_scratches=True)
        model = LiteraturePolishingModel(self.cal)
        # 스텝 길이 ≈ 1 mm 가 되도록 dt 선택 (해상도 2mm 의 절반) — dt 불변성은 단위시험 2 로 보증
        feed_m_s = recipe.feed_speed_mm_s / 1000.0
        dt = float(np.clip(0.001 / feed_m_s, 0.02, 0.5))
        removal = run_episode(model, state, recipe, self.patch, quality_dt_s=dt)
        g = self.gloss.evaluate(state)["summary"]
        cum = state.cumulative_removal_um
        return {
            "gu_mean": g["gu_mean"], "gu_p10": g["gu_p10"], "gu_min": g["gu_min"],
            "gloss_pass": g["gloss_pass"],
            "max_residual_scratch_um": removal["max_residual_scratch_um"],
            "ra_um": removal["ra_um"],
            "removal_cv": float(cum.std() / max(cum.mean(), 1e-9)),
            "healthy_overremoval_um": removal["healthy_overremoval_um"],
            "clearcoat_min_um": removal["clearcoat_min_um"],
            "heat_proxy_peak": removal["heat_proxy_peak"],
            "coverage_ratio": removal["coverage_ratio"],
            "process_time_s": removal["process_time_s"],
        }

    @staticmethod
    def _cost(ep: dict) -> float:
        O = OBJECTIVE
        gu_short = np.clip((TARGET_GU - ep["gu_mean"]) / TARGET_GU, 0.0, 1.0)
        scr = np.clip(ep["max_residual_scratch_um"] / O["scratch_scale_um"], 0.0, 1.0)
        cv = np.clip(ep["removal_cv"], 0.0, 1.0)
        over = np.clip(ep["healthy_overremoval_um"] / O["overremoval_scale_um"], 0.0, 1.0)
        t = np.clip(ep["process_time_s"] / O["time_scale_s"], 0.0, 1.0)
        ra = np.clip(abs(ep["ra_um"] - C.RA_TARGET_UM) / O["ra_scale_um"], 0.0, 1.0)
        return float(O["w_gu"] * gu_short + O["w_scratch"] * scr + O["w_map"] * cv
                     + O["w_healthy"] * over + O["w_time"] * t + O["w_ra"] * ra)

    @staticmethod
    def _feasible(recipe: Recipe, ep: dict) -> bool:
        K = CONSTRAINTS
        return bool(
            recipe.target_contact_force_n <= K["force_hard_limit_n"]
            and ep["clearcoat_min_um"] >= K["clearcoat_safety_limit_um"]
            and ep["healthy_overremoval_um"] <= K["healthy_overremoval_limit_um"]
            and ep["heat_proxy_peak"] <= K["heat_proxy_limit"]
            and ep["coverage_ratio"] >= K["coverage_min"])

    def __call__(self, x: np.ndarray) -> dict:
        recipe = Recipe(
            target_contact_force_n=float(x[0]),
            feed_speed_mm_s=float(x[1]),
            rpm=float(x[2]),
            step_over_spacing_ratio=float(x[3]),
            n_passes=int(round(x[4])),
        )
        episodes = [self._episode(recipe, s) for s in self.seeds]
        costs = [self._cost(e) for e in episodes]
        # 평균·worst 를 함께 — 한 표면의 운 좋은 결과가 지배하지 못하게
        cost = 0.7 * float(np.mean(costs)) + 0.3 * float(np.max(costs))
        feasible = all(self._feasible(recipe, e) for e in episodes)
        return {
            "x": [float(v) for v in x], "cost": cost, "feasible": feasible,
            "cost_per_seed": costs,
            "gu_mean": float(np.mean([e["gu_mean"] for e in episodes])),
            "gu_worst": float(np.min([e["gu_min"] for e in episodes])),
            "gloss_pass_all": all(e["gloss_pass"] for e in episodes),
            "process_time_s": float(np.mean([e["process_time_s"] for e in episodes])),
            "clearcoat_min_um": float(np.min([e["clearcoat_min_um"] for e in episodes])),
            "healthy_overremoval_um": float(np.max([e["healthy_overremoval_um"] for e in episodes])),
            "episodes": episodes,
        }


# ── 최소 GP (RBF-ARD) — scipy 만 사용 ─────────────────────────────────────
class GP:
    def __init__(self, X, y, rng):
        self.X, self.y_mean, self.y_std = X, y.mean(), max(y.std(), 1e-9)
        self.y = (y - self.y_mean) / self.y_std
        d = X.shape[1]
        best_ll, best = -np.inf, None
        for _ in range(200):     # 랜덤 탐색으로 주변우도 최대화 (경량)
            ls = 10 ** rng.uniform(-1.2, 0.5, size=d)
            sn = 10 ** rng.uniform(-3.0, -0.7)
            K = self._kernel(X, X, ls) + sn ** 2 * np.eye(len(X))
            try:
                L = np.linalg.cholesky(K)
            except np.linalg.LinAlgError:
                continue
            a = np.linalg.solve(L.T, np.linalg.solve(L, self.y))
            ll = -0.5 * self.y @ a - np.log(np.diag(L)).sum()
            if ll > best_ll:
                best_ll, best = ll, (ls, sn, L, a)
        self.ls, self.sn, self.L, self.alpha = best

    @staticmethod
    def _kernel(A, B, ls):
        d = (A[:, None, :] - B[None, :, :]) / ls
        return np.exp(-0.5 * (d ** 2).sum(-1))

    def predict(self, Xq):
        Ks = self._kernel(self.X, Xq, self.ls)
        mu = Ks.T @ self.alpha
        v = np.linalg.solve(self.L, Ks)
        var = np.clip(1.0 - (v ** 2).sum(0), 1e-9, None)
        return mu * self.y_std + self.y_mean, np.sqrt(var) * self.y_std


def _expected_improvement(mu, sigma, best):
    z = (best - mu) / sigma
    return (best - mu) * norm.cdf(z) + sigma * norm.pdf(z)


# ── BO 루프 ─────────────────────────────────────────────────
def run_bo(n_initial=12, n_iter=20, seed=7, evaluator: RecipeEvaluator | None = None,
           verbose=True) -> dict:
    rng = np.random.default_rng(seed)
    ev = evaluator or RecipeEvaluator()

    # 초기 표본: space-filling (Sobol) — 한쪽 구간에 몰리지 않게
    sobol = qmc.Sobol(d=5, scramble=True, seed=seed)
    U = sobol.random(n_initial)
    dataset = []
    for i, u in enumerate(U):
        r = ev(SPACE.from_unit(u))
        r["iteration"], r["phase"] = i, "initial"
        dataset.append(r)
        if verbose:
            print(f"  init {i:2d}: cost {r['cost']:.3f} GU {r['gu_mean']:.1f} "
                  f"feasible={r['feasible']}")

    infeasible_penalty = 0.5    # GP 학습용 — 선택은 feasible 만
    for it in range(n_iter):
        X = np.array([SPACE.to_unit(np.array(d["x"])) for d in dataset])
        y = np.array([d["cost"] + (0.0 if d["feasible"] else infeasible_penalty)
                      for d in dataset])
        gp = GP(X, y, rng)
        feas_costs = [d["cost"] for d in dataset if d["feasible"]]
        best = min(feas_costs) if feas_costs else y.min()

        cand = rng.uniform(0, 1, size=(4096, 5))
        mu, sd = gp.predict(cand)
        ei = _expected_improvement(mu, sd, best)
        u_next = cand[int(np.argmax(ei))]

        r = ev(SPACE.from_unit(u_next))
        r["iteration"], r["phase"] = n_initial + it, "bo"
        r["acquisition_value"] = float(ei.max())
        r["predicted_cost_mean"], r["predicted_cost_std"] = \
            float(mu[int(np.argmax(ei))]), float(sd[int(np.argmax(ei))])
        dataset.append(r)
        if verbose:
            print(f"  bo {it:2d}: cost {r['cost']:.3f} GU {r['gu_mean']:.1f} "
                  f"feasible={r['feasible']} EI {r['acquisition_value']:.4f}")

    feas = [d for d in dataset if d["feasible"]]
    best = min(feas, key=lambda d: d["cost"]) if feas else None
    return {"dataset": dataset, "best": best, "n_feasible": len(feas)}


# ── recipe JSON export ─────────────────────────────────────
def export_best_recipe(result: dict, out_dir: str = OUT_DIR) -> str:
    best = result["best"]
    if best is None:
        raise RuntimeError("feasible recipe 가 없다 — search space/제약을 점검할 것")
    x = best["x"]
    recipe_json = {
        "schema_version": "1.0",
        "recipe_id": f"recipe_{best['iteration']:05d}",
        "source": "synthetic_bo_v1_kinematic_executor",   # PPO 연결 전 잠정치임을 명시
        "polishing_model_version": C.MODEL_VERSION,
        "gloss_model_version": "literature_gu_proxy_v1",
        "pad_profile_id": "fixed_pad_v1",
        "compound_profile_id": "compound_v1",
        "target_contact_force_n": round(x[0], 3),
        "feed_speed_mm_s": round(x[1], 3),
        "rpm": round(x[2], 1),
        "step_over_spacing_ratio": round(x[3], 4),
        "n_passes": int(round(x[4])),
        "tool_path_type": "raster",
        "tool_angle_rad": [0.0, 0.0],
        "gu_target": TARGET_GU,
        "gu_type": "literature_proxy",
        "constraints": CONSTRAINTS,
        "objective_weights": OBJECTIVE,
        "search_space": {k: getattr(SPACE, k) for k in
                         ("force_n", "feed_mm_s", "rpm", "spacing_ratio", "n_passes")},
        "result": {k: best[k] for k in
                   ("cost", "gu_mean", "gu_worst", "gloss_pass_all", "process_time_s",
                    "clearcoat_min_um", "healthy_overremoval_um")},
    }
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, "bo_best_recipe.json")
    with open(path, "w", encoding="utf-8") as f:
        json.dump(recipe_json, f, ensure_ascii=False, indent=2)
    with open(os.path.join(out_dir, "bo_dataset.json"), "w", encoding="utf-8") as f:
        json.dump([{k: v for k, v in d.items() if k != "episodes"}
                   for d in result["dataset"]], f, ensure_ascii=False, indent=2)
    return path


if __name__ == "__main__":
    print("BO 실행 — 디지털 트윈 내부 탐색")
    result = run_bo()
    path = export_best_recipe(result)
    b = result["best"]
    print(f"\nfeasible {result['n_feasible']}/{len(result['dataset'])}")
    print(f"best: force {b['x'][0]:.2f}N feed {b['x'][1]:.2f}mm/s rpm {b['x'][2]:.0f} "
          f"spacing {b['x'][3]:.3f} passes {int(b['x'][4])}")
    print(f"  cost {b['cost']:.3f} | GU {b['gu_mean']:.1f} (worst tile {b['gu_worst']:.1f}) "
          f"| pass_all={b['gloss_pass_all']} | {b['process_time_s']:.0f}s")
    print(f"→ {path}")
