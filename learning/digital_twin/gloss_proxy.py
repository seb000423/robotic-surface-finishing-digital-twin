"""LiteratureGlossProxyModel.

표면상태(Ra·Scratch·균일도·Clearcoat)와 (가능하면) RTX 상대광택을 결합해
위치별 `predicted_20deg_gu_literature_proxy` 를 계산한다.

이 GU 는 디지털 트윈 결과척도다. 실제 Gloss Meter 측정값으로 표현 금지 (L-DERIVED 앵커
  + PT-DESIGN 결합식 → 출력은 SYNTHETIC).

q_optical은 현재 RTX 측정 파이프라인이 아직 없어
기본 가중치 0 이다 — 순서상 Step 8 에서 연결한다. optical 값이 주어지면
`w_optical` 을 켠 profile 로 재평가할 수 있게 훅을 유지한다.
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from . import config as C
from .roughness_metrics import _detrend, residual_scratch_depth_um

GLOSS_MODEL_VERSION = "literature_gu_proxy_v1"

# ── 논문 GU 앵커 — L-DERIVED ────────────────────────────────
DEFECTIVE_ANCHOR_GU = 25.0        # 결함 보수도장 시편 21.1~29.7 GU 의 대표값
GOOD_REFINISH_ANCHOR_GU = 78.0    # 양호한 보수도장 75.8~79.7 GU 의 대표값
HIGH_GLOSS_ANCHOR_GU = 89.0       # 상용 신차 평균 88.8 GU (별도 profile 선택 시만 사용)
TARGET_GU = 70.0                  # 저자 제안 목표 (>70 GU) — 범용 규격 아님


def gu_from_relative(relative_gloss, upper_anchor_gu: float = GOOD_REFINISH_ANCHOR_GU):
    """9장 GU 변환. 한 run 안에서 anchor 를 바꾸지 않는다.

    회귀 기준값: 0.2180 → ≈36.6 GU, 0.8815 → ≈71.7 GU.
    """
    return np.clip(
        DEFECTIVE_ANCHOR_GU + (upper_anchor_gu - DEFECTIVE_ANCHOR_GU) * np.asarray(relative_gloss),
        0.0, 100.0)


@dataclass
class GlossProxyConfig:
    """품질항 파라미터 — 전부 PT-DESIGN. 논문 직접값으로 표현 금지."""
    gloss_model_version: str = GLOSS_MODEL_VERSION
    upper_anchor_gu: float = GOOD_REFINISH_ANCHOR_GU
    ra_reference_um: float = C.RA_TARGET_UM      # 이보다 좋으면 q_ra=1
    ra_decay_scale_um: float = 0.10              # PT-DESIGN
    uniformity_decay_scale_um: float = 1.0       # PT-DESIGN  removal std [μm] 기준
    clearcoat_failure_limit_um: float = C.CLEARCOAT_SAFETY_LIMIT_UM
    scratch_epsilon_um: float = 0.01
    # 결합 가중치 — PT-DESIGN
    w_ra: float = 1.0
    w_scratch: float = 1.0
    w_uniformity: float = 1.0
    w_clearcoat: float = 1.0
    w_thermal: float = 1.0
    w_optical: float = 0.0     # RTX 파이프라인 연결(Step 8) 전까지 0
    # 판정 한계 — PT-DESIGN
    gu_p10_limit: float = 60.0
    gu_std_limit: float = 10.0
    gu_min_limit: float = 45.0
    tags: dict = field(default_factory=lambda: {
        "anchors": "L-DERIVED", "weights": "PT-DESIGN",
        "decay_scales": "PT-DESIGN", "pass_limits": "PT-DESIGN"})


def _tile_slices(shape, tiles):
    """patch 를 tiles=(K,L) 격자로 나눈 slice 목록."""
    nx, ny = shape
    xs = np.linspace(0, nx, tiles[0] + 1, dtype=int)
    ys = np.linspace(0, ny, tiles[1] + 1, dtype=int)
    return [(slice(xs[i], xs[i + 1]), slice(ys[j], ys[j + 1]), i, j)
            for i in range(tiles[0]) for j in range(tiles[1])]


class LiteratureGlossProxyModel:
    def __init__(self, cfg: GlossProxyConfig | None = None):
        self.cfg = cfg or GlossProxyConfig()

    # ── 표면상태 품질항 (타일 단위) ──────────────────────────
    def _tile_terms(self, state, sl, optical: float | None) -> dict:
        cfg = self.cfg
        height = state.micro_height_um[sl]

        # 7.1 Roughness — 타일 내 detrend 후 Ra
        z = _detrend(height)
        ra = float(np.abs(z - z.mean()).mean())
        q_ra = float(np.exp(-max(0.0, ra - cfg.ra_reference_um) / cfg.ra_decay_scale_um))

        # 7.2 Scratch — 초기 대비 잔존 (defect 없는 타일은 1.0)
        init = state.initial_scratch_depth_um[sl]
        resid = state.residual_scratch_depth_um[sl]
        init_metric = float(init.max())
        if init_metric > cfg.scratch_epsilon_um:
            q_scratch = 1.0 - float(np.clip(resid.max() / init_metric, 0.0, 1.0))
        else:
            q_scratch = 1.0

        # 7.3 균일도 — 제거량 표준편차
        q_uniformity = float(np.exp(
            -state.cumulative_removal_um[sl].std() / cfg.uniformity_decay_scale_um))

        # 7.4 Clearcoat 보전
        init_cc_mean = float(state.initial_clearcoat_um[sl].mean())
        denom = max(init_cc_mean - cfg.clearcoat_failure_limit_um, 1e-6)
        q_clearcoat = float(np.clip(
            (state.clearcoat_remaining_um[sl].min() - cfg.clearcoat_failure_limit_um) / denom,
            0.0, 1.0))

        # Synthetic thermal degradation term.  It changes the optical-quality
        # proxy only; it is not a measured GU-temperature calibration.
        q_thermal = float(np.exp(
            -state.thermal_damage_proxy[sl].mean() / C.THERMAL_GLOSS_DAMAGE_SCALE))

        # 8장: geometric combination — 한 항의 심한 결함이 평균에 숨지 않게
        terms = {"q_ra": q_ra, "q_scratch": q_scratch,
                 "q_uniformity": q_uniformity, "q_clearcoat": q_clearcoat,
                 "q_thermal": q_thermal}
        weights = {"q_ra": self.cfg.w_ra, "q_scratch": self.cfg.w_scratch,
                   "q_uniformity": self.cfg.w_uniformity, "q_clearcoat": self.cfg.w_clearcoat}
        if optical is not None and self.cfg.w_optical > 0.0:
            terms["q_optical"] = float(np.clip(optical, 0.0, 1.0))
            weights["q_optical"] = self.cfg.w_optical

        wsum = sum(weights.values())
        # Thermal damage is a one-way penalty after the established geometric
        # quality combination.  q_thermal=1 therefore preserves the pre-thermal
        # GU scale instead of diluting existing defects with another perfect term.
        log_q = (sum(w * np.log(max(terms[k], 1e-6)) for k, w in weights.items()) / wsum
                 + self.cfg.w_thermal * np.log(max(q_thermal, 1e-6)))
        q_total = float(np.exp(log_q))
        gu = float(gu_from_relative(q_total, self.cfg.upper_anchor_gu))
        return {**terms, "q_total": q_total,
                "predicted_20deg_gu_literature_proxy": gu}

    # ── 평가 ──────────────────────────────────────────────────────────────
    def evaluate(self, state, tiles=(5, 5), optical_map=None) -> dict:
        """patch 를 tiles 격자로 나눠 위치별 GU proxy 맵과 판정을 계산한다.

        optical_map: (K,L) RTX 상대 정반사 (없으면 None → w_optical=0 경로).
        """
        # 잔존 scratch 최신화 (polishing_model.evaluate 를 안 거쳤을 수 있음)
        state.residual_scratch_depth_um = residual_scratch_depth_um(
            state.micro_height_um, state.defect_mask, state.resolution_m)

        gu_map = np.zeros(tiles)
        term_maps = {k: np.zeros(tiles) for k in
                     ("q_ra", "q_scratch", "q_uniformity", "q_clearcoat", "q_thermal", "q_total")}
        for sl_i, sl_j, i, j in _tile_slices(state.shape, tiles):
            optical = None if optical_map is None else float(optical_map[i, j])
            out = self._tile_terms(state, (sl_i, sl_j), optical)
            gu_map[i, j] = out["predicted_20deg_gu_literature_proxy"]
            for k in term_maps:
                term_maps[k][i, j] = out[k]

        # 10장: 판정 — 평균만으로 판정하지 않는다
        cfg = self.cfg
        summary = {
            "gloss_model_version": cfg.gloss_model_version,
            "gu_mean": float(gu_map.mean()),
            "gu_p10": float(np.percentile(gu_map, 10)),
            "gu_std": float(gu_map.std()),
            "gu_min": float(gu_map.min()),
            "gu_target": TARGET_GU,
            "gloss_pass": bool(
                gu_map.mean() >= TARGET_GU
                and np.percentile(gu_map, 10) >= cfg.gu_p10_limit
                and gu_map.std() <= cfg.gu_std_limit
                and gu_map.min() >= cfg.gu_min_limit),
            "band_counts": {
                "target_pass": int((gu_map >= 70).sum()),
                "partial": int(((gu_map >= 60) & (gu_map < 70)).sum()),
                "low": int(((gu_map >= 30) & (gu_map < 60)).sum()),
                "severe_defect": int((gu_map < 30).sum()),
            },
            "optical_used": optical_map is not None and cfg.w_optical > 0.0,
        }
        return {"gu_map": gu_map, "term_maps": term_maps, "summary": summary}
