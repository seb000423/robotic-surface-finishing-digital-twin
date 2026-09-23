"""Ra / Rz / 잔존 Scratch 계산.

전부 micro_height_um 하나에서 각각 직접 계산한다. Rz = Ra×4~6 같은 환산 금지.
출력은 SYNTHETIC — 실제 조도계 측정값으로 표현하지 않는다.
"""
from __future__ import annotations

import numpy as np
from scipy.ndimage import maximum_filter

from . import config as C


def _detrend(height_um: np.ndarray, order: int = C.DETREND_ORDER) -> np.ndarray:
    """nominal 형상·저주파 차체곡률 제거. 평면 patch 는 1차 평면 피팅으로 충분."""
    nx, ny = height_um.shape
    xx, yy = np.meshgrid(np.arange(nx), np.arange(ny), indexing="ij")
    cols = [np.ones(height_um.size), xx.ravel(), yy.ravel()]
    if order >= 2:
        cols += [xx.ravel() ** 2, yy.ravel() ** 2, (xx * yy).ravel()]
    A = np.stack(cols, axis=1)
    coef, *_ = np.linalg.lstsq(A, height_um.ravel(), rcond=None)
    return height_um - (A @ coef).reshape(height_um.shape)


def ra_um(height_um: np.ndarray) -> float:
    """Ra = mean(|z - mean z|), detrend 후."""
    z = _detrend(height_um)
    return float(np.abs(z - z.mean()).mean())


def rz_um(height_um: np.ndarray) -> float:
    """Rz: 높은 peak 5개와 깊은 valley 5개의 대표차 (method: rz_5peak_5valley_v1).

    단순 상위 5픽셀은 한 봉우리의 이웃 픽셀 5개를 셀 수 있으므로, 국소 극값(5×5 창)만 후보로 쓴다.
    """
    z = _detrend(height_um)
    peaks = z[(z == maximum_filter(z, size=5)) & (z > 0)]
    valleys = -(-z)[((-z) == maximum_filter(-z, size=5)) & (z < 0)]
    top5 = np.sort(peaks)[-5:] if peaks.size >= 5 else np.sort(z.ravel())[-5:]
    bot5 = np.sort(valleys)[:5] if valleys.size >= 5 else np.sort(z.ravel())[:5]
    return float(top5.mean() - bot5.mean())


def residual_scratch_depth_um(height_um: np.ndarray,
                              defect_mask: np.ndarray,
                              resolution_m: float) -> np.ndarray:
    """잔존 scratch 깊이 = 국소 기준면 대비 valley 깊이.

    기준면: scratch 폭의 SCRATCH_REFERENCE_WINDOW_RATIO 배 반경의 국소 중앙값.
    defect 셀에서만 깊이를 계산하고 나머지는 0.
    """
    from scipy.ndimage import median_filter
    window_m = C.SCRATCH_WIDTH_M * C.SCRATCH_REFERENCE_WINDOW_RATIO * 2.0
    size = max(3, int(round(window_m / resolution_m)) | 1)   # 홀수 강제
    reference = median_filter(height_um, size=size, mode="reflect")
    depth = np.clip(reference - height_um, 0.0, None)
    return np.where(defect_mask, depth, 0.0)


def summarize(state) -> dict:
    """한 patch 의 품질 요약 (SYNTHETIC)."""
    res = state.residual_scratch_depth_um
    return {
        "ra_um": ra_um(state.micro_height_um),
        "rz_um": rz_um(state.micro_height_um),
        "rz_method": C.RZ_METHOD,
        "max_residual_scratch_um": float(res.max()),
        "mean_removal_um": float(state.cumulative_removal_um.mean()),
        "max_removal_um": float(state.cumulative_removal_um.max()),
        "clearcoat_min_um": float(state.clearcoat_remaining_um.min()),
        "healthy_overremoval_um": float(np.clip(
            state.cumulative_removal_um[state.healthy_mask] - C.HEALTHY_ALLOWANCE_UM,
            0.0, None).mean()) if state.healthy_mask.any() else 0.0,
        "heat_proxy_peak": float(state.heat_risk_proxy.max()),
        "temperature_mean_c": float(state.temperature_c.mean()),
        "temperature_peak_c": float(state.peak_temperature_c.max()),
        "friction_heat_flux_peak_w_m2": float(state.friction_heat_flux_w_m2.max()),
        "temperature_removal_factor_mean": float(state.temperature_removal_factor.mean()),
        "thermal_damage_mean": float(state.thermal_damage_proxy.mean()),
        "thermal_damage_peak": float(state.thermal_damage_proxy.max()),
        "coverage_ratio": float(
            (state.cumulative_removal_um >= C.MINIMUM_EFFECTIVE_REMOVAL_UM).mean()),
    }
