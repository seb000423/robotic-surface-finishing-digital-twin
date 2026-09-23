"""SurfaceState 와 초기 표면 생성 — ·3장.

`nominal_surface_xyz_m` (거시형상, m) 와 `micro_height_um` (미세형상, μm) 를 분리한다.
Ra/Rz/Scratch 는 전부 micro_height_um 에서 계산한다. 단위를 섞지 않는다.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from scipy.ndimage import gaussian_filter

from . import config as C


@dataclass
class SurfaceState:
    """한 작업 patch 의 상태맵 묶음."""
    resolution_m: float
    nominal_surface_xyz_m: np.ndarray      # (N, M, 3)
    normal_xyz: np.ndarray                 # (N, M, 3)
    micro_height_um: np.ndarray            # (N, M)
    initial_micro_height_um: np.ndarray    # (N, M)  before/after 비교용
    initial_scratch_depth_um: np.ndarray
    residual_scratch_depth_um: np.ndarray
    cumulative_removal_um: np.ndarray
    clearcoat_remaining_um: np.ndarray
    initial_clearcoat_um: np.ndarray
    dwell_time_s: np.ndarray
    pass_count: np.ndarray                 # int
    peak_contact_pressure_proxy: np.ndarray
    heat_risk_proxy: np.ndarray
    temperature_c: np.ndarray                 # synthetic surface temperature [deg C]
    peak_temperature_c: np.ndarray            # maximum synthetic temperature [deg C]
    friction_heat_flux_w_m2: np.ndarray       # latest local friction heat flux [W/m2]
    temperature_removal_factor: np.ndarray    # dimensionless piecewise material factor
    thermal_damage_proxy: np.ndarray          # dimensionless cumulative thermal exposure
    healthy_mask: np.ndarray               # bool
    defect_mask: np.ndarray                # bool
    last_active_time_s: np.ndarray         # pass debounce 용 (내부 상태)
    seed: int

    @property
    def shape(self) -> tuple:
        return self.micro_height_um.shape

    @property
    def cell_area_m2(self) -> float:
        return self.resolution_m ** 2

    def copy(self) -> "SurfaceState":
        import copy as _copy
        return _copy.deepcopy(self)


def _correlated_field(shape, correlation_m: float, resolution_m: float,
                      rng: np.random.Generator) -> np.ndarray:
    """저주파 상관 난수장. 독립난수가 아니라 공간적으로 부드럽게 변한다."""
    sigma_cells = max(correlation_m / resolution_m, 0.5)
    field = gaussian_filter(rng.standard_normal(shape), sigma=sigma_cells, mode="reflect")
    std = field.std()
    return field / std if std > 1e-12 else field


def _distance_to_segment(xx, yy, p0, p1) -> np.ndarray:
    """격자 각 점에서 선분 p0→p1 까지의 최단거리 [m]."""
    d = np.asarray(p1, float) - np.asarray(p0, float)
    length_sq = float(d @ d)
    if length_sq < 1e-12:
        return np.hypot(xx - p0[0], yy - p0[1])
    t = ((xx - p0[0]) * d[0] + (yy - p0[1]) * d[1]) / length_sq
    t = np.clip(t, 0.0, 1.0)
    return np.hypot(xx - (p0[0] + t * d[0]), yy - (p0[1] + t * d[1]))


def make_flat_patch(patch_size_m=(0.20, 0.20),
                    resolution_m: float = 0.001,
                    seed: int = 0,
                    n_scratches: int | None = None,
                    target_ra_um: float = C.RA_TARGET_UM,
                    with_scratches: bool = True) -> SurfaceState:
    """평면 patch 초기 표면 생성.

    Ra 는 `mean(|z - mean z|)` 정의로 target_ra_um 에 맞춘다.
    한계: 격자 간격이 mm 급이므로 여기서의 Ra 는 실제 조도계(μm 급 샘플링)의 Ra 와
      같은 물리량이 아니다. 모델 내부 일관성 지표로만 쓴다 (SYNTHETIC).
    """
    rng = np.random.default_rng(seed)
    nx = int(round(patch_size_m[0] / resolution_m))
    ny = int(round(patch_size_m[1] / resolution_m))
    shape = (nx, ny)

    x = (np.arange(nx) + 0.5) * resolution_m
    y = (np.arange(ny) + 0.5) * resolution_m
    xx, yy = np.meshgrid(x, y, indexing="ij")

    nominal = np.stack([xx, yy, np.zeros_like(xx)], axis=-1)
    normal = np.zeros((nx, ny, 3))
    normal[..., 2] = 1.0

    # ── 정상 roughness: band-limited noise 를 target Ra 로 스케일 (3.2) ──
    micro = _correlated_field(shape, C.RA_NOISE_CORRELATION_M, resolution_m, rng)
    micro -= micro.mean()
    mad = np.abs(micro).mean()
    micro *= target_ra_um / max(mad, 1e-12)

    # ── Clearcoat: 저주파 상관장 위에 40~50 μm (3.1) ──
    cc_field = _correlated_field(shape, C.CLEARCOAT_FIELD_CORRELATION_M, resolution_m, rng)
    cc_unit = 0.5 * (1.0 + np.clip(cc_field / 3.0, -1.0, 1.0))      # 0~1 로 압축
    clearcoat = C.CLEARCOAT_MIN_UM + cc_unit * (C.CLEARCOAT_MAX_UM - C.CLEARCOAT_MIN_UM)

    # ── Scratch: 직선 groove (3.3) ──
    scratch_depth = np.zeros(shape)
    if with_scratches:
        if n_scratches is None:
            n_scratches = int(rng.integers(C.SCRATCH_COUNT_MIN, C.SCRATCH_COUNT_MAX + 1))
        for _ in range(n_scratches):
            depth_um = float(rng.uniform(C.SCRATCH_DEPTH_MIN_UM, C.SCRATCH_DEPTH_MAX_UM))
            length_m = float(rng.uniform(C.SCRATCH_LENGTH_MIN_M, C.SCRATCH_LENGTH_MAX_M))
            angle = float(rng.uniform(0.0, np.pi))
            cx = float(rng.uniform(0.0, patch_size_m[0]))
            cy = float(rng.uniform(0.0, patch_size_m[1]))
            half = 0.5 * length_m * np.array([np.cos(angle), np.sin(angle)])
            dist = _distance_to_segment(xx, yy, (cx - half[0], cy - half[1]),
                                        (cx + half[0], cy + half[1]))
            groove = depth_um * np.exp(-(dist / C.SCRATCH_WIDTH_M) ** 2)
            scratch_depth = np.maximum(scratch_depth, groove)

    micro = micro - scratch_depth       # groove 는 표면을 파고든다

    defect = scratch_depth > (0.5 * C.SCRATCH_DEPTH_MIN_UM)

    return SurfaceState(
        resolution_m=resolution_m,
        nominal_surface_xyz_m=nominal,
        normal_xyz=normal,
        micro_height_um=micro.copy(),
        initial_micro_height_um=micro.copy(),
        initial_scratch_depth_um=scratch_depth,
        residual_scratch_depth_um=scratch_depth.copy(),
        cumulative_removal_um=np.zeros(shape),
        clearcoat_remaining_um=clearcoat.copy(),
        initial_clearcoat_um=clearcoat.copy(),
        dwell_time_s=np.zeros(shape),
        pass_count=np.zeros(shape, dtype=np.int32),
        peak_contact_pressure_proxy=np.zeros(shape),
        heat_risk_proxy=np.zeros(shape),
        temperature_c=np.full(shape, C.INITIAL_TEMPERATURE_C),
        peak_temperature_c=np.full(shape, C.INITIAL_TEMPERATURE_C),
        friction_heat_flux_w_m2=np.zeros(shape),
        temperature_removal_factor=np.ones(shape),
        thermal_damage_proxy=np.zeros(shape),
        healthy_mask=~defect,
        defect_mask=defect,
        last_active_time_s=np.full(shape, -1e9),
        seed=seed,
    )


# ── 곡면 patch 생성 ─────────────────────
def make_curved_patch(kind: str = "cylinder",
                      curvature_radius_m: float = 0.6,
                      patch_size_m=(0.12, 0.12),
                      resolution_m: float = 0.002,
                      seed: int = 0,
                      n_scratches: int | None = None,
                      target_ra_um: float = C.RA_TARGET_UM,
                      with_scratches: bool = True, quad_coeffs=None) -> SurfaceState:
    """곡면 patch — nominal 형상과 법선만 곡면으로 바꾸고 미세층(거칠기·스크래치·
    clearcoat·열)은 평면과 동일 절차로 생성한다.

    설계 근거:
      · 품질 모델은 (u,v) 격자 위 micro_height 로 동작하므로, 곡률의 효과는
        ① nominal_xyz(로봇 경로·시각화), ② normal_xyz(자세→접촉 상수·정렬),
        ③ detrend(roughness_metrics 는 2차 피팅까지 지원 — DETREND_ORDER 주의)로 들어온다.
      · kind="cylinder": u 축을 따라 반경 R 원통 (펜더/루프 가장자리 근사).
        kind="sphere"  : 반경 R 구면 캡 (보닛 중앙부 근사).
      · 곡률 sagitta 는 patch 대비 작아야 한다 (0.12 m patch, R≥0.3 m → sag ≤ 6 mm).
    SYNTHETIC — 실측 차체 곡률 아님. 실차 맵은 이후.
    """
    if kind not in ("cylinder", "sphere", "freeform", "quad"):
        raise ValueError(f"kind must be cylinder|sphere|freeform|quad, got {kind}")
    st = make_flat_patch(patch_size_m, resolution_m, seed=seed,
                         n_scratches=n_scratches, target_ra_um=target_ra_um,
                         with_scratches=with_scratches)
    xx = st.nominal_surface_xyz_m[..., 0]
    yy = st.nominal_surface_xyz_m[..., 1]
    cu = xx - patch_size_m[0] / 2.0          # patch 중심 기준 좌표
    cv = yy - patch_size_m[1] / 2.0
    R = float(curvature_radius_m)

    if kind in ("freeform", "quad"):
        nx, ny = st.shape
        h = np.zeros((nx, ny)); nrm = np.zeros((nx, ny, 3))
        for i in range(nx):
            for j in range(ny):
                hh, nn = curve_height_normal(kind, curvature_radius_m, patch_size_m,
                                             float(xx[i, j]), float(yy[i, j]),
                                             freeform_seed=seed, quad_coeffs=quad_coeffs)
                h[i, j] = hh; nrm[i, j] = nn
        st.nominal_surface_xyz_m[..., 2] = h
        st.normal_xyz = nrm
        return st

    if kind == "cylinder":
        # z = R − sqrt(R² − cu²)  (u 방향으로만 굽음), 법선 = (−cu, 0, sqrt(R²−cu²))/R
        under = np.clip(R * R - cu * cu, 1e-12, None)
        z = R - np.sqrt(under)
        n = np.stack([-cu, np.zeros_like(cv), np.sqrt(under)], axis=-1) / R
    else:  # sphere
        rho2 = cu * cu + cv * cv
        under = np.clip(R * R - rho2, 1e-12, None)
        z = R - np.sqrt(under)
        n = np.stack([-cu, -cv, np.sqrt(under)], axis=-1) / R

    st.nominal_surface_xyz_m[..., 2] = -z    # 볼록면: 중심이 가장 높게 (차체 외판)
    st.normal_xyz = n / np.linalg.norm(n, axis=-1, keepdims=True)
    return st


def _freeform_params(seed: int, patch_size_m):
    """자유곡면 = 가우시안 범프 K개의 합 (해석식 — 어느 (u,v)에서도 높이·법선 일관).

    진폭·폭은 PT-DESIGN: 최대 경사 ~8° (검증된 원통 R=0.3 의 tilt 범위 안), sagitta 수 mm.
    """
    rng = np.random.default_rng(seed + 777_000)
    K = 6
    return [(float(rng.uniform(0.15, 0.85) * patch_size_m[0]),
             float(rng.uniform(0.15, 0.85) * patch_size_m[1]),
             float(rng.uniform(-0.0025, 0.0025)),           # 진폭 ±2.5 mm
             float(rng.uniform(0.03, 0.06)))                # 폭 3~6 cm
            for _ in range(K)]


def curve_height_normal(kind: str, radius_m: float, patch_size_m, u: float, v: float,
                        freeform_seed: int = 0, quad_coeffs=None):
    """(u,v) 에서의 곡면 높이 h(중심=0, 가장자리 음수)와 단위 법선. flat 이면 (0, +z).

    make_curved_patch 의 nominal 과 동일 규약 — env 의 IK 목표·힘 투영이 이 식을 쓴다.
    """
    if kind == "flat":
        return 0.0, np.array([0.0, 0.0, 1.0])
    cu = u - patch_size_m[0] / 2.0
    cv = v - patch_size_m[1] / 2.0
    if kind == "quad":
        # 실차 스캔 셀의 국소 2차곡면 (car_cells_robot: 셀 점군 최소제곱). 계수 6개, 중심 기준.
        c = quad_coeffs if quad_coeffs is not None else (0.0,) * 6
        h = c[0] + c[1] * cu + c[2] * cv + c[3] * cu * cu + c[4] * cu * cv + c[5] * cv * cv
        du = c[1] + 2.0 * c[3] * cu + c[4] * cv
        dv = c[2] + c[4] * cu + 2.0 * c[5] * cv
        n = np.array([-du, -dv, 1.0])
        return float(h), n / np.linalg.norm(n)
    R = float(radius_m)
    if kind == "cylinder":
        under = max(R * R - cu * cu, 1e-12)
        h = np.sqrt(under) - R
        n = np.array([-cu, 0.0, np.sqrt(under)]) / R
    elif kind == "sphere":
        under = max(R * R - cu * cu - cv * cv, 1e-12)
        h = np.sqrt(under) - R
        n = np.array([-cu, -cv, np.sqrt(under)]) / R
    elif kind == "freeform":
        h, du, dv = 0.0, 0.0, 0.0
        for cx, cy, amp, w in _freeform_params(freeform_seed, patch_size_m):
            g = amp * np.exp(-(((u - cx) ** 2) + ((v - cy) ** 2)) / (2.0 * w * w))
            h += g
            du += g * (-(u - cx) / (w * w))
            dv += g * (-(v - cy) / (w * w))
        n = np.array([-du, -dv, 1.0])
        return float(h), n / np.linalg.norm(n)
    else:
        raise ValueError(f"unknown surface kind: {kind}")
    return float(h), n / np.linalg.norm(n)
