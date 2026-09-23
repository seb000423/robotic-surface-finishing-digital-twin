"""폴리싱 물리모델 파라미터 — 근거 태그 포함.


근거 태그:
    L-DIRECT   논문에서 직접 측정·사용한 값
    L-DERIVED  논문값을 단위변환·정규화·결합한 값
    PT-DESIGN 본 시뮬레이션이 시뮬레이션을 위해 선택한 값 — 논문 직접값으로 표현 금지
    SYNTHETIC  위 값을 사용해 모델이 생성한 결과 — 실제 계측값으로 표현 금지

단위 규칙 (섞지 말 것):
    길이·위치 m  /  미세높이·거칠기·도막 μm  /  시간 s  /  힘 N  /  압력 Pa  /  각속도 rad/s
"""
from __future__ import annotations

from dataclasses import dataclass, field

MODEL_VERSION = "literature_polishing_thermal_v2"

# ── 패드 (고정. 프로젝트 전 과정에서 바꾸지 않는다) ────────────────────────
PAD_DIAMETER_M = 0.110          # L-DERIVED  현행 v5 POLISHING_DISK_RADIUS=0.055 의 직경
PAD_RADIUS_M = PAD_DIAMETER_M / 2.0

# ── 초기 표면 ───────────────────────────────────────────────
CLEARCOAT_MIN_UM = 40.0         # L-DERIVED  자동차 Clearcoat 문헌 40~50 μm
CLEARCOAT_MAX_UM = 50.0         # L-DERIVED
CLEARCOAT_FIELD_CORRELATION_M = 0.05   # PT-DESIGN  두께는 독립난수가 아니라 저주파 상관장

RA_TARGET_UM = 0.08             # PT-DESIGN  anchor = Toyota 논문 0.078~0.083 μm (Alsoufi 2017)
RA_NOISE_CORRELATION_M = 0.004  # PT-DESIGN  정상 roughness 의 공간 상관길이

SCRATCH_DEPTH_MIN_UM = 0.05     # L-DIRECT   자동차 Clearcoat scratch 문헌 범위
SCRATCH_DEPTH_MAX_UM = 2.0      # L-DIRECT
SCRATCH_WIDTH_M = 0.002         # PT-DESIGN  논문에 직접분포 없음
SCRATCH_LENGTH_MIN_M = 0.03     # PT-DESIGN
SCRATCH_LENGTH_MAX_M = 0.12     # PT-DESIGN
SCRATCH_COUNT_MIN = 4           # PT-DESIGN
SCRATCH_COUNT_MAX = 12          # PT-DESIGN

# ── 패드 footprint ──────────────────────────────────────────
FOOTPRINT_SIGMA_RATIO = 0.5     # PT-DESIGN  rho=1 에서 가중치 exp(-2)=0.135
FOOTPRINT_ACTIVE_THRESHOLD = 0.05   # PT-DESIGN  dwell/pass 판정용 (정규화 가중치 기준)

# ── 제거식 계수 ─────────────────────────────────────────────
# 아래 네 개는 전부 reference force(8N)·평면(alignment=1)에서 정확히 1.0 이 되도록
#   정의돼 있다. 따라서 7장의 k 캘리브레이션 결과에 영향을 주지 않는다.
#   reference 조건을 벗어난 레시피에서만 작동한다.
REFERENCE_FORCE_N = 8.0         # L-DERIVED  reference_simulation.target_force_n
FORCE_EXPONENT = 0.0            # PT-DESIGN  압력항이 이미 힘에 선형이라 추가 지수는 0에서 시작
FORCE_SATURATION_GAIN = 0.15    # PT-DESIGN  고Force 효율포화. 근거: 항공기 프라이머 5/15/20N
                                #            Ra 1.653/1.350/1.606 — 단조증가가 아님
ALIGNMENT_EXPONENT = 1.0        # PT-DESIGN
COMPOUND_FACTOR = 1.0           # PT-DESIGN  compound 프로파일 미확정 — 1.0 고정
PAD_FACTOR = 1.0                # PT-DESIGN  고정 패드 1종

EFFECTIVE_RADIUS_RATIO = 2.0 / 3.0   # PT-DESIGN  면적가중 평균반경. k 에 흡수되므로 값 자체는
                                     #            중요하지 않으나 고정하고 기록한다

HEAT_EFFICIENCY_GAIN = 0.0      # PT-DESIGN  열에 의한 제거효율 저하. 근거 없어 0(비활성)으로
                                #            둔다. 훅만 남겨두고 열 데이터가 생기면 켠다.

# ── 미세형상 갱신 ───────────────────────────────────────────
PEAK_SELECTIVITY_GAIN = 1.0     # PT-DESIGN  sigmoid(gain * z_score)
SELECTIVE_REMOVAL_FRACTION = 1.0  # PT-DESIGN  돌출부 선택제거의 세기
BASE_REMOVAL_FRACTION = 0.5     # PT-DESIGN
# 이중합산 방지 규칙:
#   multiplier = base + selective * peak_selectivity 를 footprint 가중평균이 정확히 1.0 이
#   되도록 재정규화한다. base/selective 는 '재분배 모양'만 정하고 '총량'은 못 바꾼다.
HEIGHT_EPSILON_UM = 1e-6

# ── Ra / Rz ─────────────────────────────────────────────────
RZ_METHOD = "rz_5peak_5valley_v1"
DETREND_ORDER = 1               # PT-DESIGN  평면 patch 는 1차. 곡면은 2차로 올린다.
SCRATCH_REFERENCE_WINDOW_RATIO = 3.0   # PT-DESIGN  잔존 scratch 기준면 창 = 폭의 3배

# ── Dwell / Pass / Coverage ────────────────────────────────
MINIMUM_EFFECTIVE_REMOVAL_UM = 0.1    # PT-DESIGN  coverage 판정 하한
PASS_DEBOUNCE_S = 0.5                 # PT-DESIGN  inactive→active 재판정 최소 간격

# ── Heat proxy — 온도 단위가 아니다 ────────────────────────
HEAT_GAIN = 1e-6                # PT-DESIGN  임의 단위
COOLING_TIME_CONSTANT_S = 20.0  # PT-DESIGN
HEAT_PROXY_MAX = 10.0           # PT-DESIGN

# ── Literature-based synthetic thermal model (09 document) ──────────────────────
# PhysX does not solve clearcoat contact heat transfer.  These parameters form an
# explicit synthetic material profile; they must not be presented as measured
# temperatures for a Toyota/BMW coating.
THERMAL_MODEL_VERSION = "literature_synthetic_thermal_v1"
THERMAL_MATERIAL_PROFILE_ID = "generic_automotive_clearcoat_transfer_v1"
AMBIENT_TEMPERATURE_C = 23.0
INITIAL_TEMPERATURE_C = 23.0
FRICTION_COEFFICIENT = 0.35             # PT-DESIGN: pad/compound/clearcoat effective mu
HEAT_PARTITION_TO_COATING = 0.35        # PT-DESIGN: frictional heat entering coating stack
EFFECTIVE_AREAL_HEAT_CAPACITY_J_M2K = 2500.0  # PT-DESIGN: lumped coating+substrate capacity
THERMAL_COOLING_TIME_CONSTANT_S = 20.0  # PT-DESIGN: lumped convection/conduction cooling
TEMPERATURE_MIN_C = -50.0               # numerical guard, not an operating limit
TEMPERATURE_MAX_C = 200.0               # numerical guard, not an operating limit

# Piecewise interpolation is used because polymer wear need not increase
# monotonically with temperature. The points are a
# PT-DESIGN transfer profile, not direct um/pass data from a paper.
TEMPERATURE_FACTOR_POINTS_C = (23.0, 40.0, 60.0, 80.0)
REMOVAL_TEMPERATURE_FACTORS = (1.00, 1.12, 1.05, 1.25)

# Generic acrylic/melamine clearcoat transfer profile.  Tg~40 C is tied to the
# cited Trezona specimen only; it is not asserted for every vehicle clearcoat.
THERMAL_DAMAGE_ONSET_C = 35.0           # PT-DESIGN onset for the selected profile
THERMAL_PROFILE_TG_C = 40.0             # L-TRANSFER from the selected paper specimen
THERMAL_DAMAGE_TIME_SCALE_S = 120.0     # PT-DESIGN degree-exposure normalization
THERMAL_DAMAGE_MAX = 10.0
THERMAL_GLOSS_DAMAGE_SCALE = 1.0        # PT-DESIGN q_thermal exponential scale

# ── 안전·품질 한계 — 전부 PT-DESIGN. 논문 규격 아님 ────────
CLEARCOAT_SAFETY_LIMIT_UM = 35.0      #  차량 검사 시스템 기준으로 통일 (구 30.0)
#   BO recipe(recipe_00020) 는 30 제약으로 탐색됐으나 결과 clearcoat_min 38.38 ≥ 35 라
#     여전히 feasible — 재탐색 불필요. 관측 채널(안전여유)에도 쓰이므로 BC 챔피언은 재생성함.
HEALTHY_ALLOWANCE_UM = 1.0            # PT-DESIGN

# ── reference simulation — k 캘리브레이션 기준 ──────────────
@dataclass(frozen=True)
class ReferenceSimulation:
    """논문의 3단계 공정 집계 제거량 3 μm 를 재현하는 기준 시뮬레이션.

    실제 논문 조건과 동일하다고 표현하지 않는다 (L-DERIVED + PT-DESIGN 구성).
    """
    patch_size_m: tuple = (0.20, 0.20)          # L-DERIVED
    pad_diameter_m: float = PAD_DIAMETER_M      # L-DERIVED
    target_force_n: float = 8.0                 # L-DERIVED
    feed_speed_mm_s: float = 5.0                # L-DERIVED
    rpm_schedule: tuple = (4000.0, 3250.0, 2750.0)   # L-DERIVED  논문 OPM 중심값의 회전 proxy
    stage_time_ratio: tuple = (0.50, 1.0 / 3.0, 1.0 / 6.0)   # L-DERIVED  15/10/5분
    total_time_s: float = 1800.0                # L-DERIVED  15+10+5분.
    # ↑ 총 시간은 k 에 그대로 흡수된다 (제거량 ∝ k·시간). 즉 이 값을 바꾸면 k 가 반비례로
    #   바뀌고 다른 레시피의 예측은 동일하다. 민감한 선택이 아니다.
    path_type: str = "raster"                   # L-DERIVED
    step_over_spacing_ratio: float = 0.40       # PT-DESIGN
    target_mean_removal_um: float = 3.0         # L-DIRECT  논문 집계 제거량
    grid_resolution_m: float = 0.001            # PT-DESIGN
    quality_dt_s: float = 0.05                  # PT-DESIGN 20 Hz — 권장 주기


REFERENCE = ReferenceSimulation()


@dataclass
class PolishingModelConfig:
    """LiteraturePolishingModel 설정. 결과와 함께 저장한다."""
    model_version: str = MODEL_VERSION
    k_literature_synthetic: float | None = None   # 7장 캘리브레이션으로 채운다
    pad_radius_m: float = PAD_RADIUS_M
    footprint_sigma_ratio: float = FOOTPRINT_SIGMA_RATIO
    footprint_active_threshold: float = FOOTPRINT_ACTIVE_THRESHOLD
    effective_radius_ratio: float = EFFECTIVE_RADIUS_RATIO
    reference_force_n: float = REFERENCE_FORCE_N
    force_exponent: float = FORCE_EXPONENT
    force_saturation_gain: float = FORCE_SATURATION_GAIN
    alignment_exponent: float = ALIGNMENT_EXPONENT
    compound_factor: float = COMPOUND_FACTOR
    pad_factor: float = PAD_FACTOR
    heat_efficiency_gain: float = HEAT_EFFICIENCY_GAIN
    peak_selectivity_gain: float = PEAK_SELECTIVITY_GAIN
    base_removal_fraction: float = BASE_REMOVAL_FRACTION
    selective_removal_fraction: float = SELECTIVE_REMOVAL_FRACTION
    heat_gain: float = HEAT_GAIN
    cooling_time_constant_s: float = COOLING_TIME_CONSTANT_S
    heat_proxy_max: float = HEAT_PROXY_MAX
    thermal_enabled: bool = True
    thermal_model_version: str = THERMAL_MODEL_VERSION
    thermal_material_profile_id: str = THERMAL_MATERIAL_PROFILE_ID
    ambient_temperature_c: float = AMBIENT_TEMPERATURE_C
    friction_coefficient: float = FRICTION_COEFFICIENT
    heat_partition_to_coating: float = HEAT_PARTITION_TO_COATING
    effective_areal_heat_capacity_j_m2k: float = EFFECTIVE_AREAL_HEAT_CAPACITY_J_M2K
    thermal_cooling_time_constant_s: float = THERMAL_COOLING_TIME_CONSTANT_S
    temperature_factor_points_c: tuple = TEMPERATURE_FACTOR_POINTS_C
    removal_temperature_factors: tuple = REMOVAL_TEMPERATURE_FACTORS
    thermal_damage_onset_c: float = THERMAL_DAMAGE_ONSET_C
    thermal_profile_tg_c: float = THERMAL_PROFILE_TG_C
    thermal_damage_time_scale_s: float = THERMAL_DAMAGE_TIME_SCALE_S
    thermal_damage_max: float = THERMAL_DAMAGE_MAX
    minimum_effective_removal_um: float = MINIMUM_EFFECTIVE_REMOVAL_UM
    pass_debounce_s: float = PASS_DEBOUNCE_S
    tags: dict = field(default_factory=lambda: {
        "k_literature_synthetic": "L-DERIVED",   # 논문 3 μm 앵커에서 역산
        "force_exponent": "PT-DESIGN",
        "force_saturation_gain": "PT-DESIGN",
        "alignment_exponent": "PT-DESIGN",
        "compound_factor": "PT-DESIGN",
        "pad_factor": "PT-DESIGN",
        "peak_selectivity_gain": "PT-DESIGN",
        "base_removal_fraction": "PT-DESIGN",
        "selective_removal_fraction": "PT-DESIGN",
        "heat_gain": "PT-DESIGN",
        "thermal_model": "L-TRANSFER+PT-DESIGN",
        "thermal_enabled": "PT-DESIGN",
        "friction_coefficient": "PT-DESIGN",
        "heat_partition_to_coating": "PT-DESIGN",
        "effective_areal_heat_capacity_j_m2k": "PT-DESIGN",
        "thermal_cooling_time_constant_s": "PT-DESIGN",
        "temperature_factor_profile": "L-TRANSFER+PT-DESIGN",
        "thermal_damage_profile": "L-TRANSFER+PT-DESIGN",
    })
