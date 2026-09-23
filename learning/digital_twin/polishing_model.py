"""LiteraturePolishingModel.

패드 접촉상태(힘·위치·자세·속도)를 위치별 Clearcoat 품질 변화로 변환하는
커스텀 상태 전이모델. PhysX 마모기능이 아니다.

출력은 전부 SYNTHETIC 태그 대상 — 실제 계측값으로 표현하지 않는다.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from . import config as C
from .roughness_metrics import residual_scratch_depth_um, summarize
from .surface_state import SurfaceState


@dataclass
class ContactState:
    """한 quality step 동안의 패드 접촉 요약."""
    pad_center_uv_m: tuple          # patch 국소좌표 (u, v) [m]
    contact_force_n: float
    rpm: float
    feed_speed_m_s: float
    normal_alignment: float = 1.0   # dot(pad_normal, surface_normal), 평면=1
    in_contact: bool = True


class LiteraturePolishingModel:
    def __init__(self, cfg: C.PolishingModelConfig):
        if cfg.k_literature_synthetic is None:
            raise ValueError(
                "k_literature_synthetic 이 없다. calibrate_k() 로 reference 3μm 정규화를 "
                "먼저 수행할 것. 임의값 확정 금지.")
        self.cfg = cfg

    def _cool_temperature(self, state: SurfaceState, dt_s: float) -> None:
        """Cool every cell exactly for a first-order lumped thermal model."""
        tau = max(self.cfg.thermal_cooling_time_constant_s, 1e-9)
        decay = np.exp(-dt_s / tau)
        ambient = self.cfg.ambient_temperature_c
        state.temperature_c[:] = ambient + (state.temperature_c - ambient) * decay
        np.clip(state.temperature_c, C.TEMPERATURE_MIN_C, C.TEMPERATURE_MAX_C,
                out=state.temperature_c)
        state.friction_heat_flux_w_m2.fill(0.0)

    def _update_contact_temperature(self, state: SurfaceState, sl, shape01,
                                    pressure_pa: float, relative_speed: float,
                                    dt_s: float) -> tuple[np.ndarray, float]:
        """Update synthetic temperature and return its step-average in the footprint.

        q = mu*P*V follows the moving heat-source energy structure.  Heat partition,
        areal heat capacity and cooling are explicit PT-DESIGN transfer parameters.
        """
        cfg = self.cfg
        if not cfg.thermal_enabled:
            state.temperature_removal_factor[sl] = 1.0
            state.friction_heat_flux_w_m2.fill(0.0)
            return np.ones_like(shape01), 0.0
        tau = max(cfg.thermal_cooling_time_constant_s, 1e-9)
        decay = np.exp(-dt_s / tau)
        ambient = cfg.ambient_temperature_c
        old_local = state.temperature_c[sl].copy()

        # First cool the complete coating map so cells outside the footprint also cool.
        self._cool_temperature(state, dt_s)

        heat_flux = cfg.friction_coefficient * pressure_pa * relative_speed * shape01
        heating_rate = (cfg.heat_partition_to_coating * heat_flux
                        / max(cfg.effective_areal_heat_capacity_j_m2k, 1e-9))
        equilibrium_rise = heating_rate * tau
        end_local = (ambient + (old_local - ambient) * decay
                     + equilibrium_rise * (1.0 - decay))
        state.temperature_c[sl] = np.clip(
            end_local, C.TEMPERATURE_MIN_C, C.TEMPERATURE_MAX_C)
        state.friction_heat_flux_w_m2[sl] = heat_flux
        state.peak_temperature_c[sl] = np.maximum(
            state.peak_temperature_c[sl], state.temperature_c[sl])

        # Exact average temperature over dt for constant heat input.
        if dt_s > 0.0:
            mean_decay = tau * (1.0 - decay) / dt_s
            mean_local = (ambient + equilibrium_rise
                          + (old_local - ambient - equilibrium_rise) * mean_decay)
        else:
            mean_local = old_local
        factor = np.interp(
            mean_local,
            np.asarray(cfg.temperature_factor_points_c, dtype=float),
            np.asarray(cfg.removal_temperature_factors, dtype=float),
        )
        state.temperature_removal_factor[sl] = factor

        onset = cfg.thermal_damage_onset_c
        span = max(cfg.thermal_profile_tg_c - onset, 1e-6)
        exposure = np.clip((mean_local - onset) / span, 0.0, None)
        damage_before = state.thermal_damage_proxy[sl].copy()
        state.thermal_damage_proxy[sl] += (
            exposure * dt_s / max(cfg.thermal_damage_time_scale_s, 1e-9) * shape01)
        np.clip(state.thermal_damage_proxy, 0.0, cfg.thermal_damage_max,
                out=state.thermal_damage_proxy)
        damage_delta_mean = float(
            (state.thermal_damage_proxy[sl] - damage_before).mean())
        return factor, damage_delta_mean

    # ── 4장: footprint ────────────────────────────────────────────────────
    #  weight=raw/sum(raw) (합=1) 를 그대로 깊이에 곱하면 제거 "깊이"가
    #   격자 셀 수에 반비례해 해상도 의존이 된다 (단위시험 10 위반). 깊이는 물리량이므로
    #   무차원 모양함수 raw(0~1) 를 곱하고, 스케일은 k 가 흡수한다.
    #   sum=1 정규화 가중치는 footprint 내 통계(가중평균)에만 쓴다.
    def _footprint(self, state: SurfaceState, center_uv):
        """반환: (slice, shape01, w_dist, area_m2).
        shape01 = 무차원 모양함수(0~1, 깊이에 곱함) / w_dist = 합=1 분포(통계용)."""
        res = state.resolution_m
        R = self.cfg.pad_radius_m
        nx, ny = state.shape
        i0 = max(int((center_uv[0] - R) / res), 0)
        i1 = min(int((center_uv[0] + R) / res) + 1, nx)
        j0 = max(int((center_uv[1] - R) / res), 0)
        j1 = min(int((center_uv[1] + R) / res) + 1, ny)
        if i0 >= i1 or j0 >= j1:
            return None, None, None, 0.0

        u = (np.arange(i0, i1) + 0.5) * res
        v = (np.arange(j0, j1) + 0.5) * res
        uu, vv = np.meshgrid(u, v, indexing="ij")
        rho = np.hypot(uu - center_uv[0], vv - center_uv[1]) / R

        raw = np.exp(-0.5 * (rho / self.cfg.footprint_sigma_ratio) ** 2)
        raw[rho > 1.0] = 0.0
        total = raw.sum()
        if total <= 0.0:
            return None, None, None, 0.0
        w_dist = raw / total                      # sum = 1 — footprint 내 통계 전용
        area_m2 = float((raw > 0.0).sum()) * res * res   # 실제 접촉셀 면적 (patch 경계 잘림 반영)
        return (slice(i0, i1), slice(j0, j1)), raw, w_dist, area_m2

    # ── 5~6장: 제거식 ─────────────────────────────────────────────────────
    def _delta_removal_um(self, contact: ContactState, shape01, area_m2, dt_s):
        cfg = self.cfg
        pressure_pa = contact.contact_force_n / max(area_m2, 1e-6)

        omega = contact.rpm * 2.0 * np.pi / 60.0
        spin = omega * cfg.effective_radius_ratio * cfg.pad_radius_m
        relative_speed = float(np.hypot(spin, contact.feed_speed_m_s))
        # Feed 는 1/feed 항으로 다시 곱하지 않는다 — dwell 로 자연 반영

        alignment = np.clip(contact.normal_alignment, 0.0, 1.0) ** cfg.alignment_exponent
        force_ratio = contact.contact_force_n / cfg.reference_force_n
        force_eff = force_ratio ** cfg.force_exponent
        force_eff /= 1.0 + cfg.force_saturation_gain * max(0.0, force_ratio - 1.0) ** 2
        heat_eff = 1.0     # heat_efficiency_gain=0 — 훅만 유지 (config 참고)

        return (cfg.k_literature_synthetic * pressure_pa * relative_speed * dt_s
                * shape01 * cfg.compound_factor * cfg.pad_factor
                * alignment * force_eff * heat_eff), pressure_pa, relative_speed

    # ── 8장: 미세형상 갱신 ────────────────────────────────────────────────
    @staticmethod
    def _peak_selective_multiplier(height, weight, cfg):
        """base + selective·sigmoid(z_score) 를 footprint 가중평균 1.0 으로 재정규화.

        총 제거량은 delta_removal 이 정하고, 이 항은 '재분배 모양'만 정한다 (이중합산 금지).
        """
        mean = float((weight * height).sum())
        var = float((weight * (height - mean) ** 2).sum())
        std = max(np.sqrt(var), C.HEIGHT_EPSILON_UM)
        z = (height - mean) / std
        sel = 1.0 / (1.0 + np.exp(-cfg.peak_selectivity_gain * z))
        mult = cfg.base_removal_fraction + cfg.selective_removal_fraction * sel
        norm = float((weight * mult).sum())
        return mult / max(norm, 1e-12)

    # ── step ──────────────────────────────────────────────────────────────
    def step(self, state: SurfaceState, contact: ContactState,
             dt_s: float, sim_time_s: float = 0.0) -> dict:
        """한 quality step. state 를 in-place 갱신하고 delta 요약을 돌려준다."""
        if not contact.in_contact or contact.contact_force_n <= 0.0:
            # 접촉 없으면 제거 0 (단위시험 1). 열은 계속 식는다.
            state.heat_risk_proxy -= (state.heat_risk_proxy
                                      / self.cfg.cooling_time_constant_s) * dt_s
            np.clip(state.heat_risk_proxy, 0.0, self.cfg.heat_proxy_max,
                    out=state.heat_risk_proxy)
            self._cool_temperature(state, dt_s)
            return {"mean_removal_delta_um": 0.0}

        sl, shape01, w_dist, area_m2 = self._footprint(state, contact.pad_center_uv_m)
        if sl is None:
            self._cool_temperature(state, dt_s)
            return {"mean_removal_delta_um": 0.0}

        delta, pressure_pa, rel_speed = self._delta_removal_um(
            contact, shape01, area_m2, dt_s)
        temperature_factor, thermal_damage_delta = self._update_contact_temperature(
            state, sl, shape01, pressure_pa, rel_speed, dt_s)

        # 8장: 돌출부 선택 제거
        height = state.micro_height_um[sl]
        mult = self._peak_selective_multiplier(height, w_dist, self.cfg)
        local_removal = delta * mult * temperature_factor

        # Clearcoat 을 음수로 뚫지 않는다 (물질수지)
        local_removal = np.minimum(local_removal, state.clearcoat_remaining_um[sl])

        state.micro_height_um[sl] -= local_removal
        state.cumulative_removal_um[sl] += local_removal
        state.clearcoat_remaining_um[sl] -= local_removal

        # RL 조밀 보상용 분해:
        #   defect 셀 위 제거 = 유익 / healthy 셀에서 허용치 초과 후 제거 = 손상
        # 합계가 아니라 셀당 평균 — footprint 의 정상 셀(~2400)이 결함 셀(~50~100)보다
        #   25배 많아, 합계로 주면 정상부 감점이 지배해 "덜 문지르기"를 배운다
        #   (1차 학습에서 실측: reward ↑ 인데 GU 70→65, scratch 0.82→1.33μm 악화).
        # defect 지급은 잔여 스크래치만큼만 — 정적 defect_mask 로 무제한 지급하면
        #   이미 지운 자리를 계속 문질러 보상을 농사짓는 exploit 이 생긴다
        #   (3차 학습에서 실측: 전 구간 힘·속도 최대 포화, GU 미개선).
        defect = state.defect_mask[sl]
        over = state.cumulative_removal_um[sl] > C.HEALTHY_ALLOWANCE_UM
        remaining = np.clip(state.initial_scratch_depth_um[sl]
                            - (state.cumulative_removal_um[sl] - local_removal), 0.0, None)
        payable = np.minimum(local_removal, remaining)
        n_def, n_ho = int(defect.sum()), int((~defect & over).sum())
        defect_removal = float(payable[defect].sum() / max(n_def, 1))
        healthy_over_removal = float(local_removal[~defect & over].sum() / max(n_ho, 1))

        # 10장: dwell / pass (debounce) — 모양함수 기준 (무차원, 해상도 불변)
        active = shape01 > self.cfg.footprint_active_threshold
        state.dwell_time_s[sl][active] += dt_s
        gap_ok = (sim_time_s - state.last_active_time_s[sl]) > self.cfg.pass_debounce_s
        new_pass = active & gap_ok
        state.pass_count[sl][new_pass] += 1
        state.last_active_time_s[sl][active] = sim_time_s

        # 11장: heat proxy (임의 단위 — 온도 아님)
        heat_in = self.cfg.heat_gain * pressure_pa * rel_speed
        state.heat_risk_proxy[sl] += heat_in * dt_s * shape01
        state.heat_risk_proxy -= (state.heat_risk_proxy
                                  / self.cfg.cooling_time_constant_s) * dt_s
        np.clip(state.heat_risk_proxy, 0.0, self.cfg.heat_proxy_max,
                out=state.heat_risk_proxy)

        state.peak_contact_pressure_proxy[sl] = np.maximum(
            state.peak_contact_pressure_proxy[sl], pressure_pa)

        return {"mean_removal_delta_um": float(local_removal.mean()),
                "defect_removal_um": defect_removal,
                "healthy_over_removal_um": healthy_over_removal,
                "pressure_pa": pressure_pa,
                "temperature_mean_c": float(state.temperature_c[sl].mean()),
                "temperature_peak_c": float(state.peak_temperature_c[sl].max()),
                "temperature_removal_factor_mean": float(temperature_factor.mean()),
                "thermal_damage_delta_mean": thermal_damage_delta}

    # ── evaluate ──────────────────────────────────────────────────────────
    def evaluate(self, state: SurfaceState) -> dict:
        """에피소드 종료 시 품질 요약 (SYNTHETIC). 잔존 scratch 를 여기서 재계산한다."""
        state.residual_scratch_depth_um = residual_scratch_depth_um(
            state.micro_height_um, state.defect_mask, state.resolution_m)
        out = summarize(state)
        out["model_version"] = self.cfg.model_version
        out["k_literature_synthetic"] = self.cfg.k_literature_synthetic
        return out
