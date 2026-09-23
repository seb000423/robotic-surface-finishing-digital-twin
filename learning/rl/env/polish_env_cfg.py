"""PolishEnv 설정 — Isaac Lab DirectRLEnv.

이 증분의 범위:
  · Isaac Lab 환경 골격 + 기준 제어기(action=0) 동작
  · 접촉력 = 검증된 가상 스프링 + 어드미턴스 (learning/rl/env/contact.py — 9/9 검증)
  · 품질 = LiteraturePolishingModel + GU proxy (단위시험 10/10 + 8/8)
씬은 최소(지면+조명)다. 로봇 팔(M0609+RMPFlow)과 시각화는 다음 증분 —
접촉이 해석 모델이므로(원 시뮬도 강체 충돌 OFF) 힘·품질 거동은 팔 없이 성립한다
"""
from __future__ import annotations

import os

from isaaclab.envs import DirectRLEnvCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sim import SimulationCfg
from isaaclab.utils.configclass import configclass

_TWIN_OUT = os.path.join(
    os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))),
    "digital_twin", "outputs")


@configclass
class PolishEnvCfg(DirectRLEnvCfg):
    # ── env ──
    decimation = 3                      # 60 Hz physics → 20 Hz control/quality
    # truncation 상한. BO recipe 공칭 완주 ≈ 242 s 인데 300 이면 여유 24% — 정책이 경로의
    # 37% 이상에서 feed −50% 를 쓰면 타임아웃에 잘린다 (dwell 전략의 상한을 타이머가 정하는
    # 꼴 + baseline 은 완주하고 감속 정책만 미완주하는 판정 비대칭). 전 구간 최대 감속
    # (feed ×0.5 → ≈484 s)도 완주 가능하게 500 으로 확대.
    episode_length_s = 500.0
    action_space = 2                    # 임피던스형 잔차 [Δforce_ratio, Δfeed_ratio]
    #  두 잔차안 중 임피던스형 채택 — 기존 프로젝트 잔차 개념(힘·속도)과
    #   일치시키기 위함. 기하형(법선offset/tilt)은 로봇 팔 증분에서 재검토.
    # 관측 구성:
    #   기본 11ch + use_thermal_obs(+3: 국소 현재/최고온도·열손상) + use_spatial_obs(+4:
    #   경로 lookahead 잔여 scratch near/far mean·max). observation_space 는 학습 스크립트가
    #   플래그에 맞춰 셋팅한다 (11/14/15/18 — export 는 차원으로 구성을 식별).
    # 구 11차원 checkpoint는 새 obs env에 resume할 수 없으며 평가 시 앞 11채널만 전달한다.
    observation_space = 14
    use_thermal_obs: bool = True
    use_spatial_obs: bool = False
    state_space = 0

    # ── simulation ── (강체 동역학 없음 — sim 은 시간축·씬 관리·향후 로봇용)
    sim: SimulationCfg = SimulationCfg(dt=1 / 60, render_interval=decimation)
    scene: InteractiveSceneCfg = InteractiveSceneCfg(num_envs=4, env_spacing=1.0)

    # ── recipe ──
    # BO export 를 그대로 읽는다. 없으면 reference 기준 recipe 로 폴백.
    recipe_json_path: str = os.path.join(_TWIN_OUT, "bo_best_recipe.json")

    # ── 작업면 자세 ──
    # env 중 이 비율만큼을 side(수직면) 접촉 상수로 돌린다. 관측은 힘 오차 채널로 자세 추론.
    side_env_ratio: float = 0.0          # 0 = top 전용. side 혼합 학습 시 0.5

    # ── 표면 patch ──
    patch_size_m: tuple = (0.12, 0.12)   # 스모크 규모. 본 학습에서 0.20 으로 확대
    patch_resolution_m: float = 0.002
    surface_seed_base: int = 1000
    # ── 표면 곡률 ──
    # flat(기본)|cylinder|sphere. 곡률의 물리 효과는 PhysX 로봇 환경에서 실재(IK+PhysX 접촉),
    # 해석식 환경에서는 표면 생성만 곡면 (품질 모델은 (u,v) 격자라 동일 동작 — 명시).
    surface_kind: str = "flat"
    curvature_radius_m: float = 0.6
    freeform_seed: int = 0               # freeform 형상 seed (에피소드 표면 seed 와 별개)        # env i, episode e → seed = base + 97*i + e

    # ── 잔차 스케일 ──
    force_ratio_limit: float = 0.3       # 힘 목표 ±30 %
    feed_ratio_limit: float = 0.5        # 이송 ±50 %

    # ── 안전 ──
    force_hard_limit_n: float = 14.0     # v5 상단 hard limit
    clearcoat_safety_limit_um: float = 35.0   # 프로젝트 기준 (digital_twin.config 와 동일값 유지)

    # ── 중간(dense) 보상 가중치 ──
    # 종말 보상 도입(아래) 후 dense 항의 역할은 4가지로 제한된다:
    #   접촉력 안전(w_force) / 행동 급변 억제(w_action_rate) /
    #   Clearcoat 과다 제거 방지(w_healthy_over) / 결함 개선 방향 제공(w_defect_removal)
    w_force: float = 0.4
    force_tolerance_n: float = 1.0
    # 품질항은 footprint 셀당 평균 [μm/step] 기준 (스텝당 ~1e-3 μm → 가중치로 스케일 업).
    # 결함 위 제거를 순이득으로: 결함 구간에선 defect ≈ healthy_over 크기이므로 w 비 > 1 필요.
    w_defect_removal: float = 400.0      # defect 셀당 평균 제거 [μm]당
    w_healthy_over: float = 250.0        # 정상부 허용치 초과 셀당 평균 [μm]당
    w_action_rate: float = 0.05
    w_time: float = 0.01
    w_thermal_damage: float = 1000.0  # 열손상 증분(proxy/step) 페널티, PT-DESIGN

    # 합성 열 안전기준. 특정 실제 차량의 검증된 온도 규격이 아니다.
    thermal_hard_limit_c: float = 80.0

    # ── 종말 보상 — "논문 기반 최종 GU proxy 종말 보상" ──────────────────
    # 실측 GU 가 없는 프로젝트다. 이것은 실제 GU 가 아니라 논문 기반 디지털 트윈의
    #   최종 GU proxy 로 주는 종말 보상이다.
    # 왜: 스텝 대리보상 최적점 ≠ GU 최적점이 실험으로 확정됨.
    #   에피소드 종료 시 같은 에피소드의 전·후 품질(GU proxy·scratch·Ra·Rz·clearcoat)로
    #   최종/개선량을 함께 지급 — 최종 품질 판정이 dense 합(경험상 |≈400|)을 지배하게 한다.
    # 가중치 전부 PT-DESIGN.
    use_terminal_reward: bool = True
    t_gu_final: float = 300.0     # × (gu_after − 70)/10 — 문헌 기반 목표 대비 (지배 항)
    t_gu_delta: float = 200.0     # × (gu_after − gu_before)/10 — 개선량 (초기표면 편차 보정)
    t_scratch: float = 150.0      # × (scr_before − scr_after)/2.0 [μm]
    t_ra: float = 50.0            # × (ra_before − ra_after)/0.20 [μm]
    t_rz: float = 50.0            # × (rz_before − rz_after)/2.0 [μm]
    t_pass_bonus: float = 500.0   # 판정 5종 전부 통과 시 (GU≥70·Ra≤0.20·Rz≤2.0·CC≥35·scratch 감소)
    t_cc_fail: float = 1500.0     # 잔여 clearcoat 최소 < 안전기준(35 μm) — 큰 실패 페널티.
    # clearcoat 효율 항:
    # 에피소드가 소모한 최소-잔여량 [μm]당 벌점. 켜면 '안 깎기' 퇴화 위험.
    t_cc_use: float = 0.0
    t_thermal_damage: float = 1000.0  # × 최종 thermal_damage_peak
    t_overheat: float = 500.0         # × max(0, peak_C-Tg_C)/(80-Tg), PT-DESIGN
    #   GU 70 만 노리고 clearcoat 를 과도하게 깎는 정책을 막는 항 — pass_bonus(500)보다 크다.
    # 판정 임계값 (literature-derived project target — vehicle_export 와 동일)
    t_ra_pass_max_um: float = 0.20
    t_rz_pass_max_um: float = 2.0


def apply_obs_profile(env_cfg, profile: str):
    """관측 프로파일 → cfg 플래그·차원·열 페널티.

    basic/spatial 은 열 페널티도 0 (챔피언 학습 조건과 동일 보상) — 열 모델 자체(q_thermal
    판정·온도장)는 항상 켜져 있고, '정책이 열을 보고/벌 받는지'만 바꾼다.
    """
    env_cfg.use_thermal_obs = profile in ("thermal", "full")
    env_cfg.use_spatial_obs = profile in ("spatial", "full")
    env_cfg.observation_space = 11 + 3 * env_cfg.use_thermal_obs + 4 * env_cfg.use_spatial_obs
    if profile in ("basic", "spatial"):
        env_cfg.w_thermal_damage = 0.0
        env_cfg.t_thermal_damage = 0.0
        env_cfg.t_overheat = 0.0
    print(f"[obs_profile] {profile}: obs={env_cfg.observation_space}ch "
          f"thermal_obs={env_cfg.use_thermal_obs} spatial={env_cfg.use_spatial_obs}")
