# learning/ — 잔차 강화학습 스택

v5 폴리싱 시뮬레이션(`scripts/`)과 분리된 학습 전용 디렉터리.  
규칙 기반 힘 제어기 위에 접촉력·이송속도 보정분만 학습하는 **잔차(Residual) 정책**을 만들고, 그 정책을 다시 v5 시뮬레이션에 연결함.

```text
v5 힘 로그 ──▶ bc/ (모방학습)
                        │
digital_twin/ (표면·제거·GU 모델, BO) ──▶ rl/ (Isaac Lab env · 모방학습 · PPO)
                                              │
                                              ▼
                          scripts/polishing_v5_modules/rl_bridge.py (20 Hz 잔차 보정)
```

---

## 구성

| 디렉터리 | 내용 |
|---|---|
| `bc/` | v5 힘 로그 → (state, action) 데이터셋 추출, MLP 모방학습, 단독 추론 모듈(`bc_policy.py`) |
| `digital_twin/` | 표면 상태 13종 맵, Preston형 제거 모델, Ra/Rz/잔존 스크래치 계산, 20° GU proxy, Constrained BO(`bo_runner.py`) |
| `rl/env/` | `contact.py` 가상 스프링·어드미턴스 벡터화 이식, `polish_env.py` 해석식 접촉 DirectRLEnv, `robot_polish_env.py` PhysX 실접촉 M0609 env |
| `rl/` | 모방학습 초기화, PPO 학습(`train_ppo*.py`), 짝지은 평가(`eval_conditions*.py`), 학습된 정책 기준 공정조건 재탐색, 재폴리싱 평가, 차체 셀 순회(`car_cells_robot.py`) |
| `rl/champion/` | 학습된 정책 체크포인트 (해석식 env: `model_terminal_ppo_14ch_it800.pt`) |
| `rl/robot/champion/` | PhysX 로봇 env 체크포인트 (곡면 학습: `model_ppo_curved.pt`) |
| `vehicle_export/` | 차량 150셀 입력 → 셀별 판정 CSV (판정 5종 · 보증 한도 · 처분) |
| `ui_bridge/` | 시뮬레이션 기록(SQLite)과 모니터 피드(JSON) 작성기 |

---

## 환경 설계

| 항목 | 값 |
|---|---|
| 물리 / 제어·품질 주기 | 60 Hz / 20 Hz (decimation 3) |
| 행동 | `[Δforce_ratio, Δfeed_ratio]` ∈ [-1, 1] → 목표 힘 ±30 %, 이송 ±50 % |
| 관측 | 측정 힘 · 힘 오차 · 힘 변화 · 이송 · 진행률 · 패드 footprint 잔여 스크래치·누적 제거·클리어코트 여유 · 직전 행동 (11ch) + 국소 온도·열손상 (3ch) |
| 보상 | 매 스텝: 접촉력 안전 · 행동 급변 억제 · 클리어코트 과다 제거 방지 · 결함 제거 / 작업 종료 시: 전·후 GU · 스크래치 · Ra · Rz 개선량, 판정 통과 보너스, 클리어코트 한계 위반 페널티 |
| 안전 | hard limit 14 N (정책 출력과 무관하게 최종 단에서 적용), 클리어코트 잔여 ≥ 35 µm |
| 알고리즘 | rsl_rl PPO, 모방학습(BC)으로 초기화 |

판정 기준 5종: **GU ≥ 70 · Ra ≤ 0.20 µm · Rz ≤ 2.0 µm · 클리어코트 ≥ 35 µm · 스크래치 감소**

---

## 실행

Isaac Lab이 설치된 Isaac Sim python으로 실행함.

```bash
PY=~/isaacsim/python.sh

# 디지털 트윈 단위시험 · k 캘리브레이션 · BO
$PY -m learning.digital_twin.tests.test_unit
$PY -m learning.digital_twin.tests.test_gloss
$PY -m learning.digital_twin.bo_runner

# 접촉 모델 이식 검증 / action=0 baseline
$PY learning/rl/tests/test_contact_replay.py
$PY learning/rl/tests/test_lab_env_baseline.py --headless

# 모방학습 → PPO
$PY learning/rl/bootstrap_bc.py --headless
$PY learning/rl/train_ppo.py --headless --num_envs 16 --resume learning/rl/champion/model_bc_14ch.pt

# 같은 표면 seed에서 baseline vs 정책 비교
$PY learning/rl/eval_conditions.py --headless \
    --conditions "baseline=,policy=learning/rl/champion/model_terminal_ppo_14ch_it800.pt"

# 차량 150셀 판정 CSV
$PY learning/vehicle_export/export_vehicle_results.py

# PhysX 로봇 env — 곡면 학습 / 차체 전체 셀 순회
$PY learning/rl/train_ppo_robot.py --headless --surface_kind cylinder
bash learning/rl/run_car_cells.sh 0 482 8
```

v5 시뮬레이션에 정책을 붙여 실행:

```bash
bash scripts/run_v5_rl_view.sh              # GUI
bash scripts/run_v5_rl_view.sh --headless
```

`POLISH_RL=1`일 때만 잔차 정책이 동작하며, 기본 v5 동작은 그대로 유지됨.

---

## 결과 요약

| 항목 | 결과 |
|---|---|
| 신차 시나리오 150셀 | 147 / 150 |
| 손상차 시나리오 150셀 (윗면·측면 공정조건 분리) | 91 → 97 / 150 |
| 잔존 스크래치 (고정 공정조건 대비) | −30 % |
| 셀당 공정시간 (이송 ×1.5, 합격 수 유지) | 309 s → 177 s |
| PhysX 로봇 env 차체 전체 483셀 순회 | 321 / 483 |

결과 CSV: `rl/results/`, `rl/robot/results/`, `vehicle_export/`

> 품질 지표는 논문 근거 디지털 트윈 모델의 출력(합성 수치)이며, 실측 광택계로 보정한 값이 아님.
