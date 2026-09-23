<div align="center">

# Robotic Surface Finishing Digital Twin

### Multi-Robot Car Body Polishing with 6-Axis Cobots in NVIDIA Isaac Sim
### 6축 협동로봇 기반 차체 표면 마감(폴리싱·샌딩) 디지털 트윈

차체를 3D 스캔하고, 표면 형상에 맞는 폴리싱 경로와  
**Adaptive Force Control + Multi-Robot Rail/Gantry**를 적용하고,  
**Isaac Lab 잔차 강화학습(BC → PPO)**으로 접촉력·이송속도를 보정하는 Isaac Sim 기반 디지털 트윈 프로젝트

[![Isaac Sim](https://img.shields.io/badge/NVIDIA-Isaac%20Sim-76B900?logo=nvidia&logoColor=white)](https://developer.nvidia.com/isaac/sim)
[![ROS2](https://img.shields.io/badge/ROS2-Humble-22314E?logo=ros&logoColor=white)](https://docs.ros.org/en/humble/)
[![Python](https://img.shields.io/badge/Python-3.10-3776AB?logo=python&logoColor=white)](https://www.python.org/)
[![Isaac Lab](https://img.shields.io/badge/NVIDIA-Isaac%20Lab-76B900?logo=nvidia&logoColor=white)](https://isaac-sim.github.io/IsaacLab/)
[![PyTorch](https://img.shields.io/badge/PyTorch-2.x-EE4C2C?logo=pytorch&logoColor=white)](https://pytorch.org/)
![RL](https://img.shields.io/badge/RL-BC%20%E2%86%92%20PPO%20Residual-8A2BE2)
![Robot](https://img.shields.io/badge/Robot-6%20DOF%20Cobot-00A6A6)
[![Motion](https://img.shields.io/badge/Motion-RMPFlow-5A5A5A)](https://docs.isaacsim.omniverse.nvidia.com/)
[![UI](https://img.shields.io/badge/UI-Vite%20%2B%20Chart.js-646CFF?logo=vite&logoColor=white)](https://vitejs.dev/)

</div>

---

## Overview

기존 수작업 표면 마감(폴리싱/샌딩)과 달리, 본 프로젝트는 **협동로봇이 폴리싱 패드로 차체 표면에 직접 접촉하여 마감**하도록 구성함.

숙련공의 작업을 실제 장비에 적용하기 전에 **NVIDIA Isaac Sim 디지털 트윈에서 먼저 재현·검증**하는 것이 목표.  
자동차 차체(본네트·루프·측면)를 예시 대상으로 사용하며, 스캔 대상과 작업 종류는 교체 가능함.  
시뮬레이션의 6축 협동로봇 모델로는 **Doosan M0609**를 사용함.

차체는 윗면·측면·앞뒤 범퍼처럼 곡률과 접근 방향이 서로 달라 로봇 1대, 고정 경로 하나로는 전체를 커버하기 어려움.  
이를 해결하기 위해 **3D 스캔 → 영역별 경로 생성 → 다중 로봇(천장 C + 좌측 SL + 우측 SR) 폴리싱 → 웹 대시보드 시각화** 파이프라인으로 구성함.

---

## Demo

<div align="center">
  <img src="assets/demo.png" width="82%" alt="Isaac Sim multi-robot polishing cell">
  <br>
  <sub>Isaac Sim 작업 셀 — 천장 갠트리(C) + 좌/우 측면 리프트(SL/SR) 로봇과 폴리싱 경로</sub>
</div>

---

## Key Contributions

1. **Scan-to-Path Pipeline**  
   가상 깊이 카메라로 차체를 스캔해 Point Cloud(`.ply`)를 만들고, 지그재그(Raster) 경로 + KDTree 법선 추정으로 폴리싱 경로(`path.npy`) 자동 생성.

2. **Multi-Robot Coverage (C / SL / SR)**  
   천장 갠트리(C)는 윗면과 앞/뒤 범퍼, 텔레스코픽 리프트 위 측면 로봇(SL/SR)은 좌/우 측면 담당. 레일 정지 위치와 작업 반경은 `rail_config.json`으로 자동 계산.

3. **Adaptive Contact Force Control**  
   실측 위치 기반 **가상 스프링 + 어드미턴스 제어**로 접촉력 유지. 표면 기울기와 작업면(윗면/측면)에 따라 목표 접촉력을 3.5 ~ 8 N 범위에서 가변.

4. **Coverage-Driven Re-Polishing**  
   커버리지 맵에서 미처리(빨간 점) 영역만 다시 추출해 다음 패스 경로를 재생성하고, 가장 가까운 미완료 구간부터 재폴리싱.

5. **ROS2 Live Dashboard & Launcher**  
   진행률·로봇별 접촉력·제거량 히트맵을 ROS2 토픽으로 발행하고, rosbridge를 통해 웹 UI에서 실시간 시각화. UI 버튼으로 스캔·시뮬레이션 실행.

6. **Residual Reinforcement Learning (Isaac Lab)**  
   규칙 기반 힘 제어기는 그대로 두고, 그 위에 **[Δ접촉력 ±30 %, Δ이송속도 ±50 %] 보정분만 학습하는 잔차 정책**을 Isaac Lab DirectRLEnv에서 학습. 수제 정책 모방(BC)으로 초기화한 뒤 에피소드 종료 시점의 품질 지표를 보상으로 PPO 미세조정.

7. **Process Recipe Optimization (BO)**  
   Constrained Bayesian Optimization(GP + EI)으로 접촉력·이송·RPM·줄 간격·패스 수 레시피를 탐색하고, 학습된 정책을 고정한 outer loop로 윗면/측면 자세별 레시피를 분리.

---

## System

<div align="center">
  <img src="assets/system_architecture.png" width="100%" alt="Scan-to-path workflow and monitoring">
  <br>
  <sub>Depth Scan → Path Generation → Polishing, Coverage Feedback 재폴리싱과 ROS 2 모니터링</sub>
</div>
<br>

<div align="center">

| Phase | Description |
|---|---|
| **1. Scan** | Isaac Sim 가상 깊이 카메라(`omni.replicator`)로 차체를 다방향 촬영, 깊이 → 3D 역투영으로 Point Cloud 생성 |
| **2. Path Generation** | `path_generator.py --mode full`로 표면 법선 + Raster 경로 생성, 작업 반경(0.35 ~ 0.85 m)·표면 기울기 필터, 로봇별 영역 분할 및 레일 정지점 계산 |
| **3. Car Entry & Lift** | 반도넛형 진입 스캐너 통과 후 리프트로 차량을 작업 높이까지 상승 |
| **4. Polishing** | RMPFlow로 경로 추종, 가상 스프링 접촉력 제어 + 패드 회전(3000 RPM) |
| **5. Re-Polishing** | 커버리지 맵에서 미처리 영역을 재추출해 최대 2회 추가 패스 수행 |
| **6. Monitoring** | `ros_publisher` → ROS 2 → rosbridge(:9090) → 웹 대시보드, 완료 시 `public/data/coverage.json` 저장 |

</div>

> 접촉력은 기본적으로 시뮬레이션 패드 위치 기반 추정값이며, 커버리지는 실제 재료 제거량을 측정한 값이 아님.

---

## Multi-Robot Layout

<div align="center">

| Robot | Mount | Target Area | Implementation |
|:---:|:---:|:---:|:---|
| **C**<br>천장 | Overhead gantry | 본네트 · 루프 · 트렁크<br>+ 앞/뒤 범퍼 | Y 레일 이동 + Z 높이 추종, 범퍼 바깥까지 정지점 확장 |
| **SL**<br>좌측 | Telescopic lift<br>+ side rail | 좌측 도어 · 펜더 | Y-Z 래스터, 리프트 높이로 수직면 추종 |
| **SR**<br>우측 | Telescopic lift<br>+ side rail | 우측 도어 · 펜더 | SL과 대칭 구성 (`outward_sign` 반전) |

</div>

### C (Overhead)
- 천장 갠트리에 장착된 협동로봇이 윗면 평탄부를 담당
- 차량 길이에 맞춰 Y 정지 위치를 등간격으로 자동 계산
- 법선 기울기 ≥ 50° 인 앞/뒤 수직면까지 도달하도록 레일 여유 구간 확장

### SL / SR (Side)
- Vention 텔레스코픽 리프트(접힘 830 mm / 펼침 1700 mm) 위에 로봇 장착
- 측면은 Y로 행을 나누고 각 행에서 Z를 따라 지그재그
- 수직·곡면·중력 영향을 고려해 윗면보다 낮은 목표 접촉력 적용

---

## Control Architecture

<div align="center">
  <img src="assets/control_architecture.png" width="100%" alt="Per-robot force feedback loop">
  <br>
  <sub><code>polishing_v5_modules/agent.py</code>의 로봇별 접촉력 피드백 루프</sub>
</div>
<br>

- **runner** — 씬 구성, 차량 진입·리프트 애니메이션, 에이전트 tick · 커버리지 갱신 메인 루프
- **RailRobotAgent** — 로봇별 레일 정지점, 경로 추종, 접촉 판정 · 복구, 재폴리싱 패스 관리
- **RMPFlowController** — End-Effector를 표면 법선 방향 자세로 경로 추종
- **Virtual Spring Force** — 패드 압입량 기반 접촉력 계산 및 어드미턴스 제어
- **ros_publisher** — 상태·진행률·접촉력·히트맵 토픽 발행
- **dashboard_launcher** — Web UI [시작] 버튼 → HTTP(:8765) → `isaac_python polishing_v5.py` 실행

### Contact Force Control

```text
F_ctrl  = filtered virtual-spring force  (패드 실제 위치 기준)
F_err   = F_ctrl − F_target(tilt, mode)
accel   = (F_err − D · v) / M            # Admittance: D = 50, M = 1.0
v       = clip(v + accel · dt, ±0.02 m/s)
offset  = clip(offset + v · dt)          # 표면 법선 방향 압입량
cmd     = surface + normal · clearance   # + RMPFlow 추종 지연(lag) 보정
```

<div align="center">

| Surface | Flat (tilt 0°) | Steep (tilt ≥ 45°) |
|---|:---:|:---:|
| Top (C) | 8.0 N | 5.0 N |
| Side (SL / SR) | 6.0 N | 3.5 N |

</div>

- 목표 접촉력은 표면 기울기 0 ~ 45° 구간에서 Flat → Steep 값으로 선형 보간
- 물리 접촉 센서는 기본 OFF(`USE_PHYSICAL_CONTACT_SENSOR = False`), 힘 피드백은 패드 실제 위치 기반 가상 스프링 추정값 사용
- 패드 회전: USD `RevoluteJoint`(`pad_joint`)를 314.16 rad/s(3000 RPM)로 속도 구동
- 과압 보호: 100 N 초과 시 후퇴, 접촉 불량 구간은 일정 스텝 후 스킵

---

## Surface Scan & Coverage

<div align="center">
  <img src="assets/scan_pipeline.png" width="100%" alt="Point cloud, generated paths, and coverage map">
  <br>
  <sub>깊이 스캔 Point Cloud → 로봇별 폴리싱 경로 (C / SL / SR) → 폴리싱 커버리지 맵</sub>
</div>

---

## Web Dashboard

<div align="center">
  <img src="assets/dashboard.gif" width="82%" alt="Web dashboard">
  <br>
  <sub>Polishing before / in-progress / done monitoring.</sub>
</div>
<br>

<div align="center">

| Phase | Panels |
|---|---|
| **폴리싱 전** | 오브젝트 & 파라미터, 스캔 결과(Point Cloud), 경로 정보 |
| **폴리싱 중** | 진행률, 카메라, 로봇별 접촉력(Force) 차트, 제거량 히트맵 |
| **폴리싱 완료** | 3D 커버리지(360° 회전), 전/후 비교, 결과 지표, 분석 레포트 |

| Topic | Type | Description |
|---|---|---|
| `/polishing/state` | `std_msgs/String` | `POLISH` / `DONE` |
| `/polishing/progress` | `std_msgs/Float64` | 전체 진행률 (%) |
| `/polishing/progress/{C,SL,SR}` | `std_msgs/Float64` | 로봇별 진행률 (%) |
| `/polishing/force/{C,SL,SR}` | `std_msgs/Float64` | 로봇별 접촉력 (N) |
| `/polishing/elapsed_time` | `std_msgs/Float64` | 경과 시간 (s) |
| `/polishing/heatmap` | `std_msgs/Float64MultiArray` | 탑뷰 제거 커버리지 |

</div>

---

## Residual Reinforcement Learning (Isaac Lab)

<div align="center">

| Step | Description |
|---|---|
| **1. Contact Model Port** | v5의 가상 스프링 + 어드미턴스 접촉 모델을 `(num_envs,)` 텐서 연산으로 이식해 병렬 env 학습 (`learning/rl/env/contact.py`) |
| **2. Surface Quality Model** | 논문 근거 표면 상태(Ra · 스크래치 · 클리어코트) + Preston형 제거 모델 + 20° 광택(GU) proxy (`learning/digital_twin/`) |
| **3. BC Bootstrap** | 수제 dwell 정책을 모방해 actor 초기화 (`bootstrap_bc.py`) |
| **4. Terminal-Reward PPO** | 에피소드 종료 시 전·후 품질 개선량을 보상으로 rsl_rl PPO 미세조정 (`train_ppo.py`) |
| **5. Recipe BO** | 정책 고정 후 자세별 공정 레시피 탐색 (`bo_outer_loop.py`) |
| **6. v5 Integration** | 20 Hz 잔차 정책 브리지를 v5 `agent.py` 훅으로 연결, 차체 스캔 점군을 12 cm 셀 483개로 나눠 셀별 판정 (`rl_bridge.py`) |

</div>

```text
a = π(obs)                              # 정책 출력 = 잔차 2축, tanh ∈ [-1, 1]
F_target = F_recipe · (1 + 0.30 · a0)    # 목표 접촉력 보정
v_feed   = v_recipe · (1 + 0.50 · a1)    # 이송속도 보정
F_cmd    = clip(F_target, 0, F_hard)     # 기존 안전 한계는 정책과 무관하게 유지
```

### Reward Design

<div align="center">
  <img src="assets/rl_training_curves.png" width="100%" alt="PPO training curves">
  <br>
  <sub>PPO 학습 곡선 — 스텝 보상(좌)은 오르지만 광택 GU(중)는 떨어지는 보상 정렬 문제를 진단한 기록</sub>
</div>
<br>

<div align="center">

| Problem | Fix |
|---|---|
| 제거량 합계 보상 → 정상 셀 감점이 지배해 "덜 문지르기" 학습 | 셀당 **평균** 보상으로 변경 |
| 정적 결함 마스크 → 이미 지운 자리를 반복 문질러 보상 획득 | 지급을 **잔여 결함량**으로 게이팅 |
| 관측 범위 ≈ 패치 크기 → 공간 변별 신호 소멸 | 관측을 패드 **코어 영역 잔여 결함**으로 축소 |
| 스텝 대리 보상 최적점 ≠ 광택 최적점 | BC 부트스트랩 + **종말(에피소드 끝) 품질 보상** PPO |

</div>

### Results

<div align="center">

| Metric | Result |
|---|---|
| 신차 시나리오 150셀 (5종 판정: GU ≥ 70 · Ra ≤ 0.20 µm · Rz ≤ 2.0 µm · 클리어코트 ≥ 35 µm · 스크래치 감소) | **147 / 150** |
| 레시피 BO 자세별 분리 (손상차 시나리오 150셀) | 91 → **97 / 150** |
| 고정 레시피 대비 잔존 스크래치 (BC 정책) | **−30 %** |
| 이송 ×1.5 레시피 — 합격 수 유지 시 셀당 공정시간 | 309 s → **177 s (−43 %)** |
| PhysX 실접촉 M0609 env, 차체 전체 483셀 순회 | **321 / 483** |
| 곡면(원통 R = 0.5 m) 작업면 GU — 평면 학습 정책 → 곡면 학습 정책 | 58.4 → **65.9** |

</div>

- 합격 셀은 전부 OEM 도장 보증 제거 한도(Ford 7.5 µm) 이내. 미합격 셀은 `rework_candidate` / `spot_repaint_review`로 처분 분류
- 차체 순회의 주요 실패 원인은 강곡률 셀의 접촉 과부하 — 감압(×0.7) + 소형 패드(r = 0.035 m) 재실행으로 22셀 추가 합격

> 광택(GU)·거칠기·스크래치 지표는 논문 근거 디지털 트윈 모델의 출력(합성 수치)이며, 실측 광택계로 보정한 값이 아님.

---

## Engineering Challenges

<div align="center">

| Problem | Solution |
|---|---|
| 패드·차체 강체 충돌 시 접촉 순간 튕김(Slam) 발생 | 팔/샌더 충돌 비활성화, 실측 위치 기반 **가상 스프링**으로 접촉력 계산 |
| RMPFlow가 명령보다 덜 내려오는 정상상태 지연(~2 cm) | **Lag Feedforward**로 지연량을 추정해 명령을 더 깊게 보정 |
| 차체를 RMPFlow 장애물로 넣으면 패드까지 밀어냄 | 차체 장애물 비활성화 + 링크 단위 **Arm Guard**로 관통 감지 및 복구 |
| 로봇 특이점·도달 한계로 경로 추종 실패 | 작업 반경 0.35 ~ 0.85 m 필터 + 레일 정지점 분할 |
| 측면에서 패드가 표면 위 5 ~ 7 mm 부유 | 측면 가상 접촉 거리 0.015 → 0.010 m로 조정해 평형점을 표면에 맞춤 |
| 유리·급경사면까지 경로가 생성됨 | 표면 기울기 15° 초과 영역 제외 (윗면 평탄부만) |
| 1회 패스 후 미처리 영역 잔존 | 커버리지 맵 기반 **재폴리싱 패스** 자동 생성 |

</div>

---

## Environment

<div align="center">

| Category | Specification |
|---|---|
| OS | Ubuntu 22.04.5 LTS |
| Simulator | NVIDIA Isaac Sim 6.0.1 (standalone `python.sh`) |
| RL | Isaac Lab + rsl_rl (PPO), PyTorch (CUDA) |
| Middleware | ROS2 Humble + rosbridge_suite |
| Robot | 6축 협동로봇 (시뮬레이션 모델: Doosan M0609, URDF / USD) |
| End-Effector | Sander + Polishing Pad (`m0609_with_polisher.usd`) |
| Lift / Rail | Vention 518823 Telescopic Lift, Linear Rail |
| Language | Python 3.10 |
| Motion | RMPFlow (`rmpflow/`) |
| Contact Control | Virtual Spring + Admittance Control |
| Dashboard | Node.js 20 + Vite 8 + Chart.js 4 |
| GPU | NVIDIA GeForce RTX 5080 Laptop (RTX 계열 권장) |

</div>

> **Isaac Sim 실행기 alias** (개발 환경 기준)
> ```bash
> alias isaac_python='~/isaacsim/python.sh'
> ```
> Isaac Sim 스크립트(`scan.py`, `polishing_v*.py`)는 반드시 `isaac_python`으로 실행 (시스템 `python3` 불가).

---

## Installation

```bash
git clone https://github.com/seb000423/robotic-surface-finishing-digital-twin.git
cd robotic-surface-finishing-digital-twin

# Python 의존성 (numpy, scipy, matplotlib, Pillow, gmsh)
pip install -r requirements.txt

# ROS2 ↔ Web UI 브릿지
sudo apt install ros-humble-rosbridge-suite

# Web Dashboard
cd web_dashboard && npm install
```

- `isaacsim`, `omni.*`, `carb`, `pxr` — Isaac Sim 내장 (pip 설치 X)
- `rclpy`, `std_msgs`, `sensor_msgs` — ROS2 Humble 제공 (pip 설치 X)

---

## Usage

### 1. Web Dashboard로 실행 (권장)

```bash
# Isaac Sim 경로가 다르면 지정
export ISAAC_PYTHON=/path/to/isaacsim/python.sh

./run_dashboard.sh
```

Open:

```text
http://localhost:5173
```

- **[시작]** → 런처(:8765)가 `isaac_python polishing_v5.py` 실행
- **[스캔]** → `scan.py` → `path_generator.py` 순차 실행
- `run_dashboard.sh`가 rosbridge(:9090)도 함께 실행. rosbridge가 없으면 UI는 **데모 모드**로 동작

### 2. 전체 파이프라인 한 번에

```bash
python3 scripts/main_pipeline.py car      # 또는 cube
```

### 3. 단계별 실행

```bash
# ① 깊이 스캔
isaac_python scripts/scan.py --obj_name car

# ② 3D 경로 생성
python3 scripts/path_generator.py --obj_name car

# ③ 폴리싱 시뮬레이션
isaac_python scripts/polishing_v1.py --obj_name car      # 단일 로봇
isaac_python scripts/polishing_v5.py --obj_name car      # 다중 로봇 (C + SL + SR)
isaac_python scripts/polishing_v5.py --obj_name car --headless
```

### 4. Web UI만 실행 (시뮬레이션 없이)

```bash
cd web_dashboard
npm run dev                          # http://localhost:5173
npm run build && npm run preview     # 빌드 후 미리보기
```

### 5. 강화학습 (Isaac Lab)

```bash
PY=~/isaacsim/python.sh     # Isaac Lab이 설치된 Isaac Sim python

# 디지털 트윈 단위시험 · 레시피 BO (Isaac Sim 불필요)
$PY -m learning.digital_twin.tests.test_unit
$PY -m learning.digital_twin.bo_runner

# BC 부트스트랩 → PPO 미세조정 → 짝지은 평가
$PY learning/rl/bootstrap_bc.py --headless
$PY learning/rl/train_ppo.py --headless --num_envs 16 --max_iterations 1500 \
    --resume learning/rl/champion/model_bc_14ch.pt
$PY learning/rl/eval_conditions.py --headless \
    --conditions "baseline=,policy=learning/rl/champion/model_terminal_ppo_14ch_it800.pt"

# PhysX 실접촉 로봇 env — 차체 전체 셀 순회
bash learning/rl/run_car_cells.sh 0 482 8

# v5 다중 로봇 시뮬레이션 + 잔차 정책 (GUI)
bash scripts/run_v5_rl_view.sh
```

자세한 구성은 [`learning/README.md`](learning/README.md) 참고.

### ROS2 Topic Example

```bash
ros2 topic echo /polishing/state
ros2 topic echo /polishing/progress
ros2 topic echo /polishing/force/C
```

> 수동 실행 시 터미널 2개: `python3 scripts/dashboard_launcher.py` / `cd web_dashboard && npm run dev`

---

## Repository Structure

```text
robotic-surface-finishing-digital-twin/
├── scripts/
│   ├── scan.py                  # ① 깊이 스캔 → Point Cloud
│   ├── path_generator.py        # ② 경로 생성 + rail_config.json
│   ├── polishing_v1.py          # ③ 단일 로봇 폴리싱
│   ├── polishing_v4.py          #    4대 로봇 모드
│   ├── polishing_v5.py          #    다중 로봇 진입점 (UI 시작 버튼)
│   ├── polishing_v5_modules/
│   │   ├── bootstrap.py         #   CLI · SimulationApp 초기화
│   │   ├── runner.py            #   씬 구성 · 메인 루프
│   │   ├── agent.py             #   로봇별 제어 (RailRobotAgent)
│   │   ├── common.py            #   상수 · 힘 제어 파라미터
│   │   ├── ros_publisher.py     #   ROS2 토픽 발행
│   │   ├── pad_contact.py       #   PhysX 접촉 리포트 기반 패드 힘
│   │   ├── rl_bridge.py         #   셀 격자 · 잔차 정책 · 셀별 판정
│   │   └── visualization.py     #   커버리지 맵 · 경로 시각화
│   ├── run_v5_rl_view.sh        # v5 + 잔차 정책 실행
│   ├── main_pipeline.py         # 스캔 → 경로 → 폴리싱 일괄 실행
│   └── dashboard_launcher.py    # UI 버튼 → Isaac Sim 실행 (:8765)
│
├── learning/                    # 학습 스택
│   ├── bc/                      #   v5 로그 → 모방학습 (MLP)
│   ├── digital_twin/            #   표면 상태 · 제거 · GU proxy 모델, 레시피 BO
│   ├── rl/                      #   Isaac Lab env · PPO · 평가 · 체크포인트
│   ├── vehicle_export/          #   차량 150셀 판정 CSV 생성
│   └── ui_bridge/               #   시뮬레이션 기록 · 모니터 피드
│
├── rmpflow/                     # RMPFlow 컨트롤러 · yaml · URDF
├── scan_obj/                    # 스캔 대상 USD (car, car_small, cube)
├── scan_result/                 # 스캔 · 경로 결과 (PLY, npy, rail_config.json)
├── usd/env/                     # 로봇 · 리프트 · 레일 · 공간 USD 에셋
├── web_dashboard/               # Vite + Chart.js 대시보드
│   ├── src/                     #   main.js, ros-bridge.js, force-chart.js, coverage3d.js
│   └── public/                  #   이미지 · 데이터
│
├── assets/                      # README 이미지
├── requirements.txt
├── run_dashboard.sh             # rosbridge + 런처 + UI 통합 실행
└── README.md
```

---

<div align="center">

**NVIDIA Isaac Sim × ROS2 × 6-Axis Cobot**

Digital twin for skilled surface finishing with collaborative robots.

</div>
