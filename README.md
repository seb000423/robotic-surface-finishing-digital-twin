# Robotic Surface Finishing Digital Twin

숙련공의 표면 마감(폴리싱/샌딩) 작업을 **Doosan M0609 6축 로봇팔**로 옮기기 위한
**NVIDIA Isaac Sim** 기반 디지털 트윈 프로젝트입니다.
자동차 본네트를 예시 작업 대상으로 사용하며, 스캔 대상과 작업 종류는 교체할 수 있습니다.
3D 스캔 → 경로 생성 → 폴리싱 시뮬레이션 → 웹 대시보드 시각화
파이프라인으로 구성됩니다.

---

## 1. 시스템 설계 & 플로우 차트

### 전체 파이프라인

```
┌──────────────┐     ┌────────────────────┐     ┌──────────────────────┐
│  scan.py     │     │ path_generator.py  │     │  polishing_v*.py     │
│ (깊이 스캔)  │ ──▶ │  (3D 경로 생성)    │ ──▶ │ (폴리싱 시뮬레이션)  │
└──────────────┘     └────────────────────┘     └──────────────────────┘
       │                      │                            │
       ▼                      ▼                            ▼
 scan_result/{obj}/     scan_result/{obj}/          Isaac Sim 물리 시뮬
 points/*.ply           path.npy                    (RMPFlow + 접촉/가상힘)
                                                            │
                                                            ▼
                                                  ROS2 publish ─▶ web_dashboard
                                                  (실시간 힘/커버리지 시각화)
```

### 단계별 설계

| 단계 | 스크립트 | 실행기 | 입력 → 출력 | 핵심 기술 |
|------|----------|--------|-------------|-----------|
| ① 스캔 | `scripts/scan.py` | `isaac_python` | USD 오브젝트 → `points/*.ply` | 가상 깊이 카메라, `omni.replicator`, 깊이→3D 역투영 |
| ② 경로 생성 | `scripts/path_generator.py` | `python3` | `*.ply` → `path.npy` | 지그재그(Raster) 경로, 작업반경 필터, KDTree 법선 추정 |
| ③ 폴리싱 | `scripts/polishing_v1.py` (단일)<br>`scripts/polishing_v5.py` (다중) | `isaac_python` | `path.npy` → 시뮬레이션 | RMPFlow 모션, 가상 스프링 접촉력 제어, 패드 회전 |
| ④ 시각화 | `web_dashboard/` | `npm`(Vite) | ROS2/CSV → 웹 | 실시간 힘 차트, 3D 커버리지 맵 |

### 버전 구분

- **v1 (`polishing_v1.py`)** — 단일 로봇 폴리싱.
- **v5 (`polishing_v5.py` + `polishing_v5_modules/`)** — 레일 + 측면(SL/SR) + 천장(C) **다중 로봇** 버전.
  - `common.py` (상수/설정) · `agent.py` (로봇별 제어) · `runner.py` (씬 구성/루프)
    · `bootstrap.py` (진입점) · `ros_publisher.py` (ROS2 퍼블리시) · `visualization.py`.

### 제어 개요

- **모션**: `RMPFlowController`(`rmpflow/`)로 End-Effector를 경로(`path.npy`)에 추종.
- **접촉력**: 실측 위치 기반 **가상 스프링** 모델로 법선 방향 누름 힘 제어(목표 ≈ 1.5N).
  강체 충돌 슬램을 피하기 위해 패드/로봇 물리 충돌은 끄고 가상힘으로 처리.
- **패드 회전**: USD에 정의된 `RevoluteJoint`(`pad_joint`)를 속도 구동.

---

## 2. 운영체제 / 실행 환경

| 항목 | 사양 |
|------|------|
| OS | Ubuntu 22.04.5 LTS |
| Kernel | 6.8.0-124-generic (x86_64) |
| 시뮬레이터 | NVIDIA Isaac Sim (standalone python.sh) |
| ROS | ROS2 Humble (`/opt/ros/humble`) |
| Python (시스템) | 3.10 |
| GPU | NVIDIA GeForce RTX 5080 Laptop (Driver 580.x) |
| 웹 대시보드 | Node.js 20 + Vite 8 + Chart.js 4 |

> **Isaac Sim 실행기 alias** (개발 환경 기준)
> ```bash
> alias isaac_python='~/isaacsim/python.sh'
> alias isaac='isaac-sim.sh'
> ```
> Isaac Sim 스크립트는 반드시 `isaac_python`으로 실행해야 합니다 (시스템 `python3` 불가).

---

## 3. 사용 장비 목록

### 시뮬레이션 대상 장비 (가상)

| 장비 | 설명 |
|------|------|
| Doosan **M0609** 6축 협동로봇 | 폴리싱 로봇팔 (URDF/USD 포함) |
| OnRobot 샌더 + 폴리싱 패드 | End-Effector 공구 (`m0609_with_polisher.usd`) |
| Vention **518823** 텔레스코픽 리프트 | 측면/천장 로봇 받침대 (접힘 830mm / 펼침 1700mm) |
| 리니어 레일 시스템 | 다중 로봇(v5) 이송용 (`Rail.usd`) |
| 가상 깊이(Depth) 카메라 | 스캔용 (Isaac Sim Replicator) |
| 스캔 대상 차량 모델 | `car.usd`, `car_small.usd`, `cube.usd`, BMW Z4 등 |

### 개발/실행 PC

| 항목 | 사양 |
|------|------|
| GPU | NVIDIA RTX 5080 Laptop (RTX 계열 권장, Isaac Sim 요구) |
| OS | Ubuntu 22.04 |

---

## 4. 의존성

- **Python 패키지**: [`requirements.txt`](./requirements.txt) 참고.
  ```bash
  pip install -r requirements.txt
  ```
  (`numpy`, `scipy`, `matplotlib`, `Pillow`, `gmsh`)
- **Isaac Sim 제공** (pip 설치 X): `isaacsim`, `omni.*`, `carb`, `pxr`.
- **ROS2 Humble 제공** (pip 설치 X): `rclpy`, `std_msgs`, `sensor_msgs`.
- **웹 대시보드**: `web_dashboard/package.json` (`vite`, `chart.js`).
  ```bash
  cd web_dashboard && npm install
  ```

---

## 5. 사용 방법 (실행 순서)

### A. 웹 대시보드(UI)로 실행

UI를 띄우고 **시작 버튼**을 누르면 Isaac Sim 폴리싱 시뮬레이션이 실행됩니다.

```bash
# 1) (다른 PC라면) Isaac Sim 설치 경로 지정 — 경로가 동일하면 생략 가능
export ISAAC_PYTHON=/내/경로/isaacsim/python.sh

# 2) 한 번에 실행 (런처 서버 + 웹 UI)
./run_dashboard.sh
```

- 브라우저에서 **http://localhost:5173** 접속 → UI 표시.
- UI **[시작]** 버튼 → 런처(포트 8765)가 `isaac_python polishing_v5.py` 실행 → Isaac Sim 구동.
- UI **[스캔]** 버튼 → `scan.py` → `path_generator.py` 순차 실행.

> **동작 구조**
> ```
> [브라우저 UI :5173]
>     │  ① 시작 버튼 (HTTP POST /start)
>     ▼
> [dashboard_launcher.py :8765] ──▶ isaac_python polishing_v5.py ──┐
>                                                                  │ ② ROS2 토픽 발행
>     ┌────────────────────────────────────────────────────────────┘   (/polishing/*)
>     ▼
> [rosbridge_server :9090] ──(WebSocket)──▶ [브라우저 UI 실시간 그래프/힘/진행률]
> ```
> `run_dashboard.sh`가 **rosbridge_server(:9090)** 도 자동 실행합니다.
> rosbridge가 없으면 UI는 ROS2 데이터를 못 받고 **데모 모드**로 동작합니다
> (`sudo apt install ros-humble-rosbridge-suite` 로 설치).
> `run_dashboard.sh` 없이 수동 실행 시(터미널 2개):
> ```bash
> python3 scripts/dashboard_launcher.py          # 터미널 1 (런처)
> cd web_dashboard && npm install && npm run dev  # 터미널 2 (UI)
> ```

> **다른 PC에서 실행할 때**
> 1. Node.js 18+ , Python 3.10+ , NVIDIA Isaac Sim 설치.
> 2. ROS2 Humble + **rosbridge** 설치 (UI 실시간 데이터에 필수):
>    `sudo apt install ros-humble-rosbridge-suite`
> 3. `pip install -r requirements.txt`
> 4. `ISAAC_PYTHON` 환경변수를 본인 PC의 `python.sh` 경로로 지정.
> 5. `./run_dashboard.sh` 실행 → 브라우저에서 시작 버튼 클릭.

### B. 전체 파이프라인 한 번에

```bash
# 스캔 → 경로 생성 → 폴리싱 을 순서대로 실행
python3 scripts/main_pipeline.py car      # 또는 cube
```
> `main_pipeline.py`가 내부적으로 `scan.py`/`polishing`은 `isaac_python`,
> `path_generator.py`는 `python3`으로 호출합니다.

### C. 단계별 실행

```bash
# ① 깊이 스캔 (Isaac Sim)
isaac_python scripts/scan.py --obj_name car

# ② 3D 경로 생성 (시스템 python)
python3 scripts/path_generator.py --obj_name car

# ③ 폴리싱 시뮬레이션
isaac_python scripts/polishing_v1.py --obj_name car      # 단일 로봇
isaac_python scripts/polishing_v5.py --obj_name car      # 다중 로봇(레일+측면+천장)
```

### D. 웹 UI만 따로 실행 (시뮬 없이 화면만 확인)

```bash
cd web_dashboard
npm install        # 최초 1회
npm run dev        # 개발 서버 (http://localhost:5173)
# 또는
npm run build && npm run preview   # 빌드 후 미리보기
```

---

## 6. 디렉토리 구조

```
robotic-surface-finishing-digital-twin/
├── README.md               # (본 문서)
├── requirements.txt        # Python 의존성
├── run_dashboard.sh        # UI + Isaac Sim 런처 통합 실행 스크립트
├── scripts/                # 소스 코드 (파이프라인 + 런처)
│   ├── dashboard_launcher.py  #   UI 시작 버튼 → Isaac Sim 실행 (HTTP 서버 :8765)
│   ├── main_pipeline.py    #   파이프라인 진입점 (tkinter)
│   ├── scan.py             #   ① 깊이 스캔
│   ├── path_generator.py   #   ② 경로 생성
│   ├── polishing_v1.py     #   ③ 단일 로봇 폴리싱
│   ├── polishing_v4.py     #   ③ 폴리싱 (4대 모드)
│   ├── polishing_v5.py     #   ③ 다중 로봇 폴리싱 진입점 (UI 시작 버튼이 실행)
│   └── polishing_v5_modules/  #   v5 모듈 (common/agent/runner/bootstrap/...)
├── rmpflow/                # RMPFlow 컨트롤러 + 설정(yaml) + URDF
├── scan_obj/               # 스캔 대상 USD 오브젝트
├── scan_result/            # 스캔/경로 결과 (PLY, path.npy)
├── usd/env/                # 씬·로봇·리프트·레일 USD 에셋
└── web_dashboard/          # 웹 UI (Vite + Chart.js)
```
