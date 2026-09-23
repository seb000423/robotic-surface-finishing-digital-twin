# web_console/ — 관제·설정 웹 콘솔

차체 폴리싱 공정을 브라우저에서 설정하고, 진행 상황을 감시하고, 결과를 저장·비교하는 웹 콘솔.  
차종 · 설비 구성 · 공정 레시피를 고르고 3D 로봇 셀에서 미리 본 뒤, 시뮬레이션 기록을 재생하며 로봇별 접촉력과 셀 판정을 확인함.

---

## 화면

| 파일 | 화면 | 내용 |
|---|---|---|
| `index.html` | 랜딩 | 프로젝트 소개, 로그인 |
| `sub.html?p=<id>` | 상세 문서 | 공정별(폴리싱 · 디버링 · 샌딩 · Pick&Place) · 기술별(Physical AI · 강화학습 · 숙련공 DB · 안전 검증) 설명 |
| `console.html` | ① 사전 설정 | 차종 · 리프트 높이 · 로봇 대수 · 폴리셔 · 레시피 설정 + 3D 로봇 셀 미리보기 |
| `monitor.html` | ② 공정 감시 | 로봇별 접촉력 · 진행률 · 제거량 히트맵 · 이벤트 기록, 시뮬레이션 기록 재생 |
| `save.html` | ③ 결과 저장 | 공정 결과와 설정 스냅샷 저장 |
| `library.html` | ④ 라이브러리 | 저장된 결과 · 정답 데이터 조회 · 비교 |
| `admin.html` | 계정 관리 | 관리자 전용 |

---

## 실행

Node.js 22.5 이상 (`node:sqlite` 내장 모듈 사용).

```bash
cd web_console
npm run seed:data                 # 최초 1회 — 정답 데이터셋을 DB에 적재
PT_ADMIN_PW='원하는-비밀번호' node backend/server.js
# → http://127.0.0.1:8000
```

- `PT_ADMIN_PW`를 지정하지 않으면 최초 기동 시 관리자 비밀번호를 무작위로 만들고 서버 로그에 한 번만 출력함
- 로그인·계정 API가 필요하므로 `python -m http.server` 같은 정적 서버로는 동작하지 않음
- `seed:data`를 건너뛰면 ①·④ 화면이 데이터셋을 받지 못함

### 시뮬레이션 연동

Isaac Sim 쪽 `learning/ui_bridge/`가 시뮬레이션 기록(SQLite)과 모니터 피드(JSON)를 만들고, 콘솔이 이를 읽어 ② 공정 감시 화면에 표시함.

```bash
bash scripts/run_v5_rl_view.sh      # 실행 시 learning/ui_bridge/out/ 에 기록 생성
```

---

## 구조

```text
web_console/
├── api/index.js          # /api/* 서버리스 함수 (Vercel)
├── middleware.js         # 페이지 접근 제어 (Vercel Edge)
├── backend/              # 로컬 서버 · API 라우트 · 인증 · DB(SQLite / Turso)
├── frontend/             # 정적 화면 (배포 루트)
│   ├── assets/js/        #   화면 스크립트, 3D 뷰포트(console-viewport.js), 다국어
│   ├── assets/models/    #   로봇 · 리프트 · 레일 · 차량 glTF (meshopt 압축)
│   ├── assets/vendor/    #   three.js r169, React 18
│   └── 데이터셋/          #   세그먼트별 KPI · 궤적 데이터
├── scripts/              # 관리자 · 데이터셋 시드, 모델 변환
└── vercel.json
```

## 배포 (Vercel)

정적 화면은 CDN, `/api/*`는 서버리스 함수 하나, DB는 Turso(libSQL).

| 환경 변수 | 용도 |
|---|---|
| `TURSO_DATABASE_URL`, `TURSO_AUTH_TOKEN` | 원격 DB |
| `PT_SECRET` | 세션 서명 키 |
| `PT_ADMIN_ID`, `PT_ADMIN_PW` | 관리자 계정 시드 (`npm run seed`) |
