/**
 * Main Application — 폴리싱 모니터링 대시보드
 */
import './style.css';
import { RosBridge, ROS_TOPICS } from './ros-bridge.js';
import { ForceChart } from './force-chart.js';
import { Coverage3D } from './coverage3d.js';

let coverage3d = null;          // 3D 커버리지 뷰어(폴리싱 완료 탭)
let _cov3dLoaded = false;       // 이번 세션에서 로드 여부

// ===== State =====
let currentPhase = 'pre'; // 'pre' | 'during' | 'post'
// ROS /polishing/state 로 단계 자동 전환할지 여부.
// 사용자가 직접 탭을 누르면 false 가 되어, 좀비/잔여 퍼블리셔의 STATE 가 들어와도
// 탭이 멋대로 끌려가지 않음. '시작' 버튼을 누르면 다시 true(이번 런을 따라감).
let autoFollowState = true;
let forceCharts = {};     // { C, SL, SR } — 로봇팔 3개 접촉력 차트
let ros = null;

// 로봇팔 3개 정의 (천장 C / 좌측면 SL / 우측면 SR)
const FORCE_BOTS = [
  { key: 'C',  topicKey: 'FORCE_C',  chartId: 'forceChartC',  valId: 'forceLiveValC'  },
  { key: 'SL', topicKey: 'FORCE_SL', chartId: 'forceChartSL', valId: 'forceLiveValSL' },
  { key: 'SR', topicKey: 'FORCE_SR', chartId: 'forceChartSR', valId: 'forceLiveValSR' },
];
function forEachChart(fn) { Object.values(forceCharts).forEach(c => c && fn(c)); }

// 시작 버튼 → Isaac Sim 실행 런처 (dashboard_launcher.py)
const LAUNCHER_URL = 'http://localhost:8765';
let selectedObject = 'bmw_z4';   // 오브젝트 선택(bmw_z4/benz_coupe/ferrari_sf90/cube) — 스캔/폴리싱 공통

// 차종별 원본 사진 (public/images/*.png) — 드롭다운 선택 시 미리보기
const OBJECT_PREVIEWS = {
  bmw_z4:       '/images/bmw_z4.png',
  benz_coupe:   '/images/benz_coupe.png',
  ferrari_sf90: '/images/ferrari_sf90.png',
};

// 4코너 카메라 (드롭다운 선택). 한 번에 선택된 1개 토픽만 구독.
const CAMERAS = [
  { key: 'left_front',  topicKey: 'CAM_LEFT_FRONT'  },
  { key: 'left_back',   topicKey: 'CAM_LEFT_BACK'   },
  { key: 'right_front', topicKey: 'CAM_RIGHT_FRONT' },
  { key: 'right_back',  topicKey: 'CAM_RIGHT_BACK'  },
  { key: 'ceiling',     topicKey: 'CAM_CEILING'     },
];
let currentCameraKey = 'left_front';
let currentCameraTopic = null;

// 로봇별 진행률 막대 (좌측 SL / 우측 SR / 천장 C)
const PROGRESS_BOTS = [
  { topicKey: 'PROGRESS_SL', barId: 'progBarSL', valId: 'progValSL' },
  { topicKey: 'PROGRESS_SR', barId: 'progBarSR', valId: 'progValSR' },
  { topicKey: 'PROGRESS_C',  barId: 'progBarC',  valId: 'progValC'  },
];
function updateRobotProgress(barId, valId, pct) {
  const v = Math.min(100, Math.max(0, pct));
  const bar = document.getElementById(barId);
  const val = document.getElementById(valId);
  if (bar) bar.style.width = v + '%';
  if (val) val.textContent = Math.round(v) + '%';
}
let elapsedSeconds = 0;
let elapsedTimer = null;
let demoProgressTimer = null;
let demoProgress = 0;

// ===== DOM Ready =====
document.addEventListener('DOMContentLoaded', async () => {
  initNavigation();
  initSliders();
  initMaterialSelect();
  await loadSurfaceInfo();
  initRosBridge();
  initCameraSelect();
  initForceChart();
  initButtons();

  // Add SVG gradient for progress ring
  const svg = document.querySelector('.progress-ring');
  if (svg) {
    const defs = document.createElementNS('http://www.w3.org/2000/svg', 'defs');
    const grad = document.createElementNS('http://www.w3.org/2000/svg', 'linearGradient');
    grad.setAttribute('id', 'progressGradient');
    grad.setAttribute('x1', '0'); grad.setAttribute('y1', '0');
    grad.setAttribute('x2', '1'); grad.setAttribute('y2', '1');
    const s1 = document.createElementNS('http://www.w3.org/2000/svg', 'stop');
    s1.setAttribute('offset', '0'); s1.setAttribute('stop-color', '#4FC3F7');
    const s2 = document.createElementNS('http://www.w3.org/2000/svg', 'stop');
    s2.setAttribute('offset', '1'); s2.setAttribute('stop-color', '#FF9800');
    grad.appendChild(s1); grad.appendChild(s2);
    defs.appendChild(grad);
    svg.insertBefore(defs, svg.firstChild);
  }
});

// ===== Navigation =====
function initNavigation() {
  document.querySelectorAll('.nav-tab').forEach(tab => {
    tab.addEventListener('click', () => {
      autoFollowState = false;   // 사용자가 직접 탭 선택 → STATE 자동전환 중단(끌려가지 않음)
      switchPhase(tab.dataset.phase);
    });
  });
}

function switchPhase(phase) {
  currentPhase = phase;
  document.querySelectorAll('.nav-tab').forEach(t => t.classList.toggle('active', t.dataset.phase === phase));
  document.querySelectorAll('.phase-panel').forEach(p => p.classList.remove('active'));
  const panelMap = { pre: 'phasePre', during: 'phaseDuring', post: 'phasePost' };
  document.getElementById(panelMap[phase])?.classList.add('active');

  if (phase === 'during') {
    if (!ros || !ros.connected) {
      forEachChart(c => c.startDemo(50));
      startDemoProgress();
    }
    startElapsedTimer();
  } else {
    forEachChart(c => c.stopDemo());
    stopElapsedTimer();
    stopDemoProgress();
  }

  if (phase === 'post') loadCoverage3D();
}

// DONE 직후엔 파일이 막 기록되므로 몇 차례 재시도하며 로드
function retryLoadCoverage3D(tries = 6) {
  _cov3dLoaded = false;
  let n = 0;
  const tick = async () => {
    n++;
    await loadCoverage3D(true);
    if (!_cov3dLoaded && n < tries) setTimeout(tick, 1000);
  };
  tick();
}

// 폴리싱 완료 3D 커버리지 로드/표시 (web_dashboard/public/data/coverage.json)
async function loadCoverage3D(force = false) {
  const canvas = document.getElementById('coverage3dCanvas');
  if (!canvas) return;
  if (_cov3dLoaded && !force) { if (coverage3d) { coverage3d.resize(); coverage3d.render(); } return; }
  const overlay = document.getElementById('cov3dOverlay');
  const badge = document.getElementById('cov3dBadge');
  try {
    if (!coverage3d) coverage3d = new Coverage3D(canvas);
    await coverage3d.loadUrl('/data/coverage.json?t=' + Date.now());
    _cov3dLoaded = true;
    if (overlay) overlay.style.display = 'none';
    if (badge) badge.textContent = '완료';
    const m = coverage3d.meta || {};
    const stat = document.getElementById('cov3dStat');
    if (stat && m.total) {
      const pct = (100 * m.covered / m.total).toFixed(1);
      stat.textContent = `● 폴리싱 완료 ${pct}% (${m.covered}/${m.total})`;
    }
    applyResultMetrics(m);
  } catch (e) {
    console.warn('[cov3d] coverage.json 로드 실패(아직 폴리싱 미완료?):', e);
    if (badge) badge.textContent = '대기';
  }
}

// 결과 지표 — 폴리싱 완료 시 coverage.json 의 실데이터 반영
// (완성률=색칠/전체, 총 소요시간, 평균 접촉력, 접촉 비율). 제거깊이는 재료 선택에 연동(updateRemovalMetrics).
function applyResultMetrics(m) {
  if (!m) return;
  if (m.total) setText('metricCompletion', (100 * m.covered / m.total).toFixed(1));
  if (typeof m.elapsed_sec === 'number') {
    const mm = Math.floor(m.elapsed_sec / 60);
    const ss = Math.floor(m.elapsed_sec % 60);
    setText('metricTotalTime', `${mm}:${ss.toString().padStart(2, '0')}`);
  }
  if (typeof m.avg_force === 'number') setText('metricAvgForceResult', m.avg_force.toFixed(2));
  if (typeof m.contact_ratio === 'number') setText('metricContactRatio', m.contact_ratio.toFixed(1));
}

// 재료별 제거깊이(μm) — 물리 제거 모델이 없어 재료 스펙 기준 값으로 표시.
//   폴리머(도장) 3μm / 구조강(스탬핑) 20μm. 평균은 최대의 약 64%(기존 표기 비율 유지).
const REMOVAL_DEPTH = {
  polymer: { max: 3.0,  avg: 1.9 },
  steel:   { max: 20.0, avg: 12.8 },
};
function updateRemovalMetrics(material) {
  const r = REMOVAL_DEPTH[material] || REMOVAL_DEPTH.polymer;
  setText('metricMaxRemoval', r.max.toFixed(1));
  setText('metricAvgRemoval', r.avg.toFixed(1));
}

// ===== Sliders =====
function initSliders() {
  const sliders = [
    { id: 'paramForce', valId: 'paramForceVal', suffix: ' N' },
    { id: 'paramRPM', valId: 'paramRPMVal', suffix: ' RPM' },
    { id: 'paramSpeed', valId: 'paramSpeedVal', suffix: ' mm/s' },
    { id: 'paramPad', valId: 'paramPadVal', suffix: ' mm' }
  ];
  sliders.forEach(({ id, valId, suffix }) => {
    const slider = document.getElementById(id);
    const val = document.getElementById(valId);
    if (slider && val) {
      slider.addEventListener('input', () => { val.textContent = slider.value + suffix; });
    }
  });
}

// ===== Material Selection =====
function initMaterialSelect() {
  const select = document.getElementById('objMaterialSelect');
  const sanderMat = document.getElementById('sanderMaterial');
  
  // Elements to update
  const paramForce = document.getElementById('paramForce');
  const paramForceVal = document.getElementById('paramForceVal');
  const paramRPM = document.getElementById('paramRPM');
  const paramRPMVal = document.getElementById('paramRPMVal');
  const paramSpeed = document.getElementById('paramSpeed');
  const paramSpeedVal = document.getElementById('paramSpeedVal');
  const paramPreston = document.getElementById('paramPreston');
  
  if (select) {
    select.addEventListener('change', (e) => {
      if (e.target.value === 'steel') {
        if (sanderMat) sanderMat.textContent = 'Alumina';
        paramForce.value = 50;
        paramForceVal.textContent = '50 N';
        paramRPM.value = 10000;
        paramRPMVal.textContent = '10000 RPM';
        paramSpeed.value = 30;
        paramSpeedVal.textContent = '30 mm/s';
        paramPreston.value = '4.2e-13';
      } else { // polymer
        if (sanderMat) sanderMat.textContent = 'Polyurethane foam';
        paramForce.value = 20;
        paramForceVal.textContent = '20 N';
        paramRPM.value = 3000;
        paramRPMVal.textContent = '3000 RPM';
        paramSpeed.value = 50;
        paramSpeedVal.textContent = '50 mm/s';
        paramPreston.value = '8.0e-13';
      }
      updateRemovalMetrics(e.target.value);   // 재료별 제거깊이 갱신
    });
    updateRemovalMetrics(select.value);        // 초기값(기본 polymer) 반영
  }
}

// ===== Surface Info =====
function setText(id, text) {
  const el = document.getElementById(id);
  if (el) el.textContent = text;
}
async function loadSurfaceInfo() {
  try {
    const res = await fetch('/data/surface_info.json');
    const info = await res.json();
    const size = info.surface_size_m;
    setText('objSize', `${(size[0] * 1000).toFixed(0)} × ${(size[1] * 1000).toFixed(0)} mm`);
    setText('objHeight', `${info.height_range_cm.toFixed(1)} cm`);
    setText('objPoints', info.num_surface_points.toLocaleString());
    setText('pathWaypoints', Math.round(info.num_surface_points / 50));
  } catch (e) {
    console.warn('[App] Failed to load surface info:', e);
  }
}

// ===== ROS2 Bridge =====
function initRosBridge() {
  const urlInput = document.getElementById('rosUrl');
  const btnConnect = document.getElementById('btnConnect');
  const defaultUrl = urlInput ? urlInput.value : 'ws://localhost:9090';
  
  ros = new RosBridge(defaultUrl);
  ros.onConnectionChange = (connected) => {
    const badge = document.getElementById('rosBadge');
    const badgeText = badge.querySelector('.connection-badge__text');
    if (connected) {
      badge.classList.add('connected');
      badgeText.textContent = 'ROS2 Connected';
      // 연결만으로 LIVE 라 하지 않음 — 실제 카메라 프레임이 들어오면 renderCameraFrame 에서 LIVE 로 전환
      document.getElementById('cameraBadge').textContent = '대기중';
      if (btnConnect) {
        btnConnect.textContent = 'Disconnect';
        btnConnect.classList.add('connected');
      }
      
      // Stop demo mode if we were running it
      if (currentPhase === 'during') {
        forEachChart(c => c.stopDemo());
        stopDemoProgress();
      }
    } else {
      badge.classList.remove('connected');
      badgeText.textContent = 'ROS2 Disconnected';
      document.getElementById('cameraBadge').textContent = 'DEMO';
      if (btnConnect) {
        btnConnect.textContent = 'Connect';
        btnConnect.classList.remove('connected');
      }
      
      // Start demo mode if we are in during phase and disconnected
      if (currentPhase === 'during') {
        forEachChart(c => c.startDemo(50));
        startDemoProgress();
      }
    }
  };

  if (btnConnect && urlInput) {
    btnConnect.addEventListener('click', () => {
      if (ros.connected) {
        ros.disconnect();
      } else {
        ros.url = urlInput.value;
        ros.connect();
      }
    });
  }

  // Auto connect initially
  ros.connect();

  // Subscribe to polishing state
  ros.subscribe(ROS_TOPICS.STATE.topic, ROS_TOPICS.STATE.type, (msg) => {
    const state = msg.data;
    updateState(state);
    if (!autoFollowState) return;   // 사용자가 직접 탭을 골랐으면 자동전환 안 함
    if (state === 'POLISH' || state === 'APPROACH') switchPhase('during');
    else if (state === 'DONE') { switchPhase('post'); retryLoadCoverage3D(); }
  });

  // Subscribe to force — 로봇팔 3개 각각의 토픽
  FORCE_BOTS.forEach(({ key, topicKey, valId }) => {
    const t = ROS_TOPICS[topicKey];
    ros.subscribe(t.topic, t.type, (msg) => {
      const chart = forceCharts[key];
      if (chart) {
        chart.stopDemo();
        chart.addPoint(msg.data);
      }
      const el = document.getElementById(valId);
      if (el) el.textContent = Number(msg.data).toFixed(2);
    });
  });

  // Subscribe to progress (전체 진행률 = 색칠된 점/전체 점 % → 결과 지표 '완성률'과 동일 데이터)
  ros.subscribe(ROS_TOPICS.PROGRESS.topic, ROS_TOPICS.PROGRESS.type, (msg) => {
    stopDemoProgress();   // 실데이터 도착 → 데모 타이머와 충돌 방지(force 차트와 동일 패턴)
    updateProgress(msg.data);
    const comp = document.getElementById('metricCompletion');
    if (comp) comp.textContent = Math.min(100, Math.max(0, msg.data)).toFixed(1);
  });

  // Subscribe to per-robot progress (좌측 SL / 우측 SR / 천장 C)
  PROGRESS_BOTS.forEach(({ topicKey, barId, valId }) => {
    const t = ROS_TOPICS[topicKey];
    ros.subscribe(t.topic, t.type, (msg) => {
      stopDemoProgress();
      updateRobotProgress(barId, valId, msg.data);
    });
  });

  // Subscribe to removal heatmap (탑뷰 연마 커버리지) — 실시간 자동차 표면 반영
  ros.subscribe(ROS_TOPICS.HEATMAP.topic, ROS_TOPICS.HEATMAP.type, (msg) => {
    renderHeatmap(msg);
  });

  // Subscribe to elapsed time
  ros.subscribe(ROS_TOPICS.ELAPSED_TIME.topic, ROS_TOPICS.ELAPSED_TIME.type, (msg) => {
    stopElapsedTimer();
    elapsedSeconds = msg.data;
    updateElapsedDisplay();
  });

  // 카메라 구독은 선택된 1개만 (initCameraSelect 에서 selectCamera 호출)
  selectCamera(currentCameraKey);
}

// ===== Camera (4코너, 드롭다운 선택) =====
function renderCameraFrame(msg) {
  const canvas = document.getElementById('cameraCanvas');
  const img = document.getElementById('cameraImg');
  const overlay = document.getElementById('cameraOverlay');
  if (!canvas || !msg.data) return;

  if (img) img.style.display = 'none';
  if (overlay) overlay.style.display = 'none';
  canvas.style.display = 'block';
  const badge = document.getElementById('cameraBadge');
  if (badge) badge.textContent = 'LIVE';   // 실제 프레임 수신 시에만 LIVE

  const imageObj = new Image();
  imageObj.onload = () => {
    const ctx = canvas.getContext('2d');
    canvas.width = imageObj.width;
    canvas.height = imageObj.height;
    ctx.drawImage(imageObj, 0, 0, canvas.width, canvas.height);
  };

  let base64Data = '';
  if (typeof msg.data === 'string') {
    base64Data = msg.data;       // rosbridge 는 uint8[] 를 base64 문자열로 보냄
  } else {
    const uint8Array = new Uint8Array(msg.data);
    let binary = '';
    for (let i = 0; i < uint8Array.length; i++) binary += String.fromCharCode(uint8Array[i]);
    base64Data = btoa(binary);
  }
  imageObj.src = 'data:image/jpeg;base64,' + base64Data;
}

// 선택된 카메라 1개만 구독 (이전 구독은 해제 → 대역폭 절약)
function selectCamera(key) {
  const cam = CAMERAS.find(c => c.key === key);
  if (!cam || !ros) return;
  if (currentCameraTopic) ros.unsubscribe(currentCameraTopic);
  const t = ROS_TOPICS[cam.topicKey];
  currentCameraKey = key;
  currentCameraTopic = t.topic;
  ros.subscribe(t.topic, t.type, renderCameraFrame);

  // 카메라 전환 시 새 프레임 올 때까지 대기 표시로 리셋
  const canvas = document.getElementById('cameraCanvas');
  const overlay = document.getElementById('cameraOverlay');
  const badge = document.getElementById('cameraBadge');
  if (canvas) canvas.style.display = 'none';
  if (overlay) overlay.style.display = 'flex';
  if (badge && (!ros || !ros.connected)) badge.textContent = 'DEMO';
  else if (badge) badge.textContent = '대기중';
}

function initCameraSelect() {
  const select = document.getElementById('cameraSelect');
  if (!select) return;
  select.value = currentCameraKey;
  select.addEventListener('change', (e) => selectCamera(e.target.value));
}

// ===== Force Charts (로봇팔 3개) =====
async function initForceChart() {
  for (const { key, chartId } of FORCE_BOTS) {
    const canvas = document.getElementById(chartId);
    if (!canvas) continue;
    const chart = new ForceChart(chartId);
    await chart.loadDemoData('/data/force_log.csv');
    forceCharts[key] = chart;
  }
  // Update live force display periodically (demo mode 등)
  setInterval(() => {
    if (currentPhase !== 'during') return;
    for (const { key, valId } of FORCE_BOTS) {
      const chart = forceCharts[key];
      const el = document.getElementById(valId);
      if (chart && el) el.textContent = chart.getCurrentValue().toFixed(2);
    }
  }, 100);
}

// ===== State Updates =====
function updateState(state) {
  const chip = document.getElementById('stateChip');
  const text = document.getElementById('stateText');
  text.textContent = state;
  chip.classList.toggle('polishing', state === 'POLISH');
}

function updateProgress(pct) {
  const val = Math.min(100, Math.max(0, pct));
  document.getElementById('progressVal').textContent = Math.round(val);
  const ring = document.getElementById('progressRing');
  if (ring) {
    const circumference = 2 * Math.PI * 60; // r=60
    ring.style.strokeDashoffset = circumference * (1 - val / 100);
  }
}

// ===== Removal Heatmap (탑뷰 연마 커버리지) =====
// Float64MultiArray: layout.dim=[rows(Y), cols(X)], data=셀별 연마비율(0~1, 빈 셀 -1)
function renderHeatmap(msg) {
  const canvas = document.getElementById('heatmapCanvas');
  if (!canvas || !msg || !msg.data) return;
  const dims = (msg.layout && msg.layout.dim) || [];
  const rows = dims[0] && dims[0].size;
  const cols = dims[1] && dims[1].size;
  if (!rows || !cols) return;
  const data = msg.data;

  // 라이브 데이터 도착 → 정적 폴백 이미지 숨김
  const fallback = document.getElementById('heatmapFallback');
  if (fallback) fallback.style.display = 'none';
  canvas.style.display = 'block';

  const ctx = canvas.getContext('2d');
  const W = canvas.width, H = canvas.height;
  ctx.fillStyle = '#0d1326';            // 차량 외곽(빈 셀) 배경
  ctx.fillRect(0, 0, W, H);

  // 종횡비 유지하며 캔버스 중앙에 맞춤
  const scale = Math.min(W / cols, H / rows);
  const ox = (W - cols * scale) / 2;
  const oy = (H - rows * scale) / 2;
  const cw = Math.ceil(scale), ch = Math.ceil(scale);

  for (let r = 0; r < rows; r++) {
    for (let c = 0; c < cols; c++) {
      const v = data[r * cols + c];
      if (v == null || v < 0) continue;     // 점 없는 셀 = 투명(배경 유지)
      // 빨강(미연마 0) → 하늘색(연마완료 1)  ※ point cloud 색과 동일
      const cr = Math.round(255 * (1 - v) + 89 * v);
      const cg = Math.round(0 * (1 - v) + 191 * v);
      const cb = Math.round(0 * (1 - v) + 255 * v);
      ctx.fillStyle = `rgb(${cr},${cg},${cb})`;
      // Y는 위로 증가하도록 뒤집어 표시
      ctx.fillRect(ox + c * scale, oy + (rows - 1 - r) * scale, cw, ch);
    }
  }
}

function updateElapsedDisplay() {
  const min = Math.floor(elapsedSeconds / 60);
  const sec = Math.floor(elapsedSeconds % 60);
  document.getElementById('elapsedTime').textContent =
    `${String(min).padStart(2, '0')}:${String(sec).padStart(2, '0')}`;
}

function startElapsedTimer() {
  stopElapsedTimer();
  elapsedTimer = setInterval(() => {
    elapsedSeconds++;
    updateElapsedDisplay();
  }, 1000);
}

function stopElapsedTimer() {
  if (elapsedTimer) { clearInterval(elapsedTimer); elapsedTimer = null; }
}

// ===== Demo Progress =====
function startDemoProgress() {
  stopDemoProgress();
  demoProgress = 0;
  updateState('POLISH');
  demoProgressTimer = setInterval(() => {
    demoProgress += 0.15;
    if (demoProgress >= 100) demoProgress = 0;
    updateProgress(demoProgress);
    // 데모: 로봇별 막대도 살짝 다른 속도로
    updateRobotProgress('progBarSL', 'progValSL', Math.min(100, demoProgress * 1.1));
    updateRobotProgress('progBarSR', 'progValSR', Math.min(100, demoProgress * 0.9));
    updateRobotProgress('progBarC',  'progValC',  Math.min(100, demoProgress * 1.2));
  }, 100);
}

function stopDemoProgress() {
  if (demoProgressTimer) { clearInterval(demoProgressTimer); demoProgressTimer = null; }
}

// ===== Polishing Launcher =====
// 시작 버튼 → 런처 서버에 요청 → Isaac Sim (polishing_v5.py) 실행
async function startPolishing() {
  const btn = document.getElementById('btnStart');
  const original = btn ? btn.innerHTML : '';
  autoFollowState = true;   // 이번 런을 STATE 로 따라가도록 자동전환 재활성화
  if (btn) { btn.disabled = true; btn.innerHTML = 'Isaac Sim 실행 중…'; }
  try {
    const res = await fetch(`${LAUNCHER_URL}/start`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ obj_name: selectedObject }),
    });
    const data = await res.json();
    if (data.status === 'error') {
      alert('Isaac Sim 실행 실패: ' + (data.message || '알 수 없는 오류'));
    } else if (data.status === 'already_running') {
      console.log('[launcher] 이미 실행 중 (pid=' + data.pid + ')');
    } else {
      console.log('[launcher] 시작됨 (pid=' + data.pid + ')');
    }
    switchPhase('during');
  } catch (e) {
    console.warn('[launcher] 런처 서버 연결 실패:', e);
    alert(
      '런처 서버에 연결할 수 없습니다.\n' +
      'scripts 폴더에서 아래 명령으로 런처를 먼저 실행하세요:\n\n' +
      '  python3 dashboard_launcher.py\n\n' +
      '(데모 모니터링 화면으로 이동합니다)'
    );
    switchPhase('during');
  } finally {
    if (btn) { btn.disabled = false; btn.innerHTML = original; }
  }
}

// 런처 /scan_status 가 running:false 가 될 때까지 대기 (최대 maxMs)
async function waitScanDone(maxMs = 300000, stepMs = 2000) {
  const t0 = Date.now();
  while (Date.now() - t0 < maxMs) {
    try {
      const r = await fetch(`${LAUNCHER_URL}/scan_status`);
      const s = await r.json();
      if (typeof s.running !== 'boolean') return false;  // 구버전 런처(미지원) → 폴백
      if (!s.running) return true;
    } catch (e) {
      return false;   // 런처 연결 끊김 → 중단
    }
    await new Promise(res => setTimeout(res, stepMs));
  }
  return false;        // 타임아웃
}

// 선택된 차종의 실제 스캔 결과 이미지 표시 (없으면 정적 폴백)
function showScanImage(obj, done) {
  const img = document.getElementById('scanImg');
  const overlay = document.getElementById('scanOverlay');
  const badge = document.getElementById('scanBadge');
  if (!img) return;
  const real = `/images/scan_${obj}.png?t=${Date.now()}`;
  img.onerror = () => { img.onerror = null; img.src = '/images/scan_pointcloud.png?t=' + Date.now(); };
  img.src = real;
  img.style.display = 'block';
  if (overlay) overlay.style.display = 'none';
  if (badge) badge.textContent = done ? '완료' : '미리보기';
}

// 스캔 시작 → 런처 /scan (scan.py → path_generator.py), 완료 폴링 후 실제 포인트 클라우드 표시
async function startScan() {
  const btn = document.getElementById('btnScan');
  const original = btn ? btn.innerHTML : '';
  const badge = document.getElementById('scanBadge');
  const obj = selectedObject;           // 스캔 도중 선택이 바뀌어도 고정
  if (btn) { btn.disabled = true; btn.innerHTML = '스캔 중…'; }
  if (badge) badge.textContent = '스캔 중';
  let launched = false;
  try {
    const res = await fetch(`${LAUNCHER_URL}/scan`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ obj_name: obj }),
    });
    const data = await res.json();
    launched = data.status === 'started' || data.status === 'already_running';
    if (data.status === 'error') console.warn('[scan]', data.message);
  } catch (e) {
    console.warn('[scan] 런처 연결 실패(데모로 표시):', e);
  }

  if (launched) {
    const done = await waitScanDone();   // 완료까지 대기
    showScanImage(obj, done);
  } else {
    // 런처 미연결 → 정적 미리보기로 폴백
    showScanImage(obj, false);
  }
  if (btn) { btn.disabled = false; btn.innerHTML = original; }
}

// 오브젝트 원본 이미지 갱신 (차종 3종은 사진 표시, cube 등은 숨김)
function updateObjectPreview(obj) {
  const img = document.getElementById('objectPreviewImg');
  if (!img) return;
  const src = OBJECT_PREVIEWS[obj];
  if (src) {
    img.src = src;
    img.style.display = 'block';
  } else {
    img.style.display = 'none';
  }
}

// ===== Buttons =====
function initButtons() {
  // 오브젝트 선택 (car=BMW Z4 / cube)
  const objSel = document.getElementById('objectSelect');
  if (objSel) {
    selectedObject = objSel.value;
    updateObjectPreview(selectedObject);
    objSel.addEventListener('change', (e) => {
      selectedObject = e.target.value;
      updateObjectPreview(selectedObject);
    });
  }
  document.getElementById('btnScan')?.addEventListener('click', () => {
    startScan();
  });
  document.getElementById('btnStart')?.addEventListener('click', () => {
    startPolishing();
  });
  document.getElementById('btnNewPolish')?.addEventListener('click', () => {
    elapsedSeconds = 0;
    demoProgress = 0;
    switchPhase('pre');
  });
  // 분석레포트: 완성률 + 덜 닦인 부위 + 수동 작업 안내
  document.getElementById('btnAnalyze')?.addEventListener('click', openReport);
  const reportModal = document.getElementById('reportModal');
  document.getElementById('reportClose')?.addEventListener('click', () => { if (reportModal) reportModal.hidden = true; });
  reportModal?.addEventListener('click', (e) => { if (e.target === reportModal) reportModal.hidden = true; });
}

// ===== 분석 레포트 =====
function openReport() {
  const modal = document.getElementById('reportModal');
  const body = document.getElementById('reportBody');
  if (!modal || !body) return;
  const m = (coverage3d && coverage3d.meta) || {};
  if (!coverage3d || !coverage3d.pts || !m.total) {
    body.innerHTML = '<p>아직 폴리싱 완료 데이터가 없습니다. 폴리싱이 끝나면 분석할 수 있습니다.</p>';
    modal.hidden = false;
    return;
  }
  const completion = 100 * m.covered / m.total;
  const incomplete = Math.max(0, 100 - completion);
  const THRESH = 95;   // 이 미만 구역은 수동 폴리싱 권장
  const bad = coverage3d.analyzeRegions().filter(r => r.pct < THRESH).slice(0, 6);

  let html = '';
  html += `<div class="report-stat"><span class="report-stat__big">${completion.toFixed(1)}%</span> 완성 `
        + `<span class="report-stat__sub">(${m.covered.toLocaleString()} / ${m.total.toLocaleString()} 점)</span></div>`;
  html += `<div class="report-bar"><div class="report-bar__fill" style="width:${completion.toFixed(1)}%"></div></div>`;
  html += `<p>미완성 <b>${incomplete.toFixed(1)}%</b> 입니다.</p>`;

  if (bad.length === 0) {
    html += `<p class="report-ok">✅ 모든 부위가 충분히(≥${THRESH}%) 폴리싱되었습니다. 추가 수동 작업이 필요 없습니다.</p>`;
  } else {
    html += `<h3 class="report-h3">🔧 수동 폴리싱이 필요한 부위</h3>`;
    html += `<ul class="report-list">`;
    for (const r of bad) {
      html += `<li><b>${r.name}</b> — ${r.pct.toFixed(0)}% 완료 `
            + `<span class="report-miss">(미완 ${(100 - r.pct).toFixed(0)}%)</span></li>`;
    }
    html += `</ul>`;
    html += `<p class="report-note">작업자 안내: 위 부위는 로봇이 충분히 닦지 못했습니다. `
          + `해당 부위를 패드/그라인더로 수동 폴리싱한 뒤 재검사하세요.</p>`;
  }
  body.innerHTML = html;
  modal.hidden = false;
}
