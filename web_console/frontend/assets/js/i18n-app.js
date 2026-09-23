/* ══════════════════════════════════════════════════════════════
   앱 화면 사전 — ① 웹 UI · ② 공정 모니터링 · ④ 라이브러리

   i18n.js 보다 먼저 실려서 두 벌을 놓고 간다.

     PT_DICT_APP     텍스트 노드 하나가 통째로 맞을 때 쓴다. 기본 수단이다.
     PT_PHRASES_APP  노드 안의 한 토막만 바꾼다. 아래 경우에만 쓴다.

   왜 두 벌인가.
   이 화면들의 글은 값과 함께 자바스크립트에서 이어 붙는다.

       '도달 ' + assigned + ' / ' + points + ' pt · 미도달 구간은 …'

   런타임에 이것은 텍스트 노드 하나다. 숫자가 매번 달라지므로 노드 전체를
   키로 둘 수 없다. 그래서 앞뒤 토막을 따로 바꾼다. 토막은 긴 것부터
   맞춰 보므로, 짧은 토막이 긴 문장 안을 파고들지 않는다.

   여기 없는 문장은 EN 화면에서도 한국어로 남는다.
      화면 글을 고쳤으면 아래를 돌려라 — 빠진 것을 찍어 준다.

        python scripts/i18n-app-check.py

   토막을 새로 넣을 때는 그 토막이 다른 문장 안에도 들어 있지 않은지
      확인해라. 긴 문장을 먼저 등록하면 안전하다.
   ══════════════════════════════════════════════════════════════ */
(function () {
  'use strict';

  /* ── 노드 하나가 통째로 맞는 것 ──────────────────────────── */
  window.PT_DICT_APP = {

    /* ══ 공용 헤더 (assets/css/pt-header.css) ══ */
    '화면': 'Screens',        /* 내비 aria-label — ①②③④ 공통 */

    /* ══ ① 웹 UI — 화면 골격 ══*/
    '차량 3D · 파라미터 설정': '3D vehicle · parameters',
    '경로 간격': 'Path pitch',
    '패스': 'Passes',
    '추정': 'Est.',
    '차종(Vehicle)': 'Vehicle',
    '차체 리프트(Car Lift)': 'Car lift',
    '차체 리프트 높이': 'Car lift height',
    '로봇 팔(Robot Arms) 대수': 'Robot arms',
    '폴리셔(Polisher) 종류': 'Polisher type',
    '이동 레일(Rail) 추가': 'Add travel rail',
    '작업용 리프트(Lift) 추가': 'Add work lift',
    'RL 최적화 안내': 'About RL optimisation',
    '환경 설정 후 공정을 시작하면, AI가 해당 하드웨어 조건에서 최적의 폴리싱 파라미터(압력, 속도 등)를 탐색한다.':
      'Set the cell up and start the process, and the AI searches for the best polishing parameters — pressure, speed and the rest — under that hardware.',

    /* ── 웹 UI · 공정 패널 ──*/
    '경과': 'Elapsed',
    '남음': 'Left',
    '목표': 'Target',
    '편차': 'Deviation',
    '패드 회전': 'Pad rotation',
    '이송 속도': 'Feed rate',
    '도달률': 'Reach',
    '로봇별 진행': 'Progress by robot',

    /* ── 웹 UI · 화면 이동 ──*/
    '② 공정 감시 →': '② Monitor →',
    '③ 결과 저장 →': '③ Save result →',
    '④ 라이브러리 →': '④ Library →',
    '① 콘솔로 이동': '① Go to console',
    '① 설정': '① Setup',
    '② 공정 감시': '② Monitor',
    '③ 결과 저장': '③ Save result',
    '④ 라이브러리': '④ Library',

    /* ── 웹 UI · 대조 모달 ──*/
    'RL 시뮬레이션 완료': 'RL simulation done',
    '공정 결과 · 숙련공 정답 대조': 'Result vs. veteran reference',
    '표면 품질 · 합격 기준 대조': 'Surface quality vs. pass criteria',
    '제어 파라미터 · 숙련공 정답 대조': 'Control parameters vs. veteran reference',
    '공정 결과 · RL 탐색': 'Result · RL search',
    '숙련공 정답 · 데이터셋': 'Veteran reference · dataset',
    '항목': 'Item',
    '공정 결과': 'Result',
    '기준': 'Criterion',
    '판정': 'Verdict',
    '판정 불가': 'No verdict',

    /* 표면 품질 항목 — 데이터셋/quality_kpi.json 의 label 이 그대로 노드가 된다.
       JSON 을 고치면 여기도 같이 고쳐라. */
    '예측 20° GU proxy': 'Predicted 20° GU proxy',
    '평균 거칠기 Ra': 'Mean roughness Ra',
    '극단 거칠기 Rz': 'Peak roughness Rz',
    '잔여 클리어코트 (최소)': 'Clear coat left (min)',
    '스크래치': 'Scratches',
    '저장 완료': 'Saved',
    '저장 중': 'Saving',
    '저장 실패': 'Save failed',
    '다시 저장': 'Save again',
    '라이브러리에 기록하는 중이다.': 'Writing it to the library.',
    '일치': 'Match',
    '불일치': 'Mismatch',
    '대조할 정답이 없어 저장하지 않는다.': 'There is no reference to match against, so nothing is saved.',
    '5개 항목 모두 허용 오차 안이다. 라이브러리에 저장할 수 있다.':
      'All five items are inside tolerance. This can go to the library.',
    'RL 이 이 환경에서 찾은 파라미터를, 같은 세그먼트에서 숙련공이 낸 정답과 항목별로 맞춰 본다.':
      'The parameters RL found in this cell are matched item by item against the veteran reference for the same segment.',
    '전부 허용 오차 안에 들어야 라이브러리에 저장한다.':
      'All of them must fall inside tolerance before it goes to the library.',
    '숙련공 정답 데이터를 읽지 못했습니다': 'Could not read the veteran reference data',
    '데이터셋 없음': 'No dataset',
    '파기': 'Discard',
    '라이브러리 열기': 'Open library',
    '합격 (저장)': 'Pass (save)',
    '저장 불가': 'Cannot save',
    '품질': 'Quality',
    '파라미터': 'Parameters',

    /* ── 웹 UI · 차량과 공구 ──*/
    '벤츠 쿠페': 'Mercedes coupe',
    '페라리 SF90': 'Ferrari SF90',
    '현대 쏘나타': 'Hyundai Sonata',
    '듀얼': 'Dual',
    '싱글': 'Single',
    '기계자국과 홀로그램을 남기지 않는다. 대신 같은 면적에 시간이 더 걸린다.':
      'Leaves no machine marks or holograms. It takes longer over the same area.',
    '연마력이 월등하고 빠르다. 사람에게는 홀로그램 위험 때문에 숙련자 전용이지만, 접촉력이 일정한 로봇에는 그 제약이 없다.':
      'It cuts harder and faster. For a person it is veterans-only because of the hologram risk; a robot that holds contact force steady does not carry that limit.',
    '정면': 'Front',
    '상면': 'Top',
    '측면': 'Side',
    '자유': 'Free',
    '내림': 'Down',
    '차체가 내려와 있어 옆면 하단은 팔이 닿지 않는다. 올리면 측면 경로가 배정된다.':
      'With the body down the lower flanks stay out of reach. Raise it and side paths get assigned.',
    '레일': 'Rail',
    '고정': 'Fixed',
    '공정 모니터링': 'Process monitor',
    '공정 중단': 'Stop process',
    'RL 공정 시작': 'Start RL process',
    '저장된 최적 프리셋 불러오기': 'Load the saved best preset',
    '저장된 프리셋 없음': 'No saved preset',
    '브라우저 저장 공간이 가득 찼습니다.': 'Browser storage is full.',

    /* ── 웹 UI · 품질 판정 ──*/
    '목표 충족': 'Meets target',
    '부분': 'Partial',
    '낮음': 'Low',
    '심한 결함': 'Severe defect',
    '충족': 'Pass',
    '미달': 'Below',
    '접촉력 이탈': 'Out of band',
    '정상 추종': 'Tracking',

    /* ── 웹 UI · 파라미터 이름 (④ 와 같은 말을 쓴다) ──*/
    '접촉력 목표': 'Contact force target',
    '패드 강성 k': 'Pad stiffness k',
    '감쇠 c': 'Damping c',
    '속도 배율': 'Speed factor',
    'in-band 비율': 'In-band ratio',

    /* ══ ② 공정 모니터링 ══ */
    '공정 모니터링 · PolyTwin': 'Process monitor · PolyTwin',
    '잔여': 'Remaining',
    '일시정지': 'Pause',
    '정지': 'Stop',
    '재개': 'Resume',
    '실시간 3D 뷰': 'Live 3D view',
    '카메라': 'Camera',
    '미연마': 'Unpolished',
    '연마 완료': 'Polished',
    '지표': 'Metrics',
    '이벤트': 'Events',
    '제거량 히트맵': 'Removal heat map',
    '제거량 히트맵 — 차체 탑뷰': 'Removal heat map — body top view',
    '접촉력 — 목표대역 3–8 N': 'Contact force — target band 3–8 N',

    /* 셀 요약 줄은 숫자마다 <b> 로 끊겨 있어 토막이 각각 텍스트 노드다.
       「차종 BMW Z4 · 셀 A-02 · 로봇 3대 · 레일 2축 + 갠트리 1축」 순서로 읽힌다.
       차종이 앞에 붙으면서 첫 토막이 '셀' 이 아니라 '· 셀' 이 됐다. */
    '차종': 'Vehicle',
    '· 셀': '· Cell',
    '셀': 'Cell',
    '· 로봇': '· robots',
    '대 · 레일': '· rail',
    '축 + 갠트리': 'axes + gantry',
    '축': 'axis',
    '회': 'times',
    '전체': 'All',
    '천장': 'Ceiling',
    '천장뷰': 'Ceiling view',
    '좌측': 'Left',
    '우측': 'Right',
    '패드': 'Pad',
    '이 브라우저에서는 3D 뷰를 표시할 수 없다.': 'This browser cannot show the 3D view.',

    /* ── 모니터 · 지표 타일 ── */
    '힘 변동계수': 'Force CV',
    '목표대역 체류율': 'Time in band',
    '접촉 유지율': 'Contact retention',
    '제거 균일도': 'Removal uniformity',
    '과압 발생': 'Over-pressure events',

    /* ── 모니터 · 이벤트 로그 ── */
    '레일 이송 시작 — 구간 1/6': 'Rail travel started — segment 1/6',
    '접촉 확립 · 목표 5.2 N 추종 개시': 'Contact established · tracking 5.2 N',
    '마지막 구간 후퇴 — 리프트 하강': 'Last segment retracted — lift down',
    '운전 재개': 'Run resumed',
    '일시정지 — 오퍼레이터 요청': 'Paused — operator request',
    '정지 — 원점 복귀': 'Stopped — homing',
    '셀 A-02 로드 완료 · 로봇 3대 연결': 'Cell A-02 loaded · 3 robots connected',

    /* ══ ④ 라이브러리 ══ */
    '숙련공 정답 데이터': 'Veteran reference data',
    '세그먼트 검색': 'Search segments',
    '그리드': 'Grid',
    '리스트': 'List',
    '인증만 보기': 'Certified only',
    '레시피 비교 · 접촉력 시계열': 'Recipe comparison · contact-force series',
    '모두 닫기': 'Close all',
    '정답 run 의 접촉력 시계열': 'Contact force over time, reference run',
    '정답 run 의 접촉력': 'Contact force, reference run',
    '경로 진행 (스텝)': 'Path progress (steps)',
    '고정 게인 baseline': 'Fixed-gain baseline',
    '지표 6종': 'Six metrics',
    '지표 6종 비교 레이더': 'Radar of the six metrics',
    '파라미터 차이': 'Parameter differences',
    '불러오기': 'Load',
    '세그먼트': 'Segment',
    '태그': 'Tags',
    '점수': 'Score',
    '과압': 'Over-pressure',
    '힘 표준편차': 'Force σ',
    '등록': 'Registered',
    '액션': 'Actions',
    '원본 로그': 'Raw log',
    '취소': 'Cancel',

    /* ── 라이브러리 · 걸러 보기 ── */
    '데이터셋': 'Dataset',
    '로봇': 'Robot',
    '로봇 전체': 'All robots',
    '측면좌': 'Left flank',
    '출처 전체': 'All sources',
    '성능 전체': 'All grades',
    '스윕 최적': 'Sweep best',
    'RL 저장': 'RL saved',
    'RL 검증': 'RL validated',
    'RL 합격': 'RL pass',
    '정답 인증': 'Certified',
    'in-band 0.95 이상': 'in-band 0.95 and up',
    '0.90 미만': 'under 0.90',
    '품질순': 'By quality',
    '최신순': 'Newest',
    '비교에서 빼기': 'Remove from comparison',
    '비교에 추가': 'Add to comparison',
    '#과압_없음': '#no_overpressure',

    /* ── 라이브러리 · 레이더 축 ── */
    '목표대역 체류': 'Time in band',
    '힘 안정성': 'Force stability',
    '과압 회피': 'Over-pressure avoidance',
    'baseline 개선': 'Gain over baseline',
    '힘 정확도': 'Force accuracy',
    '힘 평균 / σ': 'Force mean / σ',

    /* ── 라이브러리 · 안내문 ── */
    '값이 갈리는 항목만 강조했다.': 'Only the items that differ are highlighted.',
    '두 장 이상 고르면 차이가 나는 항목을 짚어 준다.':
      'Pick two or more and the differing items are called out.',
    '조건에 맞는 세그먼트가 없다': 'No segment matches',
    '데이터셋을 읽는 중…': 'Reading the dataset…',
    '원본 로그를 읽는 중…': 'Reading the raw log…'
  };

  /* ── 노드 안의 한 토막만 바꾸는 것 ────────────────────────
     값과 이어 붙어 노드 전체를 키로 둘 수 없는 자리다.
     긴 것부터 맞춰 보므로 여기 적힌 순서는 상관없다.
     앞뒤 공백은 그 자리에 실제로 있는 그대로 적는다. */
  window.PT_PHRASES_APP = [

    /* ── ① 웹 UI ──*/
    ['도달 ', 'Reached '],
    [' pt · 미도달 구간은 검증 리포트로 남는다', ' pt · unreached areas stay in the validation report'],
    ['평균 ≥ ', 'mean ≥ '],
    ['에피소드 ', 'Episode '],
    [' · 셀 ', ' · cell '],
    ['세그먼트 ', 'Segment '],
    [' 로봇 · RL 검증', ' robot · RL validated'],
    [' 로봇', ' robot'],
    ['차체를 ', 'Raising the body by '],
    [' mm 올려 도어·펜더가 팔의 도달 범위에 들어온다.',
     ' mm brings the doors and fenders into the arms’ reach.'],
    ['대신 루프가 그만큼 높아진다.', 'The roof rises by the same amount.'],
    ['라이브러리에 저장했다. 세그먼트 ', 'Saved to the library. Segment '],
    [' 의 원본 CSV 와 함께 04 화면에서 열린다.', ' opens on screen 04 together with its raw CSV.'],
    ['품질 ', 'Quality: '],
    ['개 항목이 기준을 넘겼고, 파라미터 ', ' items pass, and '],
    ['개도 허용 오차 안이다. 라이브러리에 저장할 수 있다.',
     ' parameters are inside tolerance. This can go to the library.'],
    ['개 항목이 기준 미달', ' items below criterion'],
    ['개 항목이 허용 오차 밖', ' items outside tolerance'],
    ['이다. 저장하지 않는다.', '. Not saved.'],
    ['. 데이터셋/seg_best_kpi.json 이 있는지 확인하세요 — 없으면 데이터셋/make_seed.py 로 만듭니다.',
     '. Check that 데이터셋/seg_best_kpi.json exists — if not, build it with 데이터셋/make_seed.py.'],
    ['. 데이터셋/seg_best_kpi.json 이 있는지 확인하세요.',
     '. Check that 데이터셋/seg_best_kpi.json exists.'],
    [' · 최소 ', ' · min '],
    [' · 타일 ', ' · tiles '],
    /* 광택 대역 — detail 줄 끝에 붙는다. 앞의 ' · ' 까지 같이 잡아
       '부분'·'낮음' 같은 짧은 말이 다른 문장 속을 파고들지 않게 한다. */
    [' · 목표 충족', ' · meets target'],
    [' · 심한 결함', ' · severe defect'],
    [' · 부분', ' · partial'],
    [' · 낮음', ' · low'],
    /* 로봇 이름은 seg_best_kpi.json 이 들고 있어 값과 이어 붙는다.
       '· ' 까지 묶어 세그먼트 뒤의 그 자리에서만 바뀐다. */
    ['· 천장 ', '· ceiling '],
    ['· 측면좌 ', '· left-flank '],
    /* 판정 문장은 '품질 N개 …, 파라미터 N개 …' 로 이어 붙는다 */
    ['파라미터 ', 'parameters '],
    /* 저장 실패 — 서버 메시지 뒤에 이 문장이 붙는다 */
    ['저장하지 못했습니다.', 'Could not save.'],
    [' 판정은 그대로다 — 다시 눌러 저장할 수 있다.',
     ' The verdict stands — press it again to save.'],
    [' · RL 검증', ' · RL validated'],

    /* ── ② 공정 모니터링 ── */
    ['과압 임계 초과 ', 'Over-pressure threshold exceeded '],
    ['접촉 끊김 감지 ', 'Contact loss detected '],
    ['셀을 불러오지 못했다 — ', 'Could not load the cell — '],
    ['로는 fetch 가 막힌다. 이 폴더에서 서버를 띄워라:',
     'blocks fetch. Serve this folder over HTTP:'],

    /* ── ④ 라이브러리 ── */
    ['허용 힘 밴드 ', 'Allowed force band '],
    [' 기준). 이 안에 머문 스텝 비율이 in-band 다.',
     '). The share of steps inside it is the in-band ratio.'],
    ['0–1 정규화 · 힘 안정성은 CV ', '0–1 normalised · force stability takes CV '],
    ['% 를 0 점, baseline 개선은 +', '% as zero; gain over baseline takes +'],
    ['%p 를 만점으로 본다', ' points as full marks'],
    ['데이터셋을 읽지 못했습니다 — ', 'Could not read the dataset — '],
    ['원본 CSV 를 읽지 못했습니다 — ', 'Could not read the raw CSV — '],
    ['baseline 대비 ', 'vs. baseline '],
    ['#개선_+', '#improved_+'],
    ['#과압_', '#overpressure_'],
    ['과압 ', 'Over '],
    ['힘σ ', 'Force σ '],
    ['접촉력 ', 'Contact force '],

    /* 한 글자짜리 토막은 다른 낱말 속을 파고든다 — '행' 은 '진행'·'실행'
       안에도 있다. 그래서 숫자 뒤에 붙은 것만 바꾸도록 정규식으로 둔다.
       첫 칸이 정규식이면 두 번째 칸의 $1 이 잡힌 숫자다. */
    /* '초기 0.0975 대비 감소' — 영어는 어순이 반대라 통째로 뒤집는다.
       토막 두 개로 나누면 'Initial 0.0975 down from' 이 된다. */
    [/초기 ([\d.]+) 대비 감소/g, 'down from $1'],
    [/(\d)\s*행/g, '$1 rows'],
    [/(\d)\s*스텝/g, '$1 steps'],
    [/(\d)\s*건/g, '$1 items'],
    /* 「3대」 는 로봇 팔 개수 고르는 칸이다. 영어에는 세는 말이 없어 숫자만 남긴다.
       뒤에 한글이 이어지면(대비·대조…) 건드리지 않는다. */
    [/(\d)\s*대(?![가-힣])/g, '$1'],
    /* 위 토막들이 남긴 '1 items' 를 단수로 고친다. 정규식은 뒤에 모이므로
       앞의 교체가 끝난 뒤에 돈다. 한글이 있던 노드에서만 돌기 때문에
       원래 영어였던 글은 건드리지 않는다. */
    [/\b1 items\b/g, '1 item'],
    /* 파라미터 절만 남으면 문장 첫머리가 된다 */
    [/^parameters /, 'Parameters ']
  ];

  /* ── 언어 전환 상자 ────────────────────────────────────────
     웹 UI·라이브러리는 번들이라 마크업을 직접 고치기 어렵다.
     그래서 화면이 그려진 뒤에 끼워 넣는다.
     i18n.js 는 '.lang a[data-lang]' 을 보고 동작하므로 모양만 맞추면 된다.

     화면마다 헤더가 달라 앵커를 셋씩 두고 처음 맞는 것을 쓰고
     있었다 — 그래서 ① 에서는 3D 씬 한복판에 떨어졌다. 이제 ①②④ 가
     assets/css/pt-header.css 의 같은 헤더를 쓰므로 자리는 하나뿐이다.
     (③ 은 아직 사전 미비로 토글을 달지 않았다. 헤더 슬롯은 이미 있다.)
     선택자가 실제로 맞는지는 scripts/i18n-app-check.py --anchors 가 본다. */
  var ANCHORS = [
    '.pt-hdr__end'     /* 공용 헤더 오른쪽 슬롯 — 언제나 맨 끝에 붙는다 */
  ];

  var CSS =
    '.pt-lang{display:inline-flex;align-items:baseline;gap:6px;' +
    'font-size:14px;letter-spacing:.06em;white-space:nowrap;' +
    'pointer-events:auto;margin-inline-start:14px}' +
    '.pt-lang a{color:var(--text-lo);text-decoration:none;padding:2px 3px;' +
    'transition:color 160ms cubic-bezier(.4,0,.2,1)}' +
    '.pt-lang a:hover{color:var(--text-mid)}' +
    '.pt-lang a[aria-current]{color:var(--text-hi)}' +
    '.pt-lang a:focus-visible{outline:2px solid var(--accent);outline-offset:2px}' +
    '.pt-lang i{color:var(--line);font-style:normal}';

  function build() {
    var p = document.createElement('p');
    p.className = 'lang pt-lang';
    p.setAttribute('role', 'group');
    p.setAttribute('aria-label', '언어');
    p.innerHTML = '<a href="?lang=ko" hreflang="ko" data-lang="ko">KR</a>' +
                  '<i aria-hidden="true">·</i>' +
                  '<a href="?lang=en" hreflang="en" data-lang="en">EN</a>';
    return p;
  }

  function place() {
    if (document.querySelector('.pt-lang')) return true;
    for (var i = 0; i < ANCHORS.length; i++) {
      var host = document.querySelector(ANCHORS[i]);
      if (!host) continue;
      host.appendChild(build());
      if (!document.getElementById('pt-lang-css')) {
        var st = document.createElement('style');
        st.id = 'pt-lang-css';
        st.textContent = CSS;
        (document.head || document.documentElement).appendChild(st);
      }
      if (window.PTI18n) window.PTI18n.apply();   /* KR/EN 표시를 지금 칠한다 */
      return true;
    }
    return false;
  }

  /* 앱이 언제 다 그려질지 모른다. 붙을 자리가 생길 때까지 지켜본다.
     번들 화면은 document.documentElement 를 통째로 갈아 끼우므로
     document 를 본다 — body 를 보면 그 순간 관찰이 끊긴다. */
  function watch() {
    if (place()) return;
    var mo = new MutationObserver(function () {
      if (place()) mo.disconnect();
    });
    mo.observe(document, { childList: true, subtree: true });
    /* 20초가 지나도 자리가 없으면 포기한다. 계속 지켜볼 이유가 없다 */
    setTimeout(function () { mo.disconnect(); }, 20000);
  }

  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', watch);
  } else {
    watch();
  }
})();
