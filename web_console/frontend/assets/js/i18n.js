/* ══════════════════════════════════════════════════════════════
   언어 전환 — 한국어 / 영어

   원문을 키로 쓴다. data-i18n 키를 따로 두면 마크업과 사전이 어긋나는
   순간 어느 쪽이 맞는지 알 수 없게 된다. 화면에 보이는 한국어 그대로가 키다.

   적용 범위: 공통 헤더·드로어·푸터·로그인 창 + 랜딩 전체.
   랜딩의 그림은 인라인 SVG라 아래 사전이 글자를 그대로 잡는다.
   sub.html 의 그림은 래스터라 글자가 픽셀에 구워져 있다 — 영문판 파일을
   따로 굽고(scripts/i18n-figures.py) IMG_EN 이 src 만 바꿔 끼운다.
   sub.html 본문(PAGES)은 assets/js/i18n-sub.js 가 맡는다.
   ══════════════════════════════════════════════════════════════ */
(function () {
  'use strict';

  var KEY = 'pt_lang';

  function norm(s) { return String(s).replace(/\s+/g, ' ').trim(); }

  /* ── 사전 ─────────────────────────────────────────────────
     좌: 화면의 한국어 원문 (공백은 하나로 눌러서 비교한다)
     우: 영어. 형용사를 늘어놓지 않는다 — 원문과 같은 밀도로 쓴다.
     ───────────────────────────────────────────────────────── */
  var DICT = {
    /* ── 공통 골격 ── */
    '본문으로 건너뛰기': 'Skip to content',
    '메뉴 열기': 'Open menu',
    '메뉴 닫기': 'Close menu',
    '닫기': 'Close',
    'PolyTwin 홈': 'PolyTwin home',
    '언어': 'Language',
    '이 페이지': 'On this page',
    '사이트': 'Site',
    '섹션': 'Sections',
    '푸터': 'Footer',
    '맨 위로': 'Back to top',
    '홈으로': 'Home',
    '홈': 'Home',
    '관리자': 'Admin',
    '현재 위치': 'Breadcrumb',
    '게시물': 'Posts',
    '자주 묻는 질문': 'Frequently asked questions',
    '프로세스': 'Process',
    '핵심 수치': 'Key figures',
    '바로 가기': 'Shortcuts',
    '콘솔 열기': 'Open console',
    '공정 감시': 'Process monitor',
    '콘솔': 'Console',
    'PolyTwin — 로봇 폴리싱 공정 디지털 트윈': 'PolyTwin — Digital twin for robotic polishing',
    'PolyTwin — 로봇 폴리싱 공정 디지털 트윈 · 2026': 'PolyTwin — Digital twin for robotic polishing · 2026',
    'PolyTwin — 로봇 폴리싱 공정 디지털 트윈. 공정 · 기술 · 안전 검증 · 가격 정책 · 팀.':
      'PolyTwin — Digital twin for robotic polishing. Processes · technology · safety validation · pricing · team.',

    /* ── 메뉴 ── */
    '기업소개': 'Company',
    '우리의 목표': 'Our goal',
    '우리의 가치관': 'Our values',
    '우리의 성과': 'Our results',
    '가격 정책': 'Pricing',
    '폴리싱': 'Polishing',
    '디버링': 'Deburring',
    '샌딩': 'Sanding',
    '확장 로드맵': 'Roadmap',
    '기술혁신': 'Technology',
    '강화학습': 'Reinforcement learning',
    '숙련공 DB': 'Expert DB',
    '안전 검증': 'Safety validation',
    '솔루션': 'Solutions',
    '기술': 'Technology',

    /* ── 섹션 이름 ── */
    '공정': 'Process',
    '기능': 'Features',
    '사양': 'Specs',
    '라이브러리': 'Library',

    /* ── 로그인 / 회원가입 ── */
    '로그인': 'Log in',
    '회원가입': 'Sign up',
    '계정 만들기': 'Create account',
    '공정 콘솔에 접속하려면 계정 정보를 입력하세요.': 'Enter your credentials to reach the process console.',
    '입력을 확인해 주세요': 'Check your input',
    '이름': 'Name',
    '(선택)': '(optional)',
    '홍길동': 'e.g. Jane Doe',
    '사번 또는 아이디': 'Employee no. or ID',
    '비밀번호': 'Password',
    '비밀번호 (8자 이상)': 'Password (8 characters or more)',
    '비밀번호 확인': 'Confirm password',
    '비밀번호 표시': 'Show password',
    '로그인 상태 유지': 'Keep me signed in',
    '비밀번호 찾기': 'Forgot password',
    '계정이 없으신가요?': 'No account yet?',
    '이미 계정이 있으신가요?': 'Already have an account?',
    '확인 중': 'Checking',
    '등록 중': 'Registering',
    '가입 신청이 접수되었습니다.': 'Your sign-up request has been received.',
    '관리자 승인 후 콘솔 접속 권한이 부여됩니다.': 'Console access is granted once an administrator approves it.',
    'ID를 입력하세요.': 'Enter an ID.',
    'ID는 4자 이상이어야 합니다.': 'ID must be at least 4 characters.',
    '영문·숫자와 . _ - 만 쓸 수 있고 4–32자여야 합니다.':
      'Only letters, digits and . _ - are allowed, 4–32 characters.',
    '비밀번호를 입력하세요.': 'Enter a password.',
    '비밀번호는 8자 이상이어야 합니다.': 'Password must be at least 8 characters.',
    '비밀번호를 다시 입력하세요.': 'Re-enter the password.',
    '비밀번호가 일치하지 않습니다.': 'The passwords do not match.',
    '접속하려면 로그인이 필요한 화면입니다.': 'This screen requires you to log in.',
    '요청을 처리하지 못했습니다.': 'The request could not be completed.',

    /* ── 랜딩 · 정보 ──*/
    '차체 폴리싱 로봇 공정을 실시간으로 관제하고, 숙련공이 만든 파라미터를 한 벌로 저장해 다음 차종에 그대로 불러온다.':
      'Monitor the robotic body-polishing process live, save a veteran’s parameters as one set, and load it unchanged on the next model.',
    '공정을 지켜보고, 잘 된 설정을 저장하고, 다음 공정에서 다시 꺼내 쓴다.':
      'Watch the process, save the setup that worked, pull it back out on the next run.',

    /* ── 랜딩 · 히어로 ── */
    '차체 폴리싱 로봇 공정의 3차원 미리보기': '3D preview of the robotic body-polishing process',
    'AI가 찾은 최적값을': 'AI finds the optimum.',
    '숙련공이 검증한다': 'A veteran signs it off.',
    '작업 환경을 설정하여 강화학습(RL)이 도출한 최적의 파라미터로 공정을 시뮬레이션하고, 숙련공이 채점하여 합격한 결과만 라이브러리에 저장한다.':
      'Set up the work cell, simulate the process with the parameters reinforcement learning derives, and keep only the runs a veteran passes in the library.',
    '공정 보기': 'See the process',

    /* ── 랜딩 · 문장 하나 ── */
    '숙련공의 감은 은퇴와 함께': 'A veteran’s feel walks out',
    '사라진다.': 'with the retirement.',
    '남는 건 결과물뿐이다. 그 결과를 만든 암묵지(압력·속도·경로)는 어디에도 기록되지 못했다. 아래부터가 그 암묵지를 남기는 방법이다.':
      'What is left is the finished part. The tacit knowledge behind it — pressure, speed, path — was never recorded anywhere. What follows is how that knowledge gets kept.',

    /* ── 랜딩 · 01 공정 ── */
    '01 — 공정': '01 — Process',
    '잘 닦였는지는 눈이 아니라 표면 수치로 판정한다': 'A good finish is judged by surface numbers, not by eye',
    '차체를 3,600칸 격자로 나눠 칸마다 표면을 잰다. 거칠기 Ra 0.20 µm 이하, 극단 거칠기 Rz 2.0 µm 이하, 잔여 클리어코트 35 µm 이상 — 한 칸이라도 벗어나면 합격이 아니다. 숙련공이 합격을 매길 때 보는 것도 이 수치다.':
      'The body is split into a 3,600-cell grid and every cell is measured. Ra at or below 0.20 µm, Rz at or below 2.0 µm, at least 35 µm of clearcoat left — one cell outside the line and the run does not pass. These are the same numbers a veteran signs off on.',
    '차체 표면이 무광에서 유광으로 바뀌는 스캔 시연': 'Scan demo — the body surface going from matte to gloss',
    '진행': 'Progress',
    '거칠기 Ra': 'Roughness Ra',
    '합격선': 'Pass line',
    '다시 재생': 'Replay',
    '이 브라우저에서는 3D 미리보기를 표시할 수 없다.': 'This browser cannot show the 3D preview.',

    /* ── 랜딩 · 02 기능 ── */
    '02 — 기능': '02 — Features',
    '환경 설정, RL 최적화, 숙련공 평가': 'Cell setup, RL optimization, veteran review',
    '환경을 세우고, 학습이 값을 찾고, 숙련공이 판정한다. 통과한 것만 라이브러리에 남는다.':
      'Set the cell up, let learning find the values, let a veteran judge. Only what passes stays in the library.',
    '환경 설정': 'Cell setup',
    '파라미터 대신 작업 환경을 구축한다': 'Build the work cell, not the parameters',
    '값을 손으로 맞추지 않는다. 3D 뷰어에서 설비 구성만 정하면 탐색은 학습이 맡는다.':
      'Nobody dials the values in by hand. Fix the equipment layout in the 3D viewer and learning takes over the search.',
    '로봇 팔 대수 (1–3대) 지정': 'Set the number of robot arms (1–3)',
    '이동 레일 유무 설정': 'Travel rail on or off',
    '작업용 리프트 토글': 'Work lift toggle',
    '라이브 · 세션 A-2261': 'Live · session A-2261',
    '가동중': 'Running',
    '접촉력': 'Contact force',
    '이송': 'Feed',
    '토크': 'Torque',
    '최근 60초 접촉력 추이. 4.5에서 6.5뉴턴 사이를 유지하다 끝부분에서 소폭 상승.':
      'Contact force over the last 60 seconds. Held between 4.5 and 6.5 newtons, rising slightly at the end.',
    '커버리지': 'Coverage',
    '최적화 및 채점': 'Optimization and scoring',
    '채점을 통과하기 전까지는 전부 후보값이다': 'Until it passes scoring, every value is a candidate',
    '강화학습이 파라미터를 탐색해 공정을 돌리고, 숙련공이 그 결과를 채점한다.':
      'Reinforcement learning searches the parameters and runs the process; a veteran scores the result.',
    '강화학습(RL) 기반 파라미터 자동 탐색': 'Automatic parameter search driven by reinforcement learning (RL)',
    '면적 커버리지, 조도 등 품질 지표 도출': 'Quality metrics such as area coverage and surface finish',
    '숙련공의 Pass/Fail 및 별점 부여': 'Pass/fail and a star rating from a veteran',
    '프리셋 저장': 'Save preset',
    '프리셋 이름': 'Preset name',
    '세단 루프 — 김영수 반장 설정': 'Sedan roof — foreman Kim Young-su’s setup',
    '포함된 파라미터': 'Parameters included',
    '오버랩': 'Overlap',
    '패드': 'Pad',
    '경로': 'Path',
    '각도': 'Angle',
    '저장 시점의 커버리지·Ra·사이클 타임이 함께 묶인다.':
      'Coverage, Ra and cycle time at the moment of saving are bundled with it.',
    '검증된 데이터만 다시 꺼내 쓴다': 'Only validated data gets reused',
    '합격점을 받은 파라미터 세트만 라이브러리에 남는다. 다음 작업은 여기서 시작한다.':
      'Only parameter sets that passed stay in the library. The next job starts here.',
    '환경 조건별 강화학습 파라미터 저장': 'Store RL parameters per cell condition',
    '합격 레시피 비교 및 적용': 'Compare and apply passing recipes',
    '결과 지표 기반 정렬 기능': 'Sort by result metrics',
    '라이브러리 · 4건': 'Library · 4 entries',
    'Ra 낮은 순': 'Lowest Ra first',
    '프리셋': 'Preset',
    '사이클': 'Cycle',
    '세단 루프 — 김영수': 'Sedan roof — Kim Young-su',
    '적용중': 'Applied',
    '세단 후드 — 김영수': 'Sedan hood — Kim Young-su',
    'SUV 도어 — 표준': 'SUV door — standard',
    'SUV 루프 — 초기값': 'SUV roof — initial',
    '기준 미달': 'Below spec',
    '폴리싱은 네 공정 중 첫 번째다. 디버링·샌딩·Pick & Place 가 같은 스캔·경로·힘 제어 파이프라인을 쓴다.':
      'Polishing is the first of four processes. Deburring, sanding and pick & place run the same scan, path and force-control pipeline.',
    '공정 4종 보기': 'See all four processes',

    /* ── 랜딩 · 03 사양 ── */
    '03 — 사양': '03 — Specs',
    '추정값은 싣지 않았다': 'No estimates here',
    '3D 모델 4종 합계': 'Four 3D models, combined',
    '차체 모델 압축률': 'Body model compression',
    '종': 'kinds',
    '한 프리셋에 묶이는 값': 'Values bundled into one preset',
    '폰트 서브셋 3벌 · 928자': 'Three font subsets · 928 glyphs',

    /* ── 랜딩 · 마감 ── */
    '지금 돌고 있는 공정부터 기록한다': 'Start recording with the process running now',
    '설정을 만들고, 공정을 지켜보고, 결과가 좋으면 저장한다. 다음 차종에서 다시 꺼낸다.':
      'Build a setup, watch the process, save it if the result is good. Pull it back out on the next model.',
    '라이브러리 보기': 'Open library'
  };

  /* sub.html 본문 사전은 i18n-sub.js 가 먼저 실려서 놓고 간다.
     랜딩에는 싣지 않는다 — 본문 사전이 100KB 다.
     이미 있는 키는 덮지 않는다: 공통 골격의 번역이 기준이다. */
  if (window.PT_DICT_SUB) {
    for (var k in window.PT_DICT_SUB) {
      if (!Object.prototype.hasOwnProperty.call(DICT, k)) DICT[k] = window.PT_DICT_SUB[k];
    }
  }

  /* 앱 화면(① 웹 UI · ② 모니터 · ④ 라이브러리) 사전도 같은 자리에서 받는다.
     assets/js/i18n-app.js 가 놓고 간다. 겹치는 네 낱말(닫기·접촉력·커버리지·
     패드)은 공통 번역이 그대로 맞아서 덮지 않는다. */
  if (window.PT_DICT_APP) {
    for (var ka in window.PT_DICT_APP) {
      if (!Object.prototype.hasOwnProperty.call(DICT, ka)) DICT[ka] = window.PT_DICT_APP[ka];
    }
  }

  /* ── 토막 바꾸기 ───────────────────────────────────────────
     앱 화면의 글은 값과 함께 자바스크립트에서 이어 붙는다.

         '도달 ' + 1204 + ' / ' + 1370 + ' pt · 미도달 구간은 …'

     런타임에 이것은 텍스트 노드 하나이고 숫자가 매번 달라진다. 노드 전체를
     키로 둘 수 없으니 앞뒤 토막만 바꾼다. 사전이 통째로 맞추지 못한 노드에서만
     쓴다 — 통째로 맞는 쪽이 언제나 정확하기 때문이다.

     긴 토막부터 맞춘다. 짧은 토막이 긴 문장 안을 파고들면 반쪽짜리 영어가
     되는데, 그건 한국어로 남는 것보다 나쁘다. */
  var HAS_KO = /[가-힣]/;

  var PHRASES = (window.PT_PHRASES_APP || []).slice().sort(function (a, b) {
    var la = a[0] instanceof RegExp ? 0 : a[0].length;
    var lb = b[0] instanceof RegExp ? 0 : b[0].length;
    return lb - la;            /* 정규식은 뒤로 — 좁게 겨눈 것이라 순서가 덜 민감하다 */
  });

  /* 토막을 바꾼 결과에 한국어가 남을 수 있다 (파일 경로의 '데이터셋/' 같은 것).
     그런 노드는 다음 apply 때 다시 후보가 된다. 우리가 만든 값 그대로면
     손대지 않는다 — 어떤 토막이 제 결과물에 다시 걸려도 두 번 바뀌지 않는다. */
  var phrased = typeof WeakMap === 'function' ? new WeakMap() : null;

  function phrase(s) {
    var out = s, hit = false;
    for (var i = 0; i < PHRASES.length; i++) {
      var from = PHRASES[i][0], to = PHRASES[i][1];
      if (from instanceof RegExp) {
        from.lastIndex = 0;
        if (!from.test(out)) continue;
        from.lastIndex = 0;
        out = out.replace(from, to);
        hit = true;
      } else if (out.indexOf(from) >= 0) {
        out = out.split(from).join(to);
        hit = true;
      }
    }
    return hit ? out : null;
  }

  /* ── 글자가 픽셀에 구워진 그림 ────────────────────────────
     sub.html 이 쓰는 래스터 도해 중 한글이 박힌 것만 영문판이 있다.
     나머지(스캔·경로·충돌 같은 렌더)는 언어 중립이라 목록에 없다.
     영문판은 원본을 복사해 한글 자리만 덮고 Pretendard 로 다시 구운 것이다.
       python scripts/i18n-figures.py
     ───────────────────────────────────────────────────────── */
  var IMG_EN = {
    'wide_loop.webp':          'wide_loop.en.webp',
    'wide_path.webp':          'wide_path.en.webp',
    'wide_rl_overview.webp':   'wide_rl_overview.en.webp',
    'wide_result.webp':        'wide_result.en.webp',
    'wide_system.webp':        'wide_system.en.webp',
    'pdf_20_1746x1361.jpg':    'pdf_20_1746x1361.en.jpg'
  };

  /*  제목은 문장이 아니라 이름이라 따로 둔다*/
  var TITLES = {
    'PolyTwin — 로봇 폴리싱 공정 디지털 트윈': 'PolyTwin — Digital Twin for Robotic Polishing',
    'PolyTwin': 'PolyTwin'
  };

  /* 번역 대상 속성. alt·title 은 지금 없어도 넣어 둔다 — 나중에 붙는다 */
  var ATTRS = ['aria-label', 'placeholder', 'title', 'alt', 'aria-valuetext', 'content'];

  /* sub.html 은 페이지를 바꿀 때마다 document.title 을 다시 쓴다.
     로드 시점 값을 붙들고 있으면 EN 에서 탭 제목이 첫 페이지 이름으로 되돌아간다.
     그래서 스냅숏이 아니라 그때그때 현재 제목을 읽어 옮긴다. */
  function transTitle(t) {
    var n = norm(t);
    if (TITLES[n]) return TITLES[n];
    if (DICT[n]) return DICT[n];
    var m = /^(.*?)\s+—\s+PolyTwin$/.exec(n);   /* '페이지 이름 — '*/
    if (m && DICT[m[1]]) return DICT[m[1]] + ' — PolyTwin';
    return t;
  }

  var TITLE_KO = document.title;
  var TITLE_EN = transTitle(TITLE_KO);

  /* 우리가 쓴 영문 제목 그대로면 아직 그 페이지다. 다르면 화면이 바뀐 것이니
     새 한국어 제목을 원본으로 다시 잡는다. */
  function syncTitle() {
    if (document.title !== TITLE_EN) {
      TITLE_KO = document.title;
      TITLE_EN = transTitle(TITLE_KO);
    }
  }

  var cur = 'ko';
  var touchedText = [];   /* [텍스트노드, 원문] */
  var touchedAttr = [];   /* [요소, 속성명, 원문] */
  var touchedImg = [];    /* [img, 원래 src] */
  var mo = null;
  var pending = 0;
  var lastApply = 0;
  var GAP = 120;          /* 옵서버가 를 다시 훑는 최소 간격 (ms)*/

  /* 번역하지 않는 구역.
     사전은 낱말 단위도 담고 있어서(공정·경로·패드…) 아직 번역하지 않은
     본문에 그대로 풀면 한 문단 안에서 한국어와 영어가 섞인다. 번역이
     끝나지 않은 영역은 [data-i18n="off"] 로 명시해 통째로 건너뛴다. */
  function skipAttr(el) {
    return !!el.closest('[data-i18n="off"]');
  }
  /* KR/EN 글자 자체는 두 언어 어디서나 KR/EN 이다. 다만 그 상자의
     aria-label 은 읽어 주는 말이므로 번역한다 — 속성은 막지 않는다. */
  function skipText(el) {
    return !!(el.closest('.lang') || el.closest('[data-i18n="off"]'));
  }

  /* 텍스트 노드 순회 — script/style/textarea 와 제외 구역은 뺀다 */
  function eachText(fn) {
    var w = document.createTreeWalker(document.body, NodeFilter.SHOW_TEXT, {
      acceptNode: function (n) {
        var p = n.parentNode;
        if (!p || p.nodeType !== 1) return NodeFilter.FILTER_REJECT;
        var tag = p.nodeName;
        if (tag === 'SCRIPT' || tag === 'STYLE' || tag === 'TEXTAREA') return NodeFilter.FILTER_REJECT;
        if (skipText(p)) return NodeFilter.FILTER_REJECT;
        return n.nodeValue && n.nodeValue.trim() ? NodeFilter.FILTER_ACCEPT : NodeFilter.FILTER_REJECT;
      }
    });
    var n;
    while ((n = w.nextNode())) fn(n);
  }

  /* 그림 교체는 [data-i18n="off"] 안에서도 한다 — 본문 번역이 끝나지 않은
     것과 그림 안 글자가 한국어인 것은 별개 문제다. */
  function swapImages() {
    var imgs = document.getElementsByTagName('img');
    for (var i = 0; i < imgs.length; i++) {
      var el = imgs[i];
      var src = el.getAttribute('src');
      if (!src) continue;
      var q = src.indexOf('?');                    /* ?v= 캐시 버스터는 지킨다 */
      var path = q < 0 ? src : src.slice(0, q);
      var qs = q < 0 ? '' : src.slice(q);
      var slash = path.lastIndexOf('/');
      var file = slash < 0 ? path : path.slice(slash + 1);
      var en = IMG_EN[file];
      if (!en) continue;                           /* 이미 영문판이면 여기서 걸러진다 */
      touchedImg.push([el, src]);
      el.setAttribute('src', path.slice(0, slash + 1) + en + qs);
    }
  }

  function toEnglish() {
    syncTitle();
    swapImages();
    eachText(function (n) {
      /* 요소 단위 예외. 같은 한국어가 자리마다 다른 영어를 요구하는 곳이
         있다 — '이송' 은 계측 채널에서 Feed(이송속도)지만 픽앤플레이스
         간트에서는 Transfer 다. 그리고 좁은 상자(68px 레일)에서는
         사전의 정식 표현이 잘린다. 그 자리에만 data-en 을 단다.
         원문이 키라는 규칙은 그대로다 — 여기는 명시된 예외다. */
      var p = n.parentNode;
      /* 자식이 이 텍스트 하나뿐일 때만 — 여러 노드에 같은 값을 덮지 않는다 */
      var own = (p.getAttribute && p.childNodes.length === 1) ? p.getAttribute('data-en') : null;
      var en = own || DICT[norm(n.nodeValue)];
      /* 통째로 못 맞춘 노드만 토막 바꾸기로 넘긴다. 한글이 없으면 이미
         영어이거나 숫자뿐이라 볼 필요가 없다 — 다시 부를 때마다 걸리면
         touchedText 가 끝없이 자란다. */
      if (!en && PHRASES.length && HAS_KO.test(n.nodeValue)) {
        if (phrased && phrased.get(n) === n.nodeValue) return;   /* 이미 우리가 만든 값 */
        en = phrase(norm(n.nodeValue));
      }
      if (!en) return;
      /* 이미 그 영어라면 손대지 않는다. MutationObserver 가 apply 를 다시
         부를 때, 사전 항목은 영어가 키가 아니라 저절로 걸러지지만
         data-en 은 매번 걸려서 touchedText 가 무한히 자란다. */
      if (norm(n.nodeValue) === norm(en)) return;
      /* 앞뒤 공백은 그대로 둔다 — 인라인 요소 사이 간격이 여기서 나온다 */
      var m = /^(\s*)([\s\S]*?)(\s*)$/.exec(n.nodeValue);
      touchedText.push([n, n.nodeValue]);
      n.nodeValue = m[1] + en + m[3];
      if (phrased) phrased.set(n, n.nodeValue);
    });

    /* querySelectorAll 은 head 의 meta 까지 포함한다 */
    var all = document.querySelectorAll('*');
    for (var i = 0; i < all.length; i++) {
      var el = all[i];
      if (skipAttr(el)) continue;
      for (var a = 0; a < ATTRS.length; a++) {
        var k = ATTRS[a];
        if (!el.hasAttribute(k)) continue;
        var v = el.getAttribute(k);
        var en2 = DICT[norm(v)];
        if (!en2 || en2 === v) continue;
        touchedAttr.push([el, k, v]);
        el.setAttribute(k, en2);
      }
    }
    document.title = TITLE_EN;
  }

  function toKorean() {
    var i;
    for (i = touchedText.length - 1; i >= 0; i--) touchedText[i][0].nodeValue = touchedText[i][1];
    for (i = touchedAttr.length - 1; i >= 0; i--) touchedAttr[i][0].setAttribute(touchedAttr[i][1], touchedAttr[i][2]);
    for (i = touchedImg.length - 1; i >= 0; i--) touchedImg[i][0].setAttribute('src', touchedImg[i][1]);
    touchedText = [];
    touchedAttr = [];
    touchedImg = [];
    document.title = TITLE_KO;
  }

  /* 우리가 만든 변경이 다시 우리를 깨우지 않도록 옵서버를 끊고 적용한다 */
  function apply() {
    if (mo) mo.disconnect();
    if (cur === 'en') toEnglish(); else toKorean();
    document.documentElement.lang = cur;
    paintSwitch();
    lastApply = Date.now();
    if (mo) mo.observe(document.body, { childList: true, subtree: true, characterData: true });
  }

  /* ② 공정 모니터링은 매 프레임 수치를 다시 쓴다. 그때마다 옵서버가 깨어
      전체를 훑으면 3D 뷰와 프레임을 다툰다. 화면 글이 60번/초로 바뀔
     일은 없으므로 최소 간격을 둔다 — 눈에 띄지 않고 훑는 횟수는 8분의 1이 된다.
     첫 번째는 미루지 않는다. 새 마크업이 들어온 순간이 제일 눈에 띈다. */
  function schedule() {
    if (pending) return;
    var wait = Math.max(0, GAP - (Date.now() - lastApply));
    var run = function () { pending = 0; apply(); };
    pending = wait ? setTimeout(run, wait) : requestAnimationFrame(run);
  }

  function paintSwitch() {
    var links = document.querySelectorAll('.lang a[data-lang]');
    for (var i = 0; i < links.length; i++) {
      var l = links[i];
      var to = l.getAttribute('data-lang');
      if (to === cur) l.setAttribute('aria-current', 'true');
      else l.removeAttribute('aria-current');
      /* sub.html 의 ?p= 를 잃지 않도록 현재 주소에서 만든다 */
      var u = new URL(location.href);
      u.searchParams.set('lang', to);
      l.setAttribute('href', u.pathname + u.search + u.hash);
      l.setAttribute('hreflang', to);
    }
  }

  function set(lang) {
    lang = lang === 'en' ? 'en' : 'ko';
    if (lang === cur) { paintSwitch(); return; }
    cur = lang;
    try { localStorage.setItem(KEY, cur); } catch (e) { /* 사생활 보호 모드 */ }
    var u = new URL(location.href);
    u.searchParams.set('lang', cur);
    history.replaceState(null, '', u.pathname + u.search + u.hash);
    apply();
  }

  function init() {
    var q = new URLSearchParams(location.search).get('lang');
    var saved = null;
    try { saved = localStorage.getItem(KEY); } catch (e) { /* 무시 */ }
    cur = (q === 'en' || q === 'ko') ? q : (saved === 'en' ? 'en' : 'ko');
    try { localStorage.setItem(KEY, cur); } catch (e) { /* 무시 */ }

    document.addEventListener('click', function (e) {
      var t = e.target;
      var a = t && t.closest ? t.closest('.lang a[data-lang]') : null;
      if (!a) return;
      e.preventDefault();
      set(a.getAttribute('data-lang'));
    });

    /* auth-client 의 헤더 렌더처럼 나중에 들어오는 마크업도 잡는다.
       영어일 때만 감시한다 — 한국어는 원문 그대로다. */
    mo = new MutationObserver(function () { if (cur === 'en') schedule(); });
    apply();
  }

  if (document.readyState === 'loading') document.addEventListener('DOMContentLoaded', init);
  else init();

  window.PTI18n = {
    get lang() { return cur; },
    set: set,
    apply: apply
  };
})();
