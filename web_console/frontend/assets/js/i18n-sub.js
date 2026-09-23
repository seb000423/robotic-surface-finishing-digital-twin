/* ══════════════════════════════════════════════════════════════
   — sub.html 본문 사전

   i18n.js 와 같은 규칙이다. 화면에 보이는 한국어 원문 그대로가 키다.
   sub.html 에서만 읽는다 — 랜딩 첫 로드에 이 파일을 얹지 않으려고 나눴다.

   토막 키가 많은 이유: 본문이 `<b>813</b>개` 처럼 인라인 요소로 끊겨 있어
   텍스트 노드가 그 단위로 쪼개진다. 숫자를 사이에 두고도 말이 되게 골랐다.

   여기 없는 문장은 EN 에서도 한국어로 남는다. 본문에 문장을 추가하면
     이 파일에도 넣어라 — 한 문단 안에 두 언어가 섞이는 게 제일 나쁘다.

   숫자·고유명사(Isaac Sim · ROS 2 · Ansys · ISO/TS 15066 · Preston)는
   원문 그대로 둔다. 수치는 기획서 실측값이라 옮기면서 바꾸지 않는다.
   ══════════════════════════════════════════════════════════════ */
(function () {
  'use strict';

  window.PT_DICT_SUB = {

    /* ── 지표 이름 · 단위 ────────────────────────────────────
       stats 는 `<b>값</b><small>단위</small>` 라 단위가 따로 온다.
       '개' 는 영어에 대응어가 없어 공백 하나로 지운다.
       돈 단위의 '만' 은 10^4 이라 값과 붙여 읽도록 0 을 앞에 둔다
       (33 + '0k KRW/mo' = 330k KRW/mo). ── */
    '중앙값': 'median',
    '이송속도': 'Feed rate',
    '공구 자세': 'Tool pose',
    '범위': 'Range',
    '체류 시간': 'Dwell time',
    '개': ' ',
    '초': 's',
    '열': 'cols',
    '시점': 'views',
    '회': 'times',
    '단계': 'steps',
    '곳': 'spots',
    '만 원/월': '0k KRW/mo',
    '만 원/건': '0k KRW/report',
    '20초 구간 반복': '20 s loop',

    /* ── 노하우 → 물리량 ── */
    '디지털 트윈에서 같은 작업을 할 때': 'The same job run in the digital twin',
    '작업 종료 후 남는 물리량': 'Physical quantities left when the job ends',

    /* ── 광택 공정 6단계 ── */
    '세차': 'Wash',
    '먼지를 털어내고 앰블럼·구석의 찌든 때를 솔로 뺀다. 도장면 판정은 이 상태에서 한다':
      'Dust off, brush the grime out of emblems and corners. The paint is judged in this state.',
    '타르 · 철분 제거': 'Tar and iron removal',
    '일반 세차로 지워지지 않는 타르와 클리어층에 박힌 철분을 약품으로 녹여 뺀다':
      'Chemicals dissolve the tar and the iron embedded in the clear coat that a wash leaves behind.',
    '낙진 제거': 'Fallout removal',
    '표면에 앉은 분진을 클레이바·낙진패드로 걷어낸다. 남기면 연마 중에 표면을 긁는다':
      'A clay bar lifts the dust sitting on the surface. Left there, it scratches during polishing.',
    '마스킹': 'Masking',
    '고무·플라스틱부는 패드가 닿으면 회복되지 않는다. 연마 대상이 아닌 곳을 덮는다':
      'Rubber and plastic never recover once the pad touches them. Everything not being polished gets covered.',
    '1차 절삭 · 2차 기계자국 정리 · 3차 광도 마감. 기술이 집약되는 구간이다':
      'Cut, then clear the machine marks, then finish for gloss. This is where the skill sits.',
    '마무리': 'Finish',
    '마스킹을 떼고 틈에 낀 콤파운드를 걷어낸 뒤 탈지 또는 왁스로 마감한다':
      'Pull the masking, clear the compound out of the gaps, then degrease or wax.',
    '사람': 'Person',
    '시스템': 'System',
    '숙련공 공정 6단계 중 시스템이 맡는 구간은 하나다':
      'Of the veteran’s six steps the system takes one',
    '시스템이 맡는 구간. 나머지 다섯은 사람의 전·후처리로 남는다':
      'The step the system takes. The other five stay with the person, before and after.',
    '광택 공정 단계별 담당': 'Who does what, step by step',
    '담당': 'Owner',

    /* ── 도장면 단면 ── */
    '클리어코트': 'Clear coat',
    '연마 대상': 'Polished layer',
    '베이스코트': 'Base coat',
    '색상 층': 'Colour layer',
    '프리머': 'Primer',
    '방청 · 밀착': 'Anti-corrosion, adhesion',
    '차체 패널': 'Body panel',
    '강판': 'Steel',
    '스월 · 잔스크래치': 'Swirls and fine scratches',
    '광택으로 제거': 'Polishes out',
    '부분 손상': 'Partial damage',
    '일부만 가능': 'Only partly',
    '깊은 손상': 'Deep damage',
    '도색 영역': 'Repaint territory',
    '클리어코트까지 도달': 'reaches the clear coat',
    '베이스코트까지 도달': 'reaches the base coat',
    '차체 패널까지 도달': 'reaches the body panel',
    '광택이 되돌릴 수 있는 깊이는 클리어코트까지다':
      'Polishing can undo damage down to the clear coat and no further',
    '도장면 단면도. 위에서부터 클리어코트, 베이스코트, 프리머, 차체 패널 네 층이고 연마 대상은 클리어코트 한 층이다. 손상 01 스월·잔스크래치는 클리어코트 안에서 멈춰 광택으로 제거되고, 02 부분 손상은 베이스코트까지 내려가 일부만 가능하며, 03 깊은 손상은 차체 패널까지 닿아 도색 영역이다.':
      'Cross-section of the paint. From the top: clear coat, base coat, primer, body panel. Only the clear coat is polished. Damage 01, swirls and fine scratches, stops inside the clear coat and polishes out. 02, partial damage, reaches the base coat and is only partly recoverable. 03, deep damage, reaches the body panel and is repaint territory.',
    '손상 깊이별 판정': 'Verdict by damage depth',
    '손상': 'Damage',
    '도달 층': 'Layer reached',
    '판정': 'Verdict',

    /* ── 제조업 고령화 ── */
    '청년층': 'Under 30',
    '15–29세': 'age 15–29',
    /* sr 표는 둘을 한 칸에 이어 붙인다 — 이어진 형태도 키로 둔다 */
    '청년층 15–29세': 'Under 30 age 15–29',
    '30대': '30s',
    '40대': '40s',
    '50대 이상': '50 and over',
    '10년 동안 30대는 7.3%p 줄고 50대 이상은 14.4%p 늘었다':
      'In ten years the 30s fell 7.3 points and the 50-and-over rose 14.4',
    '제조업 근로자 연령대별 비중 변화. 2010년 대비 2020년 기준으로 청년층 21.6에서 15.7퍼센트, 30대 35.1에서 27.8퍼센트, 40대 27.7에서 26.9퍼센트, 50대 이상 15.7에서 30.1퍼센트.':
      'Share of manufacturing workers by age, 2010 against 2020. Under 30: 21.6 to 15.7 per cent. 30s: 35.1 to 27.8. 40s: 27.7 to 26.9. 50 and over: 15.7 to 30.1.',
    '2010년': '2010',
    '2020년': '2020',
    '제조업 근로자 연령대별 비중 (%)': 'Share of manufacturing workers by age (%)',
    '연령대': 'Age band',
    '증감': 'Change',

    /* ── 시뮬레이션 3세대 ── */
    '1세대': '1st gen',
    '기하 · 동선': 'Geometry and motion',
    '충돌 · 도달성 · 스케줄링': 'Collision, reachability, scheduling',
    '궤적 자동화': 'Trajectory automation',
    '6주 → 3일': '6 weeks → 3 days',
    '기아 용접 OLP 자동화': 'Kia welding OLP automation',
    '달성': 'Done',
    '2세대': '2nd gen',
    '공장 스케일': 'Factory scale',
    '다중 로봇 · 설비 · 물류 · 생산 흐름': 'Multiple robots, equipment, logistics, production flow',
    '가상 선검증 공장': 'Virtually pre-validated plant',
    'BMW 생산 계획 비용 절감 전망': 'BMW projected planning-cost savings',
    '3세대': '3rd gen',
    '물리력': 'Physical force',
    '접촉력 · 마찰 · 토크 계산 / 기록': 'Contact force, friction, torque — computed and logged',
    '힘을 다루는 공정': 'Processes that handle force',
    '국내 성과 사례 확인 안 됨': 'No domestic case confirmed',
    '접촉 공정 적용은 빈 구간 · 선점 지점': 'Contact processes are the empty slot, and the one to take',
    '미도달': 'Not reached',
    '시뮬레이션이 다루는 범위': 'What each generation of simulation covers',

    /* ── 가상 작업장 파이프라인 ── */
    '차체 형상 스캔': 'Body shape scan',
    '3D 포인트 클라우드로 형상을 받는다': 'The shape arrives as a 3D point cloud',
    '경로 생성': 'Path generation',
    '국소 평면과 법선을 추정해 경로를 깐다': 'Local planes and normals are estimated, then the path is laid',
    '힘 제어 연마': 'Force-controlled polishing',
    '접촉력을 추종하며 표면을 따라간다': 'It tracks contact force while following the surface',
    '숙련공의 검수': 'Veteran inspection',
    '결과를 사람이 보고 통과 여부를 정한다': 'A person looks at the result and decides pass or fail',
    '결과 저장': 'Save result',
    '통과한 회차의 파라미터를 자산으로 남긴다': 'Parameters from the runs that passed are kept as assets',
    '가상 작업장 · NVIDIA Isaac Sim': 'Virtual work cell · NVIDIA Isaac Sim',
    '실제와 같은 물리 조건으로 재현': 'Reproduced under real physics',

    /* ── 이송(pick and place) 사이클 ── */
    '들어 올리고, 옮기고, 내려놓는다': 'Lift it, move it, set it down',
    '인식': 'Detect',
    '파지': 'Grasp',
    '상승': 'Lift',
    '하강·해제': 'Lower and release',
    '복귀': 'Return',
    '이송 1 사이클의 구간별 점유': 'Where one transfer cycle spends its time',
    '구간이 가장 길다. 병목 구간 리포트가 짚는 것이 이 지점이다.':
      'is the longest segment. This is what the bottleneck report points at.',

    /* ── 안전 게이트 3단 ── */
    '기하 게이트': 'Geometry gate',
    '경로 필터링 5단계 — 가장자리·곡률·기울기·반경·도달성. 도달 불가 0개일 때만 통과':
      'Five path filters — edge, curvature, slope, radius, reachability. It passes only at zero unreachable points.',
    '접촉 게이트': 'Contact gate',
    '가상 스프링 순응 접촉 — 접촉력 3–8 N 대역을 벗어나는 파라미터는 실행 자체를 차단':
      'Virtual-spring compliant contact — parameters outside the 3–8 N band never run at all.',
    '대조 게이트': 'Cross-check gate',
    'FEA · 실기 F/T 2중 대조 — 동일 조건 3/5/8 N 접촉압력 대조, 오차 5% 초과 구간은 자동 승인 금지':
      'FEA and rig F/T both ways — 3/5/8 N contact pressure under identical conditions. Above 5 per cent error there is no automatic approval.',
    '실행 전 안전 게이트 3단': 'Three safety gates before the run',
    '세 게이트를 모두 통과한 파라미터만 실물 라인으로 나간다':
      'Only parameters through all three gates reach the real line',

    /* ── 안전 리스크 5종 ── 앞뒤가 <b>숫자</b> 로 끊긴다 ── */
    '티칭 중 인체 노출': 'Human exposure during teaching',
    '작업자가 셀 내부에서 근접 조작 — 품종 전환마다 반복':
      'The operator works inside the cell, up close — repeated at every changeover',
    '스캔 1회 → 웨이포인트': 'One scan →',
    '개 자동 생성 — 셀 내부 티칭': 'waypoints generated. Teaching inside the cell:',
    '이상 거동 (과압입 · 슬램)': 'Abnormal behaviour (over-press, slam)',
    '실물 시운전에서 발견 — 발견 시점에 이미 노출':
      'Found in the real trial run — by then it has already happened',
    '가상 스프링 순응 접촉 — 접촉력': 'Virtual-spring compliant contact — contact force',
    '이탈 시 실행 차단': 'outside the band blocks the run',
    '경로 이탈 · 도달 한계': 'Path departure, reach limit',
    '근접 관찰로 확인 — 급가속 구간은 사전 예측 불가':
      'Caught by watching up close — sudden acceleration cannot be predicted in advance',
    '경로 필터링 5단계 — 도달 불가': 'Five path filters — unreachable points',
    '개일 때만 실행 승인': 'is the only value that approves a run',
    '오검증 (가상 통과 → 현실 실패)': 'False validation (passes virtually, fails in reality)',
    '대조 기준 없음 — 실물이 유일한 판정자':
      'No reference to check against — the real machine is the only judge',
    'FEA · 실기 F/T 2중 대조 — 오차': 'FEA and rig F/T both ways — error above',
    '초과 구간 자동 승인 금지': 'gets no automatic approval',
    '설비 · 공작물 손상': 'Damage to equipment or workpiece',
    '과압입 변형·공구 파손을 실물에서 확인': 'Over-press deformation and tool breakage found on the real machine',
    '실행 전 검증 항목으로 편입 — 이상 수치 FEA 자동 플래그':
      'Folded into pre-run validation — outlying values get an automatic FEA flag',
    '위험 5종을 실물이 아니라 실행 전에 확인한다':
      'Five risks caught before the run, not on the machine',
    '리스크 항목': 'Risk',
    '기존 방식': 'The old way',
    '폴리트윈': 'PolyTwin',

    /* ── ISO/TS 15066 ── */
    '연마 접촉력은 규격 최저 한계의 12.3% 이하에서 작동한다':
      'Polishing contact force runs at or below 12.3 per cent of the standard’s lowest limit',
    '접촉력 축 도해. 0에서 65 뉴턴 축 위에 목표 접촉력 1.5 뉴턴, 작업 대역 3에서 8 뉴턴이 왼쪽 끝에 있고, ISO TS 15066 최저 허용 한계인 안면 65 뉴턴이 오른쪽 끝에 있다.':
      'Contact-force axis. On a 0 to 65 newton scale the 1.5 newton target and the 3 to 8 newton working band sit at the far left; the lowest ISO TS 15066 limit, 65 newtons for the face, sits at the far right.',
    '목표 1.5 N': 'Target 1.5 N',
    '작업 대역 3–8 N': 'Working band 3–8 N',
    '안면 허용 한계 65 N': 'Face limit 65 N',
    '인체 29개 부위 중 가장 낮은 한계(안면 65 N) 기준 — 3–8 N 은 그 4.6–12.3%':
      'Against the lowest of the 29 body-region limits (face, 65 N) — 3 to 8 N is 4.6 to 12.3 per cent of it',
    'ISO/TS 15066 대비 접촉력': 'Contact force against ISO/TS 15066',
    '목표 접촉력': 'Target contact force',
    '1.5 N — 한계의 2.3%': '1.5 N — 2.3% of the limit',
    '작업 대역': 'Working band',
    '3–8 N — 한계의 4.6–12.3%': '3–8 N — 4.6 to 12.3% of the limit',
    '규격 최저 허용 한계': 'Lowest limit in the standard',
    '안면 65 N (ISO/TS 15066:2016 Annex A)': 'Face, 65 N (ISO/TS 15066:2016 Annex A)',

    /* ── 오류 대비와 복구 ── */
    '시뮬레이션 비정상 종료': 'Simulation crashes',
    '처음부터 다시 실행': 'Start the whole run again',
    '스캔 / 경로 생성 / 실행 로그를 단계별로 분리 저장 —':
      'Scan, path generation and run logs are stored separately by stage —',
    '중단 지점부터 재개': 'resume from where it stopped',
    '통신 두절': 'Link drops',
    '영상이 끊기면 제어도 멈춤': 'If the video stops, control stops too',
    '제어 통신과 시각화 통신 분리 — 영상 오류가':
      'Control and visualisation run on separate links — a video fault has',
    '제어에 영향 없음': 'no effect on control',
    '멈춰도 처음으로 돌아가지 않는다': 'A stop does not send you back to the start',
    '상황': 'Situation',
    '대비 없을 때': 'With no provision',

    /* ── 가격 정책 ── */
    '중견기업': 'Mid-size firms',
    '월 구독 (SaaS) — 1차 유료 고객': 'Monthly subscription (SaaS) — the first paying customers',
    '월 33만 원 · 연 396만 원': '330,000 KRW a month, 3.96M a year',
    '라인을 자주 바꿔야 하나 실패를 감당하기 어려운 구간 — 발주 전 구성안 반복 검증':
      'They change lines often but cannot absorb a failure — layouts get validated over and over before the order',
    '중소기업': 'Small firms',
    '건당 과금': 'Per report',
    '검증 리포트 1건 250만 원': '2.5M KRW per validation report',
    '설비 확보가 어려워 실물 검증 곤란 — 발주 전 투자 판단':
      'Hard to get the equipment, so hard to validate for real — the call is made before the order',
    '대기업': 'Large firms',
    '프로젝트 컨설팅': 'Project consulting',
    '프로젝트 단위 협의': 'Agreed per project',
    '자체 R&D 조직 보유 — 신차 R&D 공정적용 부서 대상':
      'They have their own R&D — the buyer is the team applying processes to new models',
    '공통': 'All tiers',
    '초기 셋업비 (별도)': 'One-off setup fee (separate)',
    '120만 원': '1.2M KRW',
    '차종 모델 등록·좌표계 정렬 — 특급기술자 3일':
      'Model registration and frame alignment — three days of a senior engineer',
    '기업 규모별 3종 — 전부 공개 가격이다': 'Three tiers by company size, every price published',

    /* ── 리포트 원가 산정 ── */
    '엔지니어 공수': 'Engineer time',
    '특급기술자 401,407원 × 3일 — 엔지니어링업체 임금실태조사(국가승인통계 제372001호)':
      'Senior engineer 401,407 KRW × 3 days — engineering wage survey (national statistic no. 372001)',
    '1,204,221원': '1,204,221 KRW',
    '시뮬레이션 연산비': 'Compute',
    'GPU 인스턴스 $1.006/h × 20h × 환율 1,390원': 'GPU instance $1.006/h × 20 h at 1,390 KRW',
    '28,000원': '28,000 KRW',
    '개발비 상각': 'Amortised development',
    '초기 개발비 1억 6,500만 원 ÷ 240건 (3년)': '165M KRW of initial development ÷ 240 reports (3 years)',
    '687,500원': '687,500 KRW',
    '마진 30%': '30% margin',
    '제조업 평균 마크업 기준': 'At the manufacturing average markup',
    '575,916원': '575,916 KRW',
    '0.2%는 근거가 아니라 산정 결과다 — 상향식 합산':
      'The 0.2% is not the reason, it is the result — added up from the bottom',
    '리포트 1건 판매가': 'Price of one report',
    '설비투자(평균 18억 원)의 0.2% 미만 — 수억 원 설비의 발주 전 판단 비용':
      'Under 0.2% of the capital outlay (1.8bn KRW on average) — the cost of deciding before ordering',
    '약 250만 원': 'about 2.5M KRW',

    /* ── 유사 서비스 비교 ── */
    '도입 비용': 'Cost to adopt',
    '상용 OLP 라이선스 $15,000–100,000': 'Commercial OLP licence, $15,000–100,000',
    '월': 'a month',
    '33만 원': '330,000 KRW',
    '— 약': '— about',
    '힘 제어 검증': 'Force-control validation',
    '미지원 — 경로만 다룬다': 'Not supported — path only',
    '지원': 'Supported',
    '— 접촉력 검증까지 포함': '— contact force included',
    '곡면 경로 생성': 'Paths on curved surfaces',
    '수동 티칭 병행': 'Manual teaching alongside',
    '스캔 기반': 'From the scan',
    '자동 생성': 'generated automatically',
    '로봇 브랜드': 'Robot brand',
    '전용 또는 제한적 지원': 'Vendor-specific or limited',
    '무관': 'Irrelevant',
    '적용 가능 기업': 'Who can use it',
    '대기업 중심': 'Mostly large firms',
    '중소 ~ 대기업': 'Small through large',
    '어디로 갈지(경로)는 다들 다룬다. 얼마나 누를지(힘)는 우리만 다룬다':
      'Everyone handles where to go. Only we handle how hard to press.',
    '비교 항목': 'Compared on',
    '상용 OLP · SI 용역': 'Commercial OLP / SI work',

    /* ── 시장 규모 ── */
    '로봇 기반 표면처리 시장 전체': 'The whole robotic surface-finishing market',
    '31.8억 달러': '$3.18bn',
    '하드웨어를 포함한 최대 범위': 'The widest reading, hardware included',
    '연마·샌딩 공정 — 검증 SW 레이어': 'Polishing and sanding — the validation software layer',
    '3.5억 달러': '$350M',
    '폴리트윈이 접근 가능한 영역': 'What PolyTwin can reach',
    '초기 진입 목표 구간': 'The first slice we aim at',
    '25만 달러': '$250,000',
    'SAM 의 0.07% — 통상 기준(1–5%) 대비 보수적 적용':
      '0.07% of SAM — conservative against the usual 1–5%',
    '시장 규모 — 2026년 기준': 'Market size, 2026',

    /* ── 확장 로드맵 4단계 ── */
    '1단계': 'Stage 1',
    '자동차 차체 연마': 'Car body polishing',
    '스캔 → 경로 → 힘 제어 → 리포트 통합 — 웨이포인트 813개 · 접촉력 3–8 N 유지':
      'Scan → path → force control → report, joined up. 813 waypoints, contact force held at 3–8 N.',
    '구현 완료': 'Built',
    '2단계': 'Stage 2',
    '유리 · 위생도기 · 케이스 · 금형': 'Glass, sanitaryware, casings, moulds',
    '재질별 Preston 계수 k 를 시험 가공으로 캘리브레이션한 뒤 파라미터 프로파일로 등록':
      'Calibrate the Preston coefficient k per material on test cuts, then register it as a parameter profile',
    '개발 예정': 'Planned',
    '3단계': 'Stage 3',
    '숙련공 실측 데이터 연계': 'Measured veteran data',
    '모션캡처 실측 / Isaac Sim 대조 검증 — 재현 오차가 허용 범위에 수렴해야 성립':
      'Motion capture against Isaac Sim — it only holds if the reproduction error converges inside tolerance',
    '4단계': 'Stage 4',
    '적응형 자율 연마 · 타 공정': 'Adaptive autonomous polishing and other processes',
    '표면 상태 입력 → 파라미터 자동 조정 — 디버링 · 샌딩 · 도장 · 실링으로 확장':
      'Surface state in, parameters adjusted automatically — extending to deburring, sanding, painting, sealing',
    '가장 어려운 공정에서 절차를 세우고, 접촉하는 공정 전반으로 넓힌다':
      'Set the procedure on the hardest process, then widen it across everything that touches',

    /* ── 숙련 기술 전승 비교 ── */
    '전수 매체': 'How it is passed on',
    '어깨너머 구전·시범 — 말로 옮기면 수치가 남지 않음':
      'Watching over a shoulder, told and shown — put into words, no numbers survive',
    '17열': '17 columns',
    '물리량 로그가 곧 교재 — 접촉력·이송 속도·자세·체류 시간':
      'The log is the textbook — contact force, feed rate, pose, dwell time',
    '숙련 소요': 'Time to competence',
    '신입 숙련까지 3–5년 — 숙련공 이탈 시 처음부터 재시작':
      'Three to five years for a newcomer — when a veteran leaves it starts over',
    '정답 파라미터가 이미 등록됨 — 경험 의존 구간 축소':
      'The right parameters are already on file — less rides on experience',
    '실습 환경': 'Where you practise',
    '실물 설비 필요 — 수억 원대 투자': 'Real equipment required — hundreds of millions of won',
    '시뮬레이션에서 전 과정 실습 — 스캔·경로·힘 제어·리포트':
      'The whole thing in simulation — scan, path, force control, report',
    '숙련공의 역할': 'The veteran’s role',
    '은퇴와 함께 소멸 — 재현 여부를 확인할 수단 없음':
      'Gone at retirement, with no way to check what was reproduced',
    '교재의': 'Author of the textbook and',
    '저자이자 검증 기준': 'the standard it is checked against',
    '— 매 루프에서 계속 쓰임': '— used again on every loop',
    '직무 성격': 'The job itself',
    '분진·소음 3D 기피 직무 — 청년 유입 단절': 'Dust and noise, a job the young avoid — the pipeline is cut',
    '시뮬레이션 운영·검증 —': 'Running and validating simulations —',
    'AX 엔지니어': 'an AX engineer',
    '직무로 연결': 'role',
    '구전 전수에서 데이터 기반 교육으로': 'From word of mouth to training on data',

    /* ── 보안 4단계 ── */
    '정보자산 식별 · 위험평가': 'Identify information assets, assess risk',
    '핵심 기술 로그와 운영 이벤트 로그를 자산으로 등록하고 위험을 평가한다':
      'Core technical logs and operational event logs are registered as assets and the risk is assessed',
    '정보보호 정책 수립': 'Write the security policy',
    '로그 이원화·역할별 권한(조회/설정/실행) 등 통제 항목을 정책으로 고정한다':
      'Split logging and per-role rights (view, configure, run) are fixed as policy',
    '내부 심사': 'Internal audit',
    '정책 대비 운영 실태를 자체 점검한다': 'Practice is checked against the policy in-house',
    '인증 심사': 'Certification audit',
    'ISO 27001 Annex A 통제항목 기준 외부 심사': 'External audit against ISO 27001 Annex A controls',
    '데이터 보안 대응 4단계': 'Four stages of data security',
    'ISO 27001 Annex A 통제항목 기준': 'Against ISO 27001 Annex A controls',

    /* ── 팀 ── */
    '김두용': 'Kim Du-yong',
    '팀장 — 기획·사업화 / 힘 제어·제거량 모델':
      'Lead — planning and commercialisation / force control and removal model',
    '가상 스프링·감쇠 순응 접촉 설계(k=500) · Preston 제거량 모듈 · Ansys FEA 이중 검증 · 접촉력 1.5 N 추종 오차 검증':
      'Virtual spring-damper compliant contact (k=500), Preston removal module, dual Ansys FEA validation, 1.5 N tracking-error checks',
    '신현호': 'Shin Hyeon-ho',
    '로봇 협동 제어 / 네트워크 통신': 'Multi-robot control / networking',
    '레일·측면(SL/SR)·천장(C) 배치와 담당 영역 자동 배분 · ROS 2–rosbridge 실시간 통신 · RMPFlow 추종 지연 보정 · 기획서 구조 개편':
      'Rail, side (SL/SR) and ceiling (C) placement with automatic area assignment, ROS 2 and rosbridge in real time, RMPFlow lag compensation, proposal restructure',
    '부승언': 'Bu Seung-eon',
    'UI / 대시보드 · 데이터 시각화': 'UI / dashboards and data visualisation',
    '실행 전·중·완료 3화면 대시보드 · 실시간 접촉력 그래프·제거량 히트맵 · Blind Spot 커버리지 뷰 · 17열 로깅·CSV 내보내기':
      'Three dashboards for before, during and after the run, live contact-force chart and removal heatmap, Blind Spot coverage view, 17-column logging and CSV export',
    '정용준': 'Jeong Yong-jun',
    '3D 스캔 · 경로 계획': '3D scanning / path planning',
    '9시점 깊이 카메라 배치 · 포인트 클라우드 ICP 정합 · 표면 법선 추정 · 래스터 경로 813 웨이포인트 자동 생성·필터링 5단계':
      'Nine-view depth camera layout, ICP point-cloud registration, surface normal estimation, 813 raster waypoints generated automatically with five filter stages',
    '넷이서 스캔부터 힘 제어까지 — 역할 경계는 절대적이지 않다':
      'Four of us, from the scan to force control — the boundaries are not absolute',

    /* ── 숙련공 대조 리포트 ── */
    '정상상태 힘 변동계수 CV': 'Steady-state force CV',
    '합성 숙련공 기준(9.96%)의 약 1/3': 'About a third of the synthetic veteran baseline (9.96%)',
    '상회': 'Above target',
    '목표대역 체류율 (1.2–1.8 N)': 'Time in the target band (1.2–1.8 N)',
    '정상 폴리싱 구간 기준': 'Over the steady polishing segment',
    '과압(>1.8 N) 발생': 'Over-pressure (>1.8 N) events',
    '도장면 손상 위험이 구조적으로 0': 'Structurally zero risk of damaging the paint',
    '전이구간 CV (진입·이탈)': 'Transition CV (entry and exit)',
    '접촉의 11.3% — 가장자리 품질 편차로 이어질 구간':
      '11.3% of contact — the part that turns into edge-quality variance',
    '남은 과제': 'Still open',
    '접촉 유지율': 'Contact retention',
    '작업 시간의 1/4은 접촉이 끊겨 있었다': 'A quarter of the working time had no contact at all',
    '연마 중 힘 제어는 기준을 상회한다 — 남은 과제는 진입·이탈이다':
      'Force control beats the baseline while polishing. Entry and exit are what is left.',
    '대조 기준은 문헌 기반 합성 참조선 — 폴리싱 숙련공의 접촉력 공개 데이터셋은 존재하지 않는다 (2026-08 기준)':
      'The baseline is synthetic, built from the literature — no public contact-force dataset for polishing veterans exists (as of August 2026)',

    /* ── 로그 이원화 ── */
    '같은 공정에서 나오는 로그를 둘로 나눈다': 'One process, two kinds of log',
    '공유 가능 데이터': 'Shareable',
    '협력사·경쟁사에 공유': 'Shared with suppliers and competitors',
    '공정 중 발생한 이벤트 관련 기록': 'Records of events during the run',
    '이벤트 로그 — status 0 은 정상': 'Event log — status 0 is normal',
    '공유 불가 데이터': 'Not shareable',
    '고객사의 자산': 'The customer’s asset',
    '관절 각도, 궤적 등 민감 정보 기록': 'Joint angles, trajectories and other sensitive records',
    '스텝 로그 — 접촉력과 관절각은 밖으로 나가지 않는다':
      'Step log — contact force and joint angles never leave',

    /* ── 자산화 · 권한 ── */
    '통과 정책만 기술 자산으로 등록': 'Only policies that pass are registered as assets',
    '04 자산화': '04 Asset registry',
    '전문가 점수와 판정 — 조합 5종': 'Expert score and verdict, five combinations',
    '조합': 'Combo',
    '전문가 점수': 'Expert score',
    '등록': 'Kept',
    '제외': 'Dropped',
    '공정별 정책 · 조건 DB 등록': 'Registered in the per-process policy and condition DB',
    '재사용 가능한 디지털 자산 구축': 'A reusable digital asset',
    '협력사 A': 'Supplier A',
    '협력사 B': 'Supplier B',
    '조회 / 설정 / 제어 권한을 나눠 상황에 따라 범위를 조절한다':
      'View, configure and control are split so the scope can be tuned to the situation',
    '권한 관리 예시': 'Access, by role',
    '역할': 'Role',
    '조회': 'View',
    '설정': 'Configure',
    '제어': 'Control',
    /* 표 안의 판정 칩. JS 삼항 안에 있어서 i18n-check.py 가 못 본다
       (`${...}` 를 지우고 검사하는 방식이라 데이터 리터럴은 빠진다).
       빠져 있던 동안 EN 화면의 권한 표만 한국어로 남아 있었다. */
    '허용': 'Allowed',
    '차단': 'Blocked',

    /* ── 강화학습 환경 ── */
    '3종': '3 kinds',
    '표면 형상': 'Surface shape',
    '평면': 'Flat',
    '곡면': 'Curved',
    '복합면': 'Compound',
    '압력 범위 (N)': 'Pressure range (N)',
    '속도 범위 (RPM)': 'Speed range (RPM)',
    '폴리싱 환경의 변수를 조합해 학습 환경을 늘린다':
      'Combining the variables of a polishing cell multiplies the training environments',
    '01 환경 생성': '01 Environment build',
    '48개': '48',
    '총 환경 수': 'environments in total',
    '총 48개 환경에서 강화학습 데이터 수집 및 모델 학습 수행':
      'Data is collected and the model trained across all 48 environments',
    '학습 회차가 늘수록 숙련공 대비 검증 오차가 가파르게 줄다가 완만해지는 감쇠 곡선. 형태 예시이며 회차별 실측값은 아니다.':
      'A decay curve: validation error against the veteran falls steeply with training, then flattens. Shape only, not measured per epoch.',
    '48개 환경에서 보상을 최대화하는 정책을 학습한다':
      'The policy that maximises reward across the 48 environments is learned',
    '02 정책 학습': '02 Policy training',
    '정책': 'Policy',
    '행동 a →': 'action a →',
    '폴리트윈 환경': 'PolyTwin env',
    '48종': '48 kinds',
    '← 상태 s · 보상 r': '← state s, reward r',
    '보상 함수': 'Reward function',
    '= 접촉력 + 표면 거칠기 목표 – 사이클 타임': '= contact force + roughness target – cycle time',
    '학습이 진행될수록 숙련공 대비 검증 오차 ↓ — 곡선은 형태 예시다':
      'Validation error against the veteran falls as training runs — the curve is shape only',
    '경로 적절성': 'Path fitness',
    '연마 균일성': 'Polish uniformity',
    '접촉 안정성': 'Contact stability',
    '작업 품질': 'Work quality',
    '숙련공이 연마 결과를 보고 평가한다': 'A veteran looks at the polished result and scores it',
    '03 전문가 평가': '03 Expert review',
    '네 항목을 모두 통과한 정책만 자산화 단계로 넘어간다':
      'Only a policy that clears all four moves on to registration',

    /* ── 잔여 버 리포트 ── */
    '검사 지점': 'Inspection point',
    '양호': 'Clear',
    '잔여 버 있음': 'Burr remaining',
    '재검 필요': 'Re-check',
    '상단 좌측 홀 내부 모서리': 'Upper-left hole, inner edge',
    '좌측 수직 모서리': 'Left vertical edge',
    '하단 좌측 홀 모서리': 'Lower-left hole edge',
    '중앙 큰 홀 모서리': 'Centre large hole edge',
    '우측 외곽 홀 모서리': 'Right outer hole edge',
    '검사 지점 4곳 중 1곳에 버가 남았다': 'One of four inspection points still has a burr',
    '로드맵 4단계 · 형식 예시': 'Roadmap stage 4 · format example',
    '검사 결과 요약': 'Summary',
    '항목': 'Item',
    '지점 수': 'Points',
    '세부 검사 목록': 'Point by point',
    '리포트 형식은 폴리싱과 공유한다 — 수치는 실측이 아니다':
      'The report format is shared with polishing — the numbers are not measured',

    /* ── 실시간 공정 확인 ── */
    '패널 1': 'Panel 1',
    '패널 2': 'Panel 2',
    '패널 3': 'Panel 3',
    '패널 4': 'Panel 4',
    'Z-힘 추이': 'Z-force trend',
    '토크 추이': 'Torque trend',
    '공정 ID': 'Run ID',
    '로봇': 'Robot',
    '툴': 'Tool',
    '소재': 'Material',
    '속도': 'Speed',
    '하중': 'Load',
    '폴리싱 중 — 현재 진행률 11% · 경과 시간 02:44':
      'Polishing — 11% done, 02:44 elapsed',
    '05 실시간 공정 확인': '05 Live process view',
    '패널별 커버리지': 'Coverage by panel',
    '패널': 'Panel',
    '실행 조건': 'Run conditions',
    '현재값은 기획서 화면의 값이다. 파형은 형태 예시다':
      'The values are the ones on the proposal screen. The waveform is shape only.',

    /* ── 시장 성장률 ── */
    '산업용 로봇 시장': 'Industrial robot market',
    '하드웨어': 'Hardware',
    '디지털 트윈 시장': 'Digital twin market',
    '검증 소프트웨어': 'Validation software',
    '연평균 성장률 비교. 산업용 로봇 시장 하드웨어 11.7퍼센트, 디지털 트윈 시장 검증 소프트웨어 37.6퍼센트로 약 3.2배.':
      'Compound annual growth. Industrial robot hardware 11.7 per cent, digital twin validation software 37.6 per cent — about 3.2 times.',
    '검증 소프트웨어가 로봇 하드웨어보다 3.2배 빠르게 큰다':
      'Validation software is growing 3.2 times faster than robot hardware',
    '로봇 표면처리 시장은 31.8억 달러로 이미 크다. 폴리트윈은 그 위에서 더 빠르게 크는 쪽에 있다':
      'The robotic surface-finishing market is already $3.18bn. PolyTwin sits on the half of it that is growing faster.',

    /* ── 힘 제어 그래프 ── */
    '초기 실행 984 스텝의 접촉력 추이. 시작 직후 1.0 N 대로 올라와 목표선 1.5 N 아래에서 대체로 1.0에서 1.5 N 사이를 오가고, 300스텝 부근에서 0.45 N 까지 한 번 떨어진다.':
      'Contact force over the first 984 steps. It climbs to around 1.0 N straight away and mostly moves between 1.0 and 1.5 N under the 1.5 N target line, dropping once to 0.45 N near step 300.',
    '접촉력은 목표 1.5 N 을 넘지 않고 대역 안에 머문다':
      'Contact force stays inside the band and never passes the 1.5 N target',
    'x축 — 초기 실행 984 스텝': 'x axis — the first 984 steps',

    /* ── 충돌 · Preston · Blind Spot ── */
    '두 팔의 작업 반경이 겹치는 지점을 실행 전에 표시한다':
      'Where the two arms’ working radii overlap is marked before the run',
    '로봇 팔 두 대가 마주 보고 같은 패널을 연마하는 장면. 두 팔의 작업 반경이 만나는 가운데 지점에 표적 표시가 있다.':
      'Two robot arms facing each other, polishing the same panel. A target marker sits where their working radii meet.',
    '충돌 위험': 'Collision risk',
    '제거량은 접촉 압력과 상대 속도의 곱에 비례한다':
      'Removal is proportional to contact pressure times relative speed',
    '회전하는 연마 패드가 원판 위를 누르는 도해. 누르는 힘 P 와 상대 속도 V 가 표시되고 아래에 MRR = kp · P · V 식이 있다.':
      'A rotating polishing pad pressing on a disc. The pressing force P and relative speed V are marked, with MRR = kp · P · V below.',
    '연마 패드': 'Polishing pad',
    '임계 곡률반경': 'Critical radius of curvature',
    '패드 반경과 같다': 'Equal to the pad radius',
    '미도달 면적': 'Unreached area',
    '진단 대상 면 기준': 'Of the surface under diagnosis',
    '검출 부위': 'Sites found',
    '실행 전 좌표로 특정': 'Pinned to coordinates before the run',
    '곡률이 급한 부위의 연마 난이도를 역이용해 미연마 위험 부위를 실행 전에 특정한다':
      'The difficulty of polishing tight curvature is turned around to name the at-risk spots before the run',
    '패드 지름 150 mm — 표면 곡률반경이 패드 반경(':
      '150 mm pad — where the surface radius of curvature is smaller than the pad radius (',
    ')보다 작으면 패드가 오목·엣지 구간에 걸쳐 떠서 닿지 않는다. 그 좌표를 Blind Spot 으로 기록한다':
      '), the pad bridges the concave and edge sections and never touches. Those coordinates are logged as a Blind Spot.',

    /* ── 우리의 목표 ── */
    '숙련공의 노하우를 물리량으로 남긴다': 'Keeping a veteran’s know-how as physical quantities',
    '"이 정도 힘으로 눌러" 는 기록이 아니다. 접촉력·이송속도·공구 자세·체류 시간 — 노하우의 실체는 측정할 수 있는 물리량이고, 디지털 트윈에서는 그 값이 남는다.':
      '"Press about this hard" is not a record. Contact force, feed rate, tool pose, dwell time — know-how is made of quantities you can measure, and in the digital twin those values stay.',
    '두 시장의 연평균 성장률 — 로봇 하드웨어 11.7% 대 검증 소프트웨어 37.6%':
      'Compound annual growth of two markets — robot hardware 11.7% against validation software 37.6%',
    '현장의 기록은 말로만 남는다': 'On the floor, the record is only ever spoken',
    '사람이 연마할 때는 센서를 부착한 채 작업하지 않는다. 작업이 끝난 뒤 남는 측정값은 0개다.':
      'Nobody polishes with sensors strapped on. When the job ends, the number of measurements left is zero.',
    '"손끝에 걸리는 느낌이 사라질 때까지"': '"Until it stops catching on your fingertips"',
    '"곡면에서는 살살, 평면은 쫙"': '"Gentle on the curves, firm on the flats"',
    '작업 종료 후 남는 물리량 0개': 'Physical quantities left when the job ends: zero',
    '작업 종료 후 남는 물리량 — 수치는 기획서 기록값, 파형은 형태 예시':
      'Physical quantities left when the job ends — figures from the proposal, waveform is shape only',
    '숙련 인력은 줄고, 대체는 오지 않는다': 'The skilled are thinning out and no one is replacing them',
    '제조업 근로자 중 50대 이상 비중은 10년 사이 15.7%에서 30.1%로 늘었다. 한국 제조업 평균연령 상승세는 주요국 중 가장 가파르다.':
      'The share of manufacturing workers aged 50 and over went from 15.7% to 30.1% in ten years. Korean manufacturing is ageing faster than any comparable country.',
    '50대 이상 비중 15.7% → 30.1% (2010→2020)': '50 and over: 15.7% → 30.1% (2010→2020)',
    '30대 비중 –7.3%p': '30s: –7.3 points',
    '한·미·일 중 고령화 속도 1위': 'Ageing fastest of Korea, the US and Japan',
    '제조업 근로자 연령대별 비중 · 2010 → 2020 · 산업통상부 보도자료(2026-04-28)':
      'Manufacturing workers by age, 2010 → 2020 · Ministry of Trade, Industry and Energy release (2026-04-28)',
    '시뮬레이션은 아직 힘을 다루지 못한다': 'Simulation still cannot handle force',
    '움직임 검증(1세대)과 공장 규모 시뮬레이션(2세대)은 성과가 확인됐다. 접촉력·마찰까지 재현하는 3세대는 국내 성과 사례가 확인되지 않은 빈 구간이다.':
      'Motion validation (1st gen) and factory-scale simulation (2nd gen) have proven results. The 3rd generation, which reproduces contact force and friction, is an empty slot with no confirmed domestic case.',
    '1세대 궤적 자동화 — 달성': '1st gen, trajectory automation — done',
    '2세대 가상 선검증 공장 — 달성': '2nd gen, virtually pre-validated plant — done',
    '3세대 힘을 다루는 공정 — 선점 지점': '3rd gen, processes that handle force — the one to take',
    '시뮬레이션 세대별 범위와 국내 성과 — 3세대는 확인된 사례 없음':
      'What each generation covers and what has landed domestically — nothing confirmed for the 3rd',
    '그 빈 구간이 PolyTwin의 자리다': 'That empty slot is where PolyTwin sits',
    '물리 조건을 재현한 가상 작업장에서 폴리싱을 실행하고, 잘 된 회차의 파라미터를 저장해 다음 공정에서 다시 꺼낸다.':
      'Polishing runs in a work cell that reproduces the physics; the parameters from the runs that went well are saved and pulled back out on the next job.',
    '작업 종료 후 남는 물리량 17개': 'Physical quantities left when the job ends: 17',
    '접촉력 중앙값 1.50 N 단위로 기록': 'Median contact force logged to 1.50 N',
    '공정별 정책·조건 DB로 자산화': 'Registered in the per-process policy and condition DB',
    '가상 작업장 파이프라인 — 10단계 중 04·05·06·07·10':
      'The virtual work cell pipeline — steps 04, 05, 06, 07 and 10 of ten',

    /* ── 우리의 가치관 ── */
    '팀 CACADACA. 넷이서 스캔부터 힘 제어까지, 한 공정을 끝까지 파고든다.':
      'Team CACADACA. Four of us, from the scan through force control, digging all the way into one process.',
    '남는 것은 문서와 값이다 — 팀 CACADACA': 'What is left is the documents and the values — team CACADACA',
    '고객사의 공정은 고객사의 것이다': 'The customer’s process belongs to the customer',
    '같은 실행에서 나온 로그라도 성격이 다르다. 언제 무엇을 했는지는 나눌 수 있지만, 어떤 각도로 얼마의 힘을 줬는지는 공정 그 자체다. 저장하는 단계에서부터 둘을 갈라 놓는다.':
      'Logs from the same run are not all alike. When something happened can be shared; at what angle and under how much force is the process itself. We split the two at the point of storage.',
    '이벤트 로그는 공유, 접촉력·관절각은 분리 보관': 'Event logs shared, contact force and joint angles held separately',
    '조회·설정·제어 3단계 권한 · ISO 27001 기준 대응': 'View / configure / control — three permission tiers, aligned to ISO 27001',
    '로그 이원화 설계 — 공유 가능 데이터와 고객사 자산':
      'Split logging by design — what can be shared and what is the customer’s',
    '검증 없이는 주장하지 않는다': 'Nothing is claimed without validation',
    '시뮬레이션 파라미터는 Ansys 물리 환경과 교차 검증하고, 스캔→경로→폴리싱 엔드투엔드를 헤드리스로 교차검증한다.':
      'Simulation parameters are cross-checked against Ansys, and scan → path → polishing is cross-checked end to end, headless.',
    'Ansys FEA 교차 검증 파이프라인': 'Ansys FEA cross-check pipeline',
    '엔드투엔드 통합 실행 검증': 'End-to-end integrated run validation',
    '기획서 — Ansys 사전 검증': 'From the proposal — Ansys pre-validation',
    '숙련공이 최종 심사위원이다': 'The veteran is the final judge',
    'AI 가 만든 정책도 숙련공이 연마 결과를 보고 평가한다. 통과한 정책만 기술 자산으로 등록된다.':
      'Even a policy the AI produced is scored by a veteran looking at the polished result. Only what passes is registered as an asset.',
    '경로 적절성·연마 균일성·접촉 안정성·작업 품질 평가':
      'Scored on path fitness, polish uniformity, contact stability and work quality',
    '기준 미달 정책은 제외': 'Policies below the bar are dropped',
    '전문가 평가 단계': 'The expert review stage',
    '팀 CACADACA': 'Team CACADACA',
    '정용준·신현호·부승언·김두용. 언젠가 실물 로봇 팔에도 적용해 볼 생각이다.':
      'Jeong Yong-jun, Shin Hyeon-ho, Bu Seung-eon, Kim Du-yong. Someday we want to run this on a real robot arm.',
    '스캔·정합 / 경로 생성 / 힘제어·물리 / 협동 제어·UI 분담':
      'Split across scanning and registration, path generation, force control and physics, multi-robot control and UI',
    '팀 구성 및 역할 — 기획서 5.2 [표 7] · 소스코드는 팀 GitHub(ddoo0922/rokey_Corp3)에서 공동 관리':
      'Team and roles — proposal 5.2 [table 7] · the source is managed together on the team GitHub (ddoo0922/rokey_Corp3)',

    /* ── 우리의 성과 ── */
    '사람이 연마하고 나면 남는 측정값은 0개다. 같은 공정을 트윈에서 돌려 17열을 남겼다 — 정상 구간 접촉력 변동계수 3.23%, 목표대역 체류 100%, 과압 0회. 합성 숙련공 기준선(9.96%)의 약 1/3이다. 2026년 7–9월, 대표 항목 22개 중 완료 12 · 진행 5 · 예정 5. 수치는 전부 실측이다.':
      'When a person finishes polishing, zero measurements remain. We ran the same process in the twin and kept 17 columns — contact-force CV of 3.23% through the steady-state segment, 100% time in the target band, zero overpressure events. About a third of the synthetic veteran baseline (9.96%). July to September 2026: of 22 headline items, 12 done, 5 in progress, 5 planned. Every figure is measured.',
    '결과 분석 — 도달 영역 · 예상 공정 시간 · 권장 파라미터':
      'Result analysis — reached area, estimated cycle time, recommended parameters',
    '자동 생성 웨이포인트': 'waypoints generated automatically',
    'RMPFlow 3대 동시 실행 –33%': 'three RMPFlow robots at once, –33%',
    '힘 제어 목표 접촉력': 'target contact force under force control',
    '전 스텝 폴리싱 로깅 체계': 'logged on every polishing step',
    '스캔·정합 — 완료': 'Scan and registration — done',
    '9시점 깊이 카메라 배치·ICP 정합, 배경 제거와 표면 복원까지. 정합 오차·누락면(Blind Spot) 검출을 검증했다.':
      'Nine-view depth camera layout with ICP registration, through background removal and surface reconstruction. Registration error and missing-surface (Blind Spot) detection were validated.',
    '9시점 포인트 클라우드 정합': 'Nine-view point cloud registration',
    'Blind Spot 검출 검증': 'Blind Spot detection validated',
    '스캔 — 3D 포인트 클라우드': 'Scan — 3D point cloud',
    '경로 생성 — 완료': 'Path generation — done',
    '패드 간격 산정(0.09/0.05/0.03 m)과 래스터 경로 자동 생성. 경로 필터링 5단계와 기하·IK 도달성·충돌 간섭을 검증했다.':
      'Pad spacing worked out (0.09/0.05/0.03 m) and raster paths generated automatically. Five filter stages plus geometry, IK reachability and collision were validated.',
    '웨이포인트 813개 자동 생성': '813 waypoints generated automatically',
    '150 mm 패드 40% 중첩 기준': 'At 40% overlap on a 150 mm pad',
    '차체 국소 평면·법선 추정': 'Local plane and normal estimation on the body',
    '힘 제어·물리 — 검증 진행': 'Force control and physics — validation in progress',
    '가상 스프링·감쇠 순응 접촉과 Preston 제거량 모델을 구현하고 Ansys FEA 와 교차 검증했다. 임계값 튜닝이 진행 중이다.':
      'Virtual spring-damper compliant contact and the Preston removal model were built and cross-checked against Ansys FEA. Threshold tuning is under way.',
    'Preston 모델 F = k·침투 – c·속도, k=500': 'Preston model F = k · penetration – c · speed, k=500',
    '목표 1.5 N / 유지 3–8 N 검증 중': 'Target 1.5 N, hold 3–8 N — under validation',
    '접촉력 추종 — 초기 실행 984 스텝, 목표 1.5 N':
      'Contact-force tracking — the first 984 steps, target 1.5 N',
    '협동 제어·모니터링 — 완료': 'Multi-robot control and monitoring — done',
    '레일·측면·천장 로봇 배치와 로봇별 담당 영역 자동 배분, ROS 2 실시간 통신. 3대 동시 실행 병렬화로 사이클을 33% 줄였다.':
      'Rail, side and ceiling robot placement with areas assigned automatically, ROS 2 in real time. Running three at once cut the cycle by 33%.',
    'RMPFlow 72초 → 48초 (–33%)': 'RMPFlow 72 s → 48 s (–33%)',
    '실시간 접촉력·제거량 히트맵 대시보드': 'Live contact-force and removal heatmap dashboard',
    '실시간 공정 확인 — 수치는 기획서 화면 값, 파형은 형태 예시':
      'Live process view — figures from the proposal screen, waveform is shape only',
    '숙련공 대조 — 정상 구간은 기준의 1/3': 'Against the veteran — a third of the baseline in the steady segment',
    '문헌 기반 합성 숙련공 기준선과 힘 균일성으로 대조했다. 실제 연마가 일어나는 정상 구간의 변동계수는 3.23%로 기준(9.96%)의 약 1/3이고, 진입·이탈 구간과 접촉 유지율은 남은 과제로 그대로 공개한다.':
      'Force uniformity was compared against a synthetic veteran baseline built from the literature. In the steady segment where polishing actually happens the coefficient of variation is 3.23%, about a third of the 9.96% baseline. Entry, exit and contact retention are published as they stand — still open.',
    '정상상태 CV 3.23% · 목표대역 체류 100% · 과압 0회':
      'Steady-state CV 3.23%, 100% time in band, zero over-pressure events',
    '전이구간 CV 71.85% · 접촉 유지율 73.6% — 남은 과제':
      'Transition CV 71.85%, contact retention 73.6% — still open',
    '실측 숙련공 공개 데이터셋 부재 — 합성 참조선 기준임을 명시':
      'No public measured-veteran dataset exists — the baseline is stated as synthetic',
    '힘 균일성 대조 — 숙련공 대조 리포트 2026-08-05 · 합성 기준선은 문헌 기반 가정치':
      'Force uniformity comparison — veteran comparison report 2026-08-05 · the synthetic baseline is an assumption from the literature',

    /* ── 가격 정책 ── */
    '엔터프라이즈 디지털 트윈 툴은 전부 "견적 문의"다. 우리는 가격을 공개한다 — 설비투자(평균 18억 원)의 0.2% 수준으로, 로봇이 아니라 발주 전 판단 근거를 판다.':
      'Every enterprise digital twin tool says "contact us for a quote". We publish the price — about 0.2% of the capital outlay (1.8bn KRW on average). What we sell is not a robot but the grounds for a decision before the order.',
    '중견기업 구독 — 연 396만 원': 'Mid-size subscription — 3.96M KRW a year',
    '중소기업 검증 리포트': 'Validation report for small firms',
    '상용 OLP 라이선스 대비 도입 비용': 'the cost of adoption against a commercial OLP licence',
    '품종 전환 연 4회 기준 연간 운영비 절감': 'annual operating saving at four changeovers a year',
    '기업 규모별 수익 모델 — 1차 유료 고객은 중견기업 월 구독 · 기획서 4.3 [표 7]':
      'Revenue model by company size — the first paying customers are mid-size monthly subscriptions · proposal 4.3 [table 7]',
    '가격은 상향식으로 산정했다': 'The price was built from the bottom up',
    '인건비·연산비·개발비 상각을 공표 통계 기준으로 합산했다. 설비투자 대비 0.2%라는 비율은 목표가 아니라 이 합산의 결과다.':
      'Labour, compute and amortised development were added up against published statistics. The 0.2% of capital outlay is not a target; it is what the sum came to.',
    '특급기술자 일 단가 — 국가승인통계 제372001호 (2026년 적용)':
      'Senior engineer day rate — national statistic no. 372001 (2026)',
    '도입 1개월 내 투자 회수 — 품종 전환 연 4회 기준':
      'Paid back within a month of adoption, at four changeovers a year',
    '검증 리포트 1건 판매가 산정 — 기획서 4.3 [표 8]':
      'How the price of one validation report was worked out — proposal 4.3 [table 8]',
    '힘 제어 검증은 폴리트윈만 지원한다': 'Only PolyTwin validates force control',
    '상용 OLP 는 어디로 갈지(경로)는 다루지만 얼마나 누를지(힘)는 다루지 않는다. 연마는 접촉력이 품질을 결정하는 공정이라, 이 지점이 기존 솔루션의 공백이다.':
      'Commercial OLP handles where to go, not how hard to press. In polishing, contact force decides the quality — which is exactly the gap in what exists today.',
    '힘 제어 검증 포함 + 상용 라이선스 대비 약 1/40 가격':
      'Force-control validation included, at about a fortieth of a commercial licence',
    '적용 범위를 중소기업까지 확장': 'Reach extended down to small firms',
    '유사 서비스 비교 — 기획서 4.3 [표 10]': 'Compared with similar services — proposal 4.3 [table 10]',
    '시장과 결제 경로': 'The market and who pays',
    '하드웨어(로봇)보다 검증 소프트웨어 레이어가 빠르게 큰다. 국내에서는 스마트공장 구축 지원사업이 실질적 결제 수단이고, 우리 가격은 그 예산 한도 안에 여유 있게 들어간다.':
      'The validation software layer is growing faster than the robot hardware. In Korea the smart-factory grant programme is what actually pays, and our price sits comfortably inside its ceiling.',
    '정부형 지원 최대 2억 원 · 상생형 AI 트랙 최대 5억 원':
      'Government track up to 200M KRW, the shared-growth AI track up to 500M',
    '구독·리포트 모두 지원사업 예산 한도 이내': 'Both subscription and report fit inside the grant ceiling',
    '설비 발주 전에 검증부터 — 영업 메시지도 이 한 줄이다':
      'Validate before you order the equipment — that one line is also the sales pitch',
    '시장 규모 — 출처: Fortune Business Insights(2026) · Allied Market Research(2026)':
      'Market size — sources: Fortune Business Insights (2026), Allied Market Research (2026)',

    /* ── 폴리싱 ── */
    '차체 표면을 스캔하고, 경로를 만들고, 일정한 힘으로 연마하고, 결과를 리포트로 남긴다. 파라미터만 바꿔 다시 실행할 수 있다.':
      'Scan the body surface, build the path, polish at a steady force, leave the result as a report. Change the parameters and run it again.',
    '가상 작업장 폴리싱 실행': 'Polishing running in the virtual work cell',
    '광택 공정 6단계와 자동화 범위 — 공정 구분은 숙련공 교육자료 「자동차 광택」 3장':
      'The six steps of polishing and what is automated — the breakdown follows chapter 3 of the trade manual "Automotive Polishing"',
    '경로 생성 6단계 — 작동 범위 · 법선 추정 · 연속성 · 경계 · 회전 범위 · 충돌 · 05 패널의 축은 원본 실행 6,704점 기준':
      'Six stages of path generation — reach, normal estimation, continuity, boundary, joint range, collision. The axis in panel 05 is from the original 6,704-point run.',
    'Blind Spot 진단 — 판정 원리와 진단 결과': 'Blind Spot diagnosis — how it is judged and what it found',
    '깊이 카메라 스캔': 'depth camera scan',
    '사전 검증 리포트': 'pre-validation reports',
    '9시점 깊이 카메라로 차체를 촬영하고 포인트 클라우드를 ICP 로 정합한다. 배경을 제거하고 표면을 복원해 작업 대상 형상을 확정한다.':
      'Nine depth cameras capture the body and the point clouds are registered with ICP. The background is stripped and the surface reconstructed to fix the shape being worked on.',
    '9시점 촬영 · ICP 정합': 'Nine views, ICP registration',
    '정합 오차·누락면 검출': 'Registration error and missing surfaces detected',
    '01 스캔 — 3D 포인트 클라우드': '01 Scan — 3D point cloud',
    'KDTree 기반으로 국소 평면과 법선을 추정해 연마 도구가 표면에 직교하도록 래스터 경로를 만든다. 차량 경계·IK 도달성·충돌을 검증해 경로를 확정한다.':
      'A KDTree estimates local planes and normals so the raster path keeps the tool square to the surface. The path is fixed after checking the vehicle boundary, IK reachability and collision.',
    '패드 간격 자동 산정 (150 mm 패드 40% 중첩)':
      'Pad spacing worked out automatically (150 mm pad, 40% overlap)',
    '경로 필터링 5단계 · 웨이포인트 813개': 'Five filter stages, 813 waypoints',
    '02 경로 — 국소 평면·법선 추정': '02 Path — local plane and normal estimation',
    '연마 깊이 한계': 'How deep polishing can go',
    '광택은 도장면 최상층인 클리어코트만 깎아내는 작업이다. 그 아래 베이스코트를 건드리면 연마로는 되돌릴 수 없고 도색으로 넘어간다. 접촉력 상한과 패스 구성은 이 한 층의 두께 안에서 정해진다.':
      'Polishing cuts only the clear coat, the top layer of the paint. Touch the base coat below it and polishing cannot undo it — that becomes a repaint. The force ceiling and the pass plan are both decided inside the thickness of that one layer.',
    '연마 대상 = 클리어코트 한 층': 'What gets polished is one layer: the clear coat',
    '손상 3단계 — 제거 가능 / 일부 가능 / 도색 영역':
      'Three grades of damage — removable, partly removable, repaint',
    '클리어코트 두께 50–70 µm · 1회 광택당 제거 약 5 µm — 숙련공 인터뷰 채록 (2026-08-28)':
      'Clear coat 50–70 µm thick, about 5 µm removed per polish — from a veteran interview (2026-08-28)',
    '도장면 단면과 손상 깊이 — 층 두께는 도해 비율, 판정 기준은 숙련공 교육자료 「자동차 광택」':
      'Paint cross-section and damage depth — layer thicknesses are diagrammatic; the verdicts follow the trade manual "Automotive Polishing"',
    '가상 스프링·감쇠 순응 접촉으로 목표 접촉력을 유지하며 연마한다. 제거량은 Preston 모델로 계산하고 Ansys 물리 환경과 교차 검증했다.':
      'Virtual spring-damper compliant contact holds the target force while polishing. Removal is computed with the Preston model and cross-checked against Ansys.',
    '목표 접촉력 1.5 N 추종 · 유지 범위 3–8 N 검증 중':
      'Tracking a 1.5 N target, holding 3–8 N — under validation',
    'Preston 제거량 모델 (MRR = k': 'Preston removal model (MRR = k',
    '03 힘 제어 — 접촉력 추종, 초기 실행 984 스텝':
      '03 Force control — tracking contact force over the first 984 steps',
    '검증·리포트': 'Validation and report',
    '곡률이 급해 패드가 닿지 못하는 Blind Spot 을 실행 전에 좌표로 특정하고, 도달 영역·제거량 히트맵 등 사전 검증 리포트 4종을 산출한다.':
      'Blind Spots, where curvature is too tight for the pad to touch, are pinned to coordinates before the run, and four pre-validation reports come out — reached area, removal heatmap and the rest.',
    'Blind Spot 좌표 특정 (임계 곡률반경 = 패드 반경)':
      'Blind Spot coordinates identified (critical radius = pad radius)',
    '발주 전 사전 검증 리포트 4종': 'Four pre-order validation reports',
    '04 검증 — Blind Spot 진단': '04 Validation — Blind Spot diagnosis',

    /* ── 디버링 ── */
    '가공 후 남은 버(burr)를 모서리를 따라 제거한다. 폴리싱과 같은 스캔·경로·힘 제어 파이프라인 위에서 동작하며, 현재 공정 확장 로드맵에 있다.':
      'The burr left after machining is removed along the edge. It runs on the same scan, path and force-control pipeline as polishing, and sits on the process roadmap.',
    '스캔 · 엣지 검출': 'Scan and edge detection',
    '부품을 스캔해 포인트 클라우드를 만들고, 곡률 변화가 급한 모서리에서 버 발생 후보 구간을 검출한다.':
      'The part is scanned into a point cloud and candidate burr segments are found where curvature changes sharply.',
    '다시점 스캔 · 정합': 'Multi-view scan and registration',
    '곡률 기반 엣지 후보 추출': 'Edge candidates from curvature',
    '스캔 파이프라인 — 폴리싱과 공유': 'Scan pipeline — shared with polishing',
    '엣지 경로 생성': 'Edge path generation',
    '검출된 모서리를 따라 공구 자세를 유지하는 경로를 만들고, 도달성과 충돌을 사전 검증한다.':
      'A path that holds tool pose along the detected edge is built, with reachability and collision checked in advance.',
    '모서리 추종 경로': 'Edge-following path',
    'IK 도달성·충돌 검증': 'IK reachability and collision checked',
    '경계를 따르는 경로 생성': 'A path that follows the boundary',
    '힘 제어 디버링': 'Force-controlled deburring',
    '버 크기에 따라 제거 저항이 달라지므로, 순응 제어로 접촉력을 일정하게 유지하며 제거한다.':
      'Resistance changes with burr size, so compliant control keeps contact force steady while it cuts.',
    '순응 접촉 힘 제어': 'Compliant contact force control',
    '구간별 이송 속도 조절': 'Feed rate adjusted per segment',
    '디버링 실행 — 공구 앞은 버, 뒤는 챔퍼 · 로드맵 4단계':
      'Deburring in progress — burr ahead of the tool, chamfer behind · roadmap stage 4',
    '검증 · 리포트': 'Validation and report',
    '엣지 잔여 구간을 좌표로 리포트하고, 통과 기준을 만족한 파라미터를 프리셋으로 저장한다.':
      'Remaining edge segments are reported by coordinate, and parameters that met the bar are saved as a preset.',
    '잔여 버 좌표 리포트': 'Remaining burrs reported by coordinate',
    '파라미터 프리셋 저장': 'Parameter preset saved',
    '엣지·홀 모서리별 잔여 버 판정 — 리포트 형식 예시, 수치는 실측이 아니다':
      'Remaining burr judged per edge and hole — format example, the numbers are not measured',

    /* ── 샌딩 ── */
    '도장 전 표면을 단계별 입도로 고르게 갈아낸다. 실제 샌딩 킷 3D 자산이 준비되어 있으며, 폴리싱 파이프라인을 공유한다.':
      'Before painting, the surface is levelled through a sequence of grits. The sanding kit exists as a 3D asset and the polishing pipeline is shared.',
    '스캔 · 영역 분할': 'Scan and region split',
    '표면을 스캔하고 곡률·부위 기준으로 작업 영역을 분할한다. 영역마다 요구 거칠기가 다르다.':
      'The surface is scanned and split into work regions by curvature and location. Each region wants a different roughness.',
    '부위별 영역 분할': 'Regions split by location',
    '영역별 목표 거칠기 설정': 'A target roughness per region',
    '스캔 · 영역 분할 — 스캔 파이프라인은 폴리싱과 공유':
      'Scan and region split — the scan pipeline is shared with polishing',
    '영역별 경로 생성': 'Per-region path generation',
    '분할된 영역마다 패드 중첩률과 이송 속도를 달리한 래스터 경로를 만든다.':
      'Each region gets a raster path with its own pad overlap and feed rate.',
    '영역별 래스터 경로': 'A raster path per region',
    '중첩률·속도 개별 설정': 'Overlap and speed set separately',
    '국소 평면·법선 추정 — 영역별 경로의 기준면':
      'Local plane and normal estimation — the reference surface for each region’s path',
    '단계별 연마': 'Grit by grit',
    '거친 입도에서 고운 입도로 단계를 올려가며 같은 경로를 반복한다. 각 단계의 접촉력을 힘 제어로 유지한다.':
      'The same path is repeated from coarse grit up to fine, with force control holding the contact force at each step.',
    '입도 단계별 반복 실행': 'Repeated per grit step',
    '단계별 접촉력 프로파일': 'A contact-force profile per step',
    '입도 단계별 연마 — 거친 입도에서 고운 입도로, 도장 전이라 무광을 유지한다 · 로드맵 4단계':
      'Sanding grit by grit — coarse to fine, staying matte because paint comes after · roadmap stage 4',
    '표면 품질 검증': 'Surface quality check',
    '단계 종료마다 표면 상태를 확인하고, 미달 영역만 골라 재실행한다.':
      'The surface is checked at the end of each step and only the regions below the bar are run again.',
    '영역 단위 품질 판정': 'Quality judged per region',
    '미달 영역 선택 재실행': 'Only the regions that fell short are rerun',
    '영역 단위 품질 판정 — 미달 영역만 재실행 · 로드맵 4단계':
      'Quality judged per region — only what fell short is rerun · roadmap stage 4',

    /* ── Pick and place ── */
    '연마 공정 전후의 부품 이송을 같은 가상 작업장 안에서 다룬다. 표면처리 셀과 이송 셀을 하나의 시뮬레이션으로 검증하는 것이 목표다.':
      'Moving parts before and after polishing is handled in the same virtual work cell. The aim is to validate the finishing cell and the transfer cell in one simulation.',
    '대상 인식': 'Object detection',
    '작업대 위 부품의 위치와 자세를 인식한다. 스캔 파이프라인의 포인트 클라우드 처리를 재사용한다.':
      'The position and pose of the part on the bench are detected, reusing the point-cloud processing from the scan pipeline.',
    '위치·자세 추정': 'Position and pose estimation',
    '포인트 클라우드 처리 재사용': 'Point-cloud processing reused',
    '위치·자세 추정 — 포인트 클라우드와 부품별 좌표계 · 스캔 파이프라인 재사용':
      'Position and pose estimation — point cloud and a frame per part · scan pipeline reused',
    '파지 계획': 'Grasp planning',
    '부품 형상에서 파지 후보를 만들고, 도달성과 충돌을 검증해 파지 자세를 확정한다.':
      'Grasp candidates are generated from the part shape, then reachability and collision fix the grasp pose.',
    '파지 후보 생성': 'Grasp candidates generated',
    '충돌 가능성 확인 — 파이프라인 공유': 'Collision checked — pipeline shared',
    '이송 실행': 'Transfer',
    '들어 올리고, 옮기고, 내려놓는다. 경로는 주변 설비와의 간섭을 피해 생성된다.':
      'Lift it, move it, set it down. The path is generated to clear the surrounding equipment.',
    '간섭 회피 이송 경로': 'A transfer path that clears obstacles',
    '셀 간 연계 시퀀스': 'The sequence between cells',
    '이송 실행 — 들어 올리고, 옮기고, 내려놓는다 (21초, 소리 있음)':
      'Transfer — lift, move, set down (21 s, with sound)',
    '사이클 검증': 'Cycle validation',
    '이송을 포함한 전체 사이클 타임을 계측하고, 병목 구간을 리포트한다.':
      'The whole cycle time including transfer is measured and the bottleneck reported.',
    '사이클 타임 계측': 'Cycle time measured',
    '병목 구간 리포트': 'Bottleneck reported',
    '사이클 타임 계측 — 구간 이름은 이송 실행 절차 그대로, 길이는 형태 예시':
      'Cycle time — the segment names are the transfer procedure itself, the lengths are shape only',

    /* ── 확장 로드맵 ── */
    '이식되는 것은 결과물이 아니라 절차다. 계측 → 재현 → 판정 3단계가 성립하면 대상 공정·설비·품목은 교체할 수 있다. 곡면 차체는 접촉 공정 중 가장 까다로운 대상이라, 여기서 세운 절차는 평판·저접촉 공정으로 갈수록 요구 조건이 완화된다.':
      'What ports across is the procedure, not the output. Once measure → reproduce → judge holds, the process, the equipment and the part can all be swapped. A curved car body is the hardest thing a contact process meets, so a procedure set here only gets easier on flat and low-contact work.',
    '다음 단계는 이미 목록에 있다': 'The next steps are already on the list',
    '현직 광택 숙련공 인터뷰(2026-08-28)에서 확보한 항목이 강화학습 확장 과제로 등록돼 있다. 예산은 클리어코트 한 층의 두께다.':
      'Items from an interview with a working polishing veteran (2026-08-28) are registered as RL extension tasks. The budget is the thickness of one clear coat.',
    '도막 예산 — 클리어코트 50–70 µm · 1회 제거 약 5 µm · 누적 40 µm 초과 경고':
      'Film budget — clear coat 50–70 µm, about 5 µm off per pass, warning past 40 µm cumulative',
    '온도(마찰열) 상태변수 — 낮으면 연마가 안 되고 100 °C 에 가까우면 도장이 상한다':
      'Temperature (friction heat) as a state variable — too low and nothing cuts, near 100 °C and the paint is damaged',
    'RPM 을 고정값(3,000)에서 가변 행동변수로 · 엣지 구간 규칙 분리':
      'RPM from a fixed 3,000 to a variable action, with separate rules for edges',
    '도장면 단면 — 확장 과제의 예산은 클리어코트 한 층 안에 있다':
      'Paint cross-section — the budget for every extension sits inside one clear coat',
    '힘을 만드는 쪽은 결국 이 팔이다': 'The force comes from this arm in the end',
    '로봇 팔 불러오는 중': 'Loading the robot arm',
    'Doosan M0609 · 6축': 'Doosan M0609 · 6 axes',
    '로봇 팔을 불러오지 못했다': 'The robot arm could not be loaded',
    'WebGL 모듈을 읽지 못했다': 'WebGL modules failed to load',
    '모델 파일을 읽지 못했다': 'The model file failed to load',
    '회전 관절': 'rotary joints',
    '목표 접촉력': 'target contact force',
    '경로 웨이포인트': 'path waypoints',
    '동시 제어 — 천장·측면·레일': 'controlled at once — ceiling, side, rail',
    '실제와 같은 물리 조건으로 차체·로봇·연마 도구를 재현한다. 6축 로봇 팔은 실제 Doosan M0609 의 링크 길이를 그대로 쓰고, 명령과 측정값은 ROS 2 통신망으로 실시간 왕복한다.':
      'The body, the robot and the tooling are reproduced under real physical conditions. The six-axis arm uses the link lengths of an actual Doosan M0609, and commands and measurements travel over ROS 2 in real time.',
    '실제 로봇 형상 그대로 — 아래 모델은 시뮬레이터가 쓰는 것과 같은 파일이다':
      'The real robot geometry — the model below is the same file the simulator runs',
    '공고가 내건 조건과 우리가 이미 가진 것이 같은 자리에 있다':
      'What the calls ask for and what we already have sit in the same place',
    '2026 정책 기조': '2026 policy direction',
    '폴리트윈이 대는 것': 'What PolyTwin brings',
    '산업부 M.AX': 'MOTIE M.AX',
    '1조 1,000억 원 · 2배 확대': '1.1tn KRW, doubled',
    '제조 공정에 AI 를 넣어 실증할 것': 'Put AI into a manufacturing process and prove it',
    '재질별 파라미터 프로파일': 'Per-material parameter profiles',
    '차체에서 세운 절차를 유리·금형으로 이식': 'The procedure set on car bodies, ported to glass and moulds',
    '중기부 제조AI 스마트공장': 'MSS manufacturing-AI smart factory',
    '800억 원 · 400개 과제': '80bn KRW across 400 projects',
    '중소 제조 현장에 적용할 것': 'Apply it on small and mid-size shop floors',
    '숙련공 실측 데이터 연계': 'Linking measured veteran data',
    '검증 리포트 250만 원 — 지원 한도 안': 'A 2.5m KRW validation report — inside the support ceiling',
    '피지컬 AI 정책금융': 'Physical-AI policy finance',
    '약 16조 원': 'About 16tn KRW',
    '로봇이 물리 세계를 다룰 것': 'Have robots handle the physical world',
    '접촉력·마찰까지 계산하는 힘 제어': 'Force control that computes contact and friction',
    '목표 1.5 N 추종 · 과압 0회': 'Tracking a 1.5 N target, zero overpressure events',
    '1단계 · 완료': 'Stage 1 · done',
    '2단계': 'Stage 2',
    '3단계': 'Stage 3',
    '세 축 모두 우리가 이미 만든 것 또는 로드맵에 올린 것과 맞물린다':
      'All three tracks meet something we have already built or already put on the roadmap',
    '2·3·4단계를 굴릴 재원이 밖에 있다': 'Stages 2, 3 and 4 have funding outside the company',
    '확장 단계는 재질 시험 가공과 모션캡처 실측을 요구한다 — 우리 자체 예산으로는 감당할 수 없는 항목이다. 2026년 정부 AI 예산이 9.9조 원으로 전년의 3배가 되면서, 그 비용을 댈 과제 공고가 우리가 가려는 방향과 같은 곳을 겨냥하고 있다.':
      'The expansion stages call for trial machining on each material and motion-capture measurement — line items our own budget cannot carry. With the 2026 government AI budget at 9.9tn KRW, three times last year, the calls that would fund them point where we are already headed.',
    '산업부 M.AX 1조 1,000억 원(2배 확대) — 2단계 재질별 프로파일 확장이 여기 걸린다':
      'MOTIE M.AX 1.1tn KRW, doubled — this is where stage 2, the per-material profiles, fits',
    '중기부 제조AI 특화 스마트공장 800억 원 · 400개 과제 — 3단계 숙련공 실측 연계의 현장 확보처':
      'MSS manufacturing-AI smart factory, 80bn KRW across 400 projects — the shop floors for stage 3, the veteran measurement link',
    '피지컬 AI 정책금융 약 16조 원 — 4단계 타 공정 확장의 자금 경로':
      'About 16tn KRW of policy finance for physical AI — the funding route for stage 4, expansion to other processes',

    /* ── 기술혁신 ── */
    '움직임만 흉내 내는 시뮬레이션이 아니라, 접촉력·마찰·제거량까지 계산하는 물리 기반 가상 작업장이다. NVIDIA Isaac Sim 위에서 ROS 2 로 명령과 측정값이 실시간으로 오간다.':
      'Not a simulation that mimics motion, but a physics-based work cell that computes contact force, friction and removal. Commands and readings move over ROS 2 on top of NVIDIA Isaac Sim, in real time.',
    '전체 구성 — 작업자 Web UI 와 가상 작업장': 'The whole system — operator Web UI and virtual work cell',
    '물리 엔진 위의 작업장': 'A work cell on a physics engine',
    '실제와 같은 물리 조건으로 차체·로봇·연마 도구를 재현한다. 명령과 측정값은 ROS 2 통신망으로 실시간 왕복한다.':
      'Body, robot and polishing tool are reproduced under real physics. Commands and readings travel back and forth over ROS 2 in real time.',
    'NVIDIA Isaac Sim 물리 환경': 'NVIDIA Isaac Sim physics',
    'ROS 2 실시간 통신': 'ROS 2 in real time',
    '가상 작업장 파이프라인 — Isaac Sim 위에서 도는 10단계':
      'The virtual work cell pipeline — ten stages running on Isaac Sim',
    '접촉을 수식으로 다룬다': 'Contact handled as equations',
    '연마 패드와 표면의 접촉은 가상 스프링·감쇠 모델로, 제거량은 Preston 방정식으로 계산한다. 회전 속도·이동 속도·누르는 힘이 전부 변수다.':
      'Contact between pad and surface is a virtual spring-damper model; removal comes from the Preston equation. Rotation speed, travel speed and pressing force are all variables.',
    '순응 접촉 모델 F = k·침투 – c·속도': 'Compliant contact model F = k · penetration – c · speed',
    'Preston 제거량 MRR = k': 'Preston removal MRR = k',
    '시뮬레이션 적용 — Preston 방정식': 'Applied in simulation — the Preston equation',
    '외부 물리 환경과 교차 검증': 'Cross-checked against an outside physics engine',
    '시뮬레이션 파라미터를 Ansys 물리 환경에서 접촉력 기준으로 비교 검증한다. 자체 수치가 아니라 검증된 수치를 쓴다.':
      'Simulation parameters are compared against Ansys on contact force. We use numbers that were checked, not our own.',
    '패널 변형량 대조': 'Panel deflection compared',
    'Ansys 사전 검증': 'Ansys pre-validation',
    '일정한 힘을 유지한다': 'The force stays steady',
    '목표 접촉력을 넘지 않으면서 표면에 일정하게 힘을 가하도록 힘 제어를 구현했다. 결과는 시간축 그래프로 남는다.':
      'Force control was built to press evenly on the surface without passing the target. The result is kept as a time-series chart.',
    '목표 1.5 N 추종 · 유지 범위 3–8 N 검증 중': 'Tracking 1.5 N, holding 3–8 N — under validation',
    '경로 전 구간 접촉력 시간축 기록': 'Contact force logged over the whole path',
    '힘 제어 — 접촉력 유지, 초기 실행 984 스텝':
      'Force control — holding contact force over the first 984 steps',

    /* ── 강화학습 ── */
    '스캔 한 번이 48개의 학습 환경으로 불어난다. 추가 스캔 없이 조건만 바꿔 데이터셋을 늘리고, 보상을 최대화하는 정책을 학습시킨다.':
      'One scan grows into 48 training environments. No extra scanning — the conditions change, the dataset grows, and the policy that maximises reward is trained.',
    '스캔 1회 → 학습 환경 48개 — 환경 생성부터 자산화까지':
      'One scan → 48 training environments, from build to registry',
    '스캔 1회로 만드는 학습 환경': 'training environments from one scan',
    '경로 패턴 (오비탈·스파이럴·리니어)': 'path patterns (orbital, spiral, linear)',
    '이송 속도 30–60%': 'feed rate, 30–60%',
    '누름 강도 1.0–2.5 N': 'press force, 1.0–2.5 N',
    '환경 생성': 'Environment build',
    '한 번의 스캔에 경로 패턴 3종 × 이송 속도 4단계 × 누름 강도 4단계를 조합해 48개 학습 환경을 만든다.':
      'One scan crossed with 3 path patterns, 4 feed rates and 4 press forces makes 48 training environments.',
    '3 × 4 × 4 = 48개 환경': '3 × 4 × 4 = 48 environments',
    '깊이 카메라 9시점 포인트 클라우드 정합': 'Nine-view depth camera point cloud registration',
    '01 환경 생성 — 표면 형상 3종 × 압력 4단계 × 속도 4단계':
      '01 Environment build — 3 surface shapes × 4 pressures × 4 speeds',
    '정책 학습': 'Policy training',
    '48개 환경에서 보상을 최대화하는 정책을 학습한다. 보상은 접촉력과 표면 거칠기 목표에서 사이클 타임을 뺀 값이다.':
      'The policy that maximises reward across the 48 environments is trained. Reward is contact force and roughness target minus cycle time.',
    'R = 접촉력 + 표면 거칠기 목표 – 사이클 타임': 'R = contact force + roughness target – cycle time',
    '학습이 진행될수록 숙련공 대비 검증 오차 감소':
      'Validation error against the veteran falls as training runs',
    '02 정책 학습 — 보상 함수와 검증 오차 추이':
      '02 Policy training — reward function and validation error',
    '전문가 평가': 'Expert review',
    '숙련공이 연마 결과를 보고 평가한다. 경로 적절성·연마 균일성·접촉 안정성·작업 품질 네 항목이다. 피드백은 보상 재설계로 되돌아간다.':
      'A veteran looks at the polished result and scores it on four things: path fitness, polish uniformity, contact stability, work quality. The feedback comes back as a redesigned reward.',
    '숙련공 4항목 평가': 'Four things a veteran scores',
    '피드백 → 파라미터 보정·보상 재설계 → 재학습':
      'Feedback → parameters corrected, reward redesigned → retrain',
    '03 전문가 평가 — 숙련공이 보는 네 항목': '03 Expert review — the four things a veteran looks at',
    '자산화': 'Asset registry',
    '통과한 정책만 기술 자산으로 등록한다. 전문가 점수 기준 미달은 제외된다.':
      'Only policies that passed are registered as assets. Anything below the expert score is dropped.',
    '통과 정책만 등록 (#07 85.6 · #12 91.7 · #23 95.0 …)':
      'Only what passed is kept (#07 85.6, #12 91.7, #23 95.0 …)',
    '공정별 정책·조건 DB 등록': 'Registered in the per-process policy and condition DB',
    '04 자산화 — 조합별 전문가 점수와 판정': '04 Asset registry — expert score and verdict per combination',

    /* ── 숙련공 DB ── */
    '검증을 통과한 정책과 조건이 재사용 가능한 디지털 자산으로 쌓인다. 숙련공의 판단 기준이 다음 공정의 출발점이 된다.':
      'Policies and conditions that passed pile up as reusable digital assets. The veteran’s judgement becomes the starting point for the next job.',
    '공정 6단계 — 파라미터 설정에서 기술 보존까지':
      'Six steps — from setting parameters to keeping the technique',
    '노하우가 물리량으로 남는다': 'Know-how survives as physical quantities',
    '사람이 연마할 때 남는 측정값은 0개다. 디지털 트윈에서 같은 작업을 하면 접촉력·이송속도·공구 자세·체류 시간이 시간 단위 숫자로 남는다.':
      'When a person polishes, the number of measurements left is zero. Run the same job in the digital twin and contact force, feed rate, tool pose and dwell time stay as numbers against time.',
    '작동 시간에 따라 17개 물리량 누적': '17 physical quantities accumulated over run time',
    '접촉력 중앙값 1.50 N · 체류 시간 중앙값 4.0 초':
      'Median contact force 1.50 N, median dwell time 4.0 s',
    '노하우 → 물리량 — 수치는 기획서 기록값, 파형은 형태 예시':
      'Know-how into quantities — figures from the proposal, waveform is shape only',
    '숙련공이 등록을 결정한다': 'The veteran decides what gets registered',
    '강화학습이 만든 정책도 숙련공 평가를 통과해야 DB 에 들어간다. 점수 미달 조합은 제외된다.':
      'Even a policy from reinforcement learning has to pass a veteran before it enters the DB. Combinations below the score are dropped.',
    '전문가 점수 기준 등록/제외 판정': 'Kept or dropped on the expert score',
    '공정별 정책·조건 DB': 'Per-process policy and condition DB',
    '통과 정책만 자산으로 등록': 'Only policies that passed become assets',
    '등록된 데이터는 교재가 된다': 'What is registered becomes the teaching material',
    '어깨너머 구전에 의존하던 전수가 수치·화면 기반 훈련으로 옮겨간다. 숙련공은 교재의 저자이자 검증 기준으로 계속 쓰이고, 실물 설비가 없는 중소 협력사도 통과 구성안 프로파일을 공유받아 자사 공정을 확인한다.':
      'Passing the trade on moves from watching over a shoulder to training on numbers and screens. The veteran stays as both the author and the standard it is checked against, and small suppliers without the equipment can take a passing profile and check their own process against it.',
    '신입 숙련 3–5년 → 정답 파라미터가 이미 등록됨':
      'Three to five years to competence → the right parameters are already on file',
    '분진·소음 3D 기피 직무 → AX 엔지니어 직무로 연결':
      'A dusty, noisy job the young avoid → a route into an AX engineer role',
    '숙련 기술 전승 방식 비교 — 기획서 4.3 사회 [표 7]':
      'How the trade gets passed on, compared — proposal 4.3, social [table 7]',
    '민감 데이터는 분리 보관한다': 'Sensitive data is stored apart',
    '공유 가능한 이벤트 로그와, 관절 각도·궤적 같은 민감 정보를 이원화한다. 민감 데이터는 고객사의 자산이다.':
      'Shareable event logs and sensitive records like joint angles and trajectories are kept separate. The sensitive half is the customer’s asset.',
    '로그 이원화 — 공유 가능 / 공유 불가': 'Split logging — shareable and not',
    '이벤트 타임스탬프 기록': 'Events timestamped',
    '로그 이원화': 'Split logging',
    '권한으로 접근을 나눈다': 'Access is split by permission',
    '조회·설정·제어 권한을 역할별로 분리한다. 협력사는 조회만, 관리자는 제어까지 — 상황에 따라 범위를 조절한다.':
      'View, configure and control are separated by role. A supplier gets view only, an administrator gets control — the scope moves with the situation.',
    '조회 / 설정 / 제어 3단계 권한': 'Three levels: view, configure, control',
    '역할별 접근 분리': 'Access separated by role',
    '접근 권한 역할별 분리': 'Access permissions, by role',
    '인증 심사까지 계획에 있다': 'Certification is on the plan',
    '로그 이원화와 권한 분리는 통제 항목의 앞 단계다. ISO 27001 Annex A 통제항목 기준으로 정보자산 식별부터 인증 심사까지 4단계로 진행한다.':
      'Split logging and separated permissions come before the controls proper. Against ISO 27001 Annex A, it runs in four stages from identifying assets to the certification audit.',
    '정보자산 식별·위험평가 → 정책 수립 → 내부 심사 → 인증 심사':
      'Identify assets and assess risk → write the policy → internal audit → certification audit',
    '데이터 보안 대응 4단계 — 제출본 3.4 [그림 11]':
      'Four stages of data security — submission 3.4 [figure 11]',

    /* ── 안전 검증 ── */
    '위험은 가상이 아니라 실물에 있다. 폴리트윈의 산출물은 실기 로봇에서 실행되는 경로·파라미터다 — 그래서 검증되지 않은 구간은 통과시키지 않고, 좌표와 함께 미검증으로 표기한다.':
      'The danger is in the real machine, not the simulation. What PolyTwin produces are paths and parameters that run on a real robot — so anything unvalidated does not pass; it is marked unvalidated, with coordinates.',
    '셀 내부 티칭 — 스캔 1회로 경로 자동 생성':
      'teaching inside the cell — one scan generates the path',
    '연마 접촉력 — 대역 이탈 시 실행 차단':
      'polishing contact force — leaving the band blocks the run',
    'ISO/TS 15066 최저 한계(안면 65 N) 대비 상한':
      'ceiling against the lowest ISO/TS 15066 limit (face, 65 N)',
    '실행 전 차단 항목': 'risks blocked before the run',
    '리스크 5종을 실행 전에 차단한다': 'Five risks stopped before the run',
    '기존 방식의 위험은 전부 실물에서 발견됐다 — 발견 시점에 이미 노출된 뒤다. 폴리트윈은 다섯 항목 모두를 실행 전 검증으로 옮긴다.':
      'Under the old way every risk was found on the real machine — by which time it had already happened. PolyTwin moves all five into validation before the run.',
    '도입 기업은 이 판정 결과를 ISO/TS 15066 위험성 평가의 사전 검증 자료로 그대로 제출':
      'The adopting company can submit these verdicts as-is for the ISO/TS 15066 risk assessment',
    '검증 안 된 구간은 통과가 아니라 미검증으로 표기':
      'What was not validated is marked unvalidated, not passed',
    '규격 기준선 — ISO/TS 15066': 'The standard as baseline — ISO/TS 15066',
    '연마는 사람과 같은 공간에서 이뤄지는 협동로봇 공정이다. 정상 작업 대역 3–8 N 은 규격이 정한 인체 허용 한계에서 한 자릿수 배율만큼 떨어져 있다. 실제 위험은 이상 상황 — 과압입·경로 이탈·급가속 — 이고, 그것이 앞의 게이트가 걸러내는 대상이다.':
      'Polishing is a collaborative-robot process that happens in the same space as people. The normal 3–8 N working band sits an order of magnitude below the human limits the standard sets. The real danger is the abnormal case — over-press, path departure, sudden acceleration — and that is what the gates above filter out.',
    '인체 29개 부위별 한계 — 안면 65 N · 손 140 N · 하완 160 N':
      'Limits for 29 body regions — face 65 N, hand 140 N, forearm 160 N',
    '숙련공이 겪던 사고 — 케이블 말림·마찰 화상 — 는 셀 분리로 구조적으로 제거 (인터뷰 채록 2026-08-28)':
      'The accidents veterans used to have — cables winding in, friction burns — are removed structurally by separating the cell (interview, 2026-08-28)',
    '규격 허용 한계 대비 연마 접촉력 — ISO/TS 15066:2016 Annex A':
      'Polishing contact force against the permitted limits — ISO/TS 15066:2016 Annex A',
    '멈춰도 이어서 간다': 'A stop is not a restart',
    '오류는 없앨 수 없으니 복구를 설계한다. 로그를 단계별로 분리 저장해 중단 지점부터 재개하고, 제어와 시각화 통신을 분리해 영상 오류가 로봇 제어를 건드리지 못하게 한다.':
      'Errors cannot be removed, so recovery gets designed. Logs are stored separately by stage so a run resumes where it stopped, and control and visualisation are on separate links so a video fault cannot touch robot control.',
    '스캔 / 경로 생성 / 실행 로그 단계별 분리 저장':
      'Scan, path generation and run logs stored separately by stage',
    '제어 통신 / 시각화 통신 분리': 'Control and visualisation links separated',

    /* ── PR ── News · Media · Blog · FAQ ── */
    '보도자료와 소식. 최신순.': 'Press and news. Newest first.',
    '숙련공 인터뷰 — 도막 예산과 열 관리가 목록에 올랐다':
      'Veteran interview — film budget and heat management joined the list',
    '현직 광택 숙련공 인터뷰에서 클리어코트 두께 50–70 µm, 1회 광택당 제거 약 5 µm, 엣지 가압 규칙, RPM 가변, 마찰열 관리 등 반영 항목을 확보했다. 강화학습 정책과 발표 자료 양쪽의 확장 과제로 등록됐다.':
      'An interview with a working polishing veteran produced items to fold in: clear coat 50–70 µm thick, about 5 µm off per polish, edge pressure rules, variable RPM, friction-heat management. They are registered as extension tasks for both the RL policy and the deck.',
    '제8회 K-디지털 트레이닝 해커톤 보도자료 발행':
      'Press release for the 8th K-Digital Training hackathon',
    '"연마 숙련공의 기술을 따라하는 가상 로봇 팔" — 3D 스캔부터 경로 생성, 힘 제어 폴리싱, 검증 리포트까지 전 과정을 자동화한 공정 디지털 트윈으로 소개됐다. 사람이 정하는 값은 차종·부위·목표 접촉력뿐이다.':
      '"A virtual robot arm that copies a polishing veteran" — described as a process digital twin automating everything from 3D scanning through path generation, force-controlled polishing and the validation report. The only values a person sets are the model, the area and the target contact force.',
    '예선 심사과제 제출 — 팀 CACADACA': 'Preliminary submission — team CACADACA',
    '폴리트윈(PolyTwin): Isaac Sim · ROS 2 기반 숙련공 암묵지 재현 디지털 트윈 플랫폼. 정용준·신현호·부승언·김두용 4인. 첫 적용 공정은 연마이고, 여기서 세운 절차를 제조 공정 전반으로 확장한다.':
      'PolyTwin: a digital twin platform on Isaac Sim and ROS 2 that reproduces a veteran’s tacit knowledge. Four of us — Jeong Yong-jun, Shin Hyeon-ho, Bu Seung-eon, Kim Du-yong. Polishing is the first process; the procedure set here extends across manufacturing.',
    '영상과 시연 자료. 실행 화면 둘은 녹화 원본이고, 히어로 필름 하나만 연출 컷이다.':
      'Video and demos. Two are raw screen recordings of actual runs; only the hero film is staged.',
    '가상 작업장 폴리싱 실행 — Isaac Sim 화면 녹화 · 로봇 3대 협동':
      'Polishing in the virtual work cell — Isaac Sim screen recording, three robots together',
    '이송 실행 — 가상 작업장 시뮬레이션 (21초, 소리 있음)':
      'Transfer — virtual work cell simulation (21 s, with sound)',
    '랜딩 히어로 필름 — 시네마틱 프로덕트 컷': 'Landing hero film — cinematic product cut',
    '개발 과정의 기록. 잘 된 것보다 안 됐던 것을 먼저 적는다.':
      'Notes from building it. What did not work comes before what did.',
    '접촉이 가짜였다 — 폴리싱 v5 힘 제어 디버깅기':
      'The contact was fake — debugging force control in polishing v5',
    '패드가 차체에 닿지 않는데 접촉력이 잡히던 원인은 "투명 가상 스프링"이었다. 가상 힘이 실제 패드 위치가 아니라 명령값 기준으로 계산돼 제어기를 속이고 있었다. 실측 센서가 잡힐 때만 힘 제어로 전환하고 어드미턴스 댐핑을 이식해, SR 로봇의 무한 스킵 루프를 0회로 줄였다.':
      'Contact force was reading even though the pad never touched the body. The cause was an "invisible virtual spring": the virtual force was computed from the commanded position rather than the real pad position, and it was fooling the controller. Switching to force control only when the real sensor reads, plus admittance damping, took the SR robot’s infinite skip loop to zero.',
    '패드는 왜 통통 튀는가': 'Why the pad bounces',
    '강체끼리의 접촉에는 순응이 없다. PhysX 순응 접촉이 동작하지 않는 조건에서 접촉 순간 raw 물리력이 수백 N 까지 튀는 이유와, 임계감쇠 튜닝·마찰 제거로 그것을 억눌러 온 과정. 남은 답은 soft-body 패드다.':
      'Rigid body against rigid body has no compliance. Why raw contact force spikes to hundreds of newtons when PhysX compliant contact is not active, and how critical-damping tuning and removing friction have held it down. The real answer is still a soft-body pad.',
    '경로 간격은 눈대중이 아니다': 'Path spacing is not eyeballed',
    '줄 간격 = 패드 지름 × (1 – 오버랩). 150 mm 패드에 40% 오버랩이면 9 cm. 이 한 줄이 커버리지와 사이클 타임을 동시에 정하고, 로봇별 엔드이펙터에 따라 0.09 / 0.05 / 0.03 m 로 갈린다.':
      'Line spacing = pad diameter × (1 – overlap). A 150 mm pad at 40% overlap gives 9 cm. That one line sets coverage and cycle time at once, and splits into 0.09 / 0.05 / 0.03 m depending on each robot’s end effector.',
    '심사와 커피챗에서 받았거나 준비한 질문. 답은 짧게, 근거는 수치로.':
      'Questions from judging and coffee chats, and ones we prepared. Short answers, numbers for evidence.',
    '숙련공 데이터는 실측인가?': 'Is the veteran data measured?',
    '아직 아니다. 폴리싱 숙련공의 접촉력 공개 데이터셋은 존재하지 않아(2026-08 조사 기준), 문헌에 보고된 숙련공 특성으로 만든 합성 참조선과 대조한다. 모션캡처 실측 연계가 로드맵 3단계다 — 이 구분을 숨기지 않는 것이 원칙이다.':
      'Not yet. No public contact-force dataset for polishing veterans exists (as of our August 2026 search), so we compare against a synthetic baseline built from veteran characteristics reported in the literature. Motion-capture measurement is roadmap stage 3 — and not hiding that distinction is the rule.',
    '시뮬레이션과 현실의 차이는 어떻게 메우나?': 'How do you close the gap between simulation and reality?',
    '같은 접촉 조건을 Ansys 물리 환경에 넣어 교차 검증하고, 오차 5% 초과 구간은 자동 승인을 금지한다. 그래도 남는 차이 — RMPFlow 추종 지연, 강체 슬램 — 는 남은 과제로 명시한다.':
      'The same contact conditions go into Ansys for a cross-check, and anything over 5% error gets no automatic approval. What is still left — RMPFlow tracking lag, rigid-body slam — is stated as open.',
    '왜 하필 연마인가?': 'Why polishing of all things?',
    '2023년 연간 설치 기준 가공·연마 담당 로봇은 6천 대, 전체의 2%다(IFR World Robotics). 로봇에게 유독 까다로운 공정이라는 뜻이고, 접촉력이 품질을 결정하는 공정이라 3세대(물리력) 시뮬레이션이 아니면 검증할 수 없다.':
      'Of robots installed in 2023, 6,000 went to machining and polishing — 2% of the total (IFR World Robotics). That means it is unusually hard for robots, and because contact force decides the quality, nothing short of 3rd-generation physical-force simulation can validate it.',
    '기존 OLP 툴과 뭐가 다른가?': 'How is this different from existing OLP tools?',
    '상용 OLP 는 경로(어디로 갈지)만 다룬다. 폴리트윈은 힘(얼마나 누를지) 검증까지 포함하고, 가격은 상용 라이선스의 약 1/40 이다.':
      'Commercial OLP handles the path — where to go. PolyTwin also validates the force — how hard to press — at about a fortieth of a commercial licence.',
    '가격 근거는 무엇인가?': 'What is the price based on?',
    '상향식 합산이다. 특급기술자 공수(국가승인통계 단가) + GPU 연산비 + 개발비 상각 + 마진 30% = 리포트 1건 약 250만 원. 설비투자 대비 0.2%라는 비율은 목표가 아니라 이 합산의 결과다.':
      'It is added up from the bottom. Senior engineer time (at the national statistical rate) + GPU compute + amortised development + 30% margin = about 2.5M KRW per report. The 0.2% of capital outlay is not a target; it is what the sum came to.',
    '숙련공을 대체하는 것 아닌가?': 'Doesn’t this replace the veteran?',
    '반대다. 숙련공이 최종 심사위원이다. 강화학습이 만든 정책도 숙련공 평가를 통과해야 DB 에 등록되고, 그 데이터는 신입 교육 교재가 된다. 은퇴와 함께 사라지던 판단 기준을 남기는 쪽이다.':
      'The opposite. The veteran is the final judge. Even a policy from reinforcement learning has to pass their review before it enters the DB, and that data becomes the training material for newcomers. This keeps the judgement that used to disappear at retirement.',
    '핵심 데이터 보안은?': 'What about security for the core data?',
    '운영 이벤트 로그와 핵심 기술 로그를 분리 저장하고(로그 이원화), 조회·설정·제어 권한을 역할별로 나눈다. ISO 27001 Annex A 통제항목 기준 인증 심사까지 4단계 계획이 있다.':
      'Operational event logs and core technical logs are stored separately, and view, configure and control are split by role. There is a four-stage plan through to certification against ISO 27001 Annex A.',
    '다음 단계는?': 'What comes next?',
    '숙련공 인터뷰(2026-08-28)에서 확보한 항목들 — 도막 잔량 기반 종료 조건, 온도(마찰열) 상태변수, RPM 가변, 엣지 구간 규칙 — 을 강화학습 정책에 반영하는 것. 그 다음이 유리·위생도기·금형으로의 재질 확장이다.':
      'Folding the items from the veteran interview (2026-08-28) into the RL policy — a stop condition based on remaining film, temperature as a state variable, variable RPM, separate rules for edges. After that, extending materials to glass, sanitaryware and moulds.',

    /* ── 페이지 골격 ── */
    '개요 도해': 'Overview',
    '상세 도해': 'Detail',
    '홈으로 돌아가기': 'Back to home'
  };
})();
