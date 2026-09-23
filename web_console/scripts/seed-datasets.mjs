/* ══════════════════════════════════════════════════════════════
   숙련공 정답 데이터 시드 — 데이터셋/*.json → DB

   예전에는 ①·④ 화면이 이 JSON 두 개를 직접 fetch 했다. 이제
   segments / dataset_meta 테이블에 넣고 /api/dataset/* 로 낸다.

   사용:
     npm run seed:data                       # 로컬 backend/data/
     TURSO_DATABASE_URL=… TURSO_AUTH_TOKEN=… npm run seed:data   # 원격

   원본 JSON 은 지우지 않는다. 다시 만드는 법은 데이터셋/make_seed.py
   에 있고, 이 스크립트를 다시 돌리면 같은 seg 키를 덮어쓴다.

   CSV 원본(183MB)은 옮기지 않는다 — 세그먼트가 들고 있는
   byteStart/byteEnd 로 정적 파일에 Range 요청을 그대로 건다.
   ══════════════════════════════════════════════════════════════ */
import { createRequire } from 'node:module';
import { readFileSync } from 'node:fs';
import { fileURLToPath } from 'node:url';
import path from 'node:path';

const require = createRequire(import.meta.url);
const store = require('../backend/db.js');

const ROOT = path.join(path.dirname(fileURLToPath(import.meta.url)), '..');
const DIR = path.join(ROOT, 'frontend', '데이터셋');

function readJson(name) {
  try {
    return JSON.parse(readFileSync(path.join(DIR, name), 'utf8'));
  } catch (err) {
    throw new Error(name + ' 을 읽지 못했습니다: ' + err.message);
  }
}

/* ── ① 숙련공 정답 데이터 ── segments 는 행으로, 나머지는 머리말로 ── */
const seed = readJson('seg_best_kpi.json');
const segments = Array.isArray(seed.segments) ? seed.segments : [];
if (!segments.length) throw new Error('seg_best_kpi.json 에 segments 가 없습니다.');

const { segments: _drop, ...head } = seed;
await store.putMeta('seg_best_kpi', JSON.stringify(head));

let n = 0;
for (let i = 0; i < segments.length; i++) {
  const sg = segments[i];
  if (!sg || !sg.seg) continue;
  await store.putSegment({
    seg: String(sg.seg),
    robot: String(sg.robot || ''),
    inband: typeof sg.inband === 'number' ? sg.inband : null,
    ord: i,
    payload: JSON.stringify(sg),
  });
  n += 1;
}

/* ── ② 품질 기준 ── 작아서 통째로 둔다 ── */
await store.putMeta('quality_kpi', JSON.stringify(readJson('quality_kpi.json')));

console.log('세그먼트 ' + n + '개 · 머리말 2건 시드 완료 → ' + store.path);
