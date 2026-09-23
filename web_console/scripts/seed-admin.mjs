/* ══════════════════════════════════════════════════════════════
   관리자 계정 시드 — 로컬에서 원격 DB(Turso)를 향해 1회 실행한다.
   서버리스에는 "최초 기동"이 없어서 server.js 의 시드가 닿지 않는다.

   사용:
     TURSO_DATABASE_URL=libsql://…  TURSO_AUTH_TOKEN=…  \
     PT_ADMIN_ID=admin  PT_ADMIN_PW='강한-비밀번호'  npm run seed

   PT_ADMIN_PW 없이 원격 DB 에 시드하려 하면 거부한다 —
   약한 비밀번호가 공개 서버에 남지 않게 하기 위해서다.
   ══════════════════════════════════════════════════════════════ */
import { createRequire } from 'node:module';
const require = createRequire(import.meta.url);

const { seedAdmin } = require('../backend/routes.js');

try {
  const r = await seedAdmin();
  if (r.created) {
    console.log('관리자 계정 생성: ' + r.loginId);
  } else {
    console.log('이미 있음: ' + r.loginId + ' — 아무것도 하지 않았다.');
  }
  process.exit(0);
} catch (err) {
  console.error('시드 실패: ' + err.message);
  process.exit(1);
}
