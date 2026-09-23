/* ══════════════════════════════════════════════════════════════
   scrypt 는 node:crypto 내장이다. bcrypt 를 받으려고 npm 을
   끌어들이지 않는다 — 이 프로젝트의 의존성은 0개로 유지한다.
   ══════════════════════════════════════════════════════════════ */
'use strict';

const crypto = require('node:crypto');

const SCRYPT = { N: 16384, r: 8, p: 1, keylen: 64 };

/** 저장 형식: scrypt$N$r$p$salt(hex)$hash(hex) */
function hashPassword(plain) {
  const salt = crypto.randomBytes(16);
  const key = crypto.scryptSync(plain, salt, SCRYPT.keylen, {
    N: SCRYPT.N, r: SCRYPT.r, p: SCRYPT.p, maxmem: 128 * SCRYPT.N * SCRYPT.r * 2,
  });
  return ['scrypt', SCRYPT.N, SCRYPT.r, SCRYPT.p, salt.toString('hex'), key.toString('hex')].join('$');
}

function verifyPassword(plain, stored) {
  try {
    const [tag, N, r, p, saltHex, hashHex] = String(stored).split('$');
    if (tag !== 'scrypt') return false;
    const salt = Buffer.from(saltHex, 'hex');
    const expected = Buffer.from(hashHex, 'hex');
    const key = crypto.scryptSync(plain, salt, expected.length, {
      N: +N, r: +r, p: +p, maxmem: 128 * (+N) * (+r) * 2,
    });
    /* 길이가 다르면 timingSafeEqual 이 던진다. 먼저 걸러낸다 */
    if (key.length !== expected.length) return false;
    return crypto.timingSafeEqual(key, expected);
  } catch {
    return false;
  }
}

const newSessionToken = () => crypto.randomBytes(32).toString('base64url');

/* ══════════════════════════════════════════════════════════════

   형식: v1.<b64url(payload JSON)>.<b64url(HMAC-SHA256)>
   payload = { uid, role, exp(ms) }

   왜 서명 토큰인가: Vercel 에서 정적 HTML 은 CDN 이 서빙하고
   게이팅은 Edge Middleware 가 한다. Edge 에서 DB 를 부를 수 없으니
   서명·만료만 보고 통과/차단을 정한다. DB 의 sessions 행은 그대로
   유지한다 — 로그아웃·강제 만료(revocation)는 API 계층이 DB 대조로
   잡는다. 절충: 강제 만료된 토큰도 만료 시각까지는 HTML 게이트를
   통과할 수 있다. HTML 자체에는 데이터가 없고 데이터는 전부
   /api/* 뒤에 있으므로 수용한다.
   ══════════════════════════════════════════════════════════════ */
const DEV_SECRET = 'pt-dev-secret-not-for-production';

function sessionSecret() {
  const s = process.env.PT_SECRET;
  if (s) return s;
  if (!sessionSecret.warned) {
    sessionSecret.warned = true;
    console.warn('[auth] PT_SECRET 미설정 — 개발용 고정 키를 씁니다. 배포에서는 반드시 설정하세요.');
  }
  return DEV_SECRET;
}

const b64url = (buf) => Buffer.from(buf).toString('base64url');

function signSession(payload) {
  const body = b64url(JSON.stringify(payload));
  const sig = crypto.createHmac('sha256', sessionSecret()).update(body).digest('base64url');
  return 'v1.' + body + '.' + sig;
}

/** 서명·만료 검증. 실패하면 null. DB 대조는 하지 않는다 — 호출자 몫. */
function parseSession(token) {
  try {
    const [v, body, sig] = String(token || '').split('.');
    if (v !== 'v1' || !body || !sig) return null;
    const expect = crypto.createHmac('sha256', sessionSecret()).update(body).digest();
    const got = Buffer.from(sig, 'base64url');
    if (got.length !== expect.length || !crypto.timingSafeEqual(got, expect)) return null;
    const payload = JSON.parse(Buffer.from(body, 'base64url').toString('utf8'));
    if (!payload || typeof payload.exp !== 'number' || payload.exp < Date.now()) return null;
    return payload;
  } catch {
    return null;
  }
}

module.exports = { hashPassword, verifyPassword, newSessionToken, signSession, parseSession, sessionSecret };
