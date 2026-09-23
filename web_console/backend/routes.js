/* ══════════════════════════════════════════════════════════════
   웹 웹 UI API 라우트 — 로컬 서버와 Vercel 함수가 공유하는 한 벌

   backend/server.js (로컬) →  handleApi(req, res, pathname)
   api/[[...route]].js      →  동일 호출

   두 환경 모두 Node req/res 다. 여기 없는 것: 정적 파일 서빙
   (로컬은 server.js, Vercel 은 CDN+middleware.js 가 맡는다).
   ══════════════════════════════════════════════════════════════ */
'use strict';

const store = require('./db');
const { verifyPassword, signSession, parseSession } = require('./auth');

const SESSION_TTL = 1000 * 60 * 60 * 12;   /* 12시간 */
const COOKIE = 'pt_sid';

/* ── 유틸 ─────────────────────────────────────────────────── */
function json(res, code, body) {
  const buf = Buffer.from(JSON.stringify(body));
  res.writeHead(code, {
    'Content-Type': 'application/json; charset=utf-8',
    'Content-Length': buf.length,
    'Cache-Control': 'no-store',
  });
  res.end(buf);
}

function parseCookies(header) {
  const out = {};
  String(header || '').split(';').forEach((part) => {
    const i = part.indexOf('=');
    if (i < 0) return;
    out[part.slice(0, i).trim()] = decodeURIComponent(part.slice(i + 1).trim());
  });
  return out;
}

function readBody(req, limit) {
  /* Vercel Node 런타임은 JSON 본문을 미리 파싱해 req.body 에 둔다.
     그때 스트림을 다시 읽으면 영영 안 끝난다 — 있으면 그걸 쓴다 */
  if (req.body !== undefined) {
    if (typeof req.body === 'object' && req.body !== null) return Promise.resolve(req.body);
    if (typeof req.body === 'string') {
      try { return Promise.resolve(req.body ? JSON.parse(req.body) : {}); }
      catch { return Promise.reject(new Error('invalid json')); }
    }
  }
  const cap = limit || 64 * 1024;
  return new Promise((resolve, reject) => {
    const chunks = [];
    let n = 0;
    req.on('data', (c) => {
      n += c.length;
      if (n > cap) { reject(new Error('payload too large')); req.destroy(); return; }
      chunks.push(c);
    });
    req.on('end', () => {
      const raw = Buffer.concat(chunks).toString('utf8');
      if (!raw) { resolve({}); return; }
      try { resolve(JSON.parse(raw)); } catch { reject(new Error('invalid json')); }
    });
    req.on('error', reject);
  });
}


function publicUser(u) {
  if (!u) return null;
  return {
    id: u.id,
    loginId: u.login_id,
    name: u.name,
    role: u.role,
    status: u.status,
    createdAt: u.created_at,
    lastLoginAt: u.last_login_at,
  };
}

/* 저장 행 → 클라이언트 형태. payload 를 펴서 예전 localStorage
   레코드와 같은 모양으로 돌려준다 — 라이브러리 렌더 코드를 그대로 쓴다 */
function publicEntry(row) {
  let body = {};
  try { body = JSON.parse(row.payload); } catch { body = {}; }
  const at = new Date(Number(row.created_at));
  const pad = (n) => String(n).padStart(2, '0');
  return {
    id: String(row.ref_id),
    rowId: Number(row.id),
    seg: row.seg || '',
    name: row.name || '',
    date: pad(at.getMonth() + 1) + '-' + pad(at.getDate()) + ' ' + pad(at.getHours()) + ':' + pad(at.getMinutes()),
    savedAt: Number(row.created_at),
    owner: row.owner_name || row.owner_login || '',
    env: body.env || null,
    rl: body.rl || null,
    ref: body.ref || null,
  };
}

/* 프록시(Vercel) 뒤에서는 소켓 주소가 프록시다 — 헤더 우선 */
const clientIp = (req) =>
  String(req.headers['x-forwarded-for'] || '').split(',')[0].trim()
  || (req.socket && req.socket.remoteAddress) || '?';

const isHttps = (req) =>
  req.headers['x-forwarded-proto'] === 'https' || (req.socket && req.socket.encrypted);

function sessionCookie(req, token, maxAgeSec) {
  return COOKIE + '=' + token + '; Path=/; HttpOnly; SameSite=Lax; Max-Age=' + maxAgeSec
    + (isHttps(req) ? '; Secure' : '');
}

/** 쿠키 → 서명 검증 → DB 대조. 강제 만료(revocation)는 여기서 잡힌다 */
async function sessionOf(req) {
  const token = parseCookies(req.headers.cookie)[COOKIE];
  if (!parseSession(token)) return null;      /* 서명·만료 — 위조는 DB 도 안 간다 */
  return store.readSession(token);            /* 로그아웃·강제 만료 대조 */
}

/* ── 로그인 시도 제한 ── 무차별 대입을 늦춘다.
   서버리스에서는 인스턴스마다 따로 노는 best-effort 다 — 콜드 스타트가
   카운터를 비우지만, scrypt 비용 + 계정 열거 차단이 본 방어선이다 ── */
const attempts = new Map();
const RL = { max: 10, windowMs: 5 * 60 * 1000 };

function rateLimited(key) {
  const t = Date.now();
  const rec = attempts.get(key);
  if (!rec || rec.resetAt < t) { attempts.set(key, { n: 1, resetAt: t + RL.windowMs }); return false; }
  rec.n += 1;
  return rec.n > RL.max;
}

/* ── 검증 ─────────────────────────────────────────────────── */
const ID_RE = /^[A-Za-z0-9._-]{4,32}$/;

function validateSignup(loginId, password, name) {
  if (!loginId || !ID_RE.test(loginId)) return 'ID는 영문·숫자·. _ - 조합 4~32자여야 합니다.';
  if (!password || password.length < 8) return '비밀번호는 8자 이상이어야 합니다.';
  if (password.length > 200) return '비밀번호가 너무 깁니다.';
  if (name && String(name).length > 40) return '이름이 너무 깁니다.';
  return '';
}

/* ══════════════════════════════════════════════════════════════
   API
   ══════════════════════════════════════════════════════════════ */
async function handleApi(req, res, pathname) {
  const method = req.method;

  /* ── 가입 ── 기본 상태는 pending. 관리자 승인 전에는 못 들어온다 ── */
  if (pathname === '/api/signup' && method === 'POST') {
    const body = await readBody(req);
    const loginId = String(body.loginId || '').trim();
    const name = String(body.name || '').trim();
    const password = String(body.password || '');

    const bad = validateSignup(loginId, password, name);
    if (bad) return json(res, 400, { error: bad });
    if (await store.findByLogin(loginId)) return json(res, 409, { error: '이미 사용 중인 ID입니다.' });

    const u = await store.createUser({ loginId, name, password, role: 'engineer', status: 'pending' });
    await store.log(loginId, 'signup', loginId, 'from ' + clientIp(req));
    return json(res, 201, {
      user: publicUser(u),
      message: '가입 신청이 접수되었습니다. 관리자 승인 후 접속할 수 있습니다.',
    });
  }

  /* ── 로그인 ── */
  if (pathname === '/api/login' && method === 'POST') {
    const body = await readBody(req);
    const loginId = String(body.loginId || '').trim();
    const password = String(body.password || '');
    const key = clientIp(req) + '|' + loginId.toLowerCase();

    if (rateLimited(key)) {
      await store.log(loginId || '?', 'login.throttled', loginId, clientIp(req));
      return json(res, 429, { error: '시도가 너무 많습니다. 5분 후 다시 시도하세요.' });
    }
    if (!loginId || !password) return json(res, 400, { error: 'ID와 비밀번호를 입력하세요.' });

    const u = await store.findByLogin(loginId);
    /* ID 유무를 알려주지 않는다 — 계정 열거를 막는다 */
    if (!u || !verifyPassword(password, u.pw_hash)) {
      await store.log(loginId, 'login.fail', loginId, clientIp(req));
      return json(res, 401, { error: 'ID 또는 비밀번호가 올바르지 않습니다.' });
    }
    if (u.status === 'pending') return json(res, 403, { error: '관리자 승인 대기 중인 계정입니다.' });
    if (u.status === 'suspended') return json(res, 403, { error: '정지된 계정입니다. 관리자에게 문의하세요.' });

    attempts.delete(key);
    /* 토큰 = HMAC 서명 (Edge 게이팅용) + DB 행 (revocation 용). 형식은 auth.js 참고 */
    const token = signSession({ uid: u.id, role: u.role, exp: Date.now() + SESSION_TTL });
    await store.createSession(u.id, token, SESSION_TTL);
    await store.touchLogin(u.id);
    await store.log(u.login_id, 'login', u.login_id, clientIp(req));
    
    store.sweepSessions().catch(() => {});

    res.setHeader('Set-Cookie', sessionCookie(req, token, SESSION_TTL / 1000));
    return json(res, 200, { user: publicUser(await store.findById(u.id)) });
  }

  /* ── 로그아웃 ── */
  if (pathname === '/api/logout' && method === 'POST') {
    const token = parseCookies(req.headers.cookie)[COOKIE];
    const sess = await store.readSession(token);
    if (token) await store.dropSession(token);
    if (sess) await store.log(sess.login_id, 'logout', sess.login_id, '');
    res.setHeader('Set-Cookie', sessionCookie(req, '', 0));
    return json(res, 200, { ok: true });
  }

  
  if (pathname === '/api/me' && method === 'GET') {
    const sess = await sessionOf(req);
    return json(res, 200, { user: sess ? publicUser(sess) : null });
  }

  /* ══ 숙련공 정답 데이터 ═══════════════════════════════════
     예전에는 화면이 데이터셋/*.json 을 직접 fetch 했다. 이제 DB 다.
     돌려주는 모양은 그 JSON 과 똑같이 맞춘다 — 화면 쪽은 URL 만
     바뀌고 파싱 코드는 그대로 쓴다.
     원본 CSV(183MB)는 DB 에 넣지 않는다. 세그먼트가 들고 있는
     byteStart/byteEnd 로 여전히 정적 파일에 Range 요청을 건다. ══ */
  if (pathname.indexOf('/api/dataset/') === 0) {
    const sess = await sessionOf(req);
    if (!sess) return json(res, 401, { error: '로그인이 필요합니다.' });
    if (sess.status !== 'active') return json(res, 403, { error: '승인 대기 중인 계정입니다.' });
    if (method !== 'GET') return json(res, 405, { error: '지원하지 않는 요청입니다.' });

    if (pathname === '/api/dataset/seg-best-kpi') {
      const meta = await store.getMeta('seg_best_kpi');
      const rows = await store.listSegments();
      if (!meta && !rows.length) {
        return json(res, 503, { error: '숙련공 정답 데이터가 아직 시드되지 않았습니다 (npm run seed:data).' });
      }
      let head = {};
      try { head = meta ? JSON.parse(meta.payload) : {}; } catch { head = {}; }
      const segments = rows.map((r) => { try { return JSON.parse(r.payload); } catch { return null; } })
        .filter(Boolean);
      return json(res, 200, { ...head, segments });
    }

    if (pathname === '/api/dataset/quality-kpi') {
      const meta = await store.getMeta('quality_kpi');
      if (!meta) {
        return json(res, 503, { error: '품질 기준 데이터가 아직 시드되지 않았습니다 (npm run seed:data).' });
      }
      let body = {};
      try { body = JSON.parse(meta.payload); } catch { body = {}; }
      return json(res, 200, body);
    }

    return json(res, 404, { error: '없는 데이터셋입니다.' });
  }

  /* ══ 숙련공 데이터 라이브러리 ═════════════════════════════
     ① 웹 UI이 합격시킨 기록을 쓰고 ④ 라이브러리가 읽는다.
     예전에는 localStorage 'polytwin_saved' 였다 — 브라우저 하나에
     갇혀 있어서 다른 PC 로 가면 사라지고 저장자도 알 수 없었다.
     읽기는 로그인한 전원 공유, 지우기는 본인 또는 admin. ══ */
  if (pathname === '/api/library' || pathname.indexOf('/api/library/') === 0) {
    const sess = await sessionOf(req);
    if (!sess) return json(res, 401, { error: '로그인이 필요합니다.' });
    if (sess.status !== 'active') return json(res, 403, { error: '승인 대기 중인 계정입니다.' });

    if (pathname === '/api/library' && method === 'GET') {
      const rows = await store.listLibrary(200);
      return json(res, 200, { entries: rows.map(publicEntry) });
    }

    if (pathname === '/api/library' && method === 'POST') {
      /* ref(세그먼트 요약 스냅샷)까지 통째로 온다 — 한 건 약 7KB.
         기본 64KB 로는 여유가 빠듯해 이 경로만 올린다 */
      const body = await readBody(req, 256 * 1024);
      const refId = String(body.id || '').trim();
      const seg = String(body.seg || '').trim();
      const name = String(body.name || '').trim();

      if (!refId || refId.length > 64) return json(res, 400, { error: '기록 ID가 올바르지 않습니다.' });
      if (!body.ref || !body.rl) return json(res, 400, { error: '저장할 기록이 비어 있습니다.' });
      if (name.length > 200) return json(res, 400, { error: '이름이 너무 깁니다.' });

      /* 클라이언트가 보낸 것 중 라이브러리가 실제로 읽는 것만 남긴다 */
      const payload = JSON.stringify({ env: body.env || null, rl: body.rl, ref: body.ref });
      if (payload.length > 200 * 1024) return json(res, 413, { error: '기록이 너무 큽니다.' });

      const { entry, created } = await store.createLibraryEntry({
        userId: Number(sess.id), refId, seg, name, payload,
      });
      if (created) await store.log(sess.login_id, 'library.save', refId, seg);
      return json(res, created ? 201 : 200, {
        entry: publicEntry({
          ...entry, owner_login: sess.login_id, owner_name: sess.name,
        }),
        created,
      });
    }

    const m = pathname.match(/^\/api\/library\/(\d+)$/);
    if (m && method === 'DELETE') {
      const id = Number(m[1]);
      const row = await store.findLibraryEntry(id);
      if (!row) return json(res, 404, { error: '없는 기록입니다.' });
      const mine = Number(row.user_id) === Number(sess.id);
      if (!mine && sess.role !== 'admin') {
        return json(res, 403, { error: '본인이 저장한 기록만 지울 수 있습니다.' });
      }
      await store.deleteLibraryEntry(id);
      await store.log(sess.login_id, 'library.delete', String(row.ref_id), mine ? '' : 'by admin');
      return json(res, 200, { ok: true });
    }

    return json(res, 405, { error: '지원하지 않는 요청입니다.' });
  }

  /* ══ 관리자 전용 ══════════════════════════════════════════ */
  if (pathname.indexOf('/api/admin/') === 0) {
    const sess = await sessionOf(req);
    if (!sess) return json(res, 401, { error: '로그인이 필요합니다.' });
    if (sess.role !== 'admin' || sess.status !== 'active') {
      return json(res, 403, { error: '관리자 권한이 필요합니다.' });
    }

    if (pathname === '/api/admin/users' && method === 'GET') {
      return json(res, 200, { users: (await store.listUsers()).map(publicUser) });
    }

    if (pathname === '/api/admin/audit' && method === 'GET') {
      return json(res, 200, { entries: await store.recentAudit(60) });
    }

    const m = pathname.match(/^\/api\/admin\/users\/(\d+)$/);
    if (m) {
      const id = Number(m[1]);
      const target = await store.findById(id);
      if (!target) return json(res, 404, { error: '없는 계정입니다.' });

      if (method === 'PATCH') {
        const body = await readBody(req);
        const changes = [];

        if (body.status !== undefined) {
          if (['pending', 'active', 'suspended'].indexOf(body.status) < 0) {
            return json(res, 400, { error: '알 수 없는 상태입니다.' });
          }
          /* 마지막 관리자를 스스로 잠그지 못하게 한다 */
          if (target.role === 'admin' && target.status === 'active' && body.status !== 'active'
            && (await store.activeAdminCount()) <= 1) {
            return json(res, 409, { error: '활성 관리자가 한 명뿐입니다. 먼저 다른 관리자를 지정하세요.' });
          }
          await store.setStatus(id, body.status);
          if (body.status !== 'active') await store.dropSessionsOfUser(id);
          changes.push('status=' + body.status);
        }

        if (body.role !== undefined) {
          if (['admin', 'engineer'].indexOf(body.role) < 0) {
            return json(res, 400, { error: '알 수 없는 역할입니다.' });
          }
          if (target.role === 'admin' && body.role !== 'admin' && (await store.activeAdminCount()) <= 1) {
            return json(res, 409, { error: '활성 관리자가 한 명뿐입니다. 먼저 다른 관리자를 지정하세요.' });
          }
          await store.setRole(id, body.role);
          changes.push('role=' + body.role);
        }

        if (body.password !== undefined) {
          if (String(body.password).length < 8) {
            return json(res, 400, { error: '비밀번호는 8자 이상이어야 합니다.' });
          }
          await store.setPassword(id, String(body.password));
          await store.dropSessionsOfUser(id);   
          changes.push('password reset');
        }

        if (!changes.length) return json(res, 400, { error: '바꿀 항목이 없습니다.' });
        await store.log(sess.login_id, 'admin.update', target.login_id, changes.join(', '));
        return json(res, 200, { user: publicUser(await store.findById(id)) });
      }

      if (method === 'DELETE') {
        if (target.id === sess.id) return json(res, 409, { error: '자기 계정은 삭제할 수 없습니다.' });
        if (target.role === 'admin' && target.status === 'active' && (await store.activeAdminCount()) <= 1) {
          return json(res, 409, { error: '활성 관리자가 한 명뿐입니다.' });
        }
        await store.deleteUser(id);
        await store.log(sess.login_id, 'admin.delete', target.login_id, '');
        return json(res, 200, { ok: true });
      }
    }

    return json(res, 404, { error: '없는 경로입니다.' });
  }

  return json(res, 404, { error: '없는 경로입니다.' });
}

/* ── 관리자 계정 씨앗 ─────────────────────────────────────────
   로컬: server.js 가 기동 시 부른다 (기본값 허용 — 데모).
   Turso: PT_ADMIN_PW 없이는 거부한다 — 기본 비밀번호를 공개
   서버에 두는 사고를 원천 차단. scripts/seed-admin.mjs 로 실행. */
async function seedAdmin() {
  const loginId = process.env.PT_ADMIN_ID || 'admin';
  // 비밀번호를 지정하지 않으면 매번 무작위로 만들고 최초 1회만 웹 UI에 출력한다
  const password = process.env.PT_ADMIN_PW || require('crypto').randomBytes(9).toString('base64url');
  if (process.env.TURSO_DATABASE_URL && !process.env.PT_ADMIN_PW) {
    throw new Error('원격 DB 에는 PT_ADMIN_PW 를 명시해야 시드합니다 — 기본 비밀번호 금지.');
  }
  if (await store.findByLogin(loginId)) return { loginId, created: false };
  await store.createUser({ loginId, name: '시스템 관리자', password, role: 'admin', status: 'active' });
  await store.log('system', 'seed.admin', loginId, '');
  return { loginId, password, created: true };
}

module.exports = {
  handleApi, sessionOf, seedAdmin, json,
  COOKIE, SESSION_TTL, parseCookies, publicUser,
};
