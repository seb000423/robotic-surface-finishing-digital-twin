/* ══════════════════════════════════════════════════════════════
   Vercel Edge Middleware — HTML 게이팅

   정적 HTML 은 CDN 이 서빙하므로 302 게이팅을 여기서 한다.
   Edge 에서는 DB 를 부르지 않는다 — 쿠키의 HMAC 서명과 만료만
   검증한다 (server/auth.js 의 signSession 과 같은 형식:
   v1.<b64url(payload)>.<b64url(HMAC-SHA256)>).

   절충: 로그아웃·강제 만료된 토큰도 만료 시각까지는 이 게이트를
   통과한다. HTML 에는 데이터가 없고 데이터는 전부 /api/* 뒤에서

   ※ 접근 정책 목록은 backend/server.js 와 같이 고칠 것.
   ══════════════════════════════════════════════════════════════ */

export const config = {
  /* /api/ 와 /assets/ 는 아예 안 태운다 — 에지 호출 낭비.
     나머지는 받되 아래에서 .html 만 본다 */
  matcher: ['/((?!api/|assets/).*)'],
};

const PUBLIC_PAGES = new Set(['/', '/index.html']);
const ADMIN_PAGES = new Set(['/admin.html']);
const COOKIE = 'pt_sid';

const enc = new TextEncoder();

function b64urlToBytes(s) {
  const b64 = s.replace(/-/g, '+').replace(/_/g, '/');
  const bin = atob(b64 + '='.repeat((4 - (b64.length % 4)) % 4));
  const out = new Uint8Array(bin.length);
  for (let i = 0; i < bin.length; i++) out[i] = bin.charCodeAt(i);
  return out;
}

/** 서명·만료 검증 — auth.js parseSession 의 Edge 판 */
async function parseSession(token, secret) {
  try {
    const [v, body, sig] = String(token || '').split('.');
    if (v !== 'v1' || !body || !sig) return null;
    const key = await crypto.subtle.importKey(
      'raw', enc.encode(secret), { name: 'HMAC', hash: 'SHA-256' }, false, ['verify'],
    );
    const ok = await crypto.subtle.verify('HMAC', key, b64urlToBytes(sig), enc.encode(body));
    if (!ok) return null;
    const payload = JSON.parse(new TextDecoder().decode(b64urlToBytes(body)));
    if (!payload || typeof payload.exp !== 'number' || payload.exp < Date.now()) return null;
    return payload;
  } catch {
    return null;
  }
}

function readCookie(header, name) {
  for (const part of String(header || '').split(';')) {
    const i = part.indexOf('=');
    if (i > -1 && part.slice(0, i).trim() === name) {
      return decodeURIComponent(part.slice(i + 1).trim());
    }
  }
  return null;
}

function forbidden(message) {
  return new Response(
    '<!doctype html><meta charset="utf-8"><title>403</title>'
    + '<body style="margin:0;background:#0E1216;color:#A8B0BA;'
    + 'font:14px/1.7 Pretendard,system-ui,sans-serif;padding:64px 32px">'
    + '<p style="font-size:11px;letter-spacing:.12em;color:#5F6873;margin:0 0 12px">ERROR 403</p>'
    + '<p style="margin:0 0 20px;color:#F2F4F7;font-size:18px">' + message + '</p>'
    + '<a style="color:#3E9DBE;text-decoration:none;border-bottom:1px solid #2A6478" href="/">돌아가기</a>',
    { status: 403, headers: { 'Content-Type': 'text/html; charset=utf-8', 'Cache-Control': 'no-store' } },
  );
}

export default async function middleware(req) {
  const url = new URL(req.url);
  let pathname;
  try { pathname = decodeURIComponent(url.pathname); } catch { pathname = url.pathname; }

  /* .html 만 게이팅한다. 자산·JSON 은 그대로 통과 */
  const isHtml = pathname === '/' || /\.html$/i.test(pathname);
  if (!isHtml || PUBLIC_PAGES.has(pathname)) return;

  const secret = process.env.PT_SECRET;
  if (!secret) {
    /* 키가 없으면 전부 잠근다 — 실수로 열리는 쪽보다 낫다 */
    console.error('[middleware] PT_SECRET 미설정 — 보호 화면을 전부 차단합니다.');
  }

  const token = readCookie(req.headers.get('cookie'), COOKIE);
  const sess = secret ? await parseSession(token, secret) : null;

  if (!sess) {
    return new Response(null, {
      status: 302,
      headers: {
        Location: '/?login=1&next=' + encodeURIComponent(pathname),
        'Cache-Control': 'no-store',
      },
    });
  }
  if (ADMIN_PAGES.has(pathname) && sess.role !== 'admin') {
    return forbidden('관리자 권한이 필요한 화면입니다.');
  }
  /* 통과 — 아무것도 반환하지 않으면 CDN 이 이어서 서빙한다 */
}
