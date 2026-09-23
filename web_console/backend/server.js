/* ══════════════════════════════════════════════════════════════
   웹 웹 UI 서버 — 정적 파일 + 계정 API (로컬 개발·데모용)
   의존성 0개. node backend/server.js 만으로 뜬다.

   화면 보호는 서버에서 한다. 클라이언트 가드는 개발자도구로
   지워지므로 보호가 아니다. 여기서 막으면 HTML 이 아예 안 나간다.

   API 로직은 backend/routes.js 에 있다 — Vercel 함수
   (api/[[...route]].js)와 같은 코드를 쓴다. 여기는 정적 서빙과
   HTML 게이팅만 남았다. Vercel 에서 이 파일의 역할은
   CDN + middleware.js 가 대신한다.
   ══════════════════════════════════════════════════════════════ */
'use strict';

const http = require('node:http');
const fs = require('node:fs');
const fsp = require('node:fs/promises');
const path = require('node:path');
const { URL } = require('node:url');

const store = require('./db');
const { handleApi, sessionOf, seedAdmin, json } = require('./routes');

/* 정적 루트는 frontend/ 다. 백엔드 코드(backend/)와 부록(appendix/)은
   같은 저장소에 있어도 절대 서빙되지 않는다 — 경로 탈출 검사가 막는다 */
const ROOT = path.join(__dirname, '..', 'frontend');
const PORT = Number(process.env.PORT || 8000);
const HOST = process.env.HOST || '127.0.0.1';

/* ── 접근 정책 ─────────────────────────────────────────────────
   .html 은 기본 차단(deny-by-default). 새 화면을 추가했을 때
   실수로 열려 있는 쪽보다 실수로 막혀 있는 쪽이 안전하다.
   ※ middleware.js 의 목록과 반드시 같이 고칠 것.
   ────────────────────────────────────────────────────────────── */
const PUBLIC_PAGES = new Set(['/', '/index.html']);
const ADMIN_PAGES = new Set(['/admin.html']);

const MIME = {
  '.html': 'text/html; charset=utf-8',
  '.css': 'text/css; charset=utf-8',
  '.js': 'text/javascript; charset=utf-8',
  '.mjs': 'text/javascript; charset=utf-8',
  '.json': 'application/json; charset=utf-8',
  '.svg': 'image/svg+xml',
  '.png': 'image/png',
  '.jpg': 'image/jpeg',
  '.jpeg': 'image/jpeg',
  '.webp': 'image/webp',
  '.ico': 'image/x-icon',
  '.gif': 'image/gif',
  '.woff2': 'font/woff2',
  '.woff': 'font/woff',
  '.ttf': 'font/ttf',
  '.glb': 'model/gltf-binary',
  '.gltf': 'model/gltf+json',
  '.mp4': 'video/mp4',
  '.webm': 'video/webm',
  '.mp3': 'audio/mpeg',
  '.wasm': 'application/wasm',
  '.csv': 'text/csv; charset=utf-8',
  '.txt': 'text/plain; charset=utf-8',
  '.md': 'text/plain; charset=utf-8',
  '.map': 'application/json',
};

function redirect(res, to) {
  res.writeHead(302, { Location: to, 'Cache-Control': 'no-store' });
  res.end();
}

/* ══════════════════════════════════════════════════════════════
   정적 파일
   ══════════════════════════════════════════════════════════════ */
function sendFile(req, res, filePath, stat) {
  const ext = path.extname(filePath).toLowerCase();
  const type = MIME[ext] || 'application/octet-stream';
  const range = req.headers.range;

  /* 영상 탐색(seek)에는 Range 응답이 필요하다 */
  if (range && /^bytes=/.test(range)) {
    const parts = range.replace('bytes=', '').split('-');
    const start = parts[0] ? Number(parts[0]) : 0;
    let end = parts[1] ? Number(parts[1]) : stat.size - 1;
    if (Number.isNaN(start) || Number.isNaN(end) || start > end || start >= stat.size) {
      res.writeHead(416, { 'Content-Range': 'bytes */' + stat.size });
      res.end();
      return;
    }
    end = Math.min(end, stat.size - 1);
    res.writeHead(206, {
      'Content-Type': type,
      'Content-Length': end - start + 1,
      'Content-Range': 'bytes ' + start + '-' + end + '/' + stat.size,
      'Accept-Ranges': 'bytes',
    });
    if (req.method === 'HEAD') { res.end(); return; }
    fs.createReadStream(filePath, { start, end }).pipe(res);
    return;
  }

  res.writeHead(200, {
    'Content-Type': type,
    'Content-Length': stat.size,
    'Accept-Ranges': 'bytes',
    'Last-Modified': stat.mtime.toUTCString(),
    /* HTML 은 캐시하지 않는다 — 로그아웃 후 뒤로가기로 보호 화면이 뜨면 안 된다.
       .js 도 캐시하지 않는다 — 파일명이 고정이라 한 번 물면 고쳐도 안 바뀐다.
       (auth-client.js 를 고치고도 브라우저가 옛 파일을 계속 쓴 적이 있다) */
    'Cache-Control': ext === '.html' || ext === '.js'
      ? 'no-store'
      : 'public, max-age=3600',
  });
  if (req.method === 'HEAD') { res.end(); return; }
  fs.createReadStream(filePath).pipe(res);
}

function errorPage(res, code, message) {
  res.writeHead(code, { 'Content-Type': 'text/html; charset=utf-8', 'Cache-Control': 'no-store' });
  res.end('<!doctype html><meta charset="utf-8"><title>' + code + '</title>'
    + '<body style="margin:0;background:#0E1216;color:#A8B0BA;'
    + 'font:14px/1.7 Pretendard,system-ui,sans-serif;padding:64px 32px">'
    + '<p style="font-size:11px;letter-spacing:.12em;color:#5F6873;margin:0 0 12px">ERROR ' + code + '</p>'
    + '<p style="margin:0 0 20px;color:#F2F4F7;font-size:18px">' + message + '</p>'
    + '<a style="color:#3E9DBE;text-decoration:none;border-bottom:1px solid #2A6478" href="/">돌아가기</a>');
}

/* ══════════════════════════════════════════════════════════════
   라우팅
   ══════════════════════════════════════════════════════════════ */
const server = http.createServer(async (req, res) => {
  let pathname;
  try {
    pathname = decodeURIComponent(new URL(req.url, 'http://localhost').pathname);
  } catch {
    res.writeHead(400);
    res.end('bad request');
    return;
  }

  if (pathname.indexOf('/api/') === 0) {
    try {
      await handleApi(req, res, pathname);
    } catch (err) {
      const isJson = err && err.message === 'invalid json';
      if (!res.headersSent) {
        json(res, isJson ? 400 : 500, { error: isJson ? '요청 형식이 올바르지 않습니다.' : '서버 오류입니다.' });
      } else {
        res.end();
      }
    }
    return;
  }

  if (req.method !== 'GET' && req.method !== 'HEAD') {
    res.writeHead(405, { Allow: 'GET, HEAD' });
    res.end();
    return;
  }

  /* 경로 탈출 차단 */
  const rel = path.normalize(pathname).replace(/^[\\/]+/, '');
  const filePath = rel === '' ? path.join(ROOT, 'index.html') : path.join(ROOT, rel);
  if (filePath !== ROOT && filePath.indexOf(ROOT + path.sep) !== 0) {
    errorPage(res, 403, '접근할 수 없는 경로입니다.');
    return;
  }

  /* ── 접근 제어 ── .html 만 대상. 자산은 열어 둔다 ── */
  const isHtml = pathname === '/' || /\.html$/i.test(pathname);
  if (isHtml && !PUBLIC_PAGES.has(pathname)) {
    const sess = await sessionOf(req);
    if (!sess) {
      redirect(res, '/?login=1&next=' + encodeURIComponent(pathname));
      return;
    }
    if (ADMIN_PAGES.has(pathname) && sess.role !== 'admin') {
      errorPage(res, 403, '관리자 권한이 필요한 화면입니다.');
      return;
    }
  }

  let stat;
  let target = filePath;
  try {
    stat = await fsp.stat(target);
    if (stat.isDirectory()) {
      target = path.join(target, 'index.html');
      stat = await fsp.stat(target);
    }
  } catch {
    errorPage(res, 404, '없는 경로입니다.');
    return;
  }

  sendFile(req, res, target, stat);
});

/* ══════════════════════════════════════════════════════════════
   기동 — 관리자 계정 씨앗
   ══════════════════════════════════════════════════════════════ */
if (require.main === module) {
  (async () => {
    const seeded = await seedAdmin();
    await store.sweepSessions();
    setInterval(() => { store.sweepSessions().catch(() => {}); }, 60 * 60 * 1000).unref();

    server.listen(PORT, HOST, () => {
      const line = '─'.repeat(58);
      console.log(line);
      console.log('  Web console   http://' + HOST + ':' + PORT);
      console.log('  DB         ' + store.path);
      if (seeded.created) {
        console.log(line);
        console.log('  관리자 계정을 만들었습니다 (최초 1회)');
        console.log('    ID        ' + seeded.loginId);
        console.log('    PASSWORD  ' + seeded.password);
        console.log('  이 비밀번호는 다시 표시되지 않습니다. 로그인 후 관리자 화면에서 바꾸세요.');
      } else {
        console.log('  관리자     ' + seeded.loginId + ' (이미 있음)');
      }
      console.log(line);
    });
    /* ── 종료 ──
       WAL 을 본 파일로 접고 나간다. 안 그러면 console.db 는 헤더만
       남은 껍데기로 보이고 내용은 -wal 에 남는다 — .db 만 읽는 뷰어에서
       '테이블 없음' 으로 보인다. Ctrl+C 두 번이면 그냥 죽는다. */
    let closing = false;
    const shutdown = (sig) => {
      if (closing) process.exit(1);
      closing = true;
      console.log('');
      console.log('  ' + sig + ' — WAL 정리 후 종료합니다');
      server.close(() => {
        store.checkpoint()
          .catch((err) => console.error('  체크포인트 실패:', err.message))
          .finally(() => process.exit(0));
      });
      /* 열린 연결이 물고 있어도 3초 뒤엔 나간다 */
      setTimeout(() => {
        store.checkpoint().catch(() => {}).finally(() => process.exit(0));
      }, 3000).unref();
    };
    process.on('SIGINT', () => shutdown('SIGINT'));
    process.on('SIGTERM', () => shutdown('SIGTERM'));
  })().catch((err) => { console.error(err); process.exit(1); });
}

module.exports = { server, seedAdmin };
