/* ══════════════════════════════════════════════════════════════
   Vercel 서버리스 함수 — /api/* 전부 이 함수 하나로 받는다.
   vercel.json 의 rewrite 가 원래 경로를 __path 쿼리에 실어 보낸다
   ([[...route]] 이중 대괄호 캐치올은 Next.js 문법이라 일반 Vercel
   함수에서는 1단계 깊이만 매칭됐다 — 실측).
   로직은 backend/routes.js — 로컬 서버(backend/server.js)와 같은
   코드다. 여기서는 pathname 만 풀어서 넘긴다.

   Node 런타임이다 (Edge 아님) — scrypt 가 node:crypto 를 쓴다.
   DB 는 TURSO_DATABASE_URL 환경변수로 libSQL 백엔드가 잡힌다
   (backend/db.js 참고). 파일 SQLite 는 서버리스에서 유지되지 않는다.
   ══════════════════════════════════════════════════════════════ */
'use strict';

const { handleApi, json } = require('../backend/routes');

module.exports = async (req, res) => {
  let pathname;
  try {
    const u = new URL(req.url, 'http://localhost');
    /* rewrite 를 거치면 req.url 이 /api/index?__path=… 가 된다 — 원 경로 우선 */
    pathname = decodeURIComponent(u.searchParams.get('__path') || u.pathname);
  } catch {
    res.statusCode = 400;
    res.end('bad request');
    return;
  }

  try {
    await handleApi(req, res, pathname);
  } catch (err) {
    const isJson = err && err.message === 'invalid json';
    if (!res.headersSent) {
      json(res, isJson ? 400 : 500, { error: isJson ? '요청 형식이 올바르지 않습니다.' : '서버 오류입니다.' });
    } else {
      res.end();
    }
    if (!isJson) console.error('[api]', err);
  }
};
