/* ══════════════════════════════════════════════════════════════
   웹 웹 UI 계정 저장소 — 백엔드 이원화

   로컬:   node:sqlite (Node 22.5+ 내장) — npm install 없이
           node backend/server.js 만으로 뜬다. 기존 그대로.
   Vercel: Turso(libSQL) — 서버리스 함수의 파일시스템은 읽기
           전용이고 인스턴스 간 공유도 안 되므로 파일 DB 가
           유지되지 않는다. TURSO_DATABASE_URL 이 있으면 이쪽.

   두 백엔드 모두 같은 SQL(SQLite 문법)을 쓴다. 인터페이스는
   async 한 벌 — 로컬 sync 호출을 async 로 감싸는 비용으로
   라우트 핸들러를 한 벌만 유지한다.

   의존성은 @libsql/client 하나뿐이고, 그마저 TURSO_DATABASE_URL
   이 있을 때만 require 한다 — 로컬은 여전히 node_modules 없이 돈다.
   ══════════════════════════════════════════════════════════════ */
'use strict';

const path = require('node:path');
const { hashPassword } = require('./auth');

const now = () => Date.now();

const SCHEMA = `
CREATE TABLE IF NOT EXISTS users (
  id            INTEGER PRIMARY KEY AUTOINCREMENT,
  login_id      TEXT    NOT NULL UNIQUE COLLATE NOCASE,
  name          TEXT    NOT NULL DEFAULT '',
  pw_hash       TEXT    NOT NULL,
  role          TEXT    NOT NULL DEFAULT 'engineer'  CHECK (role IN ('admin','engineer')),
  status        TEXT    NOT NULL DEFAULT 'pending'   CHECK (status IN ('pending','active','suspended')),
  created_at    INTEGER NOT NULL,
  last_login_at INTEGER
);

CREATE TABLE IF NOT EXISTS sessions (
  token      TEXT    PRIMARY KEY,
  user_id    INTEGER NOT NULL REFERENCES users(id) ON DELETE CASCADE,
  created_at INTEGER NOT NULL,
  expires_at INTEGER NOT NULL
);
CREATE INDEX IF NOT EXISTS idx_sessions_user ON sessions(user_id);

CREATE TABLE IF NOT EXISTS audit (
  id     INTEGER PRIMARY KEY AUTOINCREMENT,
  at     INTEGER NOT NULL,
  actor  TEXT    NOT NULL,
  action TEXT    NOT NULL,
  target TEXT    NOT NULL DEFAULT '',
  detail TEXT    NOT NULL DEFAULT ''
);
CREATE INDEX IF NOT EXISTS idx_audit_at ON audit(at DESC);

/* 숙련공 데이터 라이브러리 — ① 웹 UI에서 RL 결과가 정답과 일치해
   합격시킨 기록. 예전에는 localStorage 'polytwin_saved' 였다.
   payload 는 통째 JSON 이다: env / rl / ref(세그먼트 요약 스냅샷).
   ref 안의 trace 까지 정규화할 이유가 없다 — 라이브러리는 이걸
   해석하지 않고 그대로 다시 그린다. 한 건 약 7KB.
   ref_id 는 클라이언트가 만든 'rl-…' 아이디다. UNIQUE 라서
   같은 저장을 두 번 눌러도 행이 늘지 않는다. */
CREATE TABLE IF NOT EXISTS library_entries (
  id         INTEGER PRIMARY KEY AUTOINCREMENT,
  user_id    INTEGER NOT NULL REFERENCES users(id) ON DELETE CASCADE,
  ref_id     TEXT    NOT NULL UNIQUE,
  seg        TEXT    NOT NULL DEFAULT '',
  name       TEXT    NOT NULL DEFAULT '',
  payload    TEXT    NOT NULL,
  created_at INTEGER NOT NULL
);
CREATE INDEX IF NOT EXISTS idx_library_created ON library_entries(created_at DESC);
CREATE INDEX IF NOT EXISTS idx_library_user ON library_entries(user_id);

/* 숙련공 정답 데이터 — 예전에는 데이터셋/seg_best_kpi.json 을 화면이
   직접 fetch 했다. 세그먼트 한 줄씩 넣어 질의할 수 있게 한다.
   payload 는 세그먼트 객체 통째다: KPI · 힘 시계열(trace) · 원본 CSV
   안의 바이트 범위(file/byteStart/byteEnd/sliceFile).
   바이트 범위를 DB 에 두는 이유 — 실제 CSV 는 183MB 라 DB 에 넣지
   않는다. '어디를 읽어야 하는가'만 옮기고 읽기는 그대로 Range 요청이다. */
CREATE TABLE IF NOT EXISTS segments (
  seg        TEXT    PRIMARY KEY,
  robot      TEXT    NOT NULL DEFAULT '',
  inband     REAL,
  ord        INTEGER NOT NULL DEFAULT 0,
  payload    TEXT    NOT NULL,
  updated_at INTEGER NOT NULL
);
CREATE INDEX IF NOT EXISTS idx_segments_ord ON segments(ord);

/* 데이터셋의 머리말 — generated/note/source/header 처럼 세그먼트
   바깥에 있던 것들, 그리고 quality_kpi.json 통째.
   화면에 돌려줄 때 segments 와 다시 합쳐 원래 JSON 모양을 만든다 */
CREATE TABLE IF NOT EXISTS dataset_meta (
  key        TEXT    PRIMARY KEY,
  payload    TEXT    NOT NULL,
  updated_at INTEGER NOT NULL
);
`;

/* ── 저수준 인터페이스: run / get / all — 전부 async ─────────── */

function makeSqliteBackend() {
  const fs = require('node:fs');
  const { DatabaseSync } = require('node:sqlite');

  const DATA_DIR = path.join(__dirname, 'data');
  const DB_PATH = process.env.PT_DB || path.join(DATA_DIR, 'console.db');
  fs.mkdirSync(DATA_DIR, { recursive: true });

  const db = new DatabaseSync(DB_PATH);
  /* WAL 은 읽기와 쓰기가 서로를 막지 않게 한다. 데모 중 잠김을 피하는 목적 */
  db.exec('PRAGMA journal_mode = WAL');
  db.exec('PRAGMA foreign_keys = ON');
  db.exec(SCHEMA);

  return {
    path: DB_PATH,
    /* WAL 을 본 파일로 접어 넣는다. 이걸 안 하면 console.db 는
       헤더만 남은 껍데기로 보이고 내용은 전부 -wal 에 있다 —
       .db 만 읽는 뷰어(Database Client 등)에서 '테이블 없음' 이 된다.
       종료할 때 server.js 가 부른다. */
    async checkpoint() {
      db.exec('PRAGMA wal_checkpoint(TRUNCATE)');
    },
    async run(sql, args = []) {
      const r = db.prepare(sql).run(...args);
      return { lastId: Number(r.lastInsertRowid), changes: Number(r.changes) };
    },
    async get(sql, args = []) { return db.prepare(sql).get(...args) || null; },
    async all(sql, args = []) { return db.prepare(sql).all(...args); },
  };
}

function makeLibsqlBackend() {
  /* 여기서만 require — 로컬 무의존 기동을 깨지 않는다 */
  const { createClient } = require('@libsql/client');
  const client = createClient({
    url: process.env.TURSO_DATABASE_URL,
    authToken: process.env.TURSO_AUTH_TOKEN,
  });

  /* 서버리스에는 "최초 기동"이 없다 — 스키마는 첫 질의 전에 1회 보장.
     콜드 스타트마다 IF NOT EXISTS 몇 개가 도는 비용은 수 ms 라 수용한다 */
  let ready = null;
  const ensure = () => {
    if (!ready) {
      ready = client.batch(
        SCHEMA.split(';').map((s) => s.trim()).filter(Boolean),
        'write',
      );
    }
    return ready;
  };

  return {
    path: process.env.TURSO_DATABASE_URL,
    async checkpoint() { /* 원격 DB 에는 해당 없음 */ },
    async run(sql, args = []) {
      await ensure();
      const r = await client.execute({ sql, args });
      return { lastId: Number(r.lastInsertRowid ?? 0), changes: r.rowsAffected };
    },
    async get(sql, args = []) {
      await ensure();
      const r = await client.execute({ sql, args });
      return r.rows[0] || null;
    },
    async all(sql, args = []) {
      await ensure();
      const r = await client.execute({ sql, args });
      return r.rows;
    },
  };
}

/* Vercel 인데 Turso 가 없으면 — 파일 SQLite 는 읽기 전용 FS 에서
   크래시한다. 모듈 로드는 살려 두고, 실제 호출 시 명확히 알린다 */
function makeUnconfiguredBackend() {
  const die = () => {
    throw new Error('TURSO_DATABASE_URL 미설정 — Vercel 에서는 원격 DB 가 필요합니다 (SERVER.md 참고).');
  };
  return { path: '(unconfigured)', checkpoint: async () => {}, run: die, get: die, all: die };
}

const be = process.env.TURSO_DATABASE_URL ? makeLibsqlBackend()
  : process.env.VERCEL ? makeUnconfiguredBackend()
  : makeSqliteBackend();

/* ── 스토어 — 백엔드와 무관하게 한 벌 ────────────────────────── */
const store = {
  now,

  findByLogin: (loginId) => be.get('SELECT * FROM users WHERE login_id = ?', [loginId]),
  findById: (id) => be.get('SELECT * FROM users WHERE id = ?', [id]),

  async createUser({ loginId, name = '', password, role = 'engineer', status = 'pending' }) {
    const r = await be.run(
      'INSERT INTO users (login_id, name, pw_hash, role, status, created_at) VALUES (?, ?, ?, ?, ?, ?)',
      [loginId, name, hashPassword(password), role, status, now()],
    );
    return store.findById(r.lastId);
  },

  listUsers: () => be.all(`SELECT id, login_id, name, role, status, created_at, last_login_at
                           FROM users ORDER BY
                             CASE status WHEN 'pending' THEN 0 WHEN 'active' THEN 1 ELSE 2 END,
                             created_at DESC`),
  setStatus: (id, status) => be.run('UPDATE users SET status = ? WHERE id = ?', [status, id]),
  setRole: (id, role) => be.run('UPDATE users SET role = ? WHERE id = ?', [role, id]),
  setPassword: (id, password) => be.run('UPDATE users SET pw_hash = ? WHERE id = ?', [hashPassword(password), id]),
  touchLogin: (id) => be.run('UPDATE users SET last_login_at = ? WHERE id = ?', [now(), id]),
  async deleteUser(id) {
    await be.run('DELETE FROM sessions WHERE user_id = ?', [id]);
    return (await be.run('DELETE FROM users WHERE id = ?', [id])).changes;
  },
  async activeAdminCount() {
    const r = await be.get("SELECT COUNT(*) AS n FROM users WHERE role = 'admin' AND status = 'active'");
    return Number(r.n);
  },

  createSession(userId, token, ttlMs) {
    const t = now();
    return be.run('INSERT INTO sessions (token, user_id, created_at, expires_at) VALUES (?, ?, ?, ?)',
      [token, userId, t, t + ttlMs]);
  },
  async readSession(token) {
    if (!token) return null;
    const row = await be.get(`SELECT s.token, s.expires_at, u.*
                              FROM sessions s JOIN users u ON u.id = s.user_id
                              WHERE s.token = ?`, [token]);
    if (!row) return null;
    if (Number(row.expires_at) < now()) { await be.run('DELETE FROM sessions WHERE token = ?', [token]); return null; }
    return row;
  },
  dropSession: (token) => be.run('DELETE FROM sessions WHERE token = ?', [token]),
  dropSessionsOfUser: (userId) => be.run('DELETE FROM sessions WHERE user_id = ?', [userId]),
  async sweepSessions() { return (await be.run('DELETE FROM sessions WHERE expires_at < ?', [now()])).changes; },

  /* ── 숙련공 정답 데이터 (읽기 전용 참조 자료) ──────────── */
  listSegments: () => be.all('SELECT * FROM segments ORDER BY ord ASC'),

  async putSegment({ seg, robot = '', inband = null, ord = 0, payload }) {
    await be.run(
      `INSERT INTO segments (seg, robot, inband, ord, payload, updated_at)
       VALUES (?, ?, ?, ?, ?, ?)
       ON CONFLICT(seg) DO UPDATE SET
         robot = excluded.robot, inband = excluded.inband, ord = excluded.ord,
         payload = excluded.payload, updated_at = excluded.updated_at`,
      [seg, robot, inband, ord, payload, now()],
    );
  },

  getMeta: (key) => be.get('SELECT * FROM dataset_meta WHERE key = ?', [key]),

  async putMeta(key, payload) {
    await be.run(
      `INSERT INTO dataset_meta (key, payload, updated_at) VALUES (?, ?, ?)
       ON CONFLICT(key) DO UPDATE SET payload = excluded.payload, updated_at = excluded.updated_at`,
      [key, payload, now()],
    );
  },

  /* ── 숙련공 데이터 라이브러리 ────────────────────────────
     읽기는 전원 공유다 — 합격 기록은 팀 자산이라는 전제.
     지우기는 본인 또는 admin 만 (routes.js 에서 판정). ── */
  listLibrary: (limit = 200) => be.all(
    `SELECT e.id, e.ref_id, e.seg, e.name, e.payload, e.created_at,
            u.login_id AS owner_login, u.name AS owner_name
     FROM library_entries e JOIN users u ON u.id = e.user_id
     ORDER BY e.created_at DESC LIMIT ?`, [limit]),

  findLibraryEntry: (id) => be.get('SELECT * FROM library_entries WHERE id = ?', [id]),

  async createLibraryEntry({ userId, refId, seg = '', name = '', payload }) {
    /* 같은 ref_id 면 새로 넣지 않고 기존 것을 돌려준다 — 저장 버튼을
       두 번 눌러도, 재전송이 와도 목록이 중복되지 않는다 */
    const dup = await be.get('SELECT * FROM library_entries WHERE ref_id = ?', [refId]);
    if (dup) return { entry: dup, created: false };
    const r = await be.run(
      `INSERT INTO library_entries (user_id, ref_id, seg, name, payload, created_at)
       VALUES (?, ?, ?, ?, ?, ?)`,
      [userId, refId, seg, name, payload, now()],
    );
    return { entry: await store.findLibraryEntry(r.lastId), created: true };
  },

  async deleteLibraryEntry(id) {
    return (await be.run('DELETE FROM library_entries WHERE id = ?', [id])).changes;
  },

  log: (actor, action, target = '', detail = '') =>
    be.run('INSERT INTO audit (at, actor, action, target, detail) VALUES (?, ?, ?, ?, ?)',
      [now(), actor, action, target, detail]),
  recentAudit: (n = 50) => be.all('SELECT * FROM audit ORDER BY at DESC LIMIT ?', [n]),

  checkpoint: () => be.checkpoint(),

  path: be.path,
};

module.exports = store;
