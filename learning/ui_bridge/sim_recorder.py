"""시뮬레이션 기록기 — 매 프레임(관절·베이스·힘·상태·진행·공정시간)과 이벤트·셀 판정·결과를 SQLite 한 파일에 쓴다.

UI 는 이 파일(또는 서버에 올린 같은 내용)에서 프레임을 미리 받아 보간하며 재생한다 → 실시간 폴링 렉 없음,
속도 조절·탐색 가능. 실시간도 같은 경로로 몇 초 지연 재생.

    rec = SimRecorder(path, meta={"recipe":..., "robots":[...], "scene":{...}, "dt_ctrl": 1/60})
    rec.frame(t_sim, state, progress, elapsed_s, robots)   # robots: monitor_feed.update 와 같은 dict 목록(q/base 포함)
    rec.event(t_sim, robot, level, msg); rec.cells(t_sim, cells_dict); rec.finish(result_dict)

스키마
  meta(key TEXT PK, value TEXT)                       -- JSON 값. 'scene','robots','recipe','hz','t_start','t_end'
  chunks(seq INTEGER PK, t0 REAL, t1 REAL, n INT, data BLOB)  -- gzip(JSON [frame,...]), 1 초 단위 묶음
  events(id INTEGER PK, t REAL, robot TEXT, level TEXT, msg TEXT)
  cells(id INTEGER PK, t REAL, data BLOB)             -- gzip(JSON 셀 판정 스냅샷)
  result(id INTEGER PK, data TEXT)                    -- last_run.json 과 같은 요약
프레임 = {"t": t_sim, "s": state, "p": progress, "e": elapsed_s,
          "r": [[id, force, target, state, progress, rl_force_scale, rl_feed_scale, q[6] | null, pos[3] | null, quat[4] | null,
                 tcp[3] | null, normal[3] | null], ...]}   # tcp/normal 은 결과 리플레이(관절 없음) 용 — 웹 UI가 IK 로 패드를 표면에
"""
from __future__ import annotations

import gzip
import json
import os
import sqlite3
import time

_DDL = """
CREATE TABLE IF NOT EXISTS meta (key TEXT PRIMARY KEY, value TEXT);
CREATE TABLE IF NOT EXISTS chunks (seq INTEGER PRIMARY KEY AUTOINCREMENT, t0 REAL, t1 REAL, n INTEGER, data BLOB);
CREATE TABLE IF NOT EXISTS events (id INTEGER PRIMARY KEY AUTOINCREMENT, t REAL, robot TEXT, level TEXT, msg TEXT);
CREATE TABLE IF NOT EXISTS cells (id INTEGER PRIMARY KEY AUTOINCREMENT, t REAL, data BLOB);
CREATE TABLE IF NOT EXISTS result (id INTEGER PRIMARY KEY, data TEXT);
"""


def pack(obj) -> bytes:
    return gzip.compress(json.dumps(obj, separators=(",", ":"), ensure_ascii=False).encode("utf-8"), 6)


def unpack(blob: bytes):
    return json.loads(gzip.decompress(blob).decode("utf-8"))


class SimRecorder:
    def __init__(self, path: str, meta: dict | None = None, chunk_s: float = 1.0):
        os.makedirs(os.path.dirname(os.path.abspath(path)) or ".", exist_ok=True)
        self.path = path
        self.db = sqlite3.connect(path)
        self.db.executescript("PRAGMA journal_mode=WAL; PRAGMA synchronous=NORMAL;" + _DDL)
        self.chunk_s = float(chunk_s)
        self._buf: list = []
        self._t_first = None
        self.n_frames = 0
        self.t_last = 0.0
        m = dict(meta or {})
        m.setdefault("t_start", time.time())
        for k, v in m.items():
            self.set_meta(k, v)

    # ── 메타 ──
    def set_meta(self, key: str, value):
        self.db.execute("INSERT OR REPLACE INTO meta (key, value) VALUES (?, ?)", (key, json.dumps(value, ensure_ascii=False)))
        self.db.commit()

    # ── 프레임 ──
    @staticmethod
    def _rob(r: dict):
        q = r.get("q"); b = r.get("base") or {}
        return [r.get("id"), round(float(r.get("force", 0.0)), 3), round(float(r.get("target", 0.0)), 3),
                r.get("state", ""), round(float(r.get("progress", 0.0)), 4),
                round(float(r.get("rl_force_scale", 1.0)), 3), round(float(r.get("rl_feed_scale", 1.0)), 3),
                [round(float(v), 4) for v in q] if q is not None else None,
                [round(float(v), 4) for v in b["pos"]] if b.get("pos") is not None else None,
                [round(float(v), 5) for v in b["quat"]] if b.get("quat") is not None else None,
                [round(float(v), 4) for v in r["tcp"]] if r.get("tcp") is not None else None,          # 10: 패드 목표점(선택, 결과 리플레이)
                [round(float(v), 4) for v in r["normal"]] if r.get("normal") is not None else None]    # 11: 표면 법선(선택)

    def frame(self, t: float, state: str, progress: float, elapsed_s: float, robots: list[dict]):
        fr = {"t": round(float(t), 4), "s": state, "p": round(float(progress), 4), "e": round(float(elapsed_s), 3),
              "r": [self._rob(r) for r in robots]}
        if self._t_first is None:
            self._t_first = fr["t"]
        self._buf.append(fr)
        self.n_frames += 1
        self.t_last = fr["t"]
        if fr["t"] - self._t_first >= self.chunk_s:
            self.flush()

    def flush(self):
        if not self._buf:
            return
        t0, t1 = self._buf[0]["t"], self._buf[-1]["t"]
        self.db.execute("INSERT INTO chunks (t0, t1, n, data) VALUES (?, ?, ?, ?)", (t0, t1, len(self._buf), pack(self._buf)))
        self.db.commit()
        self._buf = []; self._t_first = None

    # ── 이벤트·셀·결과 ──
    def event(self, t: float, robot: str, level: str, msg: str):
        self.db.execute("INSERT INTO events (t, robot, level, msg) VALUES (?, ?, ?, ?)", (float(t), robot, level, msg))

    def cells(self, t: float, cells: dict):
        self.db.execute("INSERT INTO cells (t, data) VALUES (?, ?)", (float(t), pack(cells)))
        self.db.commit()

    def finish(self, result: dict | None = None):
        self.flush()
        self.set_meta("t_end", time.time())
        self.set_meta("t_sim_end", self.t_last)
        self.set_meta("n_frames", self.n_frames)
        if result is not None:
            self.db.execute("INSERT OR REPLACE INTO result (id, data) VALUES (1, ?)", (json.dumps(result, ensure_ascii=False),))
        self.db.commit()

    def close(self):
        try: self.flush(); self.db.commit(); self.db.close()
        except Exception: pass


class SimReader:
    """기록 파일 읽기 — 워커 업로드·검사용."""
    def __init__(self, path: str):
        self.db = sqlite3.connect(f"file:{path}?mode=ro", uri=True)

    def meta(self) -> dict:
        return {k: json.loads(v) for k, v in self.db.execute("SELECT key, value FROM meta")}

    def chunks_after(self, seq: int = 0):
        return self.db.execute("SELECT seq, t0, t1, n, data FROM chunks WHERE seq > ? ORDER BY seq", (seq,)).fetchall()

    def events(self):
        return self.db.execute("SELECT t, robot, level, msg FROM events ORDER BY id").fetchall()

    def cells_after(self, id_: int = 0):
        return self.db.execute("SELECT id, t, data FROM cells WHERE id > ? ORDER BY id", (id_,)).fetchall()

    def result(self):
        r = self.db.execute("SELECT data FROM result WHERE id = 1").fetchone()
        return json.loads(r[0]) if r else None

    def frames(self, t0: float = -1e18, t1: float = 1e18):
        out = []
        for _, a, b, _, blob in self.db.execute("SELECT seq, t0, t1, n, data FROM chunks WHERE t1 >= ? AND t0 <= ? ORDER BY seq", (t0, t1)):
            out.extend(unpack(blob))
        return out
