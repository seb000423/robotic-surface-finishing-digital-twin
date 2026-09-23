"""시뮬레이션 → 모니터(②) 실시간 피드 파일 작성기.

백엔드 `GET /api/monitor` 가 이 JSON 파일을 읽어 그대로 돌려주고, `monitor.html` 이
0.5 s 마다 폴링해 LIVE 모드로 그린다(피드가 5 s 이상 오래되면 데모 모드로 복귀).
v5 시뮬레이션 러너가 `POLISH_MONITOR_FEED=<path>` 일 때 20 스텝마다 update 를 부른다.

스키마 (monitor.html 의 S 상태와 1:1):
  "ts": 1725180000.0,            # epoch 초 — 신선도 판단
  "state": "POLISH",            # HOME/SLIDE/APPROACH/POLISH/RETRACT/HOLD/DONE
  "progress": 0.42,              # 0~1 전체 진행(커버리지 기준)
  "elapsed_s": 812.3,
  "robots": [ {"id": "C", "name": "천장", "force": 5.61, "target": 5.6, "state": "POLISH",
               "progress": 0.5, "rl_force_scale": 1.12, "rl_feed_scale": 0.93,
               "q": [0.0, -1.05, 1.45, 0.0, 1.15, 0.0],                      # 팔 관절각(rad, 선택)
               "base": {"pos": [x, y, z], "quat": [w, x, y, z]}}, ... ],        # 베이스 자세(Isaac 월드, 선택)
  "scene": {"up": "z", "long": "y", "car_min": [...], "car_max": [...]},   # 웹 UI 좌표 정렬 기준(선택)
  "metrics": {"cv": 6.1, "band": 92.3, "keep": 97.0, "cov": 42.0, "unif": 88.0, "over": 1},
  "events": [ {"robot": "C", "msg": "...", "level": "info|warn|crit", "t": 812.3}, ... ],
  "cells": {"total": 491, "pass": 120, "rework": 30, "repaint": 40, "not_reached": 301,
            "items": [[x, y, z, "pass", gu], ...]}   # 최근 판정 스냅샷(선택)
"""
from __future__ import annotations

import json
import os
import time
from collections import deque


class MonitorFeed:
    BAND = (3.0, 8.0)          # sub.html '작업 대역 3~8 N' 과 동일

    def __init__(self, path: str, hist: int = 240):
        self.path = path
        os.makedirs(os.path.dirname(os.path.abspath(path)) or ".", exist_ok=True)
        self.t0 = time.time()
        self.events: deque = deque(maxlen=40)
        self._force_hist: dict[str, deque] = {}
        self._hist = hist
        self._over = 0
        self._n = 0

    def event(self, robot: str, msg: str, level: str = "info", sim_t: float | None = None):
        self.events.append({"robot": robot, "msg": msg, "level": level,
                            "t": float(sim_t if sim_t is not None else time.time() - self.t0)})
        if level == "crit":
            self._over += 1

    def update(self, state: str, progress: float, robots: list[dict], elapsed_s: float | None = None,
               cells: dict | None = None, scene: dict | None = None, recipe: dict | None = None):
        """robots: [{id, name, force, target, state, progress, rl_force_scale, rl_feed_scale}]"""
        self._n += 1
        forces_all = []
        for r in robots:
            h = self._force_hist.setdefault(r["id"], deque(maxlen=self._hist))
            h.append(float(r.get("force", 0.0)))
            forces_all.extend(v for v in h if v > 0.3)          # 접촉 중 표본만
        import statistics
        if len(forces_all) >= 4:
            m = statistics.mean(forces_all); sd = statistics.pstdev(forces_all)
            cv = 100.0 * sd / m if m > 1e-6 else 0.0
            band = 100.0 * sum(self.BAND[0] <= v <= self.BAND[1] for v in forces_all) / len(forces_all)
            keep = 100.0 * sum(abs(v - r.get("target", m)) <= 1.0 for v in forces_all for r in robots[:1]) / len(forces_all)
        else:
            cv = band = keep = 0.0
        payload = {
            "ts": time.time(),
            "state": state,
            "progress": float(max(0.0, min(1.0, progress))),
            "elapsed_s": float(elapsed_s if elapsed_s is not None else time.time() - self.t0),
            "robots": [{"id": r["id"], "name": r.get("name", r["id"]), "force": round(float(r.get("force", 0.0)), 3),
                        "target": round(float(r.get("target", 0.0)), 3), "state": r.get("state", state),
                        "progress": round(float(r.get("progress", 0.0)), 4),
                        "rl_force_scale": round(float(r.get("rl_force_scale", 1.0)), 3),
                        "rl_feed_scale": round(float(r.get("rl_feed_scale", 1.0)), 3),
                        # 웹 UI 3D 팔 동기화(선택): q = 관절각 6개(rad), base = {pos[3], quat[w,x,y,z]} (Isaac 월드)
                        **({"q": [round(float(v), 4) for v in r["q"]]} if r.get("q") is not None else {}),
                        **({"base": r["base"]} if r.get("base") is not None else {})} for r in robots],
            "metrics": {"cv": round(cv, 2), "band": round(band, 2), "keep": round(keep, 2),
                        "cov": round(100.0 * progress, 2),
                        "unif": round(100.0 - min(cv, 100.0) * 0.5, 2), "over": int(self._over)},
            "events": list(self.events)[-20:],
            "cells": cells or {},
            # 장면 기준(선택): {"up": "z", "long": "y", "car_min": [x,y,z], "car_max": [x,y,z]} — 웹 UI 좌표 정렬용
            **({"scene": scene} if scene else {}),
            # 유효 레시피(선택): {force_n, feed_mm_s, rpm, step_over_ratio, n_passes, pad_radius_m, robots[]} — 웹 UI 상태줄·조건 대조
            **({"recipe": recipe} if recipe else {}),
        }
        tmp = self.path + ".tmp"
        with open(tmp, "w", encoding="utf-8") as f:
            json.dump(payload, f, ensure_ascii=False)
        os.replace(tmp, self.path)
        return payload
