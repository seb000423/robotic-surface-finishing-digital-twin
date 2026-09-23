"""PhysX 접촉 리포트 기반 패드 접촉력 리더 (Isaac Sim 6).

구 `isaacsim.sensors.physics.ContactSensor` 래퍼가 이 빌드에서 로드되지 않아, omni.physx 의
contact report 이벤트(패드 프림에 PhysxContactReportAPI 적용됨)를 직접 구독해 물리 스텝마다
패드별 순접촉 임펄스를 합산하고 힘(N) = 임펄스 / dt 로 환산한다.
"""
from __future__ import annotations

import numpy as np


class PadContactReporter:
    _instance = None

    def __init__(self, physics_dt: float = 1.0 / 60.0):
        self.dt = float(physics_dt)
        self.pads: dict[str, np.ndarray] = {}      # pad prim path → 누적 임펄스(세계)
        self.normals: dict[str, np.ndarray] = {}   # 접촉 법선 평균(진단)
        self._sub = None
        self._last_forces: dict[str, np.ndarray] = {}
        self._events = 0
        try:
            import omni.physx
            self._iface = omni.physx.get_physx_simulation_interface()
            self._sub = self._iface.subscribe_contact_report_events(self._on_contact)
            print("[pad_contact] PhysX contact report 구독 시작")
        except Exception as exc:
            self._iface = None
            print(f"[pad_contact] contact report 구독 실패: {exc}")

    @classmethod
    def get(cls, physics_dt: float = 1.0 / 60.0) -> "PadContactReporter":
        if cls._instance is None:
            cls._instance = cls(physics_dt)
        return cls._instance

    def register(self, pad_prim_path: str):
        self.pads[str(pad_prim_path)] = np.zeros(3)
        self._last_forces[str(pad_prim_path)] = np.zeros(3)

    def _on_contact(self, headers, data):
        from pxr import PhysicsSchemaTools
        acc: dict[str, np.ndarray] = {}
        for h in headers:
            p0 = str(PhysicsSchemaTools.intToSdfPath(h.actor0))
            p1 = str(PhysicsSchemaTools.intToSdfPath(h.actor1))
            key, sign = None, 1.0
            for pad in self.pads:
                if p0 == pad or p0.startswith(pad + "/"):
                    key, sign = pad, 1.0; break
                if p1 == pad or p1.startswith(pad + "/"):
                    key, sign = pad, -1.0; break
            if key is None:
                continue
            tot = acc.setdefault(key, np.zeros(3))
            off, n = int(h.contact_data_offset), int(h.num_contact_data)
            for k in range(off, off + n):
                imp = data[k].impulse
                tot += sign * np.array([imp.x, imp.y, imp.z], dtype=float)
        # 이벤트는 물리 스텝마다 오므로, 이번 스텝의 값으로 교체 (없으면 0)
        for pad in self.pads:
            self._last_forces[pad] = acc.get(pad, np.zeros(3)) / self.dt
        self._events += 1

    def force(self, pad_prim_path: str) -> np.ndarray:
        """직전 물리 스텝의 순접촉력(세계, N). 접촉 없으면 0 벡터."""
        return self._last_forces.get(str(pad_prim_path), np.zeros(3))
