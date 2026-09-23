"""Coverage and point-cloud polishing visualization helpers."""
import numpy as np
from scipy.spatial import KDTree

class CoverageMap:
    """3D 복셀 기반 폴리싱 완료 영역 추적. 두 로봇이 공유."""

    def __init__(self, voxel_size=0.02):
        self.voxel_size = float(voxel_size)
        self._covered: set = set()

    def _key(self, pos):
        return (
            int(np.floor(pos[0] / self.voxel_size)),
            int(np.floor(pos[1] / self.voxel_size)),
            int(np.floor(pos[2] / self.voxel_size)),
        )

    def mark(self, pos):
        self._covered.add(self._key(pos))

    def clear(self):
        """재폴리싱 패스 시작 시 호출 — 패스 내 다중로봇 중복방지를 새로 시작."""
        self._covered.clear()

    def is_covered(self, pos):
        return self._key(pos) in self._covered

    def covered_count(self):
        return len(self._covered)

    def covered_positions(self):
        """복셀 중심 좌표 배열 반환 (시각화용)."""
        if not self._covered:
            return np.empty((0, 3))
        keys = np.array(list(self._covered), dtype=np.float32)
        return (keys + 0.5) * self.voxel_size


class PolishViz:
    """폴리싱된 표면 점(스캔 포인트 클라우드)을 패드 반경 내에서 하늘색으로 재채색.
    크기는 그대로, 색만 빨강→하늘색. 패드가 실제로 지나간 영역만 칠함."""

    def __init__(self, stage, prim_path, points, pad_radius):
        self.prim_path = prim_path
        self.points = np.asarray(points, dtype=np.float32)
        self.kdtree = KDTree(self.points) if len(self.points) else None
        self.pad_radius = float(pad_radius)
        self._sky = np.array([0.35, 0.75, 1.0], dtype=np.float32)
        self._colors = np.tile(np.array([1.0, 0.0, 0.0], dtype=np.float32), (len(self.points), 1))
        self._polished = np.zeros(len(self.points), dtype=bool)
        self._dirty = False

    def mark(self, pos):
        if self.kdtree is None:
            return
        idx = self.kdtree.query_ball_point(np.asarray(pos, dtype=np.float32), self.pad_radius)
        if not idx:
            return
        idx = np.asarray(idx)
        fresh = idx[~self._polished[idx]]
        if len(fresh):
            self._polished[idx] = True
            self._colors[idx] = self._sky
            self._dirty = True

    def covered_count(self):
        return int(self._polished.sum())

    def total_count(self):
        return int(len(self.points))

    def unpolished_positions(self):
        """아직 안 닦인(빨간) 점들의 좌표 — 재폴리싱 경로 재추정용."""
        if len(self.points) == 0:
            return np.empty((0, 3), dtype=np.float32)
        return self.points[~self._polished]

    def is_polished(self, pos):
        """이 위치의 가장 가까운 스캔 점이 이미 색칠(폴리싱)됐는지 — 재폴리싱 시 건너뛰기용."""
        if self.kdtree is None:
            return False
        _, idx = self.kdtree.query(np.asarray(pos, dtype=np.float32))
        return bool(self._polished[idx])

    def flush(self, stage):
        if not self._dirty:
            return
        from pxr import UsdGeom, Vt, Gf
        prim = stage.GetPrimAtPath(self.prim_path)
        if prim.IsValid():
            UsdGeom.Points(prim).GetDisplayColorAttr().Set(
                Vt.Vec3fArray([Gf.Vec3f(float(c[0]), float(c[1]), float(c[2])) for c in self._colors])
            )
        self._dirty = False
