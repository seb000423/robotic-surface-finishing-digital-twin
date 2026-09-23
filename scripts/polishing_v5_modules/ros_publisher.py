"""대시보드용 ROS2 퍼블리셔 (계측 전용 — 폴리싱 동작에는 일절 관여하지 않음).

runner 의 메인 루프에서 매 스텝 호출되며, 다음 토픽을 퍼블리시한다:
  /polishing/state               (std_msgs/String)         POLISH / DONE
  /polishing/progress            (std_msgs/Float64)        전체 진행률 %
  /polishing/progress/{C,SL,SR}  (std_msgs/Float64)        로봇별 진행률 %
  /polishing/force/{C,SL,SR}     (std_msgs/Float64)        로봇별 접촉력 N
  /polishing/elapsed_time        (std_msgs/Float64)        경과 초
  /polishing/heatmap             (std_msgs/Float64MultiArray)  탑뷰 제거 커버리지
  /camera/{left_front,left_back,right_front,right_back,ceiling}/compressed
                                 (sensor_msgs/CompressedImage)  코너/천장 카메라(베스트에포트)

브라우저 대시보드는 rosbridge_server(ws://localhost:9090) 를 통해 이 토픽들을 구독한다.
설계 원칙: 모든 동작을 try/except 로 감싸 실패해도 시뮬레이션은 계속된다(계측이 본체를 막지 않음).
환경변수:
  POLISH_ROS_PUBLISH=0     → 전체 비활성화
  POLISH_ROS_CAMERAS=0     → 카메라만 비활성화(스칼라/히트맵은 유지)
"""
import io
import json
import os
import time

import numpy as np

# 프로젝트 루트(final_polishing) — web_dashboard 경로 계산용
_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

_ENABLE = os.environ.get("POLISH_ROS_PUBLISH", "1") != "0"
_ENABLE_CAM = os.environ.get("POLISH_ROS_CAMERAS", "1") != "0"

# 스텝 스로틀(성능) — N 스텝마다 1회 퍼블리시
_SCALAR_EVERY = 3
_HEATMAP_EVERY = 30
_CAMERA_EVERY = 8

# 접촉 판정 임계(N) — 가짜 댐핑힘(~0.5N) 이상을 실접촉으로 간주(평균 접촉력/접촉 비율용)
_CONTACT_FORCE_THRESH = 0.5

# 카메라 정의: (dashboard 토픽키, 차 중심 기준 오프셋[dx,dy,dz]). look_at = 차 중심.
# 사용자 확인 규약: 앞=-Y, 뒤=+Y, 왼쪽=+X, 오른쪽=-X.
#   left_front=(+X,-Y) / left_back=(+X,+Y) / right_front=(-X,-Y) / right_back=(-X,+Y)
# 천장: 차중심 z≈1.44 + dz=4.1 → 절대 z≈5.5 (탑뷰 — 차 전장 대부분이 화각에 들어오게 더 올림).
#   차 크기 측정값: 길이 3.04m(Y)·폭 1.33m(X)·지붕 z≈1.87. 기본 카메라 vFOV≈35°라
#   전장 3.04m를 담으려면 시선거리 ≳4.5m 필요 → z≈5.5에서 충족(거리 ~4.2m).
#   가림 방지: 오버헤드 세로레일·캐리지는 X=0(중앙선, z≈3.6). 카메라를 X=1.2로 비켜 두면
#   카메라→차량(폭 X≈±0.67) 모든 시선이 빔면에서 X≈0.25~0.43을 지나 레일/캐리지(|X|≲0.11)·
#   가로빔(Y=±3.2)·코너 포스트를 전부 빗겨감 → 빔 위 높은 탑뷰에서도 차량을 안 가림.
_CAM_DEFS = [
    ("left_front",  ( 4.0, -3.0, 2.2)),
    ("left_back",   ( 4.0,  3.0, 2.2)),
    ("right_front", (-4.0, -3.0, 2.2)),
    ("right_back",  (-4.0,  3.0, 2.2)),
    ("ceiling",     ( 1.2,  0.0, 4.1)),
]
_CAM_RES = (320, 240)


class PolishingRosPublisher:
    def __init__(self, agents, polish_viz, car_center):
        self.ok = False
        self.agents = agents
        self.polish_viz = polish_viz
        self.car_center = np.asarray(car_center, dtype=float)
        self.node = None
        self.pubs = {}
        self.cam_annots = {}          # key -> rgb annotator
        self.cam_pubs = {}            # key -> publisher
        self._cam_disabled = False
        self._cam_fail = 0
        self.start_time = time.time()
        self.step = 0
        self._done_sent = False
        # 완료 지표 누적(실데이터) — 평균 접촉력 / 접촉 비율 계산용
        self._contact_force_sum = 0.0    # 접촉 중 힘 합
        self._contact_samples = 0        # 접촉 판정된 (로봇,틱) 샘플 수
        self._total_samples = 0          # 전체 (로봇,틱) 샘플 수
        self._Image = None
        self._CompressedImage = None
        if not _ENABLE:
            print("[ros_pub] POLISH_ROS_PUBLISH=0 → 퍼블리시 비활성화")
            return
        try:
            self._init_ros()
            self.ok = True
            print("[ros_pub] ROS2 퍼블리셔 초기화 완료 (rosbridge_server 로 대시보드 연결)")
        except Exception as exc:  # noqa: BLE001
            print(f"[ros_pub] 초기화 실패(시뮬은 계속): {exc}")

    # ── 초기화 ───────────────────────────────────────────────
    def _init_ros(self):
        import rclpy
        from std_msgs.msg import String, Float64, Float64MultiArray  # noqa: F401
        from sensor_msgs.msg import CompressedImage
        self._String = String
        self._Float64 = Float64
        self._Float64MultiArray = Float64MultiArray
        self._CompressedImage = CompressedImage
        if not rclpy.ok():
            rclpy.init(args=None)
        self._rclpy = rclpy
        self.node = rclpy.create_node("polishing_dashboard_pub")

        self.pubs["state"] = self.node.create_publisher(String, "/polishing/state", 10)
        self.pubs["progress"] = self.node.create_publisher(Float64, "/polishing/progress", 10)
        self.pubs["elapsed"] = self.node.create_publisher(Float64, "/polishing/elapsed_time", 10)
        self.pubs["heatmap"] = self.node.create_publisher(Float64MultiArray, "/polishing/heatmap", 1)
        for lab in ("C", "SL", "SR"):
            self.pubs[f"force_{lab}"] = self.node.create_publisher(Float64, f"/polishing/force/{lab}", 10)
            self.pubs[f"prog_{lab}"] = self.node.create_publisher(Float64, f"/polishing/progress/{lab}", 10)

        if _ENABLE_CAM:
            try:
                self._setup_cameras()
            except Exception as exc:  # noqa: BLE001
                print(f"[ros_pub] 카메라 설정 실패(스칼라/히트맵은 정상): {exc}")
                self._cam_disabled = True

    def _setup_cameras(self):
        import omni.replicator.core as rep
        from PIL import Image
        self._Image = Image
        cx, cy, cz = self.car_center
        for key, (dx, dy, dz) in _CAM_DEFS:
            cam = rep.create.camera(
                position=(cx + dx, cy + dy, cz + dz),
                look_at=(float(cx), float(cy), float(cz)),
            )
            rp = rep.create.render_product(cam, resolution=_CAM_RES)
            annot = rep.AnnotatorRegistry.get_annotator("rgb")
            annot.attach([rp])
            self.cam_annots[key] = annot
            self.cam_pubs[key] = self.node.create_publisher(
                self._CompressedImage, f"/camera/{key}/compressed", 1)
        print(f"[ros_pub] 카메라 {len(self.cam_annots)}대 설정(해상도 {_CAM_RES})")

    # ── 매 스텝 호출 ─────────────────────────────────────────
    def tick(self, polishing=True):
        if not self.ok:
            return
        self.step += 1
        try:
            if self.step % _SCALAR_EVERY == 0:
                self._pub_scalars(polishing)
            if self.step % _HEATMAP_EVERY == 0:
                self._pub_heatmap()
            if (not self._cam_disabled) and self.step % _CAMERA_EVERY == 0:
                self._pub_cameras()
            self._rclpy.spin_once(self.node, timeout_sec=0.0)
        except Exception as exc:  # noqa: BLE001
            print(f"[ros_pub] tick 오류(무시): {exc}")

    def _seg_progress(self, agent):
        try:
            n = max(1, len(agent.yz_stops))
            if getattr(agent, "done", False):
                return 100.0
            return float(min(100.0, 100.0 * agent.current_seg_idx / n))
        except Exception:  # noqa: BLE001
            return 0.0

    def _pub_scalars(self, polishing):
        F = self._Float64
        # 전체 진행률
        try:
            total = self.polish_viz.total_count()
            cov = self.polish_viz.covered_count()
            overall = 100.0 * cov / total if total else 0.0
        except Exception:  # noqa: BLE001
            overall = 0.0
        self.pubs["progress"].publish(F(data=float(overall)))
        self.pubs["elapsed"].publish(F(data=float(time.time() - self.start_time)))
        # 로봇별 힘/진행률
        by_label = {getattr(a, "label", ""): a for a in self.agents}
        for lab in ("C", "SL", "SR"):
            a = by_label.get(lab)
            force = float(getattr(a, "filtered_contact_force", 0.0)) if a is not None else 0.0
            self.pubs[f"force_{lab}"].publish(F(data=force))
            self.pubs[f"prog_{lab}"].publish(F(data=self._seg_progress(a) if a is not None else 0.0))
            # 완료 지표 누적(실데이터): 접촉 임계 0.5N 이상이면 접촉으로 보고 평균에 반영
            if a is not None:
                self._total_samples += 1
                if force >= _CONTACT_FORCE_THRESH:
                    self._contact_samples += 1
                    self._contact_force_sum += force
        # 상태
        all_done = bool(self.agents) and all(getattr(a, "done", False) for a in self.agents)
        if all_done and not self._done_sent:
            self.pubs["state"].publish(self._String(data="DONE"))
            self._done_sent = True
            self._dump_coverage()      # 완료 시 3D 커버리지 파일 1회 기록(대시보드 3D 뷰어용)
        elif not all_done:
            self.pubs["state"].publish(self._String(data="POLISH" if polishing else "APPROACH"))

    def _pub_heatmap(self, cell=0.04):
        try:
            pts = np.asarray(self.polish_viz.points, dtype=float)
            pol = np.asarray(self.polish_viz._polished, dtype=bool)
        except Exception:  # noqa: BLE001
            return
        if len(pts) == 0:
            return
        mn = pts[:, :2].min(axis=0)
        mx = pts[:, :2].max(axis=0)
        cols = max(1, int(np.ceil((mx[0] - mn[0]) / cell)))
        rows = max(1, int(np.ceil((mx[1] - mn[1]) / cell)))
        cols = min(cols, 200); rows = min(rows, 200)
        cnt = np.zeros((rows, cols), dtype=np.float64)
        hit = np.zeros((rows, cols), dtype=np.float64)
        ix = np.clip(((pts[:, 0] - mn[0]) / cell).astype(int), 0, cols - 1)
        iy = np.clip(((pts[:, 1] - mn[1]) / cell).astype(int), 0, rows - 1)
        for r, c, p in zip(iy, ix, pol):
            cnt[r, c] += 1.0
            if p:
                hit[r, c] += 1.0
        grid = np.full((rows, cols), -1.0, dtype=np.float64)
        nz = cnt > 0
        grid[nz] = hit[nz] / cnt[nz]
        msg = self._Float64MultiArray()
        from std_msgs.msg import MultiArrayLayout, MultiArrayDimension
        lay = MultiArrayLayout()
        lay.dim = [MultiArrayDimension(label="rows", size=int(rows), stride=int(rows * cols)),
                   MultiArrayDimension(label="cols", size=int(cols), stride=int(cols))]
        msg.layout = lay
        msg.data = grid.reshape(-1).tolist()
        self.pubs["heatmap"].publish(msg)

    def _dump_coverage(self, max_points=20000):
        """완료 시 3D 커버리지(점 좌표 + 폴리싱 여부)를 대시보드가 읽을 JSON으로 1회 기록.
        web_dashboard/public/data/coverage.json → 폴리싱 완료 탭의 360° 3D 뷰어가 로드."""
        try:
            pts = np.asarray(self.polish_viz.points, dtype=float)
            pol = np.asarray(self.polish_viz._polished, dtype=bool)
        except Exception:  # noqa: BLE001
            return
        if len(pts) == 0:
            return
        if len(pts) > max_points:
            idx = np.linspace(0, len(pts) - 1, max_points).astype(int)
            pts = pts[idx]; pol = pol[idx]
        avg_force = (self._contact_force_sum / self._contact_samples) if self._contact_samples else 0.0
        contact_ratio = (100.0 * self._contact_samples / self._total_samples) if self._total_samples else 0.0
        out = {
            "points": np.round(pts, 3).tolist(),
            "polished": pol.astype(int).tolist(),
            "min": np.round(pts.min(axis=0), 3).tolist(),
            "max": np.round(pts.max(axis=0), 3).tolist(),
            "covered": int(pol.sum()),
            "total": int(len(pol)),
            # 결과 지표용 실데이터(폴리싱 완료 시점)
            "elapsed_sec": round(time.time() - self.start_time, 1),
            "avg_force": round(float(avg_force), 2),       # 평균 접촉력 N
            "contact_ratio": round(float(contact_ratio), 1),  # 접촉 비율 %
        }
        data_dir = os.path.join(_ROOT, "web_dashboard", "public", "data")
        try:
            os.makedirs(data_dir, exist_ok=True)
            with open(os.path.join(data_dir, "coverage.json"), "w") as f:
                json.dump(out, f)
            print(f"[ros_pub] 3D 커버리지 저장: web_dashboard/public/data/coverage.json "
                  f"({out['covered']}/{out['total']}점)")
        except Exception as exc:  # noqa: BLE001
            print(f"[ros_pub] 커버리지 저장 실패(무시): {exc}")

    def _pub_cameras(self):
        if self._Image is None:
            return
        for key, annot in self.cam_annots.items():
            try:
                data = annot.get_data()
                if data is None or len(data) == 0:
                    continue
                arr = np.asarray(data)
                if arr.ndim != 3 or arr.shape[2] < 3:
                    continue
                img = self._Image.fromarray(arr[:, :, :3].astype(np.uint8), "RGB")
                buf = io.BytesIO()
                img.save(buf, format="JPEG", quality=60)
                msg = self._CompressedImage()
                msg.format = "jpeg"
                msg.data = list(buf.getvalue())
                self.cam_pubs[key].publish(msg)
            except Exception:  # noqa: BLE001
                self._cam_fail += 1
                if self._cam_fail > 15:
                    self._cam_disabled = True
                    print("[ros_pub] 카메라 반복 실패 → 카메라 퍼블리시 비활성화(스칼라/히트맵 유지)")
                return

    def shutdown(self):
        if not self.ok:
            return
        try:
            self.node.destroy_node()
        except Exception:  # noqa: BLE001
            pass


def make_publisher(agents, polish_viz, car_center):
    """실패해도 None 대신 비활성 객체를 돌려준다(호출부가 항상 .tick() 가능)."""
    try:
        return PolishingRosPublisher(agents, polish_viz, car_center)
    except Exception as exc:  # noqa: BLE001
        print(f"[ros_pub] 생성 실패(시뮬 계속): {exc}")

        class _Noop:
            ok = False
            def tick(self, *a, **k): pass
            def shutdown(self): pass
        return _Noop()
