"""차량 6영역 × 5×5 = 150셀 입력 CSV 예제 생성.

    ~/isaacsim/kit/python/bin/python3 learning/vehicle_export/make_example_input.py

실제 운용에서는 차량 검사 시스템이 이 스키마로 CSV 를 공급한다.
초기 Rq/Rz/GU 는 참고열 — 여기서는 같은 seed 로 합성한 표면에서 twin 이 잰 값을 넣어
입력 예제가 자기모순 없게 한다 (export 스크립트도 같은 절차로 재합성).
"""
import csv
import os
import sys

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
_REPO = os.path.dirname(os.path.dirname(_HERE))
sys.path.insert(0, _REPO)

from learning.digital_twin.gloss_proxy import LiteratureGlossProxyModel      # noqa: E402
from learning.digital_twin.roughness_metrics import ra_um, rz_um             # noqa: E402
from learning.vehicle_export.export_vehicle_results import (             # noqa: E402
    INPUT_COLUMNS, rq_um, synthesize_cell_patch)

# 영역: (id, 이름, 원점[m], u축, v축, 법선) — 차량 좌표계 x 전방 / y 좌 / z 상
# 셀 격자는 u,v 방향 5×5, 셀 간격은 영역 크기/5.
REGIONS = [
    ("R1", "bonnet",        (1.60, -0.60, 0.95), (1.00, 0.0, 0.10), (0.0, 1.20, 0.0), (-0.10, 0.0, 0.995)),
    ("R2", "roof",          (-0.20, -0.55, 1.45), (1.20, 0.0, 0.0),  (0.0, 1.10, 0.0), (0.0, 0.0, 1.0)),
    ("R3", "door_left",     (-0.10, 0.85, 0.55), (1.20, 0.0, 0.0),  (0.0, 0.0, 0.45), (0.0, 1.0, 0.0)),
    ("R4", "door_right",    (-0.10, -0.85, 0.55), (1.20, 0.0, 0.0), (0.0, 0.0, 0.45), (0.0, -1.0, 0.0)),
    ("R5", "fender_left",   (1.90, 0.72, 0.60), (0.60, 0.0, 0.10),  (0.0, 0.05, 0.30), (0.15, 0.97, 0.19)),
    ("R6", "fender_right",  (1.90, -0.72, 0.60), (0.60, 0.0, 0.10), (0.0, -0.05, 0.30), (0.15, -0.97, 0.19)),
]
GRID = 5


# ── 시나리오 프로파일 ──────────────────────────────────────────────────────
# correction(기본): 보정 대상 손상차 — 기존 분포 유지.
# new_car: 신차 PDI — 구조 자체가 다름:
#   · 스월 = 전면 분산 미세결함 → "스크래치 띠"가 아니라 거칠기장 Ra 로 표현.
#     Ra ≈ 0.12 는 L-DERIVED: 스월+개별자국 합산 초기 GU proxy 가 67.7 (Coatings
#     2021 의 B-세그먼트 신차 평균 20° 광택 앵커)이 되도록 역산한 값.
#     (1차 보정 0.235 는 무스크래치 기준이라 자국 감점이 겹쳐 초기 GU 58.5 로 과손상 —
#      합산 기준으로 재보정. 우리 proxy 의 q_scratch 가중상 자국이 결손 대부분을 차지)
#   · 개별 자국 = 띠 1~3개, 얕음(0.1~0.6μm) — 취급/운송 경미 자국 (PT-DESIGN)
#   · 깊은 꼬리 = 15% 셀에 0.6~1.5μm — 운송/상하차 손상 (PT-DESIGN, 실측 분포 부재)
NEW_CAR_SWIRL_RA = 0.12       # L-DERIVED (신차 합산 GU 67.7 앵커 역산, 2차 보정)

def draw_initial_state(rng, profile: str):
    """프로파일별 (ra, scratch_max, n_scratches, clearcoat)."""
    cc0 = float(rng.uniform(40.0, 50.0))
    if profile == "new_car":
        ra0 = float(np.clip(rng.normal(NEW_CAR_SWIRL_RA, 0.012), 0.09, 0.16))
        if rng.uniform() < 0.15:                      # 운송 손상 꼬리 (PT-DESIGN)
            # 상한 1.0μm = "폴리싱 가능급" 경계:
            #   ① OEM 보증 규정(누적 제거 ≤7.5μm ≈ 풀패스 2회)에서 제거 가능한 깊이
            #   ② 자체 통과율 곡선(≤1.0μm 76%+, 초과 시 붕괴)의 이중 앵커.
            #   더 깊은 손상은 상류 검사에서 재도장 분류되어 폴리싱 공정에 오지 않는다.
            scr0, n_scr = float(rng.uniform(0.5, 1.0)), int(rng.integers(2, 5))
        else:                                          # 세차 마링 (L-DERIVED ≤0.5μm, NIST 2018)
            scr0, n_scr = float(rng.uniform(0.05, 0.5)), int(rng.integers(1, 4))
    else:  # correction (기존)
        ra0 = float(np.clip(rng.normal(0.08, 0.012), 0.05, 0.12))
        scr0, n_scr = float(rng.uniform(0.4, 1.9)), None
    return ra0, scr0, n_scr, cc0


def main():
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--profile", default="correction", choices=["correction", "new_car"])
    args = ap.parse_args()
    rng = np.random.default_rng(2026)
    gloss = LiteratureGlossProxyModel()
    suffix = "" if args.profile == "correction" else "_newcar"
    out = os.path.join(_HERE, f"vehicle_150_cells{suffix}.csv")
    rows = []
    for rid, rname, origin, u, v, normal in REGIONS:
        origin, u, v = map(np.asarray, (origin, u, v))
        n = np.asarray(normal, float); n /= np.linalg.norm(n)
        for i in range(GRID):
            for j in range(GRID):
                cell = i * GRID + j
                pos = origin + u * ((i + 0.5) / GRID) + v * ((j + 0.5) / GRID)
                seed = 7000 + 101 * int(rid[1]) + cell     # export 기본 seed 와 동일 규칙
                ra0, scr0, n_scr, cc0 = draw_initial_state(rng, args.profile)
                st = synthesize_cell_patch(seed, ra0, scr0, cc0, n_scratches=n_scr)
                rows.append({
                    "region_id": rid, "region_name": rname, "cell_id": cell,
                    "position_x_m": f"{pos[0]:.3f}", "position_y_m": f"{pos[1]:.3f}",
                    "position_z_m": f"{pos[2]:.3f}",
                    "normal_x": f"{n[0]:.4f}", "normal_y": f"{n[1]:.4f}", "normal_z": f"{n[2]:.4f}",
                    "init_roughness_um": f"{rq_um(st.micro_height_um):.4f}",
                    "init_scratch_um": f"{scr0:.3f}",
                    "init_ra_um": f"{ra0:.4f}",
                    "init_rz_um": f"{rz_um(st.micro_height_um):.4f}",
                    "init_clearcoat_um": f"{cc0:.2f}",
                    "init_gu_proxy": f"{gloss.evaluate(st)['summary']['gu_mean']:.2f}",
                    "surface_seed": seed,
                    "init_scratch_count": "" if n_scr is None else n_scr,
                })
        print(f"  {rid} {rname}: {GRID*GRID} cells")
    with open(out, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=INPUT_COLUMNS)
        w.writeheader(); w.writerows(rows)
    print(f"{len(rows)} rows → {out}")


if __name__ == "__main__":
    main()
