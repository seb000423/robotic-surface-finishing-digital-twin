# -*- coding: utf-8 -*-
"""표면 품질 KPI 요약을 만든다.

    python 데이터셋/make_quality.py [원본.txt]

`data_sample.txt`(에피소드 x 셀 단위 원본, 2.4 MB)를 에피소드 하나당 한 줄로
줄여 `quality_kpi.json` 으로 뱉는다. 웹 UI이 판정 화면에서 이 파일을 읽는다.
원본을 그대로 fetch 하면 브라우저가 매번 만 줄을 파싱한다 —
seg_best_kpi.json 과 같은 이유로 같은 방식을 쓴다.

집계 방식은 판정 기준의 표현을 그대로 따른다:

    예측 20° GU proxy      타일 분포   복합 판정(아래 GLOSS 참고)
    평균 거칠기 Ra          평균        <= 0.20 um
    극단 거칠기 Rz          최대        <= 2.0 um
    잔여 클리어코트 (최소)   최소        >= 35 um
    스크래치               평균        초기 대비 감소

'극단'은 최대값, '잔여(최소)'는 최소값이다. 평균으로 뭉개면
한 군데가 뚫려도 통과해 버린다 — 기준이 그렇게 쓰여 있지 않다.
"""
import csv, io, json, os, sys, math, datetime

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.join(HERE, sys.argv[1] if len(sys.argv) > 1 else "data_sample.txt")
OUT = os.path.join(HERE, "quality_kpi.json")

# ══ GU (20° 광택) proxy ═══════════════════════════════════════
# gu_proxy.py:60-70 · gloss_proxy.py:31-39
#   GU = clip( 25.0 + (78.0 - 25.0) * q , 0, 100 )
# 앵커는 Ulbrich et al., Coatings 2021, 11, 1320 (10.3390/coatings11111320):
#   25.0  결함 보수도장 21.1-29.7 GU        (defective_anchor_gu)
#   78.0  양호 보수도장 75.8-79.7 GU        (good_refinish_anchor_gu)
#   89.0  신차 평균 88.8 GU                 (별도 profile 일 때만 상한 교체)
#   70.0  저자 제안 목표 >70 GU             (target_gu)
# !! Gloss Meter 보정을 거치지 않은 시뮬레이션 전용 proxy 다.
#    앵커는 L-DERIVED, 결합식은 PT-DESIGN -> 출력은 SYNTHETIC.
#    화면에는 '측정 광택도'가 아니라 '예측 20° GU proxy' 로 적어야 한다.
DEFECTIVE_ANCHOR_GU = 25.0
GOOD_REFINISH_ANCHOR_GU = 78.0
TARGET_GU = 70.0
ACTUAL_GLOSS_METER_CALIBRATED = False

TILE = 5            # 5x5 셀 = 타일 하나 (gloss_proxy.py:93-135)

# 가중치 기본값 — 전부 PT-DESIGN
W_RA = 1.0
W_SCRATCH = 1.0
W_UNIFORMITY = 1.0
W_CLEARCOAT = 0.0   # 잔량과 20° GU 의 직접 광학 상관 근거가 없다. 안전 실패로만 본다
W_OPTICAL = 0.0     # RTX 파이프라인 미연결 (Step 8 에서 켠다)
W_THERMAL = 1.0     # 정규화 밖에서 일방 감점

# 판정 (gloss_proxy.py:154-160) — 평균만으로 통과시키지 않는다
GLOSS_PASS = {"mean": 70.0, "p10": 60.0, "std": 10.0, "min": 45.0}
BANDS = [(70.0, "target_pass"), (60.0, "partial"), (30.0, "low"), (-1e9, "severe_defect")]


def clip(v, lo, hi):
    return lo if v < lo else (hi if v > hi else v)


def mean(xs):
    return sum(xs) / len(xs) if xs else 0.0


def std(xs):
    if len(xs) < 2:
        return 0.0
    m = mean(xs)
    return math.sqrt(sum((x - m) ** 2 for x in xs) / len(xs))


def pct(xs, p):
    """선형 보간 백분위수."""
    if not xs:
        return 0.0
    v = sorted(xs)
    if len(v) == 1:
        return v[0]
    k = (len(v) - 1) * (p / 100.0)
    lo = int(math.floor(k))
    hi = min(lo + 1, len(v) - 1)
    return v[lo] + (v[hi] - v[lo]) * (k - lo)


def gu_from_q(q):
    """q(0~1) -> 20° GU. 회귀 기준값: 0.2180 -> 36.6, 0.8815 -> 71.7"""
    return clip(DEFECTIVE_ANCHOR_GU + (GOOD_REFINISH_ANCHOR_GU - DEFECTIVE_ANCHOR_GU) * q, 0.0, 100.0)


def tile_q(cells):
    """타일 하나의 q_total. gloss_proxy.py:93-135

       ln q_total = ( Sum w_k * ln q_k ) / Sum w_k  +  w_thermal * ln q_thermal

    열항만 정규화 밖에서 일방 감점한다 — 한 항의 심한 결함이 기하평균에
    숨지 않게 하려는 것이 결합식 전체의 목적이다.
    """
    f = lambda c: [float(r[c]) for r in cells]

    # q_ra — Ra 는 detrend 후 mean|z-zbar|. 원본이 셀마다 이미 들고 있다.
    ra = mean(f("ra_after_um"))
    q_ra = math.exp(-max(0.0, ra - 0.08) / 0.10)

    # q_scratch — 잔존/초기 최대값 비. 초기에 흠이 없으면 감점할 것도 없다.
    s_init = max(f("scratch_before"))
    s_res = max(f("scratch_after"))
    q_scratch = 1.0 - clip(s_res / s_init, 0.0, 1.0) if s_init > 1e-12 else 1.0

    # q_uniformity — 누적 제거량의 산포. 고르게 깎였는가.
    q_uniformity = math.exp(-std(f("clearcoat_removed_um")) / 1.0)

    # q_clearcoat — 가중치 0. GU 결합에서 빠지고 안전 실패 판정에만 쓴다.
    cc_min = min(f("clearcoat_after_um"))
    cc_init = mean(f("clearcoat_before_um"))
    q_clearcoat = clip((cc_min - 30.0) / (cc_init - 30.0), 0.0, 1.0) if cc_init > 30.0 else 0.0

    # q_optical — RTX 상대 정반사. 파이프라인 미연결이라 가중치 0.
    q_optical = 1.0

    # q_thermal — 원본에 thermal_damage_proxy 열이 없으면 열손상 0 으로 본다.
    if "thermal_damage_proxy" in cells[0]:
        q_thermal = math.exp(-mean(f("thermal_damage_proxy")) / 1.0)
    else:
        q_thermal = 1.0

    terms = [(W_RA, q_ra), (W_SCRATCH, q_scratch), (W_UNIFORMITY, q_uniformity),
             (W_CLEARCOAT, q_clearcoat), (W_OPTICAL, q_optical)]
    wsum = sum(w for w, _ in terms if w > 0)
    if wsum <= 0:
        return 1.0
    ln_q = sum(w * math.log(max(q, 1e-12)) for w, q in terms if w > 0) / wsum
    ln_q += W_THERMAL * math.log(max(q_thermal, 1e-12))
    return clip(math.exp(ln_q), 0.0, 1.0)


def band_of(gu):
    for lo, name in BANDS:
        if gu >= lo:
            return name
    return "severe_defect"


def gloss_of(cells):
    """에피소드의 GU 분포. 셀을 5x5 타일로 묶어 타일마다 GU 를 낸다."""
    grid = {}
    for r in cells:
        grid[(int(round(float(r["x"]) * 1e6)), int(round(float(r["y"]) * 1e6)))] = r
    xs = sorted({k[0] for k in grid})
    ys = sorted({k[1] for k in grid})
    gus = []
    for i in range(0, len(xs), TILE):
        for j in range(0, len(ys), TILE):
            tile = [grid[(x, y)] for x in xs[i:i + TILE] for y in ys[j:j + TILE] if (x, y) in grid]
            if tile:
                gus.append(gu_from_q(tile_q(tile)))
    m, p10, sd, mn = mean(gus), pct(gus, 10), std(gus), min(gus)
    return {
        "glossMean": round(m, 2), "glossP10": round(p10, 2),
        "glossStd": round(sd, 2), "glossMin": round(mn, 2),
        "glossTiles": len(gus), "glossBand": band_of(m),
        "glossPass": bool(m >= GLOSS_PASS["mean"] and p10 >= GLOSS_PASS["p10"]
                          and sd <= GLOSS_PASS["std"] and mn >= GLOSS_PASS["min"]),
    }


# ── 판정 기준 ────────────────────────────────────────────────
# 값은 팀 기준표(2026-08)에서 그대로 옮겼다. 여기서 바꾸면 화면도 따라간다.
CRITERIA = [
    {"key": "glossMean", "label": "예측 20° GU proxy", "unit": "GU", "op": "gloss",
     "limit": TARGET_GU, "digits": 1},
    {"key": "ra",        "label": "평균 거칠기 Ra",       "unit": "µm", "op": "<=", "limit": 0.20, "digits": 3},
    {"key": "rz",        "label": "극단 거칠기 Rz",       "unit": "µm", "op": "<=", "limit": 2.0,  "digits": 3},
    {"key": "clearcoat", "label": "잔여 클리어코트 (최소)", "unit": "µm", "op": ">=", "limit": 35.0, "digits": 2},
    {"key": "scratch",   "label": "스크래치",             "unit": "",   "op": "<",  "limit": None, "digits": 4},
]


def main():
    if not os.path.exists(SRC):
        sys.exit("원본이 없다: " + SRC)
    rows = list(csv.DictReader(io.open(SRC, encoding="utf-8-sig")))
    if not rows:
        sys.exit("빈 파일이다")

    eps = {}
    for r in rows:
        eps.setdefault(r["episode_id"], []).append(r)

    out = []
    for eid in sorted(eps, key=lambda x: int(x)):
        R = eps[eid]
        col = lambda c: [float(r[c]) for r in R]
        avg = lambda c: mean(col(c))

        rec = {
            "episode": int(eid), "recipe": R[0]["recipe_id"],
            "seed": int(R[0]["surface_seed"]), "cells": len(R),
            # ── 판정 대상 ──
            "ra": round(avg("ra_after_um"), 4),                    # 평균
            "rz": round(max(col("rz_after_um")), 4),               # 극단 = 최대
            "clearcoat": round(min(col("clearcoat_after_um")), 3),  # 잔여 = 최소
            "scratch": round(avg("scratch_after"), 5),
            "scratchBefore": round(avg("scratch_before"), 5),
            # ── 참고 ──
            "raBefore": round(avg("ra_before_um"), 4),
            "rzBefore": round(max(col("rz_before_um")), 4),
            "clearcoatRemoved": round(avg("clearcoat_removed_um"), 4),
            "defectRatio": round(avg("is_defect"), 4),
            "passMean": round(avg("pass_count"), 2),
            "force": round(avg("force_N"), 3),
            "rpm": round(avg("rpm"), 1),
            "feed": round(avg("feed_mm_s"), 3),
            "dwell": round(avg("dwell_weighted_s"), 2),
        }
        rec.update(gloss_of(R))
        out.append(rec)

    doc = {
        "generated": datetime.date.today().isoformat(),
        "source": os.path.basename(SRC),
        "note": "에피소드 하나당 한 줄. 집계 방식은 make_quality.py 주석 참고.",
        "gloss": {
            "model": "GU = %.1f + (%.1f - %.1f) * q_total, 5x5 타일" % (
                DEFECTIVE_ANCHOR_GU, GOOD_REFINISH_ANCHOR_GU, DEFECTIVE_ANCHOR_GU),
            "anchors": "Ulbrich et al., Coatings 2021, 11, 1320 (10.3390/coatings11111320)",
            "weights": {"ra": W_RA, "scratch": W_SCRATCH, "uniformity": W_UNIFORMITY,
                        "clearcoat": W_CLEARCOAT, "optical": W_OPTICAL, "thermal": W_THERMAL},
            "pass": GLOSS_PASS,
            "calibrated": ACTUAL_GLOSS_METER_CALIBRATED,
            "tag": "SYNTHETIC",
            "thermalSource": "열손상 열이 원본에 없어 q_thermal = 1 로 둔다",
        },
        "criteria": CRITERIA,
        "episodes": out,
    }
    io.open(OUT, "w", encoding="utf-8", newline="").write(
        json.dumps(doc, ensure_ascii=False, indent=1))

    # ── 회귀 확인 ──
    for q, want in ((0.2180, 36.6), (0.8815, 71.7)):
        got = gu_from_q(q)
        print("회귀 q=%.4f -> %.1f GU (기준 %.1f) %s"
              % (q, got, want, "OK" if abs(got - want) < 0.05 else "!! 불일치"))
    print("\n%s -> %s" % (os.path.basename(SRC), os.path.basename(OUT)))
    print("  %d행 / 에피소드 %d개 / 타일 %d개" % (len(rows), len(out), out[0]["glossTiles"]))
    for e in out:
        ok = (e["glossPass"], e["ra"] <= 0.20, e["rz"] <= 2.0,
              e["clearcoat"] >= 35, e["scratch"] < e["scratchBefore"])
        print("   ep%-2d GU 평균 %5.1f p10 %5.1f σ %4.1f 최소 %5.1f [%s] %s"
              % (e["episode"], e["glossMean"], e["glossP10"], e["glossStd"],
                 e["glossMin"], e["glossBand"], "광택통과" if e["glossPass"] else "광택미달"))
        print("        Ra %.3f  Rz %.3f  잔여 %6.2f  스크래치 %.4f->%.4f  => %s"
              % (e["ra"], e["rz"], e["clearcoat"], e["scratchBefore"], e["scratch"],
                 "통과" if all(ok) else "미달"))


if __name__ == "__main__":
    main()
