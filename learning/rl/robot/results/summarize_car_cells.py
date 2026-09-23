"""차 전체 셀 순회 결과 집계 (car_cells.csv) — 면별 합격률·처분·보증 준수·실패 사유.
    python3 learning/rl/robot/results/summarize_car_cells.py [csv]
"""
import csv, sys, collections, itertools

path = sys.argv[1] if len(sys.argv) > 1 else "learning/rl/robot/results/car_cells.csv"
rows = list(csv.DictReader(open(path)))
if not rows:
    print("행 없음"); sys.exit(0)
def b(v): return str(v).strip().lower() == "true"
tot = len(rows); npass = sum(b(r["overall_pass"]) for r in rows)
print(f"== {path}: {tot}셀 | 합격 {npass} ({100*npass/tot:.1f}%) ==")
print("\n[면별]")
by_reg = {}
for r in rows: by_reg.setdefault(r["region"], []).append(r)
for reg, g in sorted(by_reg.items()):
    p = sum(b(r["overall_pass"]) for r in g)
    disp = collections.Counter(r["disposition"] for r in g)
    print(f"  {reg:12s} {p:4d}/{len(g):<4d} ({100*p/len(g):5.1f}%)  " + ", ".join(f"{k}:{v}" for k, v in sorted(disp.items())))
print("\n[처분]", dict(collections.Counter(r["disposition"] for r in rows)))
print("[결과 outcome]", dict(collections.Counter(r.get("outcome", "") for r in rows)))
w = [r for r in rows if b(r["overall_pass"])]
print(f"[보증 준수] 합격 {len(w)}셀 중 warranty_removal_ok {sum(b(r['warranty_removal_ok']) for r in w)}")
crit = {"gu_target_pass": "GU≥70", "ra_target_pass": "Ra≤0.20", "rz_target_pass": "Rz≤2.0",
        "scratch_improved": "자국 개선", "clearcoat_safe": "CC≥35"}
print("[기준별 통과율]", ", ".join(f"{lab} {100*sum(b(r[k]) for r in rows)/tot:.0f}%" for k, lab in crit.items()))
def fl(k):
    try: return [float(r[k]) for r in rows if r.get(k) not in (None, "", "None")]
    except ValueError: return []
for k, lab in (("gu_final", "GU 최종"), ("scratch_final_um", "자국 최종 μm"), ("clearcoat_min_um", "CC 최소 μm"), ("passes", "패스 수")):
    v = fl(k)
    if v: print(f"[{lab}] 평균 {sum(v)/len(v):.2f}  최소 {min(v):.2f}  최대 {max(v):.2f}")

# ── 곡률 vs 결과 (첫 배치 관찰: 과부하 실패가 강곡률 셀에 몰림) ──
try:
    def curv(r): return abs(float(r["quad_c3"])) + abs(float(r["quad_c5"]))
    buckets = [(0, 2, "|c3|+|c5| < 2 (완만)"), (2, 5, "2–5 (중간)"), (5, 99, "≥ 5 (강곡률)")]
    print("\n[곡률 구간별]")
    for lo, hi, lab in buckets:
        g = [r for r in rows if lo <= curv(r) < hi]
        if not g: continue
        p = sum(b(r["overall_pass"]) for r in g); ov = sum(r.get("outcome") == "fail_force_overload" for r in g)
        print(f"  {lab:22s} {p:4d}/{len(g):<4d} 합격 ({100*p/len(g):5.1f}%)  과부하 {ov}")
    q = [r for r in rows if r.get("outcome") == "fail_force_overload" and float(r["gu_final"] or 0) >= 70]
    print(f"[과부하 실패 중 품질(GU≥70)은 이미 충족] {len(q)}셀")
except Exception as e:
    print("(곡률 분석 생략:", e, ")")
