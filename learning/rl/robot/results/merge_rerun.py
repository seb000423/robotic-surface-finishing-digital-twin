"""순회(car_cells.csv) + 과부하 재실행(car_cells_rerun.csv) → 셀별 최선 처분(car_cells_best.csv).

규칙: 원 순회 합격이면 그대로. 아니면 재실행이 합격이면 'pass' + recipe_variant=재실행 태그(곡면용 저압·소형패드).
      둘 다 실패면 원 처분 유지(재실행 outcome 은 rerun_outcome 열에 보존).
    python3 learning/rl/robot/results/merge_rerun.py [--rerun car_cells_rerun.csv] [--out car_cells_best.csv]
"""
import argparse, csv, os, collections
_H = os.path.dirname(os.path.abspath(__file__))
ap = argparse.ArgumentParser()
ap.add_argument("--base", default=os.path.join(_H, "car_cells.csv"))
ap.add_argument("--rerun", default=os.path.join(_H, "car_cells_rerun.csv"))
ap.add_argument("--out", default=os.path.join(_H, "car_cells_best.csv"))
a = ap.parse_args()
base = list(csv.DictReader(open(a.base, encoding="utf-8")))
rer = {}
if os.path.exists(a.rerun):
    for r in csv.DictReader(open(a.rerun, encoding="utf-8")):
        rer[r["cell_id"]] = r
def ok(r): return str(r.get("overall_pass", "")).lower() in ("true", "1")
out, n_new = [], 0
for r in base:
    m = dict(r); m["recipe_variant"] = "base"; m["rerun_outcome"] = ""
    rr = rer.get(r["cell_id"])
    if rr is not None:
        m["rerun_outcome"] = rr.get("outcome", "")
        if not ok(r) and ok(rr):
            for k in ("outcome", "passes", "gu_final", "ra_final_um", "rz_final_um", "scratch_final_um", "clearcoat_min_um",
                      "temperature_peak_c", "gu_target_pass", "ra_target_pass", "rz_target_pass", "scratch_improved",
                      "clearcoat_safe", "warranty_removal_ok", "overall_pass", "disposition", "checkpoint"):
                if k in rr: m[k] = rr[k]
            m["recipe_variant"] = rr.get("checkpoint", "").split("#")[-1] if "#" in rr.get("checkpoint", "") else "rerun"
            n_new += 1
    out.append(m)
cols = list(base[0].keys()) + ["recipe_variant", "rerun_outcome"]
with open(a.out, "w", newline="", encoding="utf-8") as f:
    w = csv.DictWriter(f, fieldnames=cols); w.writeheader(); w.writerows(out)
n = len(out); p = sum(ok(r) for r in out); p0 = sum(ok(r) for r in base)
print(f"== 병합: 순회 {n}셀 합격 {p0} → 재실행 {len(rer)}셀 반영 후 합격 {p} ({100*p/n:.1f}%), 재실행으로 구제 {n_new}")
byreg = collections.defaultdict(lambda: [0, 0, 0])
for r, b in zip(out, base):
    d = byreg[r["region"]]; d[0] += 1; d[1] += ok(r); d[2] += ok(r) and not ok(b)
for k in sorted(byreg): print(f"  {k:11s} {byreg[k][1]:3d}/{byreg[k][0]:<3d} (+{byreg[k][2]} 구제)")
if rer:
    rc = collections.Counter(r.get("outcome", "") for r in rer.values())
    print("  재실행 outcome:", dict(rc))
    still = [r for r in rer.values() if not ok(r)]
    print(f"  재실행 후에도 실패 {len(still)} — 과부하 재발 {sum(1 for r in still if r.get('outcome')=='fail_force_overload')}")
print("→", a.out)
