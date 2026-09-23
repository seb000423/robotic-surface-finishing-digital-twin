# -*- coding: utf-8 -*-
"""④ 라이브러리가 읽는 구간만 잘라 둔다.

문제: 라이브러리는 `sweep_traj_csv/` 원본에 Range 요청을 걸어 run 로그를 편다.
      원본은 183 MB 인데 실제로 읽는 건 다 합쳐 4.4 MB 다.
      배포에서 원본을 빼면(=`.vercelignore`) 불러오기가 404 로 죽고,
      넣으면 8 MB 예산을 20배 넘긴다.

해법: 각 세그먼트가 읽는 바이트 범위를 그대로 파일로 떨어뜨린다.
      바이트가 같으므로 마지막 잘린 줄까지 동일하다 — 화면 동작이 안 바뀐다.

    python 데이터셋/make_traces.py

결과: `데이터셋/traces/<seg>.csv` 15개 + seg_best_kpi.json 에 sliceFile 필드.
"""
import io, json, os

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
SEED = os.path.join(HERE, "seg_best_kpi.json")
OUT = os.path.join(HERE, "traces")

CSV_BYTES = 512 * 1024        # 라이브러리의 CSV_BYTES 와 같아야 한다

d = json.load(io.open(SEED, encoding="utf-8"))
os.makedirs(OUT, exist_ok=True)

total = 0
for s in d["segments"]:
    src = os.path.join(ROOT, s["file"].replace("/", os.sep))
    end = min(s["byteEnd"] - 1, s["byteStart"] + CSV_BYTES - 1)
    n = end - s["byteStart"] + 1
    with open(src, "rb") as f:
        f.seek(s["byteStart"])
        buf = f.read(n)
    assert len(buf) == n, (s["seg"], len(buf), n)

    name = "traces/%s.csv" % s["seg"]
    with open(os.path.join(OUT, "%s.csv" % s["seg"]), "wb") as f:
        f.write(buf)

    # 원본 경로와 크기는 화면에 그대로 남는다 — "8.5 MB 중 0.17 MB" 라는 말이
    # 사실이어야 하므로 file / fileSize 는 건드리지 않는다
    s["sliceFile"] = "데이터셋/" + name
    s["sliceBytes"] = n
    total += n
    print("%-5s %7d bytes -> %s" % (s["seg"], n, name))

d["note"] += (" 배포용으로 각 run 의 읽기 구간만 데이터셋/traces/ 에 잘라 뒀다"
              " (make_traces.py). 원본 sweep_traj_csv 는 배포 제외.")

io.open(SEED, "w", encoding="utf-8", newline="").write(
    json.dumps(d, ensure_ascii=False, separators=(",", ":")))
print("합계 %.2f MB" % (total / 1048576.0))
