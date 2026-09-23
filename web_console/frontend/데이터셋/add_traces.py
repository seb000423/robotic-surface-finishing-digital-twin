# seg_best_kpi.json 에 힘 시계열(trace)을 덧붙인다. make_seed.py 다음에 돌린다.
#   python 데이터셋/add_traces.py
# 정답 run 과 baseline run 의 바이트 범위는 이미 시드에 있으므로 원본 CSV 를
# 통째로 읽지 않고 그 구간만 seek 해서 읽는다(세그먼트당 수백 KB).
# 그래프는 스텝 단위로 그리기엔 너무 길어서 BUCKETS 개 구간의 평균/최소/최대로 줄인다.
# 이 축약본 덕분에 라이브러리 카드와 시각화 패널은 추가 네트워크 요청 없이 그린다.
import os, io, json

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
SEED = os.path.join(HERE, 'seg_best_kpi.json')
BUCKETS = 120

NEED = ['filtered_force_n', 'target_force_n', 'band_lo_n', 'band_hi_n', 'contacting', 'in_band']


def trace(path, start, end, idx):
    with open(path, 'rb') as f:
        f.seek(start)
        blob = f.read(end - start)
    lines = blob.decode('ascii', 'replace').split('\n')
    if lines and not lines[-1].strip():
        lines.pop()
    n = len(lines)
    if not n:
        return None
    step = max(1, n // BUCKETS)
    out = {'f': [], 'lo': [], 'hi': [], 'c': [], 'tgt': None, 'n': n}
    for b in range(0, n, step):
        chunk = lines[b:b + step]
        fs, los, his, cs = [], [], [], 0
        for ln in chunk:
            p = ln.rstrip('\r').split(',')
            if len(p) <= idx['in_band']:
                continue
            fs.append(float(p[idx['filtered_force_n']]))
            los.append(float(p[idx['band_lo_n']]))
            his.append(float(p[idx['band_hi_n']]))
            if p[idx['contacting']] == '1':
                cs += 1
            if out['tgt'] is None:
                out['tgt'] = round(float(p[idx['target_force_n']]), 2)
        if not fs:
            continue
        out['f'].append(round(sum(fs) / len(fs), 2))
        out['lo'].append(round(sum(los) / len(los), 2))
        out['hi'].append(round(sum(his) / len(his), 2))
        out['c'].append(round(cs / len(chunk), 2))
    return out


def main():
    seed = json.load(io.open(SEED, encoding='utf-8'))
    cols = seed['header']
    idx = {c: cols.index(c) for c in NEED}
    for sg in seed['segments']:
        path = os.path.join(ROOT, *sg['file'].split('/'))
        sg['trace'] = trace(path, sg['byteStart'], sg['byteEnd'], idx)
        b = sg.get('baseline')
        if b:
            b['trace'] = trace(path, b['byteStart'], b['byteEnd'], idx)
        print(sg['seg'], len(sg['trace']['f']), 'pts', flush=True)
    io.open(SEED, 'w', encoding='utf-8').write(json.dumps(seed, ensure_ascii=False, indent=1))
    print('WROTE', SEED, os.path.getsize(SEED))


if __name__ == '__main__':
    main()
