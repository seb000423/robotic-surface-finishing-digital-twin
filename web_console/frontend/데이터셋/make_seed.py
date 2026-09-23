# seg_best_kpi.json 재생성 — 1회성. UI 가 읽는 유일한 요약 파일이다.
# 왜 필요한가: sweep_traj_csv 는 184MB 라 브라우저가 통째로 읽을 수 없다.
# 세그먼트별_최적파라미터.csv 가 '정답 파라미터'는 주지만 그 run 이 원본 CSV의
# 어디에 있는지, 실제 힘 통계가 얼마인지는 안 준다. 그 둘을 잇는다.
#   python 데이터셋/make_seed.py     (프로젝트 루트에서, 약 3분)
# 하는 일: 15개 CSV 를 한 번씩 훑어 run 단위 KPI 와 바이트 범위를 모으고,
# 세그먼트별_최적파라미터.csv 의 (target_force_n, stiffness, damping, speed_scale)
# 과 가장 가까운 run 을 그 세그먼트의 정답 run 으로 확정한다.
# 검증: 이렇게 고른 run 의 in-band(접촉 스텝 기준)는 15개 전부 best_inband 와 일치한다.
import os, csv, io, json, math

HERE = os.path.dirname(os.path.abspath(__file__))
TRAJ = os.path.join(HERE, 'sweep_traj_csv', 'sweep_traj_csv')
OPT = os.path.join(HERE, 'sweep_분석_0827', '세그먼트별_최적파라미터.csv')
OUT = os.path.join(HERE, 'seg_best_kpi.json')

NEED = ['run_id', 'filtered_force_n', 'contacting', 'in_band', 'overpressure',
        'high_force_pause_steps', 'waypoint_idx',
        'p_target_force', 'p_stiffness', 'p_damping', 'p_speed_scale']


def scan(path):
    """run_id 별 KPI 와 파일 내 바이트 범위. run 의 행은 파일 안에서 연속이다."""
    runs = {}
    with open(path, 'rb') as f:
        head = f.readline()
        cols = head.decode('utf-8-sig').strip().split(',')
        i = {c: cols.index(c) for c in NEED}
        off = len(head)
        for raw in f:
            p = raw.decode('ascii', 'replace').rstrip('\r\n').split(',')
            rid = p[i['run_id']]
            r = runs.get(rid)
            if r is None:
                r = runs[rid] = {'start': off, 'steps': 0, 'contact': 0, 'inband': 0, 'over': 0,
                                 'pause': 0, 'fsum': 0.0, 'fsq': 0.0, 'wp': 0,
                                 'p': [float(p[i['p_target_force']]), float(p[i['p_stiffness']]),
                                       float(p[i['p_damping']]), float(p[i['p_speed_scale']])]}
            off += len(raw)
            r['end'] = off
            r['steps'] += 1
            if p[i['contacting']] == '1':
                r['contact'] += 1
                v = float(p[i['filtered_force_n']])
                r['fsum'] += v; r['fsq'] += v * v
                if p[i['in_band']] == '1': r['inband'] += 1
            if p[i['overpressure']] == '1': r['over'] += 1
            if p[i['high_force_pause_steps']] not in ('0', '0.0'): r['pause'] += 1
            w = int(float(p[i['waypoint_idx']]))
            if w > r['wp']: r['wp'] = w
    for r in runs.values():
        m = r['fsum'] / r['contact'] if r['contact'] else 0.0
        r['mean'] = round(m, 3)
        r['std'] = round(math.sqrt(max(0.0, r['fsq'] / r['contact'] - m * m)) if r['contact'] else 0.0, 3)
        r['inbandRatio'] = round(r['inband'] / r['contact'], 4) if r['contact'] else None
        r['contactRatio'] = round(r['contact'] / r['steps'], 4)
    return cols, len(head), runs


def main():
    opt = list(csv.DictReader(io.open(OPT, encoding='utf-8-sig')))
    header, segs = None, []
    for row in opt:
        seg = row['segment']
        fn = 'traj_%s_steps.csv' % seg
        path = os.path.join(TRAJ, fn)
        cols, head_len, runs = scan(path)
        header = header or cols
        want = [float(row['target_force_n']), float(row['stiffness']),
                float(row['damping']), float(row['speed_scale'])]
        rid, r = min(runs.items(), key=lambda kv: sum(
            abs(a - b) / max(1e-9, abs(b)) for a, b in zip(kv[1]['p'], want)))
        bid = next((k for k in runs if 'baseline' in k), None)
        b = runs[bid] if bid else None
        assert abs(r['inbandRatio'] - float(row['best_inband'])) < 0.002, (seg, r['inbandRatio'])
        segs.append({
            'seg': seg,
            'robot': '천장' if seg.startswith('C') else '측면좌',
            'file': '데이터셋/sweep_traj_csv/sweep_traj_csv/%s' % fn,
            'fileSize': os.path.getsize(path), 'headerLen': head_len, 'runCount': len(runs),
            'bestRun': rid, 'byteStart': r['start'], 'byteEnd': r['end'],
            'steps': r['steps'], 'waypoints': r['wp'] + 1,
            'inband': float(row['best_inband']),
            'baselineInband': float(row['baseline_inband']),
            'delta': float(row['delta']),
            'contactRatio': r['contactRatio'],
            'overSteps': r['over'], 'overRatio': r['over'] / r['steps'],
            'pauseSteps': r['pause'],
            'forceMean': r['mean'], 'forceStd': r['std'],
            'cv': round(r['std'] / r['mean'] * 100, 2) if r['mean'] else None,
            'p': {'force': want[0], 'stiffness': want[1], 'damping': want[2], 'speed': want[3]},
            'baseline': ({'run': bid, 'byteStart': b['start'], 'byteEnd': b['end'],
                          'forceMean': b['mean'], 'forceStd': b['std'], 'steps': b['steps']}
                         if b else None),
        })
        print(seg, rid, 'in-band', r['inbandRatio'], flush=True)

    io.open(OUT, 'w', encoding='utf-8').write(json.dumps({
        'generated': '2026-08-29',
        'note': '데이터셋/make_seed.py 로 재생성한다. 원본 CSV 는 배포하지 않아도 되지만, '
                'UI 의 CSV 열람은 원본을 Range 요청으로 읽으므로 원본이 있어야 한다.',
        'source': '데이터셋/sweep_traj_csv/sweep_traj_csv',
        'header': header, 'segments': segs,
    }, ensure_ascii=False, indent=1))
    print('WROTE', OUT)


if __name__ == '__main__':
    main()
