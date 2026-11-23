import csv, re, sys, math, statistics as st, collections, subprocess
csvp, logp = sys.argv[1], sys.argv[2]
R = [(float(r['t']), float(r['v']), float(r['cmd_w']), float(r['gyro']), float(r['yaw'])) for r in csv.DictReader(open(csvp))]
t0, t1 = R[0][0], R[-1][0]
MK = [("경유점 지남", "경유점→앞자리(nav2)"), ("출발: 직진으로", "POI 빠져나오기"), ("곧게 들어가기 끝", "_end"), ("제자리 정렬", "출발 회전"),
      ("출발 정렬 끝", "저속 출발"), ("곡선으로 출발", "저속 출발(곡선)"), ("설정 속도 도달", "nav2 주행"),
      ("넘겨받음", "POI 직선 진입"), ("서서 돌고 들어감", "진입 전 회전"), ("nav2 를 끊고", "_cut"),
      ("도착 위치 오차", "정지"), ("대기 자리 도착", "충전소 등돌리기+후진"), ("충전소 접촉", "정지"),
      ("0.5m 곧게 나간", "충전소 빠져나오기"), ("도착 후 방향 보정", "도착 보정")]
ev = []
for l in open(logp, errors='ignore'):
    if 'api_node' not in l: continue
    m = re.search(r'\]\s*\[(\d+\.\d+)\]', l)
    if not m: continue
    t = float(m.group(1))
    if t < t0 - 60 or t > t1: continue
    for k, ph in MK:
        if k in l: ev.append((t, ph, l.split(']: ', 1)[-1].strip()[:90])); break
ev.sort()
# 진입 끝 다음은 도착 회전(POI) 또는 빠져나오기 끝(출발 회전 대기)
lab = []; cur = '정지'; prev = None; j = 0
phase_at = []
for t, ph, txt in ev:
    if ph == '_end': ph = '도착 회전' if prev in ('POI 직선 진입', '_cut', '진입 전 회전') else '빠져나오기 끝(대기)'
    if ph == '_cut': ph = 'POI 직선 진입' if prev not in ('POI 직선 진입',) else prev
    phase_at.append((t, ph, txt)); prev = ph
def phase(t):
    p = '정지'
    for tt, ph, _ in phase_at:
        if tt <= t: p = ph
        else: break
    return p
# 고주파 성분: 자이로에서 0.8초 이동평균을 뺀 것 (deg/s)
N = len(R); W = 20
G = [math.degrees(r[3]) for r in R]; C = [math.degrees(r[2]) for r in R]
pre = [0.0]
for g in G: pre.append(pre[-1] + g)
def ma(i):
    a, b = max(0, i - W), min(N, i + W + 1); return (pre[b] - pre[a]) / (b - a)
import numpy as np
_X=np.fft.rfft(np.array(G)); _f=np.fft.rfftfreq(N,0.02); _X[(_f<0.3)|(_f>3.0)]=0; HP=list(np.fft.irfft(_X,N))
_Y=np.fft.rfft(np.array(C)); _Y[(_f<0.3)|(_f>3.0)]=0; HPC=list(np.fft.irfft(_Y,N))
agg = collections.defaultdict(lambda: {'n': 0, 'ss': 0.0, 'pk': 0.0, 'rev': 0, 'cmdss': 0.0, 'sec': 0.0})
prevsign = {}
for i in range(N):
    t, v = R[i][0], R[i][1]
    if abs(v) < 0.02: continue
    p = phase(t); a = agg[p]
    a['n'] += 1; a['ss'] += HP[i] ** 2; a['pk'] = max(a['pk'], abs(HP[i])); a['sec'] += 0.02
    a['cmdss'] += HPC[i] ** 2
print('구간별 (움직이는 중만, 0.3~3Hz 차체 흔들림, 진동 제외, deg/s)')
print('%-22s %6s %10s %10s %10s' % ('구간', '시간s', '실제RMS', '최대', '지령RMS'))
for p, a in sorted(agg.items(), key=lambda x: -math.sqrt(x[1]['ss'] / max(1, x[1]['n']))):
    print('%-22s %6.1f %10.2f %10.1f %10.2f' % (p, a['sec'], math.sqrt(a['ss'] / a['n']), a['pk'], math.sqrt(a['cmdss']/a['n'])))
# 가장 큰 흔들림 1초 창 10개
win = []
for i in range(0, N - 50, 10):
    seg = [HP[k] for k in range(i, i + 50) if abs(R[k][1]) >= 0.02]
    if len(seg) < 25: continue
    win.append((math.sqrt(sum(x * x for x in seg) / len(seg)), R[i][0]))
win.sort(reverse=True); used = []
print('\n가장 흔들린 1초 (흔들림RMS, 시각, 구간, 그때 속도)')
import time
for w, t in win:
    if any(abs(t - u) < 3 for u in used): continue
    used.append(t)
    i = min(range(N), key=lambda k: abs(R[k][0] - t))
    print('  %.1f deg/s  %s  %-20s v %.2f' % (w, time.strftime('%H:%M:%S', time.localtime(t)), phase(t), R[i][1]))
    if len(used) >= 12: break
