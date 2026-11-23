#!/usr/bin/env python3
"""kick_recorder CSV 에서 곧게 달리는 구간의 조향 치우침을 본다.

    python3 tools/cruise_bias.py logs/kick_0930_1600.csv

속도가 0.15 m/s 넘게 2초 이상 이어지는 구간마다 평균 지령 회전, 평균 실제
회전(자이로), 평균 엔코더 회전을 낸다. 지령 평균이 한쪽(예: 음수=오른쪽)으로
계속 치우치고 실제 회전은 0 근처면, 무언가가 반대쪽으로 계속 밀고 있고
nav2 가 그것을 버티고 있다는 뜻이다. (2026-09-30)
"""
import csv
import math
import statistics as st
import sys

rows = [r for r in csv.DictReader(open(sys.argv[1]))]
T = [float(r["t"]) for r in rows]
V = [float(r["odom_v"]) for r in rows]
G = [float(r["gyro_z"]) for r in rows]
C = [float(r["cmd_w"]) for r in rows]
O = [float(r["odom_w"]) for r in rows]
W, p = [], 0.0
for v in V:
    if abs(v) > 0.6:
        v = p
    W.append(v)
    p = v
Vm = [sorted(W[max(0, i - 4):i + 5])[len(W[max(0, i - 4):i + 5]) // 2] for i in range(len(W))]
print("구간 시작 | 길이 | 평균 속도 | 평균 지령 | 평균 실제(자이로) | 평균 엔코더  (도/s, +왼쪽)")
allc, allg = [], []
i = 0
while i < len(T):
    if Vm[i] > 0.15:
        j = i
        while j < len(T) and Vm[j] > 0.15:
            j += 1
        # 가속·감속 끝부분 1초씩 빼고
        a, b = i + 20, j - 20
        if b - a > 40:
            c = st.mean(C[a:b]); g = st.mean(G[a:b]); o = st.mean(O[a:b])
            allc.append(c); allg.append(g)
            print("%6.1f | %.1fs | %.2f | %+5.2f | %+5.2f | %+5.2f" % (
                T[i], T[j - 1] - T[i], st.mean(Vm[a:b]), math.degrees(c),
                math.degrees(g), math.degrees(o)))
        i = j
    else:
        i += 1
if allc:
    print("전체 평균: 지령 %+.2f 도/s, 실제 %+.2f 도/s" % (
        math.degrees(st.mean(allc)), math.degrees(st.mean(allg))))
