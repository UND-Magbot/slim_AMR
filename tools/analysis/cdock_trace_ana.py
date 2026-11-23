"""충전소 후진 기록 분석 (2026-11-16): 라이다로 본 좌우 - 추정 좌우 가 틈·방향에 따라 변하는지."""
import datetime
import glob
import math
import os
import re

import numpy as np

files = sorted(glob.glob(os.path.expanduser("~/.ros/log/python3_*.log")), key=os.path.getmtime)[-80:]
runs, cur = [], None
pat = re.compile(r"틈\s+(-?\d+) 좌우\s+([+-]\d+) 방향\s+([+-][\d.]+) 목표방향\s+([+-][\d.]+) \| v ([\d.]+) "
                 r"w명령 ([+-][\d.]+) w실제 ([+-][\d.]+) \| 라이다 좌우 ([+-]\d+|-)")
for fn in files:
    with open(fn, errors="ignore") as f:
        for line in f:
            if "충전소" not in line:
                continue
            m = re.search(r"\[(\d+\.\d+)\]", line)
            t = float(m.group(1)) if m else 0.0
            if "충전소 인식" in line:
                cur = {"t": t, "rows": [], "end": ""}
                runs.append(cur)
                continue
            if cur is None:
                continue
            if "도킹 완료" in line:
                cur["end"] = line.split("완료:", 1)[1].strip()[:60]
            m = pat.search(line)
            if m:
                g, la, dy, hd, v, wc, wa, rl = m.groups()
                cur["rows"].append((int(g), int(la), float(dy), None if rl == "-" else int(rl)))
runs.sort(key=lambda r: r["t"])
for r in runs:
    R = [x for x in r["rows"] if x[3] is not None and x[0] > 130]
    if len(R) < 8:
        continue
    g = np.array([x[0] for x in R]) / 1000.0
    la = np.array([x[1] for x in R])
    dy = np.array([x[2] for x in R])
    d = np.array([x[3] - x[1] for x in R])
    A = np.c_[g, np.radians(dy), np.ones(len(g))]
    co = np.linalg.lstsq(A, d, rcond=None)[0]
    sg = [1 if x > 3 else -1 if x < -3 else 0 for x in la]
    sg = [x for x in sg if x]
    flips = sum(1 for a, b in zip(sg, sg[1:]) if a != b)
    print("%s n=%2d 시작 좌우%+4d 방향%+5.1f | 라이다-추정 평균%+5.1f 표준편차%4.1f | 틈1m당%+6.1f 방향1rad당%+7.1f | 좌우 %+d~%+d 부호바뀜%d | %s"
          % (datetime.datetime.fromtimestamp(r["t"]).strftime("%m-%d %H:%M"), len(R),
             r["rows"][0][1], r["rows"][0][2], d.mean(), d.std(), co[0], co[1],
             la.min(), la.max(), flips, r["end"]))
