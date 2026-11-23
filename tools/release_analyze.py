#!/usr/bin/env python3
"""출발 저속(0.04 m/s) 구간이 끝나는 순간의 휘청임 — sway_probe 기록 + bringup.log.

    python3 tools/release_analyze.py logs/sway_1008_1529.csv [더 많은 파일...]

api_node 로그 '출발 Nmm 저속 끝 — ...' 시각을 기준으로 앞 0.5초 ~ 뒤 4초를 본다.
  예전 : '... nav2 속도 제한 풂'        (한 번에 풂)
  새것 : '... 속도를 서서히 올림'        (2026-10-08, DEPART_RAMP_M 동안 올림)

지표
  회전최대 : 자이로 |각속도| 최대 [도/s]
  흔들림   : 자이로에서 1초 이동평균을 뺀 값의 RMS [도/s] (꺾임이 아니라 떨림)
  방향폭   : 자이로 적분 방향의 최대-최소 [도]
  지령최대 : 회전 지령 |w| 최대 [도/s]
  속도4s   : 4초 뒤 속도 [m/s]
  바퀴방향폭: 오도메트리(EKF, 방향은 바퀴 위주) 방향의 최대-최소 [도]
  좌우폭   : 지도 위 위치가 처음 방향 직선에서 벗어난 최대-최소 [mm]

⚠ 이 로봇의 자이로는 카메라(D435i) 안에 있다. 0.4 m/s 에서 자이로 떨림 9.8 도/s,
  바퀴 1.6 도/s 였다(2026-10-08) — 빠를수록 카메라 거치대가 떤다. 차체가
  실제로 휘청했는지는 '바퀴방향폭' 과 '좌우폭' 으로 본다.
(2026-10-08, 사용자: '0.04 제한을 푸는 순간' 휘청)
"""
import csv
import math
import os
import re
import statistics as st
import sys

LOG = os.path.expanduser("~/projects/UND_AMR/logs/bringup.log")
D = math.degrees
PRE, POST = 0.5, 4.0


def load(path):
    R = list(csv.DictReader(open(path)))
    T = [float(r["t"]) for r in R]
    t0 = os.path.getmtime(path) - T[-1]
    out = []
    for r in R:
        mx, my, myaw = float(r["mo_x"]), float(r["mo_y"]), float(r["mo_yaw"])
        ox, oy = float(r["od_x"]), float(r["od_y"])
        c, s = math.cos(myaw), math.sin(myaw)
        out.append({"t": t0 + float(r["t"]), "v": float(r["v"]),
                    "cmd": D(float(r["cmd_w"])), "g": D(float(r["gyro"])),
                    "oy": float(r["od_yaw"]),
                    "x": mx + c * ox - s * oy, "y": my + s * ox + c * oy})
    return out


def releases(t_a, t_b):
    out = []
    for l in open(LOG, encoding="utf-8", errors="ignore"):
        m = re.search(r"\[(\d{10}\.\d+)\] \[api_node\]: 출발 (\d+)mm 저속 끝 — (.*)", l)
        if m and t_a + PRE < float(m.group(1)) < t_b - POST:
            out.append((float(m.group(1)), "천천히" if "서서히" in m.group(3) else "한번에"))
    return out


def one(R, tr):
    w = [r for r in R if tr - PRE <= r["t"] <= tr + POST]
    if len(w) < 20:
        return None
    g = [r["g"] for r in w]
    n = max(1, int(len(w) / (PRE + POST)))          # 1초 표본 수
    hf = []
    for i in range(len(g)):
        a = g[max(0, i - n // 2):i + n // 2 + 1]
        hf.append(g[i] - sum(a) / len(a))
    h, hs = 0.0, [0.0]
    for i in range(1, len(w)):
        h += w[i]["g"] * (w[i]["t"] - w[i - 1]["t"])
        hs.append(h)
    o0 = w[0]["oy"]
    oys = [D(math.atan2(math.sin(r["oy"] - o0), math.cos(r["oy"] - o0))) for r in w]
    # 처음 0.5초(저속) 방향으로 직선을 긋고 좌우를 잰다
    ux, uy = math.cos(w[0]["oy"]), math.sin(w[0]["oy"])
    a = [r for r in w if r["t"] <= tr]
    if len(a) >= 2:
        dx, dy = a[-1]["x"] - a[0]["x"], a[-1]["y"] - a[0]["y"]
        L = math.hypot(dx, dy)
        if L > 0.005:
            ux, uy = dx / L, dy / L
    lat = [((r["x"] - w[0]["x"]) * -uy + (r["y"] - w[0]["y"]) * ux) * 1000 for r in w]
    return {"oyaw": max(oys) - min(oys), "lat": max(lat) - min(lat),
            "gmax": max(abs(x) for x in g),
            "rms": math.sqrt(sum(x * x for x in hf) / len(hf)),
            "yaw": max(hs) - min(hs),
            "cmax": max(abs(r["cmd"]) for r in w),
            "v4": w[-1]["v"]}


def main():
    res = []
    for p in sys.argv[1:]:
        R = load(p)
        if not R:
            continue
        for tr, kind in releases(R[0]["t"], R[-1]["t"]):
            x = one(R, tr)
            if x:
                x["kind"] = kind
                res.append(x)
                print("  %-4s 바퀴방향폭 %4.1f도 | 좌우폭 %4.0fmm | 자이로최대 %4.1f | 지령최대 %4.1f | 속도4s %.2f"
                      % (kind, x["oyaw"], x["lat"], x["gmax"], x["cmax"], x["v4"]))
    print("== 평균")
    for kind in ("한번에", "천천히"):
        xs = [x for x in res if x["kind"] == kind]
        if xs:
            print("  %-4s %2d회 | 바퀴방향폭 %4.1f도 | 좌우폭 %4.0fmm | 자이로최대 %4.1f도/s | 지령최대 %4.1f도/s | 속도4s %.2f"
                  % (kind, len(xs), st.mean(x["oyaw"] for x in xs), st.mean(x["lat"] for x in xs),
                     st.mean(x["gmax"] for x in xs), st.mean(x["cmax"] for x in xs),
                     st.mean(x["v4"] for x in xs)))


if __name__ == "__main__":
    main()
