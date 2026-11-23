#!/usr/bin/env python3
"""출발마다 흔들림 분석 — sway_probe 기록 + bringup.log 의 api_node 줄을 맞춘다.

    python3 tools/depart_analyze.py logs/sway_1002_1535.csv [더 많은 파일...]

출발 종류 (api_node 로그 '이동 목표 ... — xxx')
  정렬   : '먼저 제자리 정렬' (nav2 방식, 목표가 옆·뒤 -> 돌고 nav2 저속)
  정면   : 'nav2 저속 출발'  (nav2 방식, 목표가 앞)
  creep  : '먼저 천천히 출발' (예전 방식, api_node 가 기어가다 nav2 로)
  nav2만 : 접미사 없음 (출발 동작 없이 nav2)

지표 (전진을 시작한 순간부터; 출발점 -> 목표점 직선 기준)
  방향4s : 처음 4초 자이로 적분 방향의 최대-최소 [도]
  방향1.5: 처음 1.5m 동안 같은 값 (방식마다 4초 안 속도가 달라 거리로도 본다)
  좌우1.5: 처음 1.5m 동안 직선에서 좌우로 벗어난 최대 [mm]
  좌우전체: 목표 0.4m 앞까지 최대 [mm]
  반전   : 조향 지령(0.3초 평균)이 ±0.5도/s 를 넘나들며 뒤집힌 횟수 (달리는 동안)
(2026-10-02, '정렬 유지 + api_node 직접 전진 제거' 비교용)
"""
import csv
import math
import os
import re
import statistics as st
import sys

LOG = os.path.expanduser("~/projects/UND_AMR/logs/bringup.log")
D = math.degrees


def load(path):
    R = list(csv.DictReader(open(path)))
    T = [float(r["t"]) for r in R]
    t0 = os.path.getmtime(path) - T[-1]
    rows = []
    for r in R:
        mx, my, myaw = float(r["mo_x"]), float(r["mo_y"]), float(r["mo_yaw"])
        ox, oy, oyaw = float(r["od_x"]), float(r["od_y"]), float(r["od_yaw"])
        c, s = math.cos(myaw), math.sin(myaw)
        rows.append({"t": t0 + float(r["t"]), "x": mx + c * ox - s * oy,
                     "y": my + s * ox + c * oy, "v": float(r["v"]),
                     "cmd": D(float(r["cmd_w"])), "g": D(float(r["gyro"]))})
    return rows


def goals(t_a, t_b):
    out, err = [], None
    for l in open(LOG, encoding="utf-8", errors="ignore"):
        m = re.search(r"\[(\d{10}\.\d+)\] \[api_node\]: (.*)", l)
        if not m:
            continue
        t, msg = float(m.group(1)), m.group(2)
        if not (t_a - 30 < t < t_b):
            continue
        e = re.match(r"출발 방향 오차 ([-+][\d.]+)도", msg)
        if e:
            err = float(e.group(1))
            continue
        g = re.match(r"이동 목표 \(([-\d.]+), ([-\d.]+)\)[^—]*(— (.*))?", msg)
        if g:
            tail = g.group(4) or ""
            kind = ("정렬" if "제자리 정렬" in tail else "정면" if "nav2 저속" in tail
                    else "creep" if "천천히" in tail else "nav2만")
            out.append((t, float(g.group(1)), float(g.group(2)), kind, err))
            err = None
    return out


def analyze(path):
    R = load(path)
    if not R:
        return []
    G = goals(R[0]["t"], R[-1]["t"])
    res = []
    for k, (tg, gx, gy, kind, err0) in enumerate(G):
        # 기록 앞뒤로 잘린 출발은 뺀다 (시작 전에 보낸 목표, 끝나기 전 다음 목표가 없는 것)
        if tg < R[0]["t"] + 0.5 or k + 1 >= len(G) or G[k + 1][0] > R[-1]["t"]:
            continue
        t_next = G[k + 1][0]
        idx = [i for i, r in enumerate(R) if tg <= r["t"] < t_next]
        if not idx:
            continue
        i0 = next((i for i in idx if R[i]["v"] > 0.02), None)
        if i0 is None:
            continue
        x0, y0 = R[i0]["x"], R[i0]["y"]
        L = math.hypot(gx - x0, gy - y0)
        if L < 1.0:
            continue
        ux, uy = (gx - x0) / L, (gy - y0) / L
        lat15, latall, yaw, h = [], [], [], 0.0
        yaw15, h15 = [], 0.0
        for i in range(i0, idx[-1] + 1):
            r = R[i]
            along = (r["x"] - x0) * ux + (r["y"] - y0) * uy
            lat = ((r["x"] - x0) * (-uy) + (r["y"] - y0) * ux) * 1000
            if along < 1.5:
                lat15.append(lat)
                if i > i0:
                    h15 += r["g"] * (r["t"] - R[i - 1]["t"])
                    yaw15.append(h15)
            if along < L - 0.4:
                latall.append(lat)
            if r["t"] - R[i0]["t"] <= 4.0 and i > i0:
                h += r["g"] * (r["t"] - R[i - 1]["t"])
                yaw.append(h)
        # 조향 반전
        seg = [i for i in range(i0, idx[-1] + 1) if R[i]["v"] > 0.1]
        rev, sgn = 0, 0
        for j, i in enumerate(seg):
            w = st.mean(R[q]["cmd"] for q in seg[max(0, j - 7):j + 8])
            s_ = 1 if w > 0.5 else -1 if w < -0.5 else 0
            if s_ and sgn and s_ != sgn:
                rev += 1
            if s_:
                sgn = s_
        if not lat15 or not yaw:
            continue
        res.append({"kind": kind, "err0": err0, "yaw4": max(yaw) - min(yaw),
                    "yaw15": (max(yaw15) - min(yaw15)) if yaw15 else float("nan"),
                    "lat15": max(abs(v) for v in lat15),
                    "latall": max(abs(v) for v in latall) if latall else float("nan"),
                    "rev": rev, "t": tg})
    return res


def main():
    allr = []
    for p in sys.argv[1:]:
        r = analyze(p)
        print("== %s: 출발 %d회" % (os.path.basename(p), len(r)))
        for x in r:
            print("  %-5s 처음 오차 %6s | 방향4s %4.1f도 | 방향1.5 %4.1f도 | 좌우1.5 %4.0fmm | 좌우전체 %4.0fmm | 반전 %d"
                  % (x["kind"], "%+.1f" % x["err0"] if x["err0"] is not None else "-",
                     x["yaw4"], x["yaw15"], x["lat15"], x["latall"], x["rev"]))
        allr += r
    print("== 종류별 평균")
    for kind in ("정면", "정렬", "creep", "nav2만"):
        xs = [x for x in allr if x["kind"] == kind]
        if xs:
            print("  %-5s %2d회 | 방향4s %.1f도 | 방향1.5 %.1f도 | 좌우1.5 %.0fmm | 좌우전체 %.0fmm | 반전 %.1f"
                  % (kind, len(xs), st.mean(x["yaw4"] for x in xs), st.mean(x["yaw15"] for x in xs),
                     st.mean(x["lat15"] for x in xs),
                     st.mean(x["latall"] for x in xs), st.mean(x["rev"] for x in xs)))


if __name__ == "__main__":
    main()
