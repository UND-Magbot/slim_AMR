#!/usr/bin/env python3
"""라이다-지도 일치율 — 지금 SLAM 위치(또는 주어진 자세)에서 스캔이 지도 벽에 얼마나 붙나.

    python3 tools/map_fit_check.py                 지금 TF 자세만
    python3 tools/map_fit_check.py 4.45 -3.37 131  그 자세도 같이 비교 (도 단위)

로봇이 서 있을 때 쓴다(움직이면 스캔과 TF 시각이 어긋나 낮게 나온다).
'5cm 안 비율' 이 높을수록 잘 맞는다. 2026-10-08: 재기동 전 23~31%, 재기동 후 57~59%.
지도 칸이 5cm 라 '중앙 거리' 는 0/50/100mm 로만 나온다.
scipy 는 이 PC 에서 numpy 2 와 안 맞아 numpy 만 쓴다.
"""
import math
import sys
import time

import numpy as np
import rclpy
import tf2_ros
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, QoSProfile, ReliabilityPolicy,
                       qos_profile_sensor_data)
from sensor_msgs.msg import LaserScan


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def main():
    rclpy.init()
    n = Node("map_fit_check")
    st = {}
    n.create_subscription(OccupancyGrid, "/map", lambda m: st.__setitem__("map", m),
                          QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                     reliability=ReliabilityPolicy.RELIABLE))
    n.create_subscription(LaserScan, "/scan", lambda m: st.__setitem__("scan", m),
                          qos_profile_sensor_data)
    buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(buf, n)
    t0, lt, pose = time.time(), None, None
    while time.time() - t0 < 20 and (lt is None or pose is None):
        rclpy.spin_once(n, timeout_sec=0.2)
        if "map" in st and "scan" in st:
            try:
                lt = buf.lookup_transform("base_footprint", st["scan"].header.frame_id,
                                          rclpy.time.Time())
                pose = buf.lookup_transform("map", "base_footprint", rclpy.time.Time())
            except Exception:                                # noqa: BLE001
                pass
    if lt is None or pose is None:
        sys.exit("지도/스캔/TF 를 20초 안에 못 받았다 (map=%s scan=%s)"
                 % ("map" in st, "scan" in st))
    sc, m = st["scan"], st["map"]
    lx, ly, lyaw = lt.transform.translation.x, lt.transform.translation.y, yaw_of(lt.transform.rotation)
    r = np.array(sc.ranges)
    a = sc.angle_min + np.arange(len(r)) * sc.angle_increment
    ok = np.isfinite(r) & (r > 0.1) & (r < 8)
    r, a = r[ok], a[ok]
    px, py = lx + r * np.cos(a + lyaw), ly + r * np.sin(a + lyaw)
    g = np.array(m.data).reshape(m.info.height, m.info.width)
    res = m.info.resolution
    ox, oy = m.info.origin.position.x, m.info.origin.position.y
    occ = g >= 65
    dist = np.full(g.shape, 99.0)
    cur = occ.copy()
    dist[cur] = 0
    for k in range(1, 11):
        nb = cur.copy()
        nb[1:, :] |= cur[:-1, :]
        nb[:-1, :] |= cur[1:, :]
        nb[:, 1:] |= cur[:, :-1]
        nb[:, :-1] |= cur[:, 1:]
        dist[nb & ~cur] = k
        cur = nb
    dist *= res

    def score(x, y, yaw):
        c, s = math.cos(yaw), math.sin(yaw)
        wx, wy = x + c * px - s * py, y + s * px + c * py
        i, j = ((wy - oy) / res).astype(int), ((wx - ox) / res).astype(int)
        v = (i >= 0) & (i < g.shape[0]) & (j >= 0) & (j < g.shape[1])
        d = dist[i[v], j[v]]
        return np.median(d) * 1000, np.mean(d < 0.05) * 100

    t = pose.transform
    cands = {"지금 TF": (t.translation.x, t.translation.y, math.degrees(yaw_of(t.rotation)))}
    if len(sys.argv) >= 4:
        cands["주어진 자세"] = tuple(float(v) for v in sys.argv[1:4])
    for name, (x, y, yd) in cands.items():
        md, f = score(x, y, math.radians(yd))
        print("%s (%.3f, %.3f, %.1f도): 점-벽 중앙 %.0fmm, 5cm 안 %.0f%%" % (name, x, y, yd, md, f))


if __name__ == "__main__":
    main()
