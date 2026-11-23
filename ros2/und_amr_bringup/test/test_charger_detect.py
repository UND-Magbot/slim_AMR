"""충전 스테이션 V 홈 인식 — 2026-11-11 실제 스캔 (스테이션이 로봇 바로 뒤 683mm)."""
import math
import os

import numpy as np

from und_amr_bringup.charger_detect import detect, detect_scan

DATA = os.path.join(os.path.dirname(__file__), "data", "charger_scan_20261111.npz")


def load():
    d = np.load(DATA)
    return d["r"], d["a"]


def test_real_scan_station_behind_robot():
    r, a = load()
    c = detect_scan(r, a[0], a[1] - a[0])
    assert c is not None
    # 앞면 중심 (-1.120, 0) 근처, 바깥 방향 = 로봇 쪽 (+x) = yaw 0
    assert abs(c["x"] + 1.120) < 0.01 and abs(c["y"]) < 0.015
    assert abs(math.degrees(c["yaw"])) < 2.0
    assert abs(c["angle"] - 118) < 4 and c["resid"] < 0.003


def test_real_scan_rotated_and_shifted():
    """로봇이 비스듬히 서 있어도 같은 스테이션 자세를 낸다."""
    r, a = load()
    ok = np.isfinite(r)
    x, y = r[ok] * np.cos(a[ok]), r[ok] * np.sin(a[ok])
    th = math.radians(25.0)
    X = math.cos(th) * x - math.sin(th) * y + 0.3
    Y = math.sin(th) * x + math.cos(th) * y - 0.2
    # 라이다 스캔 순서 그대로(0~360도, 뒤가 가운데) — 센서는 (0.3, -0.2)
    o = np.argsort(np.mod(a[ok], 2 * math.pi))
    c = detect(X[o], Y[o], sensor=(0.3, -0.2))
    fx, fy = math.cos(th) * -1.120 + 0.3, math.sin(th) * -1.120 - 0.2
    assert c is not None
    assert math.hypot(c["x"] - fx, c["y"] - fy) < 0.015
    assert abs(math.degrees(math.atan2(math.sin(c["yaw"] - th), math.cos(c["yaw"] - th)))) < 2.0


def test_room_corner_and_flat_wall_are_not_station():
    t = np.linspace(0, 1, 120)
    # 오목한 118도 꺾임이지만 변이 길다 (벽 모서리)
    L = np.c_[-1.5 + 0.8 * t * math.cos(math.radians(31)), 0.8 * t * math.sin(math.radians(31))]
    R = np.c_[-1.5 + 0.8 * t * math.cos(math.radians(31)), -0.8 * t * math.sin(math.radians(31))]
    P = np.r_[R[::-1], L[1:]]
    assert detect(P[:, 0], P[:, 1]) is None
    W = np.c_[np.full(200, -1.2), np.linspace(-0.6, 0.6, 200)]
    assert detect(W[:, 0], W[:, 1]) is None
    # 90도 방 모서리
    C = np.r_[np.c_[-1.2 + 0.4 * t, -0.4 * t][::-1], np.c_[-1.2 + 0.4 * t, 0.4 * t][1:]]
    assert detect(C[:, 0], C[:, 1]) is None


def test_no_scan_points():
    assert detect([], []) is None
