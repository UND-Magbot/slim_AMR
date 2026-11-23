"""충전소 POI (2026-11-11): 실제 스캔으로 등록 -> 도킹 자세, 이동은 대기 자리, 왕복 제외."""
import math
import os
import threading
import types

import numpy as np

DATA = os.path.join(os.path.dirname(__file__), "data", "charger_scan_20261111.npz")


def make(ApiNode, robot=(2.0, 1.0, math.radians(30))):
    d = np.load(DATA)

    class N(ApiNode):
        k = 0

        @property
        def scan(self):                      # 읽을 때마다 새 장 (시각만 다름)
            N.k += 1
            return types.SimpleNamespace(
                header=types.SimpleNamespace(
                    stamp=types.SimpleNamespace(sec=N.k, nanosec=0), frame_id="laser"),
                ranges=d["r"], angle_min=float(d["a"][0]),
                angle_increment=float(d["a"][1] - d["a"][0]))

    n = N.__new__(N)
    n.lock = threading.Lock()
    n.tf_pose = lambda *a, **k: {"x": robot[0], "y": robot[1], "yaw": robot[2]}
    n._poi_laser = lambda f: (0.0, 0.0, 0.0)
    return n


def test_measure_gives_dock_pose_behind_robot(ApiNode):
    rx, ry, ryaw = 2.0, 1.0, math.radians(30)
    n = make(ApiNode, (rx, ry, ryaw))
    st = n._charger_measure()
    assert st is not None and st["n"] >= 4 and st["spread"] < 0.005
    # 스테이션 앞면은 로봇 뒤 1.119m, 바깥 방향 = 로봇이 보는 쪽
    fx = rx + math.cos(ryaw) * -1.119 - math.sin(ryaw) * -0.005
    fy = ry + math.sin(ryaw) * -1.119 + math.cos(ryaw) * -0.005
    assert math.hypot(st["face"][0] - fx, st["face"][1] - fy) < 0.01
    dx, dy, dyaw = st["dock"]
    # 도킹 자세: 앞면에서 바깥으로 0.437m, 방향 = 로봇과 같은 쪽
    assert abs(math.hypot(dx - st["face"][0], dy - st["face"][1]) - 0.43675) < 1e-6
    assert abs(math.degrees(math.atan2(math.sin(dyaw - ryaw), math.cos(dyaw - ryaw)))) < 2.0


def test_charger_poi_goto_goes_to_approach_not_into_station(ApiNode):
    n = ApiNode.__new__(ApiNode)
    n.lock = threading.Lock()
    p = {"name": "충전소1", "kind": "charger", "x": 1.0, "y": 2.0, "yaw": math.pi / 2,
         "standoff": 0.8}
    n.pois = [p]
    sent = []
    n.nav_goto = lambda x, y, yaw, **k: sent.append((x, y, yaw, k)) or (True, "ok")
    assert n.poi_goto("충전소1")[0]
    x, y, yaw, k = sent[0]
    assert abs(x - 1.0) < 1e-9 and abs(y - 2.8) < 1e-9 and abs(yaw - math.pi / 2) < 1e-9
    assert k.get("poi") is None


def test_charger_kind_kept_and_route_refuses_cart(ApiNode):
    """2026-11-18 충전소는 왕복 끝점이 된다(도킹으로 도착). 대차 POI 만 거절."""
    n = ApiNode.__new__(ApiNode)
    out = n._clean_pois([{"name": "충전소1", "kind": "charger", "x": 0, "y": 0, "yaw": 0,
                          "standoff": 0.8}])
    assert out[0]["kind"] == "charger" and out[0]["standoff"] == 0.8
    n.estop = False
    n.control_mode = "auto"
    n.job = {"state": "idle"}
    n.lock = threading.Lock()
    n.pois = out + [{"name": "A", "kind": "point", "x": 1, "y": 0, "yaw": 0},
                    {"name": "렉", "kind": "cart", "x": 2, "y": 0, "yaw": 0}]
    n._lift_moving = lambda: False
    ok, msg = n.route_start("A", "렉", 1)
    assert not ok and "대차" in msg
