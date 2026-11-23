"""SLAM 감시·저장 자세 검사 (2026-09-30 검토).

- 지도 밖·무한대 자세는 저장하지도 읽지도 않는다
- SLAM 이 혼자 죽으면 세우고, 위치추정이었으면 다시 띄운다(10분 3회까지)
- 띄우기·내리기 중(busy)에는 감시가 손대지 않는다
"""
import math
import threading

import pytest


class _P:
    def __init__(self, code):
        self.returncode = code

    def poll(self):
        return self.returncode


def _log():
    return type("L", (), {k: (lambda *a, **kw: None)
                          for k in ("info", "warn", "error")})()


@pytest.fixture
def n(ApiNode, tmp_path):
    n = ApiNode.__new__(ApiNode)
    n.get_logger = _log
    n.map_meta = {"resolution": 0.05, "width": 200, "height": 100,
                  "origin": {"x": -5.0, "y": -2.0, "yaw": 0.0}}
    n.map_dir = str(tmp_path)
    n._pose_file = lambda: str(tmp_path / ".last_pose")
    n._slam_busy = 0
    n._slam_busy_lock = threading.Lock()
    n._slam_restarts = []
    n.slam_proc = None
    n.slam_mode = None
    n.active_map = "m1"
    n.nav_state, n.nav_msg = "idle", ""
    n.stops = 0
    n.stop_now = lambda: setattr(n, "stops", n.stops + 1)
    n.spawned = []

    def respawn(name):
        n.spawned.append(name)
        n._slam_busy_add(-1)
    n._slam_respawn = respawn
    return n


def test_pose_sane_bounds(n):
    # 지도 x -5..5, y -2..3 (+1m 여유)
    assert n._pose_sane(0.0, 0.0, 0.0)
    assert n._pose_sane(-5.9, 3.9, 1.0)
    assert not n._pose_sane(-6.1, 0.0, 0.0)
    assert not n._pose_sane(-1154679.0, -1730631.0, 0.0)
    assert not n._pose_sane(float("nan"), 0.0, 0.0)
    assert not n._pose_sane(0.0, 0.0, math.inf)


def test_load_pose_rejects_outside(n, tmp_path):
    (tmp_path / ".last_pose").write_text("m1 -369079.0 -613501.0 0.1")
    assert n._load_pose("m1") is None
    (tmp_path / ".last_pose").write_text("m1 1.0 0.5 0.1")
    assert n._load_pose("m1") == (1.0, 0.5, 0.1)
    (tmp_path / ".last_pose").write_text("m1 1.0 0.5 nan")
    assert n._load_pose("m1") is None


def test_watch_ignores_alive_and_busy(n):
    n.slam_proc, n.slam_mode = _P(None), "localization"
    n._slam_watch()
    assert n.stops == 0 and n.slam_proc is not None
    n.slam_proc = _P(-11)
    n._slam_busy = 1                  # 내리는 중
    n._slam_watch()
    assert n.stops == 0 and n.slam_proc is not None


def test_watch_death_stops_and_respawns(n):
    n.slam_proc, n.slam_mode = _P(-11), "localization"
    n._slam_watch()
    import time
    time.sleep(0.05)
    assert n.stops == 1
    assert n.slam_proc is None and n.slam_mode is None
    assert n.nav_state == "failed" and "SLAM" in n.nav_msg
    assert n.spawned == ["m1"]
    assert n._slam_busy == 0


def test_watch_gives_up_after_three(n):
    import time
    for _ in range(4):
        n.slam_proc, n.slam_mode = _P(-11), "localization"
        n._slam_watch()
        time.sleep(0.02)
    assert n.stops == 4
    assert n.spawned == ["m1"] * 3


def test_watch_mapping_death_no_respawn(n):
    n.slam_proc, n.slam_mode = _P(1), "mapping"
    n._slam_watch()
    assert n.stops == 1 and n.spawned == []


def test_watch_failed_spawn_is_quiet(n):
    n.slam_proc, n.slam_mode = _P(1), None      # 띄우다 실패한 것
    n._slam_watch()
    assert n.stops == 0 and n.slam_proc is None


def test_map_kick_when_localizing_without_map(ApiNode):
    """2026-11-17 위치추정인데 map 좌표가 5초 넘게 없으면 마지막 자리로 /initialpose."""
    import types
    n = ApiNode.__new__(ApiNode)
    n.slam_mode, n._slam_busy, n.active_map = "localization", 0, "m"
    n.tf_pose = lambda *a, **k: None
    n._load_pose = lambda name: (0.77, -0.09, -0.25)
    sent = []
    n._publish_initialpose = lambda x, y, yaw: sent.append((x, y, yaw))
    n.get_logger = lambda: types.SimpleNamespace(warn=lambda *a: None, info=lambda *a: None)
    import und_amr_bringup.api_node as am
    t = [100.0]
    real = am.time.monotonic
    am.time.monotonic = lambda: t[0]
    try:
        n._map_kick_check()
        t[0] += 3.0
        n._map_kick_check()
        assert sent == []
        t[0] += 3.0
        n._map_kick_check()
        assert sent == [(0.77, -0.09, -0.25)]
        t[0] += 5.0
        n._map_kick_check()
        assert len(sent) == 1                       # 15초에 한 번만
        n.tf_pose = lambda *a, **k: {"x": 0, "y": 0, "yaw": 0}
        n._map_kick_check()
        assert n._map_missing_t is None
    finally:
        am.time.monotonic = real
