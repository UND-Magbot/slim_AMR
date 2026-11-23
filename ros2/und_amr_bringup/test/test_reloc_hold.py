"""수동 위치 고정 (2026-11-13): SLAM 보정을 멈추고 EKF 자세를 넣어 손으로 놓은 자리 그대로."""
import math
import types


def make(ApiNode, mo):
    n = ApiNode.__new__(ApiNode)
    n.slam_mode = "localization"
    n.loc_hold = None
    n.paused = False
    n.calls = []

    def set_paused(want):
        n.calls.append(want)
        n.paused = want
        return True, None

    n._slam_set_paused = set_paused
    n._slam_paused = lambda: n.paused
    n.tf_pose = lambda t="map", s="base_footprint": (
        {"x": mo[0], "y": mo[1], "yaw": mo[2]} if (t, s) == ("map", "odom") else None)
    n.sent = []
    n.pub_ekf_pose = types.SimpleNamespace(publish=lambda m: n.sent.append(m))
    n.inits = []
    n._publish_initialpose = lambda x, y, yaw, **k: n.inits.append((x, y, yaw))
    n.slam = {}
    n.get_clock = lambda: types.SimpleNamespace(
        now=lambda: types.SimpleNamespace(to_msg=lambda: __import__("builtin_interfaces.msg", fromlist=["Time"]).Time()))
    n.get_logger = lambda: types.SimpleNamespace(info=lambda *a: None, warn=lambda *a: None)
    return n


def compose(mo, m):
    p = m.pose.pose
    oyaw = 2 * math.atan2(p.orientation.z, p.orientation.w)
    c, s = math.cos(mo[2]), math.sin(mo[2])
    return (mo[0] + c * p.position.x - s * p.position.y,
            mo[1] + s * p.position.x + c * p.position.y, mo[2] + oyaw)


def test_hold_puts_robot_exactly_where_placed(ApiNode):
    mo = (1.2, -0.7, math.radians(35))             # 지금 SLAM 의 map -> odom
    n = make(ApiNode, mo)
    ok, msg = n.relocalize_manual(-2.72, 0.45, math.radians(-91.4), hold=True)
    assert ok and n.calls == [True] and n.loc_hold is not None
    assert n.inits == []                            # SLAM 에 initialpose 는 안 보낸다
    m = n.sent[-1]
    assert m.header.frame_id == "odom"
    x, y, yaw = compose(mo, m)
    assert abs(x + 2.72) < 1e-9 and abs(y - 0.45) < 1e-9
    assert abs(math.atan2(math.sin(yaw - math.radians(-91.4)), math.cos(yaw - math.radians(-91.4)))) < 1e-9


def test_non_hold_releases_and_uses_lidar(ApiNode):
    n = make(ApiNode, (0, 0, 0))
    n.relocalize_manual(1.0, 2.0, 0.3, hold=True)
    ok, _ = n.relocalize_manual(1.5, 2.5, 0.1, hold=False)
    assert ok and n.paused is False and n.loc_hold is None
    assert n.inits == [(1.5, 2.5, 0.1)]


def test_release(ApiNode):
    n = make(ApiNode, (0, 0, 0))
    n.relocalize_manual(1.0, 2.0, 0.3, hold=True)
    ok, msg = n.relocalize_release()
    assert ok and n.paused is False and n.loc_hold is None
