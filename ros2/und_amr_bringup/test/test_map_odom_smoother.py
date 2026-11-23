"""map_odom_smoother 계산 (2026-11-16)."""
import math

from und_amr_bringup.map_odom_smoother import blend, compose, far, inverse, target_from


def _close(a, b, tol=1e-9):
    return all(abs(x - y) < tol for x, y in zip(a, b))


def test_inverse_compose():
    a = (1.2, -0.7, 0.9)
    assert _close(compose(a, inverse(a)), (0, 0, 0))


def test_target_matches_slam():
    mo = (3.0, -1.0, 0.3)
    ob = (10.0, 4.0, -1.2)
    mb = compose(mo, ob)
    assert _close(target_from(mb, ob), mo)


def test_blend_ends():
    cur, tgt, ob = (0, 0, 0), (0.01, -0.02, 0.005), (40.0, 30.0, 1.0)
    assert _close(blend(cur, tgt, ob, 0.0), cur)
    assert _close(blend(cur, tgt, ob, 1.0), tgt)


def test_blend_robot_moves_straight_far_from_odom_origin():
    # odom 원점에서 50m, 방향 보정 0.6° → 로봇 자리는 두 끝 사이 직선 위를 지나야 한다
    cur = (0.0, 0.0, 0.0)
    ob = (50.0, 0.0, 0.0)
    tgt = target_from(compose((0.0, 0.0, math.radians(0.6)), ob), ob)
    tgt = (tgt[0], tgt[1] + 0.01, tgt[2])
    p0, p1 = compose(cur, ob), compose(tgt, ob)
    pm = compose(blend(cur, tgt, ob, 0.5), ob)
    assert abs(pm[0] - (p0[0] + p1[0]) / 2) < 1e-6
    assert abs(pm[1] - (p0[1] + p1[1]) / 2) < 1e-6


def test_step_spread_over_time():
    # 0.3m/s 로 곧게 가는 중 옆으로 10mm 계단 보정 → 한 틱(20ms) 에 0.3mm 넘게 안 튄다
    cur = (0.0, 0.0, 0.0)
    tgt = (0.0, 0.010, 0.0)
    dt, tau = 0.02, 0.8
    a = 1 - math.exp(-dt / tau)
    x, prev, jumps = 0.0, None, []
    for _ in range(200):
        x += 0.3 * dt
        cur = blend(cur, tgt, (x, 0.0, 0.0), a)
        y = compose(cur, (x, 0.0, 0.0))[1]
        if prev is not None:
            jumps.append(y - prev)
        prev = y
    assert max(jumps) < 0.0003
    assert abs(prev - 0.010) < 0.0005      # 4초 뒤 거의 다 따라갔다


def test_far():
    assert far((0, 0, 0), (0.3, 0, 0))
    assert far((0, 0, 0), (0, 0, math.radians(9)))
    assert not far((0, 0, 0), (0.05, 0.05, math.radians(2)))


def test_pose_before_enable_is_kept_and_used(monkeypatch):
    """2026-11-17 재부팅: 켜지기 전에 온 첫 /pose 를 버려 지도 좌표가 안 생겼다."""
    import types
    from und_amr_bringup.map_odom_smoother import MapOdomSmoother
    n = MapOdomSmoother.__new__(MapOdomSmoother)
    n.enabled, n.cur, n.tgt, n._pending, n._param_fail = False, None, None, None, 0
    n.get_logger = lambda: types.SimpleNamespace(info=lambda *a: None)
    n._odom_base = lambda stamp=None: (0.0, 0.0, 0.0)
    m = types.SimpleNamespace(
        header=types.SimpleNamespace(stamp=types.SimpleNamespace(sec=1, nanosec=0)),
        pose=types.SimpleNamespace(pose=types.SimpleNamespace(
            position=types.SimpleNamespace(x=0.8, y=-0.1),
            orientation=types.SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0))))
    monkeypatch.setattr("und_amr_bringup.map_odom_smoother.Time",
                        types.SimpleNamespace(from_msg=lambda s: s))
    n._on_pose(m)
    assert n.cur is None and n._pending is m
    n._param_result(0.0)
    assert n.enabled and n.cur is not None and abs(n.cur[0] - 0.8) < 1e-9
