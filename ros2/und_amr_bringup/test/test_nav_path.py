"""화면용 경로: 지나간 부분은 잘리고, 이동이 끝나면 비어야 한다 (2026-11-13)."""


def make(ApiNode, state, robot):
    n = ApiNode.__new__(ApiNode)
    n.nav_state = state
    n.nav_path = [[0.0, 0.0], [2.0, 0.0], [2.0, 2.0]]
    n.tf_pose = lambda *a, **k: {"x": robot[0], "y": robot[1], "yaw": 0.0}
    return n


def test_passed_part_is_trimmed(ApiNode):
    p = make(ApiNode, "active", (1.2, 0.05)).nav_path_json()["points"]
    assert p[0] == [1.2, 0.05] and p[1:] == [[2.0, 0.0], [2.0, 2.0]]
    p = make(ApiNode, "active", (2.02, 1.0)).nav_path_json()["points"]
    assert p == [[2.02, 1.0], [2.0, 2.0]]


def test_cleared_when_not_moving(ApiNode):
    for st in ("done", "canceled", "failed", "idle"):
        assert make(ApiNode, st, (1.0, 0.0)).nav_path_json()["points"] == []
