#!/usr/bin/env python3
"""도착 정밀도 측정 — 보낸 목표와 실제로 선 자리를 비교한다.

POI 로 보낼 때 얼마나 정확히 서는지 숫자로 남긴다. 설정을 만지기 전에
지금 값을 알아야 나아졌는지 판단할 수 있다.

로봇을 앞뒤로 오가게 하며 여러 번 재고, 매번 오차를 기록한다.
    - 거리 오차   목표 지점에서 얼마나 떨어져 섰나
    - 방향 오차   목표 방향에서 얼마나 틀어져 섰나

    python3 tools/goal_accuracy.py --dist 2.0 --reps 4

앞뒤로 (dist + 1m) 씩 비워 두세요. 로봇이 실제로 왕복합니다.
"""
import argparse
import json
import math
import time
import urllib.request

API = "http://127.0.0.1:8090"


def api(path, body=None):
    req = urllib.request.Request(
        API + path,
        data=json.dumps(body).encode() if body is not None else None,
        headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=10) as r:
        return json.load(r)


def pose():
    p = api("/chassis/pose").get("map")
    return (p["x"], p["y"], p["yaw"]) if p else None


def wrap(d):
    while d > math.pi:
        d -= 2 * math.pi
    while d < -math.pi:
        d += 2 * math.pi
    return d


def go(x, y, yaw, limit=120.0):
    """목표를 보내고 끝날 때까지 기다린다. (성공여부, 걸린시간)"""
    r = api("/nav/goto", {"x": x, "y": y, "yaw": yaw})
    if not r.get("ok"):
        return False, r.get("message", ""), 0.0
    t0 = time.time()
    while time.time() - t0 < limit:
        time.sleep(1.0)
        st = api("/nav/status")
        if st["state"] == "done":
            return True, "", time.time() - t0
        if st["state"] in ("failed", "canceled"):
            return False, st.get("message", st["state"]), time.time() - t0
    api("/nav/cancel", {})
    return False, "시간 초과", limit


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dist", type=float, default=2.0, help="왕복 거리[m]")
    ap.add_argument("--reps", type=int, default=4, help="왕복 횟수")
    a = ap.parse_args()

    st = api("/nav/status")
    if not st.get("available"):
        print("!! nav2 가 떠 있지 않습니다")
        return
    p0 = pose()
    if p0 is None:
        print("!! 로봇 위치가 없습니다 (위치추정 확인)")
        return

    # 현재 자세를 기준으로 앞뒤 두 지점을 잡는다.
    x0, y0, th0 = p0
    ax = x0 + a.dist / 2 * math.cos(th0)
    ay = y0 + a.dist / 2 * math.sin(th0)
    bx = x0 - a.dist / 2 * math.cos(th0)
    by = y0 - a.dist / 2 * math.sin(th0)
    pts = [(ax, ay, th0), (bx, by, th0)]

    print("=" * 62)
    print("  기준 자세 (%.3f, %.3f) %.1f도" % (x0, y0, math.degrees(th0)))
    print("  %.1f m 씩 %d 회 왕복하며 도착 오차를 잰다" % (a.dist, a.reps))
    print("=" * 62)

    errs = []
    for i in range(a.reps):
        tx, ty, tyaw = pts[i % 2]
        ok, msg, sec = go(tx, ty, tyaw)
        time.sleep(2.0)          # 관성으로 더 가는 몫이 끝난 뒤에 잰다
        cur = pose()
        if not ok or cur is None:
            print("  [%d] 실패 — %s" % (i + 1, msg))
            continue
        de = math.hypot(cur[0] - tx, cur[1] - ty)
        dy = math.degrees(wrap(cur[2] - tyaw))
        errs.append((de, dy))
        print("  [%d] 목표(%.3f, %.3f) → 도착(%.3f, %.3f)   "
              "거리오차 %5.1f mm   방향오차 %+5.1f 도   %4.1f초"
              % (i + 1, tx, ty, cur[0], cur[1], de * 1000, dy, sec))

    print()
    print("=" * 62)
    if not errs:
        print("  성공한 회차가 없습니다")
        return
    d = [e[0] for e in errs]
    y = [abs(e[1]) for e in errs]
    try:
        import yaml
        g = yaml.safe_load(open("/home/und_amr1/ros2_ws/src/und_amr_bringup/"
                                "config/nav2.yaml"))["controller_server"][
            "ros__parameters"]["general_goal_checker"]
        tol_d, tol_y = g["xy_goal_tolerance"] * 1000, math.degrees(g["yaw_goal_tolerance"])
    except Exception:                                    # noqa: BLE001
        tol_d, tol_y = float("nan"), float("nan")
    print("  거리 오차  평균 %.0f mm   최대 %.0f mm   (판정 기준 %.0f mm)"
          % (sum(d) / len(d) * 1000, max(d) * 1000, tol_d))
    print("  방향 오차  평균 %.1f 도   최대 %.1f 도   (판정 기준 %.1f 도)"
          % (sum(y) / len(y), max(y), tol_y))
    print()
    print("  판정 기준보다 한참 작으면 기준을 조여서 더 정확히 세울 수 있다.")
    print("  기준에 붙어 있으면 컨트롤러가 그 이상 못 맞추는 것이므로,")
    print("  기준만 조이면 목표를 영영 못 맞추고 맴돈다.")
    print("=" * 62)


if __name__ == "__main__":
    main()
