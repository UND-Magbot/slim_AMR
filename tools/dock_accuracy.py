#!/usr/bin/env python3
"""렉 진입 반복 정밀도 — 같은 렉에 여러 번 들어가 보고 얼마나 흩어지는지 잰다.

한 번 잘 들어가는 것과 매번 같은 자리에 들어가는 것은 다른 문제다. 리프팅을
붙이면 중요한 것은 후자다. 그래서 반복해서 재고 산포를 낸다.

재는 것 — 진입을 마친 뒤 로봇과 대차의 어긋남

    좌우   대차 중심이 로봇 옆으로 얼마나 벗어나 있나 (이것이 '중심 안 맞음')
    앞뒤   대차 중심까지 얼마나 남았나 (0 이면 정확히 중앙)
    방향   대차 축과 얼마나 틀어져 있나 (리프팅에 가장 중요하다)

한 회차는 다음과 같다.

    (1) 렉 밖으로 빠져나온다 (수동으로 앞으로. 도킹을 마치면 나가는 쪽을 본다)
    (2) 자동으로 바꾸고 작업을 건다
    (3) 끝나면 대차와의 어긋남을 잰다

    python3 tools/dock_accuracy.py --poi 렉1 --reps 5

⚠ 로봇이 실제로 움직인다. 렉 앞뒤로 3m 씩 비우고 비상정지를 잡고 있을 것.
"""
import argparse
import json
import math
import time
import urllib.error
import urllib.request

API = "http://127.0.0.1:8090"
BUSY = ("moving", "looking", "staging", "aligning", "entering")


def api(path, body=None, method=None):
    req = urllib.request.Request(
        API + path,
        data=json.dumps(body).encode() if body is not None else None,
        headers={"Content-Type": "application/json"}, method=method)
    try:
        with urllib.request.urlopen(req, timeout=10) as r:
            return json.load(r)
    except urllib.error.HTTPError as e:
        return json.load(e)


def wrap(d):
    return math.atan2(math.sin(d), math.cos(d))


def fold(d):
    """대차 방향은 90도 주기다. -45~45도로 접는다."""
    return (d + math.pi / 4) % (math.pi / 2) - math.pi / 4


def pose():
    p = api("/chassis/pose").get("map")
    return (p["x"], p["y"], p["yaw"]) if p else None


def cart_near(xy, radius=0.6, want=None, min_score=0.70):
    """지도 좌표 xy 근처에서 가장 가까운 대차. 카메라 판정은 보지 않는다.

    ⚠ 점수와 종류를 따진다. 렉 안에서는 다리를 비스듬히 보게 되어 엉뚱한
      조합이 잡히기도 한다. 그것으로 재면 방향이 9도씩 튄다(2026-09-16:
      작업은 -0.09도로 봤는데 밖에서는 -9.40도로 쟀다).
    """
    best = None
    for c in (api("/carts").get("carts") or []):
        if want and c.get("cart") != want:
            continue
        if float(c.get("score") or 0.0) < min_score:
            continue
        m = c.get("map") or {}
        p = m.get("xy")
        if not p:
            continue
        off = math.hypot(p[0] - xy[0], p[1] - xy[1])
        if off > radius:
            continue
        if best is None or off < best[0]:
            best = (off, p[0], p[1], m.get("yaw", 0.0), c.get("cart"))
    return best


def measure(poi):
    """진입을 마친 로봇과 대차의 어긋남. (좌우mm, 앞뒤mm, 방향도) 또는 None."""
    rp = pose()
    if rp is None:
        return None
    c = cart_near((poi["x"], poi["y"]), want=poi.get("cart"))
    if c is None:
        return None
    _, cx, cy, cyaw, _ = c
    dx, dy = cx - rp[0], cy - rp[1]
    co, sn = math.cos(rp[2]), math.sin(rp[2])
    return (round((-dx * sn + dy * co) * 1000, 1),      # 좌우
            round((dx * co + dy * sn) * 1000, 1),       # 앞뒤
            round(math.degrees(fold(cyaw - rp[2])), 2))  # 방향


def drive_out(poi, speed=0.15, limit=40.0):
    """렉 밖으로 빠져나온다. 도킹을 마치면 나가는 쪽을 보고 있다."""
    api("/chassis/status", {"control_mode": "manual"}, "PATCH")
    t0 = time.time()
    want = float(poi.get("standoff") or 1.8) + 0.3
    ok = False
    while time.time() - t0 < limit:
        rp = pose()
        if rp is None:
            break
        if math.hypot(poi["x"] - rp[0], poi["y"] - rp[1]) >= want:
            ok = True
            break
        api("/chassis/twist", {"linear": speed, "angular": 0.0})
        time.sleep(0.1)
    api("/chassis/twist", {"linear": 0.0, "angular": 0.0})
    time.sleep(0.5)
    api("/chassis/status", {"control_mode": "auto"}, "PATCH")
    time.sleep(0.5)
    return ok


def run_job(name, limit=240.0):
    r = api("/job/start", {"poi": name})
    if not r.get("ok"):
        return False, r.get("message", "")
    t0 = time.time()
    last = ""
    while time.time() - t0 < limit:
        time.sleep(0.5)
        st = api("/job/status")
        s = st.get("state")
        if st.get("step") and st["step"] != last:
            last = st["step"]
        if s == "done":
            return True, "%.0f초" % (time.time() - t0)
        if s in ("failed", "canceled"):
            return False, st.get("message", s)
    api("/job/cancel", {})
    return False, "시간 초과"


def stat(vals):
    n = len(vals)
    m = sum(vals) / n
    sd = math.sqrt(sum((v - m) ** 2 for v in vals) / n) if n > 1 else 0.0
    return m, sd, max(abs(v) for v in vals)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--poi", required=True, help="대차 POI 이름")
    ap.add_argument("--reps", type=int, default=5)
    ap.add_argument("--out-speed", type=float, default=0.15, dest="out_speed",
                    help="렉에서 빠져나올 때 속도 [m/s]")
    a = ap.parse_args()

    pois = {p["name"]: p for p in api("/pois").get("pois", [])}
    poi = pois.get(a.poi)
    if poi is None or poi.get("kind") != "cart":
        print("!! '%s' 가 없거나 대차 지점이 아닙니다" % a.poi)
        return
    if not api("/nav/status").get("available"):
        print("!! nav2 가 떠 있지 않습니다")
        return

    print("=" * 66)
    print("  '%s' 에 %d 회 들어가 본다" % (a.poi, a.reps))
    print("  대차 자리 (%.2f, %.2f), 접근 %.1fm"
          % (poi["x"], poi["y"], poi.get("standoff") or 1.8))
    print("  ⚠ 로봇이 움직입니다. 비상정지를 잡고 계세요.")
    print("=" * 66)

    rows = []
    for i in range(a.reps):
        rp = pose()
        if rp and math.hypot(poi["x"] - rp[0], poi["y"] - rp[1]) < 1.0:
            if not drive_out(poi, a.out_speed):
                print("  [%d] 렉에서 빠져나오지 못했습니다" % (i + 1))
                break
        ok, msg = run_job(a.poi)
        if not ok:
            print("  [%d] 실패 — %s" % (i + 1, msg))
            continue
        time.sleep(1.5)                    # 가라앉기를 기다린 뒤에 잰다
        m = measure(poi)
        if m is None:
            print("  [%d] 성공했으나 대차가 안 보여 못 쟀습니다" % (i + 1))
            continue
        rows.append(m)
        print("  [%d] 좌우 %+7.1f mm   앞뒤 %+7.1f mm   방향 %+6.2f 도   (%s)"
              % (i + 1, m[0], m[1], m[2], msg))

    print()
    print("=" * 66)
    if len(rows) < 2:
        print("  잰 회차가 모자라 산포를 못 냅니다 (%d 회)" % len(rows))
        return
    names = ("좌우[mm]", "앞뒤[mm]", "방향[도]")
    print("  %d 회 결과" % len(rows))
    print("  %-10s %9s %9s %9s" % ("", "평균", "산포(1σ)", "최대"))
    for k, nm in enumerate(names):
        m, sd, mx = stat([r[k] for r in rows])
        print("  %-10s %9.2f %9.2f %9.2f" % (nm, m, sd, mx))
    print()
    print("  평균은 '치우친 정도' 이고 산포는 '매번 달라지는 정도' 다.")
    print("  평균이 크면 기준이 어긋난 것이라 값을 고치면 되지만,")
    print("  산포가 크면 제어나 차체를 봐야 한다.")
    print("=" * 66)


if __name__ == "__main__":
    main()
