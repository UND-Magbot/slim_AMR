import sys, math, types, collections
sys.path.insert(0, 'test'); sys.path.insert(0, '.')
from und_amr_bringup.api_node import ApiNode
from test_narrow_poi import make
POI = {"name": "p", "x": 2.0, "y": 0.0, "yaw": 0.0}
def sim(pre_len, lat, yaw_deg, wmax=None, wacc=None, pred=0.0, lag=0.3, gain=1.25, v0=0.2):
    pre_x = 2.0 - pre_len
    n = make(ApiNode, (pre_x - 0.3, lat, math.radians(yaw_deg)), POI)
    n._slow_timer = None
    if wacc: n.SP_W_ACC = wacc
    n._np.update(mode="open", entry=dict(POI), pre=(pre_x, 0.0, 0.0, pre_len))
    n.nav_goal = types.SimpleNamespace(cancel_goal_async=lambda: None)
    st = {"w": 0.0}
    n.odom = types.SimpleNamespace(twist=types.SimpleNamespace(twist=types.SimpleNamespace(linear=types.SimpleNamespace(x=v0), angular=types.SimpleNamespace(z=0.0))))
    n.SP_HANDOFF_M = 10; n.SP_HANDOFF_LAT = 1; n.SP_HANDOFF_YAW = 1.5; n.SP_HANDOFF_ERR = 1.5; n.SP_HANDOFF_LAT_PAR = 1
    if wmax: n.SP_W_MAX_HO = wmax
    n.SP_YAW_PRED_S = pred
    assert n.np_handoff()
    dt = 1 / n.NP_HZ; q = collections.deque([0.0] * int(lag / dt)); s = n._sim; yaws = []; ws = []; wa = 0.0
    t = 0
    while getattr(n, "_np_timer", None) is not None and t < 30:
        n.odom.twist.twist.angular.z = wa
        n._np_tick(); q.append(s["w"]); wc = q.popleft()
        wa += (gain * wc - wa) * min(1, dt / 0.15)       # 실제: 지연 + 15% 넘침 + 1차 반응
        ws.append(wa)
        s["yaw"] += wa * dt; s["x"] += s["v"] * math.cos(s["yaw"]) * dt; s["y"] += s["v"] * math.sin(s["yaw"]) * dt
        yaws.append(math.degrees(s["yaw"])); t += dt
    over = max(yaws) if yaw_deg < 0 else -min(yaws)
    return "최대 실제회전 %4.1f도/s  반대로 넘어감 %+4.1f도  끝 좌우 %+5.1fmm 방향 %+4.1f도  %.1f초" % (
        max(abs(x) for x in ws) * 57.3, over, s["y"] * 1000, math.degrees(s["yaw"]), t)
cases = [(0.087, -12.6), (0.06, -9), (0.12, -17), (0.16, -21)]
for name, kw in [("지금 (0.6m)", dict(pre_len=0.6)),
                 ("0.6m + 9도/s + 지연보정", dict(pre_len=0.6, wmax=0.157, wacc=0.35, pred=0.3)),
                 ("1.0m + 9도/s + 지연보정", dict(pre_len=1.0, wmax=0.157, wacc=0.35, pred=0.3)),
                 ("1.0m + 9도/s", dict(pre_len=1.0, wmax=0.157, wacc=0.35))]:
    print(name)
    for lat, yd in cases:
        print("   %3.0fmm %5.1f도: %s" % (lat * 1000, yd, sim(lat=lat, yaw_deg=yd, **kw)))
