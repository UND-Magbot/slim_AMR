"""경로 직선도 (2026-11-17, 사용자: 경로에도 웬만하면 직선주행). 시작=로봇 자리 또는 POI, 끝=POI."""
import math, json, sys, urllib.request, rclpy, numpy as np
from rclpy.action import ActionClient
from nav2_msgs.action import ComputePathToPose, SmoothPath
from geometry_msgs.msg import PoseStamped
rclpy.init(); n = rclpy.create_node("pstr")
pc = ActionClient(n, ComputePathToPose, "/compute_path_to_pose"); pc.wait_for_server(timeout_sec=5)
sc = ActionClient(n, SmoothPath, "/smooth_path"); sc.wait_for_server(timeout_sec=5)
pois = {p["name"]: p for p in json.load(urllib.request.urlopen("http://localhost:8090/pois", timeout=3))["pois"]}
def ps(x, y, yaw):
    p = PoseStamped(); p.header.frame_id = "map"; p.pose.position.x = x; p.pose.position.y = y
    p.pose.orientation.z = math.sin(yaw / 2); p.pose.orientation.w = math.cos(yaw / 2); return p
def call(cli, g):
    f = cli.send_goal_async(g); rclpy.spin_until_future_complete(n, f, timeout_sec=5); h = f.result()
    if h is None or not h.accepted: return None
    r = h.get_result_async(); rclpy.spin_until_future_complete(n, r, timeout_sec=10); return r.result()
def stats(P):
    P = np.array(P)
    if len(P) < 3: return "점 %d" % len(P)
    d = np.diff(P, axis=0); seg = np.hypot(d[:, 0], d[:, 1]); h = np.arctan2(d[:, 1], d[:, 0])
    # 5cm 간격으로 다시 뽑아 방향 변화
    L = np.r_[0, np.cumsum(seg)]; s = np.arange(0, L[-1], 0.05)
    Q = np.c_[np.interp(s, L, P[:, 0]), np.interp(s, L, P[:, 1])]
    dq = np.diff(Q, axis=0); hq = np.unwrap(np.arctan2(dq[:, 1], dq[:, 0])); dh = np.degrees(np.diff(hq))
    curved = (np.abs(dh) > 0.5).sum() * 0.05            # 휘는 구간 길이 [m]
    turns = np.abs(dh).sum()
    u = (P[-1] - P[0]) / max(np.hypot(*(P[-1] - P[0])), 1e-6)
    dev = np.abs((P - P[0])[:, 0] * u[1] - (P - P[0])[:, 1] * u[0]).max()
    return "길이 %.2fm 직선거리 %.2fm, 휘는 구간 %.2fm, 방향 바뀐 총량 %.0f도, 직선에서 최대 %.2fm" % (L[-1], np.hypot(*(P[-1]-P[0])), curved, turns, dev)
TRAVEL = "--travel" in sys.argv
pairs = [tuple(a.split(">")) for a in sys.argv[1:] if ">" in a] or [("1", "2"), ("2", "1"), ("1", "test1"), ("2", "test2")]
for a, b in pairs:
    A, B = pois[a], pois[b]
    gyaw = math.atan2(B["y"] - A["y"], B["x"] - A["x"]) if "--travel" in sys.argv[0:1] or TRAVEL else B["yaw"]
    g = ComputePathToPose.Goal(); g.goal = ps(B["x"], B["y"], gyaw); g.start = ps(A["x"], A["y"], A["yaw"]); g.use_start = True; g.planner_id = "GridBased"
    r = call(pc, g)
    if r is None or r.status != 4: print(a, ">", b, "경로 없음"); continue
    raw = [(q.pose.position.x, q.pose.position.y) for q in r.result.path.poses]
    sg = SmoothPath.Goal(); sg.path = r.result.path; sg.smoother_id = "SmoothPath"; sg.max_smoothing_duration.sec = 1; sg.check_for_collisions = False
    sr = call(sc, sg)
    sm = [(q.pose.position.x, q.pose.position.y) for q in sr.result.path.poses] if sr and sr.status == 4 else []
    print("%s > %s\n  원래(Theta*)  %s\n  스무더 뒤     %s" % (a, b, stats(raw), stats(sm) if sm else "실패"))
