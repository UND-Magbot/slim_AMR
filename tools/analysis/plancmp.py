import math, time, statistics as st, threading
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from rclpy.executors import SingleThreadedExecutor
from nav2_msgs.action import ComputePathToPose
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
R = math.radians
def ps(x, y, yaw):
    p = PoseStamped(); p.header.frame_id = 'map'; p.pose.position.x = x; p.pose.position.y = y
    p.pose.orientation.z = math.sin(yaw / 2); p.pose.orientation.w = math.cos(yaw / 2); return p
def toward(a, b): return math.atan2(b[1] - a[1], b[0] - a[0])
P1 = (4.174, -1.928, R(-20.1)); P2 = (9.285, -3.633, R(169.5)); CH = (0.766, -0.039, R(-15.8))
def out(p, d): return (p[0] + d * math.cos(p[2]), p[1] + d * math.sin(p[2]))
e1, e2, ec = out(P1, 0.3), out(P2, 0.3), out(CH, 0.5)
G = {'1->2': (e1, (8.70, -3.52, R(-11))), '2->1': (e2, (4.74, -2.13, R(160))),
     '충전소->1': ((ec[0], ec[1], CH[2]), (3.61, -1.72, R(-20))), '1->충전소': (e1, (1.48, -0.24, R(-16))),
     '1->test1': (e1, (3.916, -0.520, R(73.1))), '1->test2(좁음)': (e1, (8.084, 1.771, R(165.2))), '1->렉1': (e1, (-3.142, -3.179, R(-12.5)))}
rclpy.init(); n = Node('plan_cmp'); cli = ActionClient(n, ComputePathToPose, 'compute_path_to_pose')
cm = {}
n.create_subscription(OccupancyGrid, '/global_costmap/costmap', lambda m: cm.__setitem__('m', m),
                      QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE))
ex = SingleThreadedExecutor(); ex.add_node(n); threading.Thread(target=ex.spin, daemon=True).start()
cli.wait_for_server(timeout_sec=10); t0 = time.time()
while 'm' not in cm and time.time() - t0 < 5: time.sleep(0.1)
def cost_at(x, y):
    m = cm.get('m')
    if m is None: return -1
    i = int((x - m.info.origin.position.x) / m.info.resolution); j = int((y - m.info.origin.position.y) / m.info.resolution)
    if 0 <= i < m.info.width and 0 <= j < m.info.height: return m.data[j * m.info.width + i]
    return -1
def plan(start, goal, pid):
    g = ComputePathToPose.Goal(); g.goal = ps(*goal); g.start = ps(*start); g.use_start = True; g.planner_id = pid
    t = time.time(); f = cli.send_goal_async(g)
    while not f.done(): time.sleep(0.005)
    h = f.result()
    if not h.accepted: return None
    r = h.get_result_async()
    while not r.done(): time.sleep(0.005)
    dt = time.time() - t; res = r.result().result; pts = [(q.pose.position.x, q.pose.position.y) for q in res.path.poses]
    if len(pts) < 2: return {'t': dt, 'fail': True}
    L = sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(pts, pts[1:]))
    # 마지막 0.6m 들어오는 방향 vs 목표 방향
    acc, k = 0.0, len(pts) - 1
    while k > 0 and acc < 0.6: acc += math.hypot(pts[k][0] - pts[k-1][0], pts[k][1] - pts[k-1][1]); k -= 1
    a = math.atan2(pts[-1][1] - pts[k][1], pts[-1][0] - pts[k][0]); da = math.degrees(math.atan2(math.sin(a - goal[2]), math.cos(a - goal[2])))
    # 처음 0.6m 나가는 방향 vs 출발 방향
    acc, k = 0.0, 0
    while k < len(pts) - 1 and acc < 0.6: acc += math.hypot(pts[k+1][0] - pts[k][0], pts[k+1][1] - pts[k][1]); k += 1
    b = math.atan2(pts[k][1] - pts[0][1], pts[k][0] - pts[0][0]); db = math.degrees(math.atan2(math.sin(b - start[2]), math.cos(b - start[2])))
    # 방향 변화 총합 (0.1m 간격으로 다시 뽑아)
    rs = [pts[0]]; acc = 0
    for a_, b_ in zip(pts, pts[1:]):
        acc += math.hypot(b_[0] - a_[0], b_[1] - a_[1])
        if acc >= 0.1: rs.append(b_); acc = 0
    hd = [math.atan2(b_[1] - a_[1], b_[0] - a_[0]) for a_, b_ in zip(rs, rs[1:])]
    turns = [math.degrees(math.atan2(math.sin(y - x), math.cos(y - x))) for x, y in zip(hd, hd[1:])]
    signs = sum(1 for x, y in zip(turns, turns[1:]) if x * y < 0 and abs(x) > 1 and abs(y) > 1)
    mc = max(cost_at(*p) for p in pts)
    return {'t': dt, 'L': L, 'end': da, 'start': db, 'turn': sum(abs(x) for x in turns), 'maxturn': max([abs(x) for x in turns] or [0]), 'S': signs, 'cost': mc, 'fail': False}
print('%-16s %-9s %6s %6s %8s %8s %8s %6s %5s' % ('경우', '계획기', '시간s', '길이m', '끝방향차', '출발방향', '꺾임합', '최대꺾', '최대비용'))
for name, (s_, g_) in G.items():
    s3 = (s_[0], s_[1], s_[2] if len(s_) == 3 else toward(s_, g_))
    for pid in ('GridBased', 'Hybrid'):
        rr = [plan(s3, g_, pid) for _ in range(3)]
        rr = [r for r in rr if r]
        if not rr or rr[0].get('fail'):
            print('%-16s %-9s 실패 (%.2fs)' % (name, pid, rr[0]['t'] if rr else -1)); continue
        r = rr[0]
        print('%-16s %-9s %6.2f %6.2f %+7.1f도 %+7.1f도 %7.0f도 %5.1f도 %5d' % (name, pid, max(x['t'] for x in rr), r['L'], r['end'], r['start'], r['turn'], r['maxturn'], r['cost']))
rclpy.shutdown()
