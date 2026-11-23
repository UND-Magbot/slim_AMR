#!/usr/bin/env python3
"""오도메트리 검증 — /odom 과 엔코더 카운트 원본을 직접 대조.

base_node 가 발행하는 /diagnostics 에 축별 엔코더 position(counts) 이 들어있다.
카운트에서 직접 계산한 거리와 /odom 이 말하는 거리가 다르면 노드 적분 문제,
같으면 오도메트리는 맞고 실제 주행(슬립/라이다 정합) 쪽을 봐야 한다.
"""
import argparse, math, sys, time
import rclpy, yaml
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import Twist
from safe_stop import install_safe_stop
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data

ap = argparse.ArgumentParser()
ap.add_argument("--secs", type=float, default=6.0)
ap.add_argument("--margin", type=float, default=1.0, help="주행거리 외 추가로 요구할 전방 여유[m]")
ap.add_argument("--no-clearance-check", action="store_true")
ap.add_argument("--speed", type=float, default=0.20)
ap.add_argument("--params", default="/home/und_amr1/ros2_ws/src/und_amr_bringup/config/amr_params.yaml")
a = ap.parse_args()

P = yaml.safe_load(open(a.params))["base_node"]["ros__parameters"]
D, K = P["wheel_diameter"], P.get("wheel_diameter_ratio", 1.0)
CPR = float(P["counts_per_wheel_rev"])
RL = D * (2 * K / (1 + K)) / 2.0
RR = D * (2 / (1 + K)) / 2.0
print("파라미터: 직경 좌 %.5f / 우 %.5f m, %d counts/rev" % (RL*2, RR*2, CPR))

rclpy.init()
class R(Node):
    def __init__(s):
        super().__init__("diag_odom")
        s.od=None; s.js=None; s.enc={}; s.scan=None
        s.create_subscription(Odometry,"/odom",lambda m:setattr(s,'od',m),10)
        s.create_subscription(JointState,"/joint_states",lambda m:setattr(s,'js',m),10)
        s.create_subscription(DiagnosticArray,"/diagnostics",s._dg,10)
        s.create_subscription(LaserScan,"/scan",lambda m:setattr(s,'scan',m),
                              qos_profile_sensor_data)
        s.pub=s.create_publisher(Twist,"/cmd_vel",10)
    def _dg(s,m):
        for st in m.status:
            for kv in st.values:
                if kv.key=="position":
                    ax=int(st.name.split("axis")[-1])
                    s.enc[ax]=int(kv.value)
    def spin(s,t):
        t0=time.time()
        while time.time()-t0<t: rclpy.spin_once(s,timeout_sec=0.02)

    def front_clear(s, half_deg=35.0):
        """전방 +-half_deg 최소거리. /scan 이 없으면 None."""
        m = s.scan
        if m is None: return None
        best=None
        for i,r in enumerate(m.ranges):
            if not math.isfinite(r) or not (m.range_min < r < m.range_max): continue
            d=math.degrees(m.angle_min+i*m.angle_increment)
            if -half_deg<=d<=half_deg and (best is None or r<best): best=r
        return best

n=R(); install_safe_stop(n); n.spin(2.5)
if n.od is None or not n.enc:
    print("!! /odom 또는 /diagnostics 미수신 (enc=%s)"%n.enc); sys.exit(1)

def snap():
    n.spin(0.4)
    p=n.od.pose.pose.position
    return (p.x,p.y), dict(n.enc), (list(n.js.position) if n.js else None)

# ---- 안전 점검: 실제로 갈 거리를 계산하고 전방 여유와 비교 ----
need = a.secs * a.speed + a.margin
clr = n.front_clear()
if not a.no_clearance_check:
    if clr is None:
        print("!! /scan 이 없어 전방 확인 불가. 라이다를 띄우거나 "
              "--no-clearance-check 로 강제 실행하세요."); sys.exit(1)
    if clr < need:
        print("!! 전방 여유 %.2f m < 필요 %.2f m (주행 %.2f + 여유 %.2f) — 중단"
              %(clr, need, a.secs*a.speed, a.margin)); sys.exit(1)
print("전방 여유 %.2f m / 필요 %.2f m — 진행"%(clr if clr else -1, need))

(p0,e0,j0)=snap()
print(">> %.1f초간 %.2f m/s 직진 (예상 %.2f m)"%(a.secs,a.speed,a.secs*a.speed))
t=Twist(); t.linear.x=a.speed
t0=time.time()
stopped_by=None
while time.time()-t0<a.secs:
    n.pub.publish(t); rclpy.spin_once(n,timeout_sec=0.02)
    c=n.front_clear()
    if c is not None and c < 0.6:            # 주행 중 비상 정지
        stopped_by="전방 %.2f m 접근"%c
        break
el=time.time()-t0
for _ in range(15): n.pub.publish(Twist()); n.spin(0.05)
n.spin(2.0)
(p1,e1,j1)=snap()

od=math.hypot(p1[0]-p0[0],p1[1]-p0[1])
dl=(e1[1]-e0[1])/CPR*2*math.pi*RL
dr=(e1[2]-e0[2])/CPR*2*math.pi*RR
enc=(dl+dr)/2.0
print()
if stopped_by: print("  ⚠ 주행 중 비상정지: %s"%stopped_by)
print("  주행시간 %.2f s (지령속도 기준 %.4f m)"%(el,el*a.speed))
print("  엔코더 카운트 좌 %+d / 우 %+d"%(e1[1]-e0[1],e1[2]-e0[2]))
print("  카운트로 직접 계산한 거리 : %.4f m  (좌 %.4f / 우 %.4f)"%(enc,dl,dr))
print("  /odom 이 말하는 거리      : %.4f m"%od)
print("  차이 %+.1f mm (%+.2f%%)"%((od-enc)*1000,(od/enc-1)*100 if enc else 0))
if j0 and j1:
    wl=(j1[0]-j0[0])*RL; wr=(j1[1]-j0[1])*RR
    print("  /joint_states 적분 거리   : %.4f m (좌 %.4f / 우 %.4f)"%((wl+wr)/2,wl,wr))
print()
print("  판정: %s"%("오도메트리 적분 정상 (엔코더와 일치)" if abs(od-enc)<0.02
      else "!! 노드 적분이 엔코더와 어긋남 — base_node 문제"))
rclpy.shutdown()
