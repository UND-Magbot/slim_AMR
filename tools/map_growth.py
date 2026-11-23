#!/usr/bin/env python3
"""매핑 확인 — 로봇이 움직일 때 지도가 실제로 넓어지는가.

주행 전후로 /map 의 관측 셀 수를 비교한다. 이전에 "라이다를 돌려도
지도가 안 그려진다" 는 문제가 있었으므로, 넓어지는지를 수치로 확인한다.
map->odom 보정량도 같이 본다(SLAM 이 오도메트리를 얼마나 고쳐주는지).
"""
import argparse, math, sys, time
import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy,
                       qos_profile_sensor_data)
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from safe_stop import install_safe_stop
from tf2_ros import Buffer, TransformListener

ap=argparse.ArgumentParser()
ap.add_argument("--meters",type=float,default=1.5)
ap.add_argument("--speed",type=float,default=0.15)
ap.add_argument("--legs",type=int,default=2,help="전진/후진 왕복 횟수")
a=ap.parse_args()

mq=QoSProfile(depth=1); mq.durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
mq.reliability=QoSReliabilityPolicy.RELIABLE

rclpy.init()
class S(Node):
    def __init__(s):
        super().__init__("mapgrow"); s.map=None; s.od=None; s.scan=None
        s.create_subscription(OccupancyGrid,"/map",lambda m:setattr(s,'map',m),mq)
        s.create_subscription(Odometry,"/odom",lambda m:setattr(s,'od',m),10)
        s.create_subscription(LaserScan,"/scan",lambda m:setattr(s,'scan',m),qos_profile_sensor_data)
        s.pub=s.create_publisher(Twist,"/cmd_vel",10)
        s.buf=Buffer(); s.tl=TransformListener(s.buf,s)
    def sp(s,t):
        t0=time.time()
        while time.time()-t0<t: rclpy.spin_once(s,timeout_sec=0.02)
    def stats(s):
        s.sp(1.5)
        m=s.map
        if m is None: return None
        d=m.data
        return {"known":sum(1 for v in d if v>=0),
                "occ":sum(1 for v in d if v>50),
                "w":m.info.width,"h":m.info.height,"res":m.info.resolution}
    def correction(s):
        try:
            t=s.buf.lookup_transform("map","odom",rclpy.time.Time())
            tr=t.transform.translation; q=t.transform.rotation
            return math.hypot(tr.x,tr.y), math.degrees(math.atan2(
                2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)))
        except Exception:
            return None,None
    def clr(s,rear=False,half=35.0):
        m=s.scan
        if m is None: return None
        b=None
        for i,r in enumerate(m.ranges):
            if not math.isfinite(r) or not (m.range_min<r<m.range_max): continue
            d=math.degrees(m.angle_min+i*m.angle_increment)
            hit=(abs(d)>=180-half) if rear else (-half<=d<=half)
            if hit and (b is None or r<b): b=r
        return b
    def drive(s,dist,speed):
        p=s.od.pose.pose.position; x0,y0=p.x,p.y
        t=Twist(); t.linear.x=speed; t0=time.time(); stop=None
        while time.time()-t0 < abs(dist)/abs(speed)+15:
            s.pub.publish(t); rclpy.spin_once(s,timeout_sec=0.02)
            c=s.clr(rear=(speed<0))
            if c is not None and c<0.7: stop="장애물 %.2fm"%c; break
            p=s.od.pose.pose.position
            if math.hypot(p.x-x0,p.y-y0)>=abs(dist): break
        for _ in range(12): s.pub.publish(Twist()); s.sp(0.05)
        s.sp(1.5)
        p=s.od.pose.pose.position
        return math.hypot(p.x-x0,p.y-y0), stop

n=S(); n.sp(3.0)
install_safe_stop(n)
if n.map is None or n.od is None or n.scan is None:
    print("!! /map, /odom, /scan 중 미수신"); sys.exit(1)
s0=n.stats()
c0=n.correction()
print("주행 전: 관측 셀 %d, 점유 셀 %d  (%dx%d @ %.2fm)"
      %(s0["known"],s0["occ"],s0["w"],s0["h"],s0["res"]))
print("         map->odom 보정 %.3f m / %+.2f deg"%(c0[0] or 0,c0[1] or 0))
print()
try:
    for k in range(a.legs):
        for name,sp in (("전진",a.speed),("후진",-a.speed)):
            need=a.meters+0.9
            c=n.clr(rear=(sp<0))
            if c is None or c<need:
                print("  [%s] 여유 %.2f m < %.2f m — 건너뜀"%(name,c or -1,need)); continue
            d,stop=n.drive(a.meters,sp)
            st=n.stats(); cc=n.correction()
            print("  [%s %d] %.3f m 이동%s → 관측 셀 %d (%+d), 점유 %d (%+d) | map->odom %.3f m %+.2f deg"
                  %(name,k+1,d," ("+stop+")" if stop else "",
                    st["known"],st["known"]-s0["known"],st["occ"],st["occ"]-s0["occ"],
                    cc[0] or 0,cc[1] or 0))
            s0=st
finally:
    for _ in range(12): n.pub.publish(Twist()); n.sp(0.05)
sf=n.stats(); cf=n.correction()
print()
print("="*70)
print("  최종: 관측 셀 %d, 점유 셀 %d, 격자 %dx%d"%(sf["known"],sf["occ"],sf["w"],sf["h"]))
print("  map->odom 누적 보정 %.3f m / %+.2f deg"%(cf[0] or 0,cf[1] or 0))
print("="*70)
rclpy.shutdown()
