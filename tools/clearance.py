#!/usr/bin/env python3
"""주변 여유 공간 확인 — 주행 테스트 전 안전 점검용."""
import math, sys, time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

rclpy.init()
class S(Node):
    def __init__(s):
        super().__init__("clearance"); s.m=None
        s.create_subscription(LaserScan,"/scan",lambda m:setattr(s,'m',m),qos_profile_sensor_data)
n=S()
t0=time.time()
while n.m is None and time.time()-t0<10: rclpy.spin_once(n,timeout_sec=0.05)
if n.m is None:
    print("!! /scan 없음"); sys.exit(1)
m=n.m
pts=[(math.degrees(m.angle_min+i*m.angle_increment), r) for i,r in enumerate(m.ranges)
     if math.isfinite(r) and m.range_min<r<m.range_max]
allmin=min(r for _,r in pts)
print("전 방향 최소거리: %.2f m  (어느 방향으로든 안전한 여유)"%allmin)
best=[]
for s0 in range(-180,180,15):
    sel=[r for a,r in pts if s0<=a<s0+15]
    if sel:
        mn=min(sel); best.append((mn,s0))
        print("  %+4d ~ %+4d deg : %6.2f m %s"%(s0,s0+15,mn,"#"*int(min(mn,12)*3)))
best.sort(reverse=True)
print()
print("가장 넓은 방향: " + ", ".join("%+d~%+ddeg %.1fm"%(s,s+15,v) for v,s in best[:3]))
rclpy.shutdown()
