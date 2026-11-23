#!/usr/bin/env python3
"""정지 상태에서 라이다가 흔들리는지 확인 — 마운트 이완/충격 점검용.

로봇을 세워둔 채 연속 스캔을 비교한다. 정상이면 스캔끼리 거의 같고
ICP 결과가 0 에 가깝다. 마운트가 흔들리면 값이 튄다.
"""
import math, statistics, sys, time
sys.path.insert(0,"/home/und_amr1/projects/UND_AMR/tools")
import numpy as np, rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from scan_match import match

rclpy.init()
class S(Node):
    def __init__(s):
        super().__init__("lidar_stable"); s.m=None; s.meta=None
        s.create_subscription(LaserScan,"/scan",s._cb,qos_profile_sensor_data)
    def _cb(s,m): s.m=m; s.meta=m
    def one(s):
        s.m=None; t0=time.time()
        while s.m is None and time.time()-t0<8: rclpy.spin_once(s,timeout_sec=0.02)
        return s.m
n=S()
frames=[]
t0=time.time()
while len(frames)<20 and time.time()-t0<25:
    m=n.one()
    if m: frames.append(np.array(m.ranges,dtype=np.float64))
meta=n.meta
print("스캔 %d개 수집, 유효점 평균 %.0f/%d"
      %(len(frames), statistics.mean([np.isfinite(f).sum() for f in frames]), len(frames[0])))

# 1) 같은 빈의 거리값이 얼마나 흔들리는가
stack=np.vstack(frames)
ok=np.all(np.isfinite(stack)&(stack<12),axis=0)
if ok.sum()>50:
    sd=np.std(stack[:,ok],axis=0)
    print("정지 상태 거리 잡음: 중앙값 %.4f m, 90퍼센타일 %.4f m (정상이면 0.01 m 내외)"
          %(float(np.median(sd)),float(np.percentile(sd,90))))
else:
    print("!! 전 프레임에서 유효한 빈이 부족 (%d개)"%ok.sum())

# 2) 정지 상태 스캔끼리 ICP — 0 이 나와야 정상
print()
print("정지 상태 ICP (0 에 가까워야 정상):")
for i in (1,5,10,19):
    if i>=len(frames): break
    (tx,ty,th),info=match(frames[0],frames[i],meta,init=(0,0,0),rmax=10.0)
    print("  프레임0 vs %2d: dx=%+.4f dy=%+.4f dth=%+.3fdeg  rms=%.4f  %s"
          %(i,tx,ty,math.degrees(th),info["rms"],
            "OK" if (math.hypot(tx,ty)<0.03 and abs(math.degrees(th))<0.5) else "!! 흔들림"))
rclpy.shutdown()
