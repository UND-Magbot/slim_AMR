#!/usr/bin/env python3
"""위치추정 드리프트 감시 — 스캔이 지도와 얼마나 맞는지 시간에 따라 본다.

로봇을 세워 둔 채로도 값이 나빠지면 위치추정이 흘러가고 있다는 뜻이다.
움직일 때만 나빠지면 오도메트리 오차를 스캔정합이 못 따라잡는 것이다.
"""
import math, sys, time
import numpy as np, yaml
from PIL import Image
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf2_ros import Buffer, TransformListener

name = sys.argv[1] if len(sys.argv) > 1 else None
secs = float(sys.argv[2]) if len(sys.argv) > 2 else 40.0
if not name:
    import json, urllib.request
    with urllib.request.urlopen("http://127.0.0.1:8090/mapping/status", timeout=8) as r:
        name = json.load(r).get("active_map")
base = "/home/und_amr1/ros2_ws/maps/" + name
info = yaml.safe_load(open(base + ".yaml"))
res = float(info["resolution"]); ox, oy, _ = [float(v) for v in info["origin"]]
img = np.array(Image.open(base + ".pgm").convert("L"))
H, W = img.shape
occ = img < 100
steps = max(1, int(round(0.6 / res)))
d = np.full(occ.shape, steps, np.int16); d[occ] = 0
cur = occ.copy()
for k in range(1, steps + 1):
    nxt = cur.copy()
    nxt[1:,:] |= cur[:-1,:]; nxt[:-1,:] |= cur[1:,:]
    nxt[:,1:] |= cur[:,:-1]; nxt[:,:-1] |= cur[:,1:]
    new = nxt & ~cur
    if not new.any(): break
    d[new] = k; cur = nxt
lik = np.exp(-((d.astype(np.float32)*res)**2)/(2*0.15*0.15))
print("지도 '%s' %dx%d, 감시 %.0f초" % (name, W, H, secs))

rclpy.init()
class S(Node):
    def __init__(s):
        super().__init__("locdrift"); s.m=None; s.od=None
        s.create_subscription(LaserScan,"/scan",lambda m:setattr(s,'m',m),qos_profile_sensor_data)
        s.create_subscription(Odometry,"/odom",lambda m:setattr(s,'od',m),10)
        s.buf=Buffer(); s.tl=TransformListener(s.buf,s)
    def sp(s,t):
        t0=time.time()
        while time.time()-t0<t: rclpy.spin_once(s,timeout_sec=0.02)
n=S(); n.sp(3.0)

def yaw_of(q): return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
def score(px,py,pyaw,mnt):
    m=n.m
    rs=np.asarray(m.ranges); ang=m.angle_min+np.arange(len(rs))*m.angle_increment
    ok=np.isfinite(rs)&(rs>m.range_min)&(rs<12.0)
    rs,ang=rs[ok],ang[ok]
    if len(rs)<40: return None
    i=np.linspace(0,len(rs)-1,180).astype(int); rs,ang=rs[i],ang[i]
    a=pyaw+mnt+ang
    wx=px+rs*np.cos(a); wy=py+rs*np.sin(a)
    gx=((wx-ox)/res).astype(np.int32); gy=(H-1-((wy-oy)/res)).astype(np.int32)
    msk=(gx>=0)&(gx<W)&(gy>=0)&(gy<H)
    if msk.sum()<20: return 0.0
    return float(lik[gy[msk],gx[msk]].sum()/len(rs))

MNT=0.0366   # 라이다 장착 yaw (URDF)
print("  %6s %10s %10s %8s %10s %8s"%("시각","map x","map y","yaw°","일치도","오도 이동"))
t0=time.time(); o0=None; first=None
while time.time()-t0<secs:
    n.sp(2.0)
    try:
        t=n.buf.lookup_transform("map","base_footprint",rclpy.time.Time())
    except Exception as e:
        print("   TF 없음:",str(e)[:50]); continue
    px,py=t.transform.translation.x,t.transform.translation.y
    pyaw=yaw_of(t.transform.rotation)
    sc=score(px,py,pyaw,MNT)
    p=n.od.pose.pose.position
    if o0 is None: o0=(p.x,p.y)
    od=math.hypot(p.x-o0[0],p.y-o0[1])
    if first is None: first=sc
    print("  %5.1fs %10.3f %10.3f %8.1f %9.1f%% %8.3f m"
          %(time.time()-t0,px,py,math.degrees(pyaw),(sc or 0)*100,od))
print()
print("  일치도 %.1f%% → %.1f%%  (떨어지면 위치추정이 흘러가는 중)"
      %((first or 0)*100,(sc or 0)*100))
rclpy.shutdown()
