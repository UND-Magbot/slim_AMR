#!/usr/bin/env python3
"""EKF 융합 검증 — 바퀴 슬립이 방향을 망치는 걸 자이로가 막아주는가.

한 번의 직진에서 세 가지 방향 변화량을 비교한다.
    /odom              바퀴 오도메트리만  (슬립에 그대로 오염됨)
    /odometry/filtered EKF 융합 (yaw 는 IMU 자이로에서 가져옴)
    라이다 ICP         정답

바퀴가 헛돈 회차에서 EKF 쪽이 라이다에 가까우면 융합이 제 역할을 하는 것이다.

    ros2 launch und_amr_bringup bringup.launch.py
    python3 tools/verify_ekf.py --meters 1.5 --reps 5
"""
import argparse, math, statistics, sys, time
sys.path.insert(0, "/home/und_amr1/projects/UND_AMR/tools")
import numpy as np, rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from safe_stop import install_safe_stop
from diagnostic_msgs.msg import DiagnosticArray
from std_srvs.srv import Trigger
from scan_match import match

ap = argparse.ArgumentParser()
ap.add_argument("--meters", type=float, default=1.5)
ap.add_argument("--speed", type=float, default=0.15)
ap.add_argument("--prealign", type=float, default=0.5)
ap.add_argument("--reps", type=int, default=5)
ap.add_argument("--scans", type=int, default=14)
a = ap.parse_args()

def yaw(q):
    return math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))

def wrap(d):
    while d > math.pi: d -= 2*math.pi
    while d < -math.pi: d += 2*math.pi
    return d

class Rig(Node):
    def __init__(s):
        super().__init__("verify_ekf")
        s.scan=None; s.meta=None; s.od=None; s.ek=None; s.enc={}; s.st={}
        s.create_subscription(LaserScan,"/scan",s._sc,qos_profile_sensor_data)
        s.create_subscription(Odometry,"/odom",lambda m:setattr(s,'od',m),10)
        s.create_subscription(Odometry,"/odometry/filtered",lambda m:setattr(s,'ek',m),10)
        s.create_subscription(DiagnosticArray,"/diagnostics",s._dg,10)
        s.pub=s.create_publisher(Twist,"/cmd_vel",10)
        s.cli_en=s.create_client(Trigger,"/base_node/enable")
    def _sc(s,m): s.scan=m; s.meta=m

    def ensure_enabled(s):
        """축이 풀려 있으면 다시 켠다. 풀린 채로 달리면 로봇이 전혀
        움직이지 않는데 측정은 '회전 0도'로 기록되어 결과가 오염된다."""
        s.sp(0.5)
        en = all(s.st.get(ax,{}).get("enabled")=="True" for ax in (1,2))
        if en: return True
        if not s.cli_en.wait_for_service(timeout_sec=3.0):
            print("      !! enable 서비스 없음"); return False
        fut=s.cli_en.call_async(Trigger.Request())
        t0=time.time()
        while not fut.done() and time.time()-t0<5: rclpy.spin_once(s,timeout_sec=0.05)
        s.sp(1.5)
        en = all(s.st.get(ax,{}).get("enabled")=="True" for ax in (1,2))
        print("      축 재기동: %s"%("성공" if en else "실패"))
        return en
    def _dg(s,m):
        for st in m.status:
            if not st.name.startswith("motor/axis"): continue
            ax=int(st.name.split("axis")[-1])
            kv={k.key:k.value for k in st.values}
            s.st[ax]=kv
            if "position" in kv: s.enc[ax]=int(kv["position"])
    def sp(s,t):
        t0=time.time()
        while time.time()-t0<t: rclpy.spin_once(s,timeout_sec=0.02)
    def state(s):
        s.sp(0.35)
        p=s.od.pose.pose
        e=s.ek.pose.pose if s.ek else None
        return ((p.position.x,p.position.y,yaw(p.orientation)),
                ((e.position.x,e.position.y,yaw(e.orientation)) if e else None),
                dict(s.enc))
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
    def grab(s,n):
        s.sp(1.0)
        acc=None;got=0;t0=time.time()
        while got<n and time.time()-t0<14:
            s.scan=None
            while s.scan is None and time.time()-t0<14: rclpy.spin_once(s,timeout_sec=0.05)
            m=s.scan
            if acc is None: acc=[[] for _ in m.ranges]
            for i,r in enumerate(m.ranges):
                if math.isfinite(r) and m.range_min<r<m.range_max: acc[i].append(r)
            got+=1
        return np.array([statistics.median(v) if len(v)>=3 else np.nan for v in acc])
    def drive(s,dist,speed):
        dist=abs(dist)
        (o0,_,_)=s.state()
        t=Twist(); t.linear.x=speed; t0=time.time(); stop=None
        while time.time()-t0 < dist/abs(speed)+15:
            s.pub.publish(t); rclpy.spin_once(s,timeout_sec=0.02)
            c=s.clr(rear=(speed<0))
            if c is not None and c<0.6: stop="장애물 %.2fm"%c; break
            p=s.od.pose.pose.position
            if math.hypot(p.x-o0[0],p.y-o0[1])>=dist: break
        for _ in range(12): s.pub.publish(Twist()); s.sp(0.05)
        s.sp(1.5)
        return stop

rclpy.init(); n=Rig(); install_safe_stop(n); n.sp(3.0)
if n.od is None or n.ek is None or n.scan is None:
    print("!! /odom, /odometry/filtered, /scan 중 미수신 있음"); sys.exit(1)

rows=[]
try:
    for k in range(a.reps):
        if not n.ensure_enabled():
            print("  축을 켤 수 없어 중단"); break
        f=n.clr()
        if f is None or f < a.meters+0.8:
            print("  전방 %.2f m < 필요 %.2f m — 중단"%(f or -1, a.meters+0.8)); break
        n.drive(a.prealign,a.speed); n.sp(0.8)
        A=n.grab(a.scans); meta=n.meta
        (o0,e0,c0)=n.state()
        stop=n.drive(a.meters,a.speed)
        (o1,e1,c1)=n.state()
        B=n.grab(a.scans)
        od_d=math.hypot(o1[0]-o0[0],o1[1]-o0[1])
        (tx,ty,th),info=match(A,B,meta,init=(od_d,0.0,0.0),rmax=10.0)
        alts=[]
        for fct in (0.85,1.15):
            (_,_,t2),_=match(A,B,meta,init=(od_d*fct,0.0,0.0),rmax=10.0)
            alts.append(math.degrees(t2))
        asp=max([math.degrees(th)]+alts)-min([math.degrees(th)]+alts)
        moved = od_d > a.meters*0.5
        ok = info["rms"]<0.10 and asp<0.5 and moved
        d_od=math.degrees(wrap(o1[2]-o0[2]))
        d_ek=math.degrees(wrap(e1[2]-e0[2]))
        d_gt=math.degrees(th)
        dc1=c1.get(1,0)-c0.get(1,0); dc2=c1.get(2,0)-c0.get(2,0)
        skew=(dc1-dc2)/max(abs(dc1),abs(dc2),1)*100
        if ok: rows.append((skew,d_od,d_ek,d_gt))
        print("  [%d] 좌우차 %+6.2f%% | 방향변화  바퀴 %+7.2f°  EKF %+7.2f°  라이다 %+7.2f° "
              "| 바퀴오차 %+6.2f°  EKF오차 %+6.2f° | rms %.3f %s"
              %(k+1,skew,d_od,d_ek,d_gt,d_od-d_gt,d_ek-d_gt,info["rms"],
                "OK" if ok else ("안움직임" if not moved else "기각")))
        r=n.clr(rear=True)
        if r and r > od_d+a.prealign+0.9: n.drive(od_d+a.prealign,-a.speed); n.sp(0.8)
        else: print("      후방 %.2f m — 복귀 생략"%(r or -1))
finally:
    for _ in range(12): n.pub.publish(Twist()); n.sp(0.05)

print()
print("="*78)
if rows:
    eo=[abs(r[1]-r[3]) for r in rows]; ee=[abs(r[2]-r[3]) for r in rows]
    print("  유효 %d회"%len(rows))
    print("  바퀴 오도메트리 방향오차: 평균 %.2f°  최대 %.2f°"%(statistics.mean(eo),max(eo)))
    print("  EKF 융합    방향오차: 평균 %.2f°  최대 %.2f°"%(statistics.mean(ee),max(ee)))
    if statistics.mean(eo)>0.1:
        print("  → EKF 가 방향오차를 %.1f 배 줄임"%(statistics.mean(eo)/max(statistics.mean(ee),1e-3)))
    slip=[r for r in rows if abs(r[0])>3.0]
    if slip:
        print()
        print("  슬립 회차(좌우차 3%% 초과) %d개만 보면:"%len(slip))
        print("    바퀴 오차 평균 %.2f°  /  EKF 오차 평균 %.2f°"
              %(statistics.mean([abs(r[1]-r[3]) for r in slip]),
                statistics.mean([abs(r[2]-r[3]) for r in slip])))
print("="*78)
rclpy.shutdown()
