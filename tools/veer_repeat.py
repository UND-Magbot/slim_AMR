#!/usr/bin/env python3
"""직진 휨 재현성 확인 — 구동륜 접지 상태 진단.

같은 지령으로 짧게 여러 번 직진시켜 휨(deg/2m)이 재현되는지 본다.
값이 매번 튀고 부호까지 뒤집히면 구동륜이 간헐적으로 접지를 잃는 것이고,
그 상태에서는 어떤 주행 보정도 측정이 불가능하다.

휨은 엔코더만으로 계산한다(라이다 정합 불필요):  theta = (d_r - d_l)/sep
"""
import argparse, math, statistics, sys, time
import rclpy, yaml
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import Twist
from safe_stop import install_safe_stop
from sensor_msgs.msg import LaserScan

ap=argparse.ArgumentParser()
ap.add_argument("--reps",type=int,default=4)
ap.add_argument("--meters",type=float,default=2.0,help="측정 주행거리[m]")
ap.add_argument("--speed",type=float,default=0.15)
ap.add_argument("--prealign",type=float,default=0.5,help="캐스터 정렬 주행거리[m]")
ap.add_argument("--params",default="/home/und_amr1/ros2_ws/src/und_amr_bringup/config/amr_params.yaml")
a=ap.parse_args()
P=yaml.safe_load(open(a.params))["base_node"]["ros__parameters"]
D,K=P["wheel_diameter"],P.get("wheel_diameter_ratio",1.0)
SEP=P["wheel_separation"]; CPR=float(P["counts_per_wheel_rev"])
RL=D*(2*K/(1+K))/2.0; RR=D*(2/(1+K))/2.0

rclpy.init()
class R(Node):
    def __init__(s):
        super().__init__("veer_repeat"); s.d={}; s.scan=None
        s.create_subscription(DiagnosticArray,"/diagnostics",s._dg,10)
        s.create_subscription(LaserScan,"/scan",lambda m:setattr(s,'scan',m),qos_profile_sensor_data)
        s.pub=s.create_publisher(Twist,"/cmd_vel",10)
    def _dg(s,m):
        cur={}
        for st in m.status:
            if st.name.startswith("motor/axis"):
                cur[int(st.name.split("axis")[-1])]={kv.key:kv.value for kv in st.values}
        if cur: s.d=cur
    def sp(s,t):
        t0=time.time()
        while time.time()-t0<t: rclpy.spin_once(s,timeout_sec=0.02)
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
    def go(s,meters,speed,RL=None,RR=None):
        """엔코더로 |meters| 만큼 갈 때까지 주행. 시간이 아니라 거리 기준이라
        사전정렬 거리가 의도대로 나온다(초 단위로 두었다가 0.18m 밖에
        안 굴러 캐스터 과도구간이 측정에 그대로 섞인 적이 있다)."""
        p0={x:int(s.d[x]["position"]) for x in s.d}
        t=Twist(); t.linear.x=speed; t0=time.time(); stop=None
        limit=abs(meters)/max(abs(speed),1e-3)+12.0
        while time.time()-t0<limit:
            s.pub.publish(t); rclpy.spin_once(s,timeout_sec=0.02)
            c=s.clr(rear=(speed<0))
            if c is not None and c<0.6: stop="장애물 %.2fm"%c; break
            cur={x:int(s.d[x]["position"]) for x in s.d}
            if 1 in cur and 2 in cur and 1 in p0 and 2 in p0:
                dd=(abs(cur[1]-p0[1])+abs(cur[2]-p0[2]))/2.0/CPR*math.pi*D
                if dd>=abs(meters): break
        for _ in range(12): s.pub.publish(Twist()); s.sp(0.05)
        s.sp(1.2)
        p1={x:int(s.d[x]["position"]) for x in s.d}
        return p1[1]-p0[1], p1[2]-p0[2], stop

n=R(); n.sp(2.5)
install_safe_stop(n)
if not n.d: print("!! /diagnostics 미수신"); sys.exit(1)
need=a.meters+a.prealign+0.9
rows=[]
try:
    for k in range(a.reps):
        f=n.clr()
        if f is None or f<need:
            print("  전방 %.2f m < 필요 %.2f m — 중단"%(f or -1,need)); break
        n.go(a.prealign,a.speed); n.sp(0.8)          # 캐스터 정렬
        d1,d2,stop=n.go(a.meters,a.speed)            # 측정
        dl=d1/CPR*2*math.pi*RL; dr=d2/CPR*2*math.pi*RR
        dist=(dl+dr)/2.0
        th=math.degrees((dr-dl)/SEP)
        per2=th/dist*2 if dist>0.05 else float("nan")
        rows.append(per2)
        print("  [%d] 좌 %+d / 우 %+d counts | 이동 %.3f m | 휨 %+.2f deg → %+.2f deg/2m%s"
              %(k+1,d1,d2,dist,th,per2," ("+stop+")" if stop else ""))
        r=n.clr(rear=True)
        back_need=dist+a.prealign+0.9
        if r and r>back_need:
            n.go(dist+a.prealign,-a.speed); n.sp(0.8)
        else:
            print("      후방 %.2f m < %.2f m — 복귀 생략"%(r or -1,back_need))
finally:
    for _ in range(12): n.pub.publish(Twist()); n.sp(0.05)

v=[x for x in rows if x==x]
print()
print("="*66)
if len(v)>=2:
    print("  휨 평균 %+.2f deg/2m   표준편차 %.2f   범위 %+.2f ~ %+.2f"
          %(statistics.mean(v),statistics.pstdev(v),min(v),max(v)))
    sign_flip = min(v)<0<max(v)
    print("  부호 뒤집힘: %s"%("있음 → 접지 불량 지속" if sign_flip else "없음"))
    print()
    print("  [기준] 어제 정상 바닥 캐스터정렬 후 : -0.81 deg/2m")
    print("  [기준] 캐스터 높이기 전            : -18.7 ~ +16.8 (부호 뒤집힘)")
    if statistics.pstdev(v)<1.5 and not sign_flip:
        print("  → 재현성 확보. 재보정 진행 가능")
    else:
        print("  → 아직 재현성 부족. 재보정해도 신뢰할 수 없음")
print("="*66)
rclpy.shutdown()
