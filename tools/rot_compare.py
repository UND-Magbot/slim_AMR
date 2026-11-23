#!/usr/bin/env python3
"""제자리 회전에서 방향 추정 4종 비교.

    라이다 ICP        정답
    바퀴 오도메트리    /odom      (슬립에 오염)
    자이로 3축 적분    원시 IMU
    EKF               /odometry/filtered 와 (있으면) /odometry/test2

IMU 광학 프레임은 base_link 에 대해 rpy(-90,0,-90) 이라
로봇의 요(yaw)축은 IMU 의 -y 축에 해당한다. 세 축을 모두 적분해 확인한다.
"""
import argparse, math, statistics, sys, time
sys.path.insert(0,"/home/und_amr1/projects/UND_AMR/tools")
import numpy as np, rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from safe_stop import install_safe_stop
from scan_match import match_rotation

ap=argparse.ArgumentParser()
ap.add_argument("--rate",type=float,default=0.35)
ap.add_argument("--secs",type=float,default=3.5)
ap.add_argument("--scans",type=int,default=12)
ap.add_argument("--passive",action="store_true",
                help="모터를 쓰지 않고 사람이 손으로 돌린다. "
                     "제자리회전을 모터로 시키면 캐스터가 비틀려 바퀴가 크게 "
                     "미끄러진다(8/25: 같은 지령 998도/1183도). 윤거를 재려면 "
                     "이 모드를 쓸 것.")
ap.add_argument("--params",default="/home/und_amr1/ros2_ws/src/und_amr_bringup/config/amr_params.yaml")
ap.add_argument("--target",type=float,default=90.0,
                help="passive 에서 목표 회전각[deg]. 여기 닿으면 바로 끝낸다")
a=ap.parse_args()
def yaw(q): return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
def wrap(d):
    while d>math.pi: d-=2*math.pi
    while d<-math.pi: d+=2*math.pi
    return d

rclpy.init()
class S(Node):
    def __init__(s):
        super().__init__("rotcmp")
        s.g=None; s.scan=None; s.meta=None; s.a=None; s.b=None; s.c=None; s.o=None
        s.create_subscription(Imu,"/camera/camera/imu",lambda m:setattr(s,'g',m),qos_profile_sensor_data)
        s.create_subscription(LaserScan,"/scan",s._sc,qos_profile_sensor_data)
        s.create_subscription(Odometry,"/odometry/filtered",lambda m:setattr(s,'a',m),10)
        s.create_subscription(Odometry,"/odometry/test2",lambda m:setattr(s,'b',m),10)
        s.create_subscription(Odometry,"/odometry/test3",lambda m:setattr(s,'c',m),10)
        s.create_subscription(Odometry,"/odom",lambda m:setattr(s,'o',m),10)
        s.pub=s.create_publisher(Twist,"/cmd_vel",10)
    def _sc(s,m): s.scan=m; s.meta=m
    def sp(s,t):
        t0=time.time()
        while time.time()-t0<t: rclpy.spin_once(s,timeout_sec=0.02)
    def grab(s,n):
        s.sp(1.0); acc=None;got=0;t0=time.time()
        while got<n and time.time()-t0<12:
            s.scan=None
            while s.scan is None and time.time()-t0<12: rclpy.spin_once(s,timeout_sec=0.05)
            m=s.scan
            if acc is None: acc=[[] for _ in m.ranges]
            for i,r in enumerate(m.ranges):
                if math.isfinite(r) and m.range_min<r<m.range_max: acc[i].append(r)
            got+=1
        return np.array([statistics.median(v) if len(v)>=3 else np.nan for v in acc])
    def clr(s):
        m=s.scan
        return min((r for r in m.ranges if math.isfinite(r) and m.range_min<r<m.range_max), default=0)

n=S(); n.sp(3.0)
install_safe_stop(n)
if n.g is None or n.scan is None or n.o is None:
    print("!! 토픽 미수신"); sys.exit(1)
c=n.clr()
if c<0.55:
    print("!! 전 방향 최소 %.2f m — 제자리 회전에 부족"%c); sys.exit(1)
print("전 방향 여유 %.2f m — 진행"%c)

A=n.grab(a.scans); meta=n.meta
yo0=yaw(n.o.pose.pose.orientation)
ya0=yaw(n.a.pose.pose.orientation) if n.a else None
yb0=yaw(n.b.pose.pose.orientation) if n.b else None
yc0=yaw(n.c.pose.pose.orientation) if n.c else None
gi=[0.0,0.0,0.0]; last=time.time()
t=Twist(); t.angular.z=a.rate
if a.passive:
    print(">> [손으로] 지금부터 %.0f초 안에 로봇을 제자리에서 돌려 주세요."%a.secs)
    print("   90도 정도면 충분합니다. 다 돌리셨으면 그대로 두세요.")
else:
    print(">> 제자리 회전 %.2f rad/s x %.1f초 (약 %.0f도)"%(a.rate,a.secs,math.degrees(a.rate*a.secs)))
t0=time.time(); shown=-1.0
while time.time()-t0<a.secs:
    if not a.passive: n.pub.publish(t)
    rclpy.spin_once(n,timeout_sec=0.02)
    now=time.time(); dt=now-last; last=now
    gi[0]+=n.g.angular_velocity.x*dt; gi[1]+=n.g.angular_velocity.y*dt; gi[2]+=n.g.angular_velocity.z*dt
    if a.passive:
        # 바퀴 오도메트리로 지금까지 돈 양을 실시간으로 보여준다.
        # 얼마나 돌았는지 안 보이면 사람이 조금만 돌리고 만다(9/9: 4.2도).
        cur=abs(math.degrees(wrap(yaw(n.o.pose.pose.orientation)-yo0)))
        if cur-shown>=1.0:
            shown=cur
            sys.stdout.write("\r   돌린 각도 %5.1f / %.0f deg   남은 시간 %4.1f s   "
                             %(cur,a.target,a.secs-(now-t0)))
            sys.stdout.flush()
        if cur>=a.target:
            print("\n   목표 도달 — 그대로 두세요"); break
if not a.passive:
    for _ in range(15): n.pub.publish(Twist()); n.sp(0.05)
n.sp(2.0)
B=n.grab(a.scans)

d_od=math.degrees(wrap(yaw(n.o.pose.pose.orientation)-yo0))
(tx,ty,th),info=match_rotation(A,B,meta,lever=0.0,
                               guess_rad=math.radians(d_od),search_deg=45.0,rmax=10.0)
gt=math.degrees(th)
print()
print("="*70)
print("  라이다 ICP (정답)        %+8.2f deg   rms %.4f"%(gt,info["rms"]))
print("  바퀴 오도메트리          %+8.2f deg   오차 %+7.2f"%(d_od,d_od-gt))
print("  자이로 x 적분            %+8.2f deg"%math.degrees(gi[0]))
print("  자이로 y 적분 (부호반전)  %+8.2f deg   오차 %+7.2f   <-- 로봇 요축"
      %(math.degrees(-gi[1]),math.degrees(-gi[1])-gt))
print("  자이로 z 적분            %+8.2f deg"%math.degrees(gi[2]))
if n.a: print("  기존 EKF                %+8.2f deg   (yaw공분산 %.1f)"
              %(math.degrees(wrap(yaw(n.a.pose.pose.orientation)-ya0)),n.a.pose.covariance[35]))
if n.b: print("  EKF2 쿼터니언만 보정      %+8.2f deg   (yaw공분산 %.1f)"
              %(math.degrees(wrap(yaw(n.b.pose.pose.orientation)-yb0)),n.b.pose.covariance[35]))
if n.c: print("  EKF3 base_link 축변환     %+8.2f deg   (yaw공분산 %.1f)   <-- TF 불필요"
              %(math.degrees(wrap(yaw(n.c.pose.pose.orientation)-yc0)),n.c.pose.covariance[35]))

# 윤거 역산 — 오도메트리 회전량은 윤거에 반비례한다.
#   yaw_odo = (d_r - d_l) / sep   이므로  sep_true = sep_now * yaw_odo / yaw_true
if abs(gt) < 5.0:
    print("  회전량이 %.1f도로 너무 작아 윤거를 역산하지 않는다 (30도 이상 권장)"%abs(gt))
else:
    try:
        import yaml
        sep=yaml.safe_load(open(a.params))["base_node"]["ros__parameters"]["wheel_separation"]
        sep_t=sep*d_od/gt
        print("  현재 wheel_separation %.4f m  ->  역산값 %.4f m  (%+.2f %%)"
              %(sep,sep_t,(sep_t/sep-1)*100))
        if not a.passive:
            print("  ⚠ 모터 회전은 캐스터 비틀림으로 슬립이 섞인다. --passive 로 다시 잴 것")
        elif abs(info["rms"])>0.05:
            print("  ⚠ ICP rms %.4f 로 정합이 나쁘다. 주변에 벽이 보이는 곳에서 다시 잴 것"%info["rms"])
        else:
            print("  -> 이 값을 amr_params.yaml 의 wheel_separation 에 넣으면 된다")
    except Exception as e:
        print("  윤거 역산 실패: %s"%e)
print("="*70)
rclpy.shutdown()
