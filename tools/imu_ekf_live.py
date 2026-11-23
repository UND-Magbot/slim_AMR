#!/usr/bin/env python3
"""자이로 → EKF 경로 점검. 로봇을 손으로 돌리면서 세 값을 같이 본다.

    gyro          IMU 원시 각속도 (센서 프레임)
    EKF yaw       /odometry/filtered 의 방향
    바퀴 yaw      /odom 의 방향

로봇을 손으로 90도쯤 돌렸을 때 EKF yaw 가 따라오면 융합이 동작하는 것이고,
그대로 0 이면 IMU 가 필터에 반영되지 않는 것이다.
축이 해제된 상태에서 실행할 것.
"""
import argparse, math, time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry

ap=argparse.ArgumentParser(); ap.add_argument("--secs",type=float,default=20.0)
a=ap.parse_args()

def yaw(q): return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))

rclpy.init()
class S(Node):
    def __init__(s):
        super().__init__("imu_ekf_live"); s.g=None; s.ek=None; s.od=None
        s.create_subscription(Imu,"/camera/camera/imu",lambda m:setattr(s,'g',m),qos_profile_sensor_data)
        s.create_subscription(Odometry,"/odometry/filtered",lambda m:setattr(s,'ek',m),10)
        s.create_subscription(Odometry,"/odom",lambda m:setattr(s,'od',m),10)
    def sp(s,t):
        t0=time.time()
        while time.time()-t0<t: rclpy.spin_once(s,timeout_sec=0.02)
n=S(); n.sp(2.5)
if n.g is None or n.ek is None:
    print("!! IMU 또는 EKF 미수신"); raise SystemExit(1)
e0=yaw(n.ek.pose.pose.orientation); o0=yaw(n.od.pose.pose.orientation)
gi={"x":0.0,"y":0.0,"z":0.0}
print("="*72)
print("  준비하세요. 카운트다운 후 %.0f초간 측정합니다."%a.secs)
for c in (5,4,3,2,1):
    print("   %d ..."%c, flush=True); n.sp(1.0)
print("  >>> 지금 돌리세요! 제자리에서 손으로 90도쯤 천천히 <<<", flush=True)
print("="*72)
e0=yaw(n.ek.pose.pose.orientation); o0=yaw(n.od.pose.pose.orientation)
print("   시각   gyro x/y/z [rad/s]        적분 x/y/z [deg]        EKF yaw    바퀴 yaw")
last=time.time(); t0=time.time()
while time.time()-t0<a.secs:
    n.sp(0.25)
    now=time.time(); dt=now-last; last=now
    if n.g:
        gi["x"]+=n.g.angular_velocity.x*dt
        gi["y"]+=n.g.angular_velocity.y*dt
        gi["z"]+=n.g.angular_velocity.z*dt
    ek=math.degrees(yaw(n.ek.pose.pose.orientation)-e0)
    od=math.degrees(yaw(n.od.pose.pose.orientation)-o0)
    print("  %5.1fs  %+6.3f %+6.3f %+6.3f   %+7.1f %+7.1f %+7.1f   %+8.2f°  %+8.2f°"
          %(now-t0,n.g.angular_velocity.x,n.g.angular_velocity.y,n.g.angular_velocity.z,
            math.degrees(gi["x"]),math.degrees(gi["y"]),math.degrees(gi["z"]),ek,od))
print()
print("="*72)
print("  자이로 적분 총합: x %+.1f°  y %+.1f°  z %+.1f°"
      %(math.degrees(gi["x"]),math.degrees(gi["y"]),math.degrees(gi["z"])))
print("  EKF yaw 변화 %+.2f° / 바퀴 yaw 변화 %+.2f°"%(ek,od))
big=max(abs(gi["x"]),abs(gi["y"]),abs(gi["z"]))
ax=[k for k in gi if abs(gi[k])==big][0]
print("  로봇 yaw 에 해당하는 IMU 축: %s (적분 %+.1f°)"%(ax,math.degrees(gi[ax])))
if abs(ek)<0.5*abs(math.degrees(gi[ax])):
    print("  → EKF 가 자이로를 반영하지 못하고 있다")
else:
    print("  → EKF 가 자이로를 정상 반영 중")
print("="*72)
rclpy.shutdown()
