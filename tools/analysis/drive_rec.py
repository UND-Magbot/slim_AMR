"""주행 S자 분석용 기록 (2026-11-16): /cmd_vel(nav2→base), /cmd_vel_noradius, /odom, IMU, map 자세 20Hz."""
import time, sys, math, rclpy
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import Imu
import tf2_ros
rclpy.init(); n=rclpy.create_node("driverec"); f=open(sys.argv[1],"w"); f.write("t,src,a,b,c\n")
buf=tf2_ros.Buffer(); tl=tf2_ros.TransformListener(buf,n)
n.create_subscription(Twist,"/cmd_vel",lambda m: f.write("%.3f,cmd,%.4f,%.4f,0\n"%(time.time(),m.linear.x,m.angular.z)),50)
n.create_subscription(Twist,"/cmd_vel_noradius",lambda m: f.write("%.3f,cmdnr,%.4f,%.4f,0\n"%(time.time(),m.linear.x,m.angular.z)),50)
n.create_subscription(Odometry,"/odom",lambda m: f.write("%.3f,odom,%.4f,%.4f,0\n"%(time.time(),m.twist.twist.linear.x,m.twist.twist.angular.z)),50)
cnt={"i":0}
def imu(m):
    cnt["i"]+=1
    if cnt["i"]%4==0: f.write("%.3f,imu,0,%.4f,0\n"%(time.time(),m.angular_velocity.z))
n.create_subscription(Imu,"/imu/base_link",imu,qos_profile_sensor_data)
from geometry_msgs.msg import PoseWithCovarianceStamped
def _sp(m):
    q=m.pose.pose.orientation
    f.write("%.3f,slam,%.4f,%.4f,%.4f\n"%(time.time(),m.pose.pose.position.x,m.pose.pose.position.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))))
n.create_subscription(PoseWithCovarianceStamped,"/pose",_sp,10)
n.create_subscription(Path,"/plan",lambda m: f.write("%.3f,plan,%d,%.3f,%.3f\n"%(time.time(),len(m.poses),m.poses[-1].pose.position.x if m.poses else 0,m.poses[-1].pose.position.y if m.poses else 0)),5)
t0=time.time(); last=0
while time.time()-t0<float(sys.argv[2]):
    rclpy.spin_once(n,timeout_sec=0.01)
    if time.time()-last>=0.05:
        last=time.time()
        try:
            tr=buf.lookup_transform("map","base_footprint",rclpy.time.Time()); q=tr.transform.rotation
            f.write("%.3f,pose,%.4f,%.4f,%.4f\n"%(last,tr.transform.translation.x,tr.transform.translation.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))))
        except Exception: pass
f.close()
