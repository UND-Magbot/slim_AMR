import time, sys, rclpy
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
rclpy.init(); n=rclpy.create_node("turnrec"); f=open(sys.argv[1],"w"); f.write("t,src,v,w\n")
def w(src):
    def cb(m):
        if src in ("cmd_vel","noradius"): f.write("%.3f,%s,%.4f,%.4f\n"%(time.time(),src,m.linear.x,m.angular.z))
        elif src=="odom": f.write("%.3f,odom,%.4f,%.4f\n"%(time.time(),m.twist.twist.linear.x,m.twist.twist.angular.z))
        else: f.write("%.3f,imu,0,%.4f\n"%(time.time(),m.angular_velocity.z))
    return cb
n.create_subscription(Twist,"/cmd_vel",w("cmd_vel"),50)
n.create_subscription(Twist,"/cmd_vel_noradius",w("noradius"),50)
n.create_subscription(Odometry,"/odom",w("odom"),50)
n.create_subscription(Imu,"/imu/base_link",w("imu"),qos_profile_sensor_data)
t0=time.time()
while time.time()-t0<float(sys.argv[2]): rclpy.spin_once(n,timeout_sec=0.05)
f.close()
