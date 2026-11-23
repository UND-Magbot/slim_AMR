import rclpy, time, sys
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from tf2_msgs.msg import TFMessage
rclpy.init(); n=Node('gap_mon'); last={}; out=open(sys.argv[1],'a')
def note(k):
    now=time.time(); p=last.get(k)
    if p and now-p>0.3:
        out.write('%.3f %s 간격 %.2f초\n'%(now,k,now-p)); out.flush()
    last[k]=now
def on_tf(m):
    for t in m.transforms:
        if t.header.frame_id=='odom' and t.child_frame_id=='base_footprint': note('tf odom->base_footprint')
        if t.header.frame_id=='map' and t.child_frame_id=='odom': note('tf map->odom')
n.create_subscription(TFMessage,'/tf',on_tf,QoSProfile(depth=100,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.VOLATILE))
n.create_subscription(Odometry,'/odometry/filtered',lambda m: note('odometry/filtered'),20)
n.create_subscription(Odometry,'/odom',lambda m: note('odom'),20)
n.create_subscription(Imu,'/imu/base_link',lambda m: note('imu'),qos_profile_sensor_data)
end=time.time()+float(sys.argv[2])
while time.time()<end: rclpy.spin_once(n,timeout_sec=0.1)
