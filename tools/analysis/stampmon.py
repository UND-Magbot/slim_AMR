import rclpy, time, sys
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from tf2_msgs.msg import TFMessage
rclpy.init(); n=Node('stamp_mon'); out=open(sys.argv[1],'a'); mx={}
def age(k, h):
    now=time.time(); a=now-(h.stamp.sec+h.stamp.nanosec*1e-9)
    if abs(a)>0.4: out.write('%.3f %s 시각차 %+.3f초\n'%(now,k,a)); out.flush()
    m=mx.setdefault(k,[9,-9]); m[0]=min(m[0],a); m[1]=max(m[1],a)
def on_tf(m):
    for t in m.transforms:
        if t.header.frame_id=='odom' and t.child_frame_id=='base_footprint': age('tf odom->base',t.header)
        if t.header.frame_id=='map' and t.child_frame_id=='odom': age('tf map->odom',t.header)
n.create_subscription(TFMessage,'/tf',on_tf,QoSProfile(depth=100,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.VOLATILE))
n.create_subscription(Odometry,'/odometry/filtered',lambda m: age('odometry/filtered',m.header),20)
n.create_subscription(Odometry,'/odom',lambda m: age('odom',m.header),20)
n.create_subscription(Imu,'/imu/base_link',lambda m: age('imu',m.header),qos_profile_sensor_data)
end=time.time()+float(sys.argv[2]); nxt=time.time()+30
while time.time()<end:
    rclpy.spin_once(n,timeout_sec=0.1)
    if time.time()>nxt:
        out.write('%.3f 30초 범위 %s\n'%(time.time(),' | '.join('%s %+.2f~%+.2f'%(k,v[0],v[1]) for k,v in mx.items()))); out.flush(); mx.clear(); nxt+=30
