import rclpy, time, sys, csv, math, threading
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.executors import SingleThreadedExecutor
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
rclpy.init(); n = Node('wob_rec')
s = {'v': 0.0, 'cmd': 0.0, 'cmd_t': 0.0, 'cv': 0.0, 'g': 0.0, 'yaw': 0.0}
def yaw_of(q): return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
def on_ekf(m): s['v'] = m.twist.twist.linear.x; s['yaw'] = yaw_of(m.pose.pose.orientation)
def on_cmd(m): s['cmd'], s['cv'], s['cmd_t'] = m.angular.z, m.linear.x, time.time()
n.create_subscription(Odometry, '/odometry/filtered', on_ekf, 50)
n.create_subscription(Twist, '/cmd_vel', on_cmd, 50)
n.create_subscription(Twist, '/cmd_vel_noradius', on_cmd, 50)
n.create_subscription(Imu, '/imu/base_link', lambda m: s.__setitem__('g', m.angular_velocity.z), qos_profile_sensor_data)
ex = SingleThreadedExecutor(); ex.add_node(n); threading.Thread(target=ex.spin, daemon=True).start()
t0 = time.time(); nxt = t0
with open(sys.argv[1], 'w', newline='') as f:
    w = csv.writer(f); w.writerow(['t', 'v', 'cmd_v', 'cmd_w', 'gyro', 'yaw'])
    while time.time() - t0 < float(sys.argv[2]):
        time.sleep(0.004)
        if time.time() >= nxt:
            nxt += 0.02; fresh = time.time() - s['cmd_t'] < 0.5
            w.writerow(['%.3f' % time.time(), '%.4f' % s['v'], '%.4f' % (s['cv'] if fresh else 0), '%.4f' % (s['cmd'] if fresh else 0), '%.4f' % s['g'], '%.5f' % s['yaw']])
            f.flush()
