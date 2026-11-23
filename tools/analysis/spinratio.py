import math, time, threading, statistics as st
import rclpy, tf2_ros
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.executors import SingleThreadedExecutor
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
rclpy.init(); n = Node('spin_ratio'); buf = tf2_ros.Buffer(); tf2_ros.TransformListener(buf, n)
pub = n.create_publisher(Twist, '/cmd_vel_noradius', 10); s = {'g': 0.0, 'wh': 0.0}
n.create_subscription(Imu, '/imu/base_link', lambda m: s.__setitem__('g', m.angular_velocity.z), qos_profile_sensor_data)
n.create_subscription(Odometry, '/odom', lambda m: s.__setitem__('wh', m.twist.twist.angular.z), 20)
ex = SingleThreadedExecutor(); ex.add_node(n); threading.Thread(target=ex.spin, daemon=True).start()
def yaw():
    tr = buf.lookup_transform('odom', 'base_footprint', rclpy.time.Time()); q = tr.transform.rotation
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
def stop():
    for _ in range(5): pub.publish(Twist()); time.sleep(0.02)
def spin(deg, wmax=0.7, acc=0.6):
    prev = yaw(); done = 0.0; tgt = math.radians(abs(deg)); d = 1 if deg > 0 else -1; t0 = time.time(); rec = []; ig = iw = 0.0; tl = time.time()
    while True:
        y = yaw(); dy = math.atan2(math.sin(y-prev), math.cos(y-prev)); prev = y; done += dy * d
        e = tgt - done; el = time.time() - t0
        if e < math.radians(0.7) or el > 20: break
        cap = min(wmax, 0.08 + el * acc); w = max(0.06, min(cap, 1.5 * e))
        t = Twist(); t.angular.z = d * w; pub.publish(t)
        now = time.time(); ig += s['g'] * (now - tl); iw += s['wh'] * (now - tl); tl = now
        rec.append((el, w, s['g'] * d, s['wh'] * d)); time.sleep(0.02)
    stop()
    t1 = time.time()
    while time.time() - t1 < 1.5:
        now = time.time(); ig += s['g'] * (now - tl); iw += s['wh'] * (now - tl); tl = now; time.sleep(0.01)
    steady = [r for r in rec if abs(r[1] - wmax) < 1e-3 and r[0] > 1.5]
    return (math.degrees(abs(ig)), math.degrees(abs(iw)),
            st.mean(r[2] for r in steady) / wmax, st.mean(r[3] for r in steady) / wmax,
            max(r[2] for r in rec) / wmax, max(r[3] for r in rec) / wmax)
t0 = time.time()
while time.time() - t0 < 10:
    try: yaw(); break
    except Exception: time.sleep(0.2)
try:
    for deg in (180, -180, 180, -180):
        r = spin(deg)
        print('%+4d도: 총각 자이로 %.1f 바퀴 %.1f (비 %.3f) | 정속 평균 자이로/지령 %.3f 바퀴/지령 %.3f | 최고 자이로/지령 %.2f 바퀴/지령 %.2f' % ((deg, r[0], r[1], r[1] / r[0]) + r[2:]), flush=True)
finally:
    stop()
rclpy.shutdown()
