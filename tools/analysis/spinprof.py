"""제자리 회전 방식 비교 (2026-11-23). /cmd_vel_noradius 로 직접 돈다. 서 있고 주변이 비어 있을 때만."""
import math, sys, time, threading, statistics as st
import rclpy, tf2_ros
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.executors import SingleThreadedExecutor
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
rclpy.init(); n = Node('spin_prof'); buf = tf2_ros.Buffer(); tf2_ros.TransformListener(buf, n)
pub = n.create_publisher(Twist, '/cmd_vel_noradius', 10); g = {'w': 0.0}
n.create_subscription(Imu, '/imu/base_link', lambda m: g.__setitem__('w', m.angular_velocity.z), qos_profile_sensor_data)
ex = SingleThreadedExecutor(); ex.add_node(n); threading.Thread(target=ex.spin, daemon=True).start()
def yaw():
    tr = buf.lookup_transform('odom', 'base_footprint', rclpy.time.Time()); q = tr.transform.rotation
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
def stop():
    for _ in range(5): pub.publish(Twist()); time.sleep(0.02)
def spin(deg, prof):
    wmax, mode, acc = prof
    y0 = yaw(); prev = y0; done = 0.0; tgt = math.radians(abs(deg)); d = 1 if deg > 0 else -1
    t0 = time.time(); rec = []
    while True:
        y = yaw(); dy = math.atan2(math.sin(y-prev), math.cos(y-prev)); prev = y; done += dy * d
        e = tgt - done; el = time.time() - t0
        if e < math.radians(0.7) or el > 20: break
        if mode == 'lin': cap = min(wmax, 0.08 + el * acc)
        else:
            T = wmax / acc * 1.5; x = min(1.0, el / T); cap = 0.08 + (wmax - 0.08) * x * x * (3 - 2 * x)
        w = max(-cap, min(cap, 1.5 * e)); w = max(w, 0.06)
        t = Twist(); t.angular.z = d * w; pub.publish(t)
        rec.append((el, w, g['w'] * d)); time.sleep(0.02)
    stop(); time.sleep(1.5)
    over = math.degrees(done - tgt)
    # 출렁임: 지령이 최대에 머무는 동안 실제 회전의 흔들림, 그리고 처음 넘침
    steady = [a for (el, w, a) in rec if abs(w - wmax) < 1e-3 and el > 0.3]
    peak = max(a for (_, _, a) in rec)
    sd = st.pstdev(steady) if len(steady) > 5 else float('nan')
    return el, math.degrees(peak), math.degrees(wmax), math.degrees(sd), over
profs = {'A 지금(직선가속 0.6, 0.70)': (0.70, 'lin', 0.6), 'B S자가속, 0.70': (0.70, 's', 0.6), 'C S자가속, 0.55': (0.55, 's', 0.6)}
t0 = time.time()
while time.time() - t0 < 10:
    try: yaw(); break
    except Exception: time.sleep(0.2)
res = {k: [] for k in profs}
try:
    for rep in range(2):
        for k, p in profs.items():
            for deg in (160, -160):
                r = spin(deg, p); res[k].append(r)
                print('%-28s %+4d도  %.1f초  최고 %.0f도/s (지령 %.0f)  출렁임 ±%.1f도/s  끝 %+.1f도' % ((k, deg) + r), flush=True)
finally:
    stop()
print()
for k, v in res.items():
    print('%-28s 평균 %.2f초  최고 %.0f도/s  출렁임 ±%.1f도/s  끝 지나침 %+.1f도' % (k, st.mean(x[0] for x in v), st.mean(x[1] for x in v), st.mean(x[3] for x in v), st.mean(x[4] for x in v)))
rclpy.shutdown()
