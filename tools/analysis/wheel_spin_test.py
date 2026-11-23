"""제자리 회전 중 바퀴별 속도·전류 기록 (2026-11-16, 사용자 허락)."""
import time, math, json, urllib.request, numpy as np, rclpy
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu, LaserScan
from diagnostic_msgs.msg import DiagnosticArray
rclpy.init(); n = rclpy.create_node("wheelspin")
pub = n.create_publisher(Twist, "/cmd_vel_noradius", 10)
imu = []; diag = []; scan = {}
n.create_subscription(Imu, "/imu/base_link", lambda m: imu.append((time.time(), m.angular_velocity.z)), qos_profile_sensor_data)
n.create_subscription(LaserScan, "/scan", lambda m: scan.__setitem__("s", m), qos_profile_sensor_data)
def on_diag(m):
    t = time.time()
    for s in m.status:
        if s.name.startswith("motor/axis"):
            kv = {k.key: k.value for k in s.values}
            diag.append((t, int(s.name[-1]), float(kv["position"]), float(kv["cmd_rpm"]), float(kv["act_rpm"]), float(kv.get("iq") or 0)))
n.create_subscription(DiagnosticArray, "/diagnostics", on_diag, 50)
def spin_for(sec):
    t = time.time()
    while time.time() - t < sec: rclpy.spin_once(n, timeout_sec=0.01)
def busy():
    for s in ("nav", "job", "charger", "route"):
        st = json.load(urllib.request.urlopen("http://localhost:8090/%s/status" % s, timeout=2))["state"]
        if st in ("active", "sending", "moving", "backing", "aligning", "going", "lifting", "staging", "entering", "exiting", "looking"):
            return s
def clear():
    m = scan.get("s"); r = np.array(m.ranges); ok = np.isfinite(r) & (r > m.range_min) & (r < m.range_max)
    return r[ok].min() > 0.95
def rotate(w, ang):
    t0 = time.time(); a = 0.0; last = None; i0 = len(imu)
    while abs(a) < ang - math.radians(4):
        if busy(): raise SystemExit("다른 이동 — 중단")
        t = Twist(); t.angular.z = w * max(0.35, min(1.0, (time.time() - t0) / 1.0)); pub.publish(t)
        spin_for(0.05)
        for ti, wz in imu[i0:]:
            if last is not None: a += wz * (ti - last)
            last = ti
        i0 = len(imu)
        if time.time() - t0 > 20: break
    t1 = time.time()
    for _ in range(20): pub.publish(Twist()); spin_for(0.05)
    return t0, t1
spin_for(2.0)
if busy() or not clear(): raise SystemExit("이동 중이거나 0.95m 안 장애물 — 시작 안 함")
segs = []
for w in (0.30, 0.60):
    for sgn in (1, -1):
        if not clear(): raise SystemExit("0.95m 안 장애물 — 중단")
        segs.append((w * sgn,) + rotate(w * sgn, math.radians(90))); spin_for(1.5)
D = np.array(diag); I = np.array(imu)
for w, t0, t1 in segs:
    lo, hi = t0 + 0.25 * (t1 - t0), t0 + 0.85 * (t1 - t0)
    out = []
    sp = {}
    for ax in (1, 2):
        d = D[(D[:, 1] == ax) & (D[:, 0] >= lo) & (D[:, 0] <= hi)]
        # 위치 차분 -> rpm (4096 counts/rev)
        t = d[:, 0]; p = d[:, 2]
        k = 3
        v = (p[k:] - p[:-k]) / (t[k:] - t[:-k]) / 4096.0 * 60.0
        sp[ax] = (t[k:] + t[:-k]) / 2, v
        out.append("축%d 명령 %.2f rpm, 실제 %.2f±%.2f rpm (%.0f%%), 전류 %.2f±%.2f A" % (ax, np.median(d[:, 3]), np.abs(v).mean(), v.std(), 100 * v.std() / max(1e-6, np.abs(v).mean()), d[:, 5].mean(), d[:, 5].std()))
    tt = np.linspace(lo + 0.3, hi - 0.3, 200)
    a1 = np.interp(tt, *sp[1]); a2 = np.interp(tt, *sp[2])
    c = np.corrcoef(np.abs(a1) - np.abs(a1).mean(), np.abs(a2) - np.abs(a2).mean())[0, 1]
    im = I[(I[:, 0] >= lo) & (I[:, 0] <= hi)]
    print("회전 %+.2f rad/s: %s | 두 바퀴 속도 출렁임 상관 %+.2f | IMU %.3f±%.3f" % (w, " / ".join(out), c, abs(im[:, 1].mean()), im[:, 1].std()))
np.save("/home/und_amr1/projects/UND_AMR/logs/wheel_spin_diag.npy", D); np.save("/home/und_amr1/projects/UND_AMR/logs/wheel_spin_imu.npy", I)
