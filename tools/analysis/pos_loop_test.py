"""드라이브 위치 고리 이득별 제자리 회전 출렁임 (2026-11-16, 사용자: 로봇 넓은 곳에 옮겨 둠).
각 설정마다 +0.30 rad/s 180도, -0.30 rad/s 180도. 끝나면 위치이득 10/앞먹임 256 으로 되돌린다."""
import time, math, json, subprocess, urllib.request, numpy as np, rclpy, sys
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu, LaserScan
from diagnostic_msgs.msg import DiagnosticArray
SETS = [(10, 256), (0, 256), (30, 256), (10, 256)]
rclpy.init(); n = rclpy.create_node("posloop")
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
        if st in ("active", "sending", "moving", "backing", "aligning", "going", "lifting", "staging", "entering", "exiting", "looking", "maneuver", "leaving"):
            return s
def clear():
    m = scan.get("s"); r = np.array(m.ranges); ok = np.isfinite(r) & (r > m.range_min) & (r < m.range_max)
    return r[ok].min()
def fw(cmd):
    subprocess.run(["python3", "/home/und_amr1/projects/UND_AMR/tools/fw_cmd.py", cmd, "sleep:0.8"], capture_output=True, timeout=20)
def rotate(w, ang):
    t0 = time.time(); a = 0.0; last = None; i0 = len(imu)
    while abs(a) < ang - math.radians(4):
        if busy(): stop(); raise SystemExit("다른 이동 — 중단")
        t = Twist(); t.angular.z = w * max(0.35, min(1.0, (time.time() - t0) / 1.0)); pub.publish(t)
        spin_for(0.05)
        for ti, wz in imu[i0:]:
            if last is not None: a += wz * (ti - last)
            last = ti
        i0 = len(imu)
        if time.time() - t0 > 25: break
    t1 = time.time(); stop()
    return t0, t1
def stop():
    for _ in range(20): pub.publish(Twist()); spin_for(0.05)
spin_for(2.0)
d0 = clear()
print("가장 가까운 점 %.2fm" % d0)
if busy() or d0 < 0.75: raise SystemExit("이동 중이거나 0.75m 안 장애물 — 시작 안 함")
segs = []
try:
    for kp, ff in SETS:
        fw("g 0 %d %d" % (kp, ff)); spin_for(1.0)
        for sgn in (1, -1):
            if clear() < 0.75: raise SystemExit("0.75m 안 장애물 — 중단")
            segs.append(((kp, ff), 0.30 * sgn) + rotate(0.30 * sgn, math.radians(180))); spin_for(1.5)
finally:
    stop(); fw("g 0 10 256")
D = np.array(diag); I = np.array(imu)
print("설정            회전     IMU w(평균±표준편차)  출렁 %   주 주파수  바퀴1 rpm(±)      바퀴2 rpm(±)      전류1±   전류2±")
for (kp, ff), w, t0, t1 in segs:
    lo, hi = t0 + 0.2 * (t1 - t0), t0 + 0.9 * (t1 - t0)
    im = I[(I[:, 0] >= lo) & (I[:, 0] <= hi)]
    tt = np.arange(lo, hi, 0.01); ww = np.interp(tt, im[:, 0], im[:, 1]); x = ww - ww.mean()
    F = np.abs(np.fft.rfft(x * np.hanning(len(x)))); f = np.fft.rfftfreq(len(x), 0.01); s = (f > 0.3) & (f < 8)
    wh = []
    for ax in (1, 2):
        d = D[(D[:, 1] == ax) & (D[:, 0] >= lo) & (D[:, 0] <= hi)]
        t = d[:, 0]; p = d[:, 2]; k = 3
        v = (p[k:] - p[:-k]) / (t[k:] - t[:-k]) / 4096.0 * 60.0
        wh.append((np.abs(v).mean(), v.std(), d[:, 5].std()))
    print("kp %3d ff %3d  %+.2f   %.3f±%.3f            %4.0f%%   %.2f Hz    %.2f±%.2f        %.2f±%.2f        %.2f    %.2f" % (
        kp, ff, w, abs(ww.mean()), ww.std(), 100 * ww.std() / abs(ww.mean()), f[s][np.argmax(F[s])],
        wh[0][0], wh[0][1], wh[1][0], wh[1][1], wh[0][2], wh[1][2]))
np.save("/home/und_amr1/projects/UND_AMR/logs/pos_loop_diag.npy", D); np.save("/home/und_amr1/projects/UND_AMR/logs/pos_loop_imu.npy", I)
