"""드라이브 속도 이득별 제자리 회전 출렁임 시험 (2026-11-16, 사용자 허락)."""
import os, sys, time, math, json, urllib.request
import numpy as np, rclpy
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu, LaserScan
sys.path.insert(0, os.path.expanduser("~/projects/UND_AMR/tools"))
from fw_cmd import crc_line
LOG = os.path.expanduser("~/projects/UND_AMR/logs/bringup.log")

def fw(cmds):
    with open(LOG, "rb") as f: n0 = sum(1 for _ in f)
    fd = os.open("/dev/ttyUSB0", os.O_WRONLY | os.O_NOCTTY)
    for c in cmds:
        os.write(fd, ("\r\n" + crc_line(c) + "\r\n").encode()); time.sleep(0.4)
    os.close(fd); time.sleep(1.2)
    with open(LOG, "rb") as f: lines = f.read().decode("utf-8", "replace").splitlines()[n0:]
    return [l.split("펌웨어: ", 1)[1].replace("> ", "") for l in lines if "펌웨어: " in l]

def busy():
    try:
        for s in ("nav", "job", "charger", "route"):
            st = json.load(urllib.request.urlopen("http://localhost:8090/%s/status" % s, timeout=2))["state"]
            if st in ("active", "sending", "moving", "backing", "aligning", "going", "lifting", "staging", "entering", "exiting", "looking"):
                return s + ":" + st
    except Exception as e:
        return "api? %s" % e
    return None

rclpy.init(); n = rclpy.create_node("gainspin")
pub = n.create_publisher(Twist, "/cmd_vel_noradius", 10)
imu = []; scan = {}
n.create_subscription(Imu, "/imu/base_link", lambda m: imu.append((time.time(), m.angular_velocity.z)), qos_profile_sensor_data)
n.create_subscription(LaserScan, "/scan", lambda m: scan.__setitem__("s", m), qos_profile_sensor_data)

def spin_for(sec):
    t = time.time()
    while time.time() - t < sec: rclpy.spin_once(n, timeout_sec=0.01)

def clear():
    m = scan.get("s")
    if m is None: return False
    r = np.array(m.ranges); ok = np.isfinite(r) & (r > m.range_min) & (r < m.range_max)
    return r[ok].min() > 0.95

def rotate(w, ang):
    """w 로 ang [rad] 돌기 (IMU 적분). 기록 (t, wz) 반환."""
    imu.clear(); t0 = time.time(); a = 0.0; last = t0; rec = []
    while abs(a) < ang - math.radians(4):
        b = busy()
        if b: raise SystemExit("다른 이동 시작(%s) — 중단" % b)
        ramp = min(1.0, (time.time() - t0) / 1.0)
        t = Twist(); t.angular.z = w * max(0.35, ramp); pub.publish(t)
        spin_for(0.05)
        while imu:
            ti, wz = imu.pop(0); a += wz * (ti - last) if ti > last else 0.0; last = ti; rec.append((ti, wz))
        if time.time() - t0 > 20: break
    for _ in range(20):
        pub.publish(Twist()); spin_for(0.05)
    return np.array(rec)

def metric(rec):
    if len(rec) < 50: return None
    t = rec[:, 0] - rec[0, 0]; T = t[-1]; m = (t > 0.25 * T) & (t < 0.85 * T)
    x = rec[m, 1]; k = 10; sm = np.convolve(x, np.ones(k) / k, "valid")
    return abs(sm.mean()), sm.std(), sm.min(), sm.max()

spin_for(2.0)
if busy(): raise SystemExit("이동 중 — 시작 안 함")
sets = [(35, 64), (35, 32), (35, 16), (60, 32)]
res = []
try:
    for kp, ki in sets:
        out = fw(["k 0 %d %d" % (kp, ki), "q"])
        g = [l for l in out if "speed_kp" in l]
        print("이득 %d/%d 적용:" % (kp, ki), g[0][g[0].find("speed_kp"):].split(" iq")[0] if g else out[-3:])
        spin_for(1.5)
        if not clear(): raise SystemExit("0.95m 안 장애물 — 중단")
        rl = rotate(+0.30, math.radians(90)); spin_for(1.5)
        if not clear(): raise SystemExit("0.95m 안 장애물 — 중단")
        rr = rotate(-0.30, math.radians(90)); spin_for(1.5)
        iq = [l[l.find("iq="):] for l in fw(["q"]) if "iq=" in l]
        ml, mr = metric(rl), metric(rr)
        res.append((kp, ki, ml, mr, iq))
        print("   왼쪽: 평균 %.3f 출렁임 ±%.3f (%.3f~%.3f) | 오른쪽: 평균 %.3f ±%.3f (%.3f~%.3f) | 정지 전류 %s" % (ml + mr + (" ".join(iq),)))
finally:
    for _ in range(10):
        pub.publish(Twist()); spin_for(0.05)
    out = fw(["k 0 35 64", "q"])
    g = [l for l in out if "speed_kp" in l]
    print("원래 이득으로 되돌림:", [x[x.find("speed_kp"):].split(" iq")[0] for x in g])
