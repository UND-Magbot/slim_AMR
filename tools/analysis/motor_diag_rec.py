"""축별 드라이브 상태(알람·전압·전류·지령/실제 rpm) 를 진단 주기마다 기록 (2026-11-18, 축1 알람 0x4000 원인 찾기)."""
import sys, time, rclpy
from diagnostic_msgs.msg import DiagnosticArray
rclpy.init(); n = rclpy.create_node("motordiagrec")
f = open(sys.argv[1], "w"); f.write("t,axis,online,enabled,error,voltage,iq,cmd_rpm,act_rpm\n")
def cb(m):
    t = time.time()
    for s in m.status:
        if s.name.startswith("motor/axis"):
            kv = {k.key: k.value for k in s.values}
            f.write("%.3f,%s,%s,%s,%s,%s,%s,%s,%s\n" % (t, s.name[-1], kv.get("online"), kv.get("enabled"), kv.get("error"),
                    kv.get("voltage"), kv.get("iq"), kv.get("cmd_rpm"), kv.get("act_rpm")))
n.create_subscription(DiagnosticArray, "/diagnostics", cb, 50)
t0 = time.time()
while time.time() - t0 < float(sys.argv[2]):
    rclpy.spin_once(n, timeout_sec=0.2)
f.close()
