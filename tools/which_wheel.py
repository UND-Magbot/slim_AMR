#!/usr/bin/env python3
"""어느 축이 어느 쪽 바퀴인지 눈으로 확인한다.

한 번에 한 바퀴만 돌린다. 사람이 보고 답하면 끝이다.
지도나 오도메트리로 역산하는 방법은 '양쪽이 다 정상일 때만' 맞는다.
한쪽이 고장나 있으면 그 추론이 통째로 틀어진다(9/11 에 그렇게 됐다).

원리: 차동구동에서 v = w*sep/2 로 주면 한쪽 바퀴 속도가 정확히 0 이 된다.
      base_node 의 정상 경로를 그대로 타므로 설정이 틀렸으면 틀린 대로 보인다.

    python3 tools/which_wheel.py

로봇이 서 있는 바퀴를 축으로 천천히 돕니다. 옆으로 1 m 쯤 여유를 두세요.
바퀴를 띄워 놓고 해도 됩니다(그 편이 안전합니다).
"""
import math
import sys
import time

import rclpy
import yaml
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_srvs.srv import Trigger

sys.path.insert(0, "/home/und_amr1/projects/UND_AMR/tools")
from safe_stop import install_safe_stop          # noqa: E402

PARAMS = "/home/und_amr1/ros2_ws/src/und_amr_bringup/config/amr_params.yaml"
W = 0.5          # 각속도 [rad/s]
SECS = 4.0       # 한쪽당 돌리는 시간


class N(Node):
    def __init__(s):
        super().__init__("which_wheel")
        s.enc = {}
        s.pub = s.create_publisher(Twist, "/cmd_vel", 10)
        s.create_subscription(DiagnosticArray, "/diagnostics", s._dg, 10)
        s.cli = s.create_client(Trigger, "/base_node/enable")

    def _dg(s, m):
        for st in m.status:
            if not st.name.startswith("motor/axis"):
                continue
            ax = int(st.name.split("axis")[-1])
            for kv in st.values:
                if kv.key == "position":
                    s.enc[ax] = int(kv.value)

    def sp(s, t):
        t0 = time.time()
        while time.time() - t0 < t:
            rclpy.spin_once(s, timeout_sec=0.02)

    def spin_one(s, v, w):
        t = Twist()
        t.linear.x = v
        t.angular.z = w
        e0 = dict(s.enc)
        t0 = time.time()
        while time.time() - t0 < SECS:
            s.pub.publish(t)
            rclpy.spin_once(s, timeout_sec=0.02)
        for _ in range(12):
            s.pub.publish(Twist())
            s.sp(0.05)
        s.sp(1.0)
        e1 = dict(s.enc)
        return (e1.get(1, 0) - e0.get(1, 0)), (e1.get(2, 0) - e0.get(2, 0))


def main():
    rclpy.init()
    n = N()
    install_safe_stop(n)
    n.sp(3.0)
    if not n.enc:
        print("!! /diagnostics 미수신 — base_node 확인")
        return
    if not n.cli.wait_for_service(timeout_sec=5.0):
        print("!! /base_node/enable 없음")
        return
    cur = yaml.safe_load(open(PARAMS))["base_node"]["ros__parameters"]
    sep = cur["wheel_separation"]
    v = W * sep / 2.0          # 이 값이면 한쪽 바퀴 지령이 정확히 0 이 된다

    print("=" * 62)
    print("  현재 설정  좌=축%d(%+.0f)  우=축%d(%+.0f)"
          % (cur["left_axis"], cur["left_sign"],
             cur["right_axis"], cur["right_sign"]))
    print("  한 번에 한 바퀴만 돌립니다. 어느 쪽이 도는지 보세요.")
    print("  (로봇은 서 있는 바퀴를 축으로 천천히 돕니다)")
    print("=" * 62)
    n.cli.call_async(Trigger.Request())
    n.sp(2.5)

    print("\n>> 1) 설정상 '오른쪽' 바퀴만 돌립니다 (%.1f초)" % SECS)
    print("   ... 지금 보세요")
    d1a, d2a = n.spin_one(v, W)
    print("   엔코더  축1 %+d  /  축2 %+d" % (d1a, d2a))

    n.sp(2.0)
    print("\n>> 2) 설정상 '왼쪽' 바퀴만 돌립니다 (%.1f초)" % SECS)
    print("   ... 지금 보세요")
    d1b, d2b = n.spin_one(v, -W)
    print("   엔코더  축1 %+d  /  축2 %+d" % (d1b, d2b))

    print("\n" + "=" * 62)
    print("  1) 에서 돈 축: %s" % ("축1" if abs(d1a) > abs(d2a) else "축2"))
    print("  2) 에서 돈 축: %s" % ("축1" if abs(d1b) > abs(d2b) else "축2"))
    print()
    print("  1) 에서 실제로 '오른쪽' 바퀴가 돌았고")
    print("  2) 에서 실제로 '왼쪽' 바퀴가 돌았으면 설정이 맞습니다.")
    print("  반대로 보였다면 left_axis 와 right_axis 를 서로 바꿔야 합니다.")
    print()
    print("  ※ 좌우 기준은 '로봇 뒤에서 진행 방향을 보고 섰을 때' 입니다.")
    print("=" * 62)
    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
