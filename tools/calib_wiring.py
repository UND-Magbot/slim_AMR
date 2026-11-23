#!/usr/bin/env python3
"""바퀴 배선 판정 — 어느 축이 어느 쪽이고 부호가 어느 쪽인지 자동으로 찾는다.

베이스를 다시 만들거나 모터 커넥터를 손대면 아래 네 값이 바뀐다.
    left_axis / right_axis / left_sign / right_sign   (amr_params.yaml)
틀리면 로봇이 반대로 가거나 반대로 돈다.

⚠ 오도메트리는 이 설정을 그대로 따라 계산되므로 판단 근거가 못 된다.
   여기서는 slam_toolbox 의 map->base_link 를 정답으로 쓴다.

두 번 움직여서 판정한다.
    1) 전진 지령 -> 지도에서 앞으로 갔나 뒤로 갔나   => 부호
    2) 좌회전 지령 -> 지도에서 왼쪽으로 돌았나        => 축 대응

    python3 tools/calib_wiring.py

작고 느리게 움직인다(기본 0.25 m, 0.08 m/s). 앞뒤로 1 m 쯤 비워 두면 된다.
"""
import math
import sys
import time

import rclpy
import yaml
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener

sys.path.insert(0, "/home/und_amr1/projects/UND_AMR/tools")
from safe_stop import install_safe_stop          # noqa: E402

PARAMS = "/home/und_amr1/ros2_ws/src/und_amr_bringup/config/amr_params.yaml"
DIST = 0.25          # 전진 시험 거리 [m]
SPEED = 0.08         # 아주 느리게. 잘못된 방향으로 갈 수 있다
TURN = 0.35          # 회전 시험 각속도 [rad/s]
TURN_S = 2.5         # 회전 시험 시간 [s]
NEED_M = 0.05        # 이보다 적게 움직이면 판정 불가


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(d):
    while d > math.pi:
        d -= 2 * math.pi
    while d < -math.pi:
        d += 2 * math.pi
    return d


class W(Node):
    def __init__(s):
        super().__init__("calib_wiring")
        s.odom = None
        s.scan = None
        s.buf = Buffer()
        s.lis = TransformListener(s.buf, s)
        s.pub = s.create_publisher(Twist, "/cmd_vel", 10)
        s.create_subscription(Odometry, "/odom", lambda m: setattr(s, "odom", m), 10)
        s.create_subscription(LaserScan, "/scan", lambda m: setattr(s, "scan", m),
                              qos_profile_sensor_data)
        s.cli = s.create_client(Trigger, "/base_node/enable")

    def sp(s, t):
        t0 = time.time()
        while time.time() - t0 < t:
            rclpy.spin_once(s, timeout_sec=0.02)

    def pose(s):
        try:
            t = s.buf.lookup_transform("map", "base_link", rclpy.time.Time())
        except Exception:
            return None
        p = t.transform.translation
        return p.x, p.y, yaw_of(t.transform.rotation)

    def clear(s, half=40.0, back=False):
        m = s.scan
        if m is None:
            return 0.0
        v = []
        for i, r in enumerate(m.ranges):
            if not math.isfinite(r) or not (m.range_min < r < m.range_max):
                continue
            d = math.degrees(m.angle_min + i * m.angle_increment)
            if back:
                d = math.degrees(wrap(math.radians(d) - math.pi))
            if -half <= d <= half:
                v.append(r)
        return min(v) if v else 0.0

    def run(s, lin, ang, secs):
        t = Twist()
        t.linear.x = lin
        t.angular.z = ang
        t0 = time.time()
        while time.time() - t0 < secs:
            s.pub.publish(t)
            rclpy.spin_once(s, timeout_sec=0.02)
        for _ in range(12):
            s.pub.publish(Twist())
            s.sp(0.05)
        s.sp(1.5)


def main():
    rclpy.init()
    n = W()
    install_safe_stop(n)
    n.sp(3.0)
    if n.pose() is None:
        print("!! map->base_link 없음 — 위치추정이 돌고 있어야 합니다")
        return
    if not n.cli.wait_for_service(timeout_sec=5.0):
        print("!! /base_node/enable 없음 — base_node 확인")
        return

    cur = yaml.safe_load(open(PARAMS))["base_node"]["ros__parameters"]
    print("=" * 62)
    print("  현재 설정  좌=축%d(%+.0f)  우=축%d(%+.0f)"
          % (cur["left_axis"], cur["left_sign"],
             cur["right_axis"], cur["right_sign"]))
    print("  전진 %.2f m, 좌회전 %.0f 도를 시험합니다. 앞뒤로 1 m 비워 두세요."
          % (DIST, math.degrees(TURN * TURN_S)))
    print("=" * 62)
    fc, rc = n.clear(), n.clear(back=True)
    print("  여유  앞 %.2f m / 뒤 %.2f m" % (fc, rc))
    if min(fc, rc) < DIST + 0.5:
        print("!! 여유 부족 — 앞뒤로 %.2f m 이상 필요" % (DIST + 0.5))
        return

    n.cli.call_async(Trigger.Request())
    n.sp(2.5)

    # 1) 전진 시험 — 부호
    p0 = n.pose()
    n.run(SPEED, 0.0, DIST / SPEED)
    p1 = n.pose()
    dx, dy = p1[0] - p0[0], p1[1] - p0[1]
    fwd = dx * math.cos(p0[2]) + dy * math.sin(p0[2])     # 출발 헤딩 방향 성분
    print("\n  [1] 전진 지령 -> 지도에서 %+.3f m (%s)"
          % (fwd, "앞으로" if fwd > 0 else "뒤로"))
    if abs(fwd) < NEED_M:
        print("      %.0f mm 밖에 안 움직였습니다. 축이 꺼졌거나 막힌 것"
              % (abs(fwd) * 1000))
        return
    sign_ok = fwd > 0

    # 2) 좌회전 시험 — 축 대응
    p0 = n.pose()
    n.run(0.0, TURN, TURN_S)
    p1 = n.pose()
    dth = math.degrees(wrap(p1[2] - p0[2]))
    print("  [2] 좌회전 지령 -> 지도에서 %+.1f 도 (%s)"
          % (dth, "왼쪽" if dth > 0 else "오른쪽"))
    if abs(dth) < 5.0:
        print("      %.1f 도 밖에 안 돌았습니다. 판정 불가" % abs(dth))
        return
    turn_ok = dth > 0

    # 판정
    #   부호를 뒤집으면  직진과 회전이 '함께' 뒤집힌다
    #   축을 바꾸면      회전만 뒤집힌다 (직진은 좌우 지령이 같아 영향 없음)
    # 그래서 직진으로 부호를 먼저 정하고, 그 뒤 남은 회전 오류를 축으로 잡는다.
    la, ra = cur["left_axis"], cur["right_axis"]
    ls, rs = cur["left_sign"], cur["right_sign"]
    turn_now = turn_ok
    if not sign_ok:
        ls, rs = -ls, -rs
        turn_now = not turn_now      # 부호를 뒤집었으니 회전도 같이 뒤집혔다
    if not turn_now:
        la, ra = ra, la

    print("\n" + "=" * 62)
    if (la, ra, ls, rs) == (cur["left_axis"], cur["right_axis"],
                            cur["left_sign"], cur["right_sign"]):
        print("  설정이 맞습니다. 고칠 것 없음")
    else:
        print("  amr_params.yaml 을 이렇게 고치세요")
        print("    left_axis: %d" % la)
        print("    right_axis: %d" % ra)
        print("    left_sign: %.1f" % ls)
        print("    right_sign: %.1f" % rs)
        print("  고친 뒤 로봇을 다시 올리고 이 시험을 한 번 더 돌려 확인하세요")
    print("=" * 62)
    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
