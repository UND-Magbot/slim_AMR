#!/usr/bin/env python3
"""윤거(wheel_separation) 역산 — 지도 기준 자세를 정답으로 쓴다.

    yaw_odom = (d_r - d_l) / sep      이므로
    sep_true = sep_now * yaw_odom / yaw_true

정답을 어디서 가져오는가가 핵심이다.

  rot_compare.py 의 라이다 ICP(match_rotation)는 "제자리 회전 전용" 이라
  로봇이 회전하면서 위치까지 옮기면 정합이 깨진다. 손으로 돌리면 반드시
  옮겨지므로(9/9: rms 0.0795, 각도 8도 어긋남) 그 방법을 쓸 수 없다.

  여기서는 slam_toolbox 가 지도에 맞춰 낸 map->base_link 자세를 쓴다.
  회전과 이동을 같이 풀기 때문에 손으로 돌려도 된다.

쓰는 법 — 모터를 쓰지 않는다. 손으로 돌리는 것이 정확하다.
  (모터 제자리회전은 캐스터가 비틀려 바퀴가 크게 미끄러진다. 8/25: 998/1183도)

    python3 tools/calib_sep.py --target 180

  로봇을 제자리에서 목표각까지 천천히 돌린 뒤 그대로 두세요.
  많이 돌릴수록 정확해진다. 180도 이상을 권한다.
"""
import argparse
import math
import sys
import time

import rclpy
import yaml
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu
from tf2_ros import Buffer, TransformListener

ap = argparse.ArgumentParser()
ap.add_argument("--target", type=float, default=180.0, help="목표 회전각[deg]")
ap.add_argument("--secs", type=float, default=90.0, help="제한 시간[s]")
ap.add_argument("--params",
                default="/home/und_amr1/ros2_ws/src/und_amr_bringup/config/amr_params.yaml")
a = ap.parse_args()


JUMP = math.radians(15.0)   # 0.1초에 이보다 크게 변하면 위치추정 점프로 본다


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(d):
    while d > math.pi:
        d -= 2 * math.pi
    while d < -math.pi:
        d += 2 * math.pi
    return d


class C(Node):
    def __init__(s):
        super().__init__("calib_sep")
        s.odom = None
        s.buf = Buffer()
        s.lis = TransformListener(s.buf, s)
        s.create_subscription(Odometry, "/odom", lambda m: setattr(s, "odom", m), 10)
        # 자이로도 같이 본다. 한 번 돌려서 윤거와 축대응을 동시에 확인한다.
        # IMU 는 best effort 로 나온다. 기본(reliable)이면 한 건도 못 받는다.
        q = QoSProfile(depth=50, reliability=ReliabilityPolicy.BEST_EFFORT)
        s.raw = None
        s.base = None
        s.create_subscription(Imu, "/camera/camera/imu",
                              lambda m: setattr(s, "raw", m), q)
        s.create_subscription(Imu, "/imu/base_link",
                              lambda m: setattr(s, "base", m), q)

    def gyro(s):
        """(카메라 원시 xyz, base_link z). 없으면 None."""
        r = s.raw.angular_velocity if s.raw else None
        b = s.base.angular_velocity if s.base else None
        return ([r.x, r.y, r.z] if r else None), (b.z if b else None)

    def map_yaw(s):
        try:
            t = s.buf.lookup_transform("map", "base_link", rclpy.time.Time())
        except Exception:
            return None
        return yaw_of(t.transform.rotation)

    def sp(s, t):
        t0 = time.time()
        while time.time() - t0 < t:
            rclpy.spin_once(s, timeout_sec=0.02)


def wait_still(n, need=1.5, tol=0.3, limit=20.0):
    """로봇이 멈출 때까지 기다린다. tol[deg] 안에서 need 초 유지되면 정지로 본다."""
    t0 = time.time()
    said = False
    while time.time() - t0 < limit:
        y0 = yaw_of(n.odom.pose.pose.orientation)
        n.sp(need)
        if abs(math.degrees(wrap(yaw_of(n.odom.pose.pose.orientation) - y0))) < tol:
            return True
        if not said:
            print("  ... 로봇이 멈추기를 기다리는 중")
            said = True
    return False


def main():
    rclpy.init()
    n = C()
    n.sp(3.0)
    if n.odom is None:
        print("!! /odom 미수신 — base_node 가 떠 있는지 확인하세요")
        return
    if n.map_yaw() is None:
        print("!! map->base_link 없음 — 위치추정(localization)이 돌고 있어야 합니다")
        return

    sep = yaml.safe_load(open(a.params))["base_node"]["ros__parameters"]["wheel_separation"]
    print("=" * 62)
    print("  현재 wheel_separation %.4f m" % sep)
    print("  로봇을 제자리에서 %.0f 도까지 천천히 돌린 뒤 그대로 두세요." % a.target)
    print("  (모터를 쓰지 마세요. 손으로 돌리는 것이 정확합니다)")
    print("=" * 62)

    # 자이로 치우침(바이어스)을 먼저 잰다. 안 빼면 회전량에 그대로 섞인다.
    # ⚠ 반드시 정지 상태여야 한다. 재는 동안 로봇이 돌면 그 회전이 바이어스로
    #   잡히고, 이후 전 구간에서 그만큼 빼버린다.
    #   (9/10: 재는 중 돌려서 y 가 0.208 rad/s = 11.9도/s 로 잡혔고
    #    적분값이 -891도까지 튀었다)
    if not wait_still(n):
        print("!! 로봇이 계속 움직입니다. 완전히 세운 뒤 다시 실행하세요")
        return
    bs, bb, t_b = [0.0, 0.0, 0.0], 0.0, 0
    tb = time.time()
    while time.time() - tb < 2.5:
        n.sp(0.05)
        r, b = n.gyro()
        if r is None:
            continue
        for i in range(3):
            bs[i] += r[i]
        bb += b if b is not None else 0.0
        t_b += 1
    if t_b:
        bs = [v / t_b for v in bs]
        bb /= t_b
        print("  자이로 바이어스 x %+.5f y %+.5f z %+.5f  base_z %+.5f rad/s"
              % (bs[0], bs[1], bs[2], bb))
        if max(abs(v) for v in bs) > 0.05:
            print("!! 바이어스가 0.05 rad/s 를 넘습니다. 측정 중 로봇이 움직였을 수")
            print("   있습니다. 완전히 세운 뒤 다시 실행하세요")
            return
    gi = [0.0, 0.0, 0.0]
    gb = 0.0
    tg = time.time()

    m0, o0 = n.map_yaw(), yaw_of(n.odom.pose.pose.orientation)
    mp, op = m0, o0
    macc = oacc = 0.0          # 누적 회전(180도 넘어도 되도록 펴서 더한다)
    t0 = time.time()
    shown = -1.0
    still = 0.0
    jumps = 0
    while time.time() - t0 < a.secs:
        n.sp(0.1)
        mc = n.map_yaw()
        if mc is None:
            continue
        oc = yaw_of(n.odom.pose.pose.orientation)
        now = time.time()
        dt = now - tg
        tg = now
        # 위치추정이 한 번에 크게 튀면(재정합) 그건 로봇이 돈 것이 아니다.
        # 손으로 돌리는 속도로는 나올 수 없는 변화량이라 그 구간은 버린다.
        if abs(wrap(mc - mp)) < JUMP:
            macc += wrap(mc - mp)
            oacc += wrap(oc - op)
        else:
            jumps += 1
        mp, op = mc, oc
        r, b = n.gyro()
        if r is not None:
            for i in range(3):
                gi[i] += (r[i] - bs[i]) * dt
            if b is not None:
                gb += (b - bb) * dt
        cur = abs(math.degrees(macc))
        if cur - shown >= 1.0:
            shown = cur
            still = 0.0
            sys.stdout.write("\r   지도기준 %6.1f / %.0f deg   바퀴 %6.1f deg   남은 %4.1f s   "
                             % (cur, a.target, math.degrees(oacc), a.secs - (time.time() - t0)))
            sys.stdout.flush()
        else:
            still += dt
        if cur >= a.target:
            print("\n   목표 도달")
            break
        # 목표에 못 닿아도 충분히 돌리고 멈췄으면 거기서 끝낸다.
        # 남은 시간을 헛돌면 그동안의 위치추정 보정이 회전량에 섞인다(9/10).
        if cur >= 30.0 and still >= 2.5:
            print("\n   회전이 멈춤 — 여기까지로 계산합니다")
            break
    else:
        print("\n   시간 초과")

    # 끝점은 로봇이 완전히 선 뒤에 읽는다. 회전 중 위치추정은 뒤처지거나
    # 재정합으로 튀지만, 세워 두면 지도에 정확히 맞춰진다.
    wait_still(n)
    n.sp(2.0)
    if jumps:
        print("   ⚠ 위치추정이 %d 회 크게 튀어 그 구간은 제외했습니다" % jumps)

    # 누적값은 '몇 바퀴 돌았는지' 세는 데만 쓰고, 크기는 정지 상태의
    # 시작/끝 자세로 정한다. 이렇게 하면 회전 중 누적 오차가 들어오지 않는다.
    def settle(acc, y_end, y_start):
        d = wrap(y_end - y_start)
        turns = round((acc - d) / (2 * math.pi))
        return d + turns * 2 * math.pi

    macc = settle(macc, n.map_yaw(), m0)
    oacc = settle(oacc, yaw_of(n.odom.pose.pose.orientation), o0)

    gt = math.degrees(macc)
    od = math.degrees(oacc)
    print()
    print("=" * 62)
    print("  지도 기준 회전 (정답)   %+8.2f deg" % gt)
    print("  바퀴 오도메트리        %+8.2f deg   오차 %+.2f" % (od, od - gt))
    if abs(gt) < 30.0:
        print("  회전량이 %.1f 도로 부족하다. 180 도 이상 돌려서 다시 재세요." % abs(gt))
    else:
        sep_t = sep * od / gt
        print()
        print("  역산 wheel_separation  %.4f m   (현재 대비 %+.2f %%)"
              % (sep_t, (sep_t / sep - 1) * 100))
        print("  기하 실측 0.1750 m 대비 스크럽 계수 %.4f" % (sep_t / 0.175))
        err = abs(od - gt) / abs(gt) * 100
        if err < 1.0:
            print("  현재 값으로 이미 %.2f %% 안에 든다. 바꿀 필요가 적다." % err)
        else:
            print("  -> amr_params.yaml 의 wheel_separation 을 이 값으로")
        print()
        print("  같은 방향으로 한 번 더 재서 값이 재현되는지 확인할 것.")

    # 자이로 축 대응 — 어느 축이 로봇의 요(yaw)인가
    if t_b and abs(gt) >= 30.0:
        print()
        print("  자이로 축 대응 (바이어스 뺀 적분값, 정답 %+.2f deg)" % gt)
        cand = [("카메라 raw x", math.degrees(gi[0])),
                ("카메라 raw y", math.degrees(gi[1])),
                ("카메라 raw z", math.degrees(gi[2])),
                ("/imu/base_link z", math.degrees(gb))]
        best = None
        for name, v in cand:
            err = (v - gt) / gt * 100
            mark = ""
            if abs(err) < 10:
                mark = "  <-- 일치"
                if best is None or abs(err) < abs(best[1]):
                    best = (name, err)
            print("    %-18s %+8.2f deg   %+7.1f %%%s" % (name, v, err, mark))
        if best:
            print("    -> %s 가 로봇 요축. 오차 %+.1f %%" % (best[0], best[1]))
            # 바퀴와 자이로는 서로 독립이다. 둘이 가깝게 붙어 있는데
            # 지도만 멀면, 틀린 쪽은 지도다(회전 중 정합이 뒤처진 경우).
            gyz = math.degrees(gb)
            if abs(gyz) > 1.0:
                gap_og = abs(od - gyz)
                gap_m = abs(gt - (od + gyz) / 2)
                print()
                print("    교차검증  바퀴 %+.1f / 자이로 %+.1f / 지도 %+.1f deg"
                      % (od, gyz, gt))
                if gap_og < 5.0 and gap_m > 10.0:
                    print("    ⚠ 바퀴와 자이로는 %.1f 도 안에서 일치하는데 지도만 %.1f 도"
                          % (gap_og, gap_m))
                    print("      떨어져 있다. 이번 지도값은 믿지 말 것 (다시 측정)")
                elif gap_og < 5.0:
                    print("    세 값이 모두 %.1f 도 안에 든다. 신뢰할 만하다"
                          % max(gap_og, gap_m))
        else:
            print("    -> 어느 축도 맞지 않는다. 부호가 반대인 축이 있으면")
            print("       imu_adapter 의 축 변환을 그 부호로 고쳐야 한다.")
    print("=" * 62)
    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
