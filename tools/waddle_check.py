#!/usr/bin/env python3
"""주행 중 좌우로 왔다갔다 하는 것을 재고, 원인을 가른다.

재는 것은 두 가지다.

    경로이탈  nav2 가 낸 경로에서 좌우로 몇 mm 벗어나 있나.
              눈에 보이는 '왔다갔다' 가 바로 이것이다. 길이 단위라
              얼마나 심한지 바로 감이 온다. nav2 이동 중에만 잰다.
    각속도    코가 좌우로 흔들리는 속도를 세 갈래로 본다.
              지령   /cmd_vel 이 시킨 회전
              바퀴   엔코더로 만든 오도메트리가 말하는 회전
              자이로 카메라(D435i) 안의 자이로가 잰 회전
              지도   map->base_footprint 를 미분한 회전

              ⚠ 지도는 자이로와 독립이 아니다. map->base_footprint 는
                slam 보정(map->odom)에 EKF(odom->base)를 얹은 것이고,
                EKF 의 방향은 자이로에서 온다. 게다가 TF 가 매 주기 갱신되지
                않아 계단처럼 들어와 미분하면 크기가 줄어든다. 참고로만 보고
                '자이로가 거짓말하나' 를 가리는 근거로는 쓰지 말 것.
                자이로를 검증하려면 정지 상태에서 손으로 돌려 라이다 정합과
                직접 맞춰야 한다 (tools/odom_smooth.py).
              바퀴와 자이로가 같이 흔들리면 바퀴가 진짜로 다르게 돈
              것이고(정수 rpm 양자화·디더·모터 제어), 바퀴는 잠잠한데
              자이로만 흔들리면 바깥에서 몸이 밀린 것이다(캐스터·바닥).

⚠ 대역을 잘못 잡으면 아무것도 못 본다. 처음에는 1 초보다 느린 성분을
  다 빼 버렸는데, 실제 흔들림이 0.6~0.85 Hz(주기 1.2~1.7 초)라 본체가
  통째로 잘려 나가고 꼬리만 남았다. 눈에 보이는 사행은 보통 주기 1~5 초
  다. 위쪽(45 Hz 대 노면 진동)은 반드시 잘라야 한다(2026-09-14).

쓰는 법 — 재는 동안 로봇을 몰아 둔다. 한 속도로 오래 달리면 속도 구간별로
나뉘어 나오고, 렉 진입처럼 짧은 동작이면 '움직인 구간 전체' 한 줄로 나온다.
앞뒤는 가리지 않는다 — 후진도 그대로 잡힌다.

    python3 tools/waddle_check.py --seconds 40

앞뒤로 충분히 비워 두세요.
"""
import argparse
import math

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Imu
from tf2_ros import Buffer, TransformListener

# 사행(蛇行)이 있는 대역만 남긴다.
#   아래: 8 초보다 느린 것은 진짜 방향전환이지 흔들림이 아니다
#   위:   0.3 초보다 빠른 떨림은 노면·모터 진동이다
HP_WIN_S = 8.0
LP_WIN_S = 0.3
BINS = [(0.05, 0.15), (0.15, 0.25), (0.25, 0.35),
        (0.35, 0.45), (0.45, 0.55), (0.55, 0.70)]
MIN_SEC = 2.0            # 이보다 짧게 머문 속도 구간은 판정에서 뺀다
MOVE_MIN = 0.03          # 이보다 빠르면 '움직이는 중' 으로 본다 [m/s]
RATE = 20.0


class Rec(Node):
    def __init__(self):
        super().__init__("waddle_check")
        self.rows = []       # (t, v, w_cmd, w_gyro, 경로이탈[m], w_wheel, 지도yaw)
        self.w_cmd = 0.0
        self.w_gyro = 0.0
        self.w_wheel = 0.0
        self.v_meas = 0.0
        self.path = None
        self.path_t = 0.0
        self.tfb = Buffer()
        self.tfl = TransformListener(self.tfb, self)
        self.create_subscription(Twist, "/cmd_vel", self.on_cmd, 10)
        self.create_subscription(Odometry, "/odometry/filtered", self.on_odom, 20)
        # ⚠ EKF 출력(/odometry/filtered)은 자이로를 이미 섞은 값이라
        #   비교 대상이 못 된다. base_node 가 내는 순수 바퀴 오도메트리를 쓴다.
        self.create_subscription(Odometry, "/odom", self.on_wheel, 20)
        # 센서 토픽은 BEST_EFFORT 다. 기본 QoS 로 걸면 한 장도 안 온다.
        self.create_subscription(Imu, "/imu/base_link", self.on_imu,
                                 qos_profile_sensor_data)
        self.create_subscription(Path, "/plan", self.on_plan, 10)
        self.create_timer(1.0 / RATE, self.tick)

    def on_cmd(self, m):
        self.w_cmd = m.angular.z

    def on_odom(self, m):
        self.v_meas = m.twist.twist.linear.x

    def on_wheel(self, m):
        self.w_wheel = m.twist.twist.angular.z

    def on_imu(self, m):
        self.w_gyro = m.angular_velocity.z

    def on_plan(self, m):
        pts = [(p.pose.position.x, p.pose.position.y) for p in m.poses]
        self.path = np.array(pts) if len(pts) >= 2 else None
        self.path_t = self.get_clock().now().nanoseconds * 1e-9

    def xtrack(self, x, y):
        """경로에서 좌우로 벗어난 거리 [m]. 왼쪽이 양수. 경로가 없으면 nan.

        ⚠ 경로가 끊기면 버려야 한다. 직전 경로를 계속 들고 있으면, 자율주행이
          끝난 뒤(렉 진입처럼 직접 모는 구간) 옛 경로와 견준 엉뚱한 값이
          나온다(2026-09-16).
        """
        p = self.path
        now = self.get_clock().now().nanoseconds * 1e-9
        if p is None or now - getattr(self, "path_t", 0.0) > 2.0:
            return float("nan")
        a, b = p[:-1], p[1:]
        d = b - a
        ln = np.einsum("ij,ij->i", d, d)
        ln[ln == 0] = 1e-9
        w = np.stack([x - a[:, 0], y - a[:, 1]], axis=1)
        u = np.clip(np.einsum("ij,ij->i", w, d) / ln, 0.0, 1.0)
        foot = a + d * u[:, None]
        dist = np.hypot(x - foot[:, 0], y - foot[:, 1])
        i = int(np.argmin(dist))
        # 진행 방향 기준 왼쪽이면 양수
        side = d[i, 0] * (y - a[i, 1]) - d[i, 1] * (x - a[i, 0])
        return float(math.copysign(dist[i], side))

    def tick(self):
        try:
            tr = self.tfb.lookup_transform("map", "base_footprint", Time())
        except Exception:                                    # noqa: BLE001
            return
        t = tr.header.stamp.sec + tr.header.stamp.nanosec * 1e-9
        x = tr.transform.translation.x
        y = tr.transform.translation.y
        q = tr.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.rows.append((t, self.v_meas, self.w_cmd, self.w_gyro,
                          self.xtrack(x, y), self.w_wheel, yaw))


def smooth(t, x, win_s):
    dt = float(np.median(np.diff(t)))
    if not np.isfinite(dt) or dt <= 0:
        dt = 1.0 / RATE
    n = max(3, int(round(win_s / dt)) | 1)
    pad = n // 2
    return np.convolve(np.pad(x, pad, mode="edge"),
                       np.ones(n) / n, mode="valid")[:len(x)]


def band(t, x):
    """사행 대역만 남긴다."""
    x = np.nan_to_num(x, nan=0.0, posinf=0.0, neginf=0.0)
    if len(t) < 8:
        return np.zeros(len(x))
    lo = smooth(t, x, LP_WIN_S)
    return lo - smooth(t, lo, HP_WIN_S)


def period(t, x):
    """가장 센 흔들림의 주기 [초]. 없으면 0."""
    if len(x) < 32:
        return 0.0
    dt = float(np.median(np.diff(t)))
    if not np.isfinite(dt) or dt <= 0:
        return 0.0
    sp = np.abs(np.fft.rfft((x - x.mean()) * np.hanning(len(x))))
    fr = np.fft.rfftfreq(len(x), dt)
    keep = (fr > 1.0 / HP_WIN_S) & (fr < 1.0 / (2 * LP_WIN_S))
    if not keep.any():
        return 0.0
    f = float(fr[keep][int(np.argmax(sp[keep]))])
    return 1.0 / f if f > 0 else 0.0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=40.0)
    a = ap.parse_args()

    rclpy.init()
    n = Rec()
    print("=" * 68)
    print("  %.0f 초 동안 기록한다. 지금부터 로봇을 몰아 주세요." % a.seconds)
    print("  한 속도로 오래 달려야 그 구간이 판정에 쓰입니다.")
    print("=" * 68)
    t0 = n.get_clock().now().nanoseconds * 1e-9
    while rclpy.ok():
        rclpy.spin_once(n, timeout_sec=0.1)
        if n.get_clock().now().nanoseconds * 1e-9 - t0 > a.seconds:
            break
    rows = list(n.rows)
    n.destroy_node()
    rclpy.shutdown()

    if len(rows) < 40:
        print("!! 표본이 너무 적습니다 (%d). map->base_footprint 가 있는지 보세요."
              % len(rows))
        return

    arr = np.array(rows, dtype=float)
    t, v, wc, wg, xt, ww, my = (arr[:, i] for i in range(7))
    # 지도 자세는 각도라 각속도로 바꿔야 나머지와 견줄 수 있다.
    myaw = np.unwrap(my)
    # TF 가 같은 시각으로 두 번 들어오면 dt 가 0 이 되어 미분이 무한이 된다.
    tg = np.maximum.accumulate(t)
    tg = tg + np.arange(len(tg)) * 1e-9
    wm = np.gradient(myaw, tg)
    wm = np.clip(wm, -3.0, 3.0)        # 정합이 튄 순간을 잘라낸다
    has_path = int(np.isfinite(xt).sum()) > len(xt) * 0.5
    bxt = band(t, xt) * 1000.0                    # mm
    bwc = np.degrees(band(t, wc))
    bwg = np.degrees(band(t, wg))
    bww = np.degrees(band(t, ww))
    bwm = np.degrees(band(t, wm))

    print()
    if not has_path:
        print("  ※ nav2 경로가 없어 경로이탈은 못 쟀습니다 (수동 주행이면 정상).")
    print("  %-12s %5s │ %11s │ %6s %6s %6s %6s │ %5s" %
          ("전진속도[m/s]", "초", "경로이탈[mm]",
           "지령", "바퀴", "자이로", "지도", "주기"))
    print("  %-12s %5s │ %5s %5s │ %6s %6s %6s %6s │ %5s" %
          ("", "", "rms", "최대", "[도/s]", "[도/s]", "[도/s]", "[도/s]", "[초]"))
    print("  " + "-" * 73)

    # ⚠ 속도는 절댓값으로 본다. 후진은 음수로 들어와 예전에는 어느 칸에도
    #   안 들어갔다 — 렉 진입(후진)을 재려는데 표가 통째로 비었다(2026-09-16).
    av = np.abs(v)

    good = []
    for lo, hi in BINS:
        m = (av >= lo) & (av < hi)
        sec = m.sum() / RATE
        if sec < 0.5:
            continue
        rx = float(np.sqrt(np.mean(bxt[m] ** 2))) if has_path else float("nan")
        px = float(np.max(np.abs(bxt[m]))) if has_path else float("nan")
        rc = float(np.sqrt(np.mean(bwc[m] ** 2)))
        rg = float(np.sqrt(np.mean(bwg[m] ** 2)))
        rw = float(np.sqrt(np.mean(bww[m] ** 2)))
        rm = float(np.sqrt(np.mean(bwm[m] ** 2)))
        pr = period(t[m], bxt[m] if has_path else bwg[m])
        thin = " " if sec >= MIN_SEC else "*"
        print("  %.2f ~ %.2f%s %5.1f │ %5.0f %5.0f │ %6.2f %6.2f %6.2f %6.2f │ %5.1f"
              % (lo, hi, thin, sec, rx, px, rc, rw, rg, rm, pr))
        if sec >= MIN_SEC:
            good.append(((lo, hi), rx, px, rc, rg, pr, rw, rm))

    # 짧은 동작(렉 진입은 6~24초)은 구간별로 나누면 표본이 안 모인다.
    # 움직인 구간 전체를 한 줄로도 낸다.
    mv = av >= MOVE_MIN
    whole = None
    if mv.sum() / RATE >= 1.0:
        whole = (float(np.sqrt(np.mean(bxt[mv] ** 2))) if has_path else float("nan"),
                 float(np.max(np.abs(bxt[mv]))) if has_path else float("nan"),
                 float(np.sqrt(np.mean(bwc[mv] ** 2))),
                 float(np.sqrt(np.mean(bww[mv] ** 2))),
                 float(np.sqrt(np.mean(bwg[mv] ** 2))),
                 float(np.sqrt(np.mean(bwm[mv] ** 2))),
                 period(t[mv], bxt[mv] if has_path else bwg[mv]),
                 mv.sum() / RATE, float(np.mean(av[mv])))
        print("  " + "-" * 73)
        print("  움직인 구간 전체  %5.1f │ %5.0f %5.0f │ %6.2f %6.2f %6.2f %6.2f │ %5.1f"
              % (whole[7], whole[0], whole[1], whole[2], whole[3], whole[4],
                 whole[5], whole[6]))
        print("     (평균 속력 %.2f m/s, 앞뒤 무관)" % whole[8])
    print("  (* %.0f 초에 못 미쳐 판정에서 뺀 구간)" % MIN_SEC)
    print()
    print("=" * 68)
    if not good:
        if whole is None:
            print("  움직인 시간이 1초도 안 됩니다. 재는 동안 몰아 주세요.")
            return
        # 짧은 동작이면 전체 한 줄로 판정한다
        rc, rw, rg = whole[2], whole[3], whole[4]
        print("  구간별로 나눌 만큼 길지 않아 전체로 판정합니다 (%.1f초)" % whole[7])
        print("    지령 %.2f  →  바퀴 %.2f  →  자이로 %.2f 도/s" % (rc, rw, rg))
        print()
        if rg < 1.5:
            print("  흔들림이라 부를 만한 크기가 아닙니다.")
        elif rc > rg * 0.6:
            print("  지령이 이미 그만큼 출렁입니다 -> 제어 쪽입니다.")
            print("  볼 곳: 진입 이득(JOB_K_LAT/K_YAW)과 조향 상한(JOB_W_ENTER).")
        elif rw > rg * 0.6:
            print("  지령은 잔잔한데 바퀴가 출렁입니다 -> 지령이 바퀴에 실리는")
            print("  과정입니다. 정수 rpm 양자화·디더(rpm_dither)를 보세요.")
        else:
            print("  바퀴는 고르게 도는데 몸이 흔들립니다 -> 캐스터·바닥입니다.")
            print("  소프트웨어로는 한계가 있습니다.")
        print("=" * 68)
        return

    print("  속도에 따른 변화 (느린 구간 → 빠른 구간)")
    if has_path:
        print("    경로이탈 rms  %s mm"
              % " → ".join("%.0f" % g[1] for g in good))
    print("    지도          %s 도/s" % " → ".join("%.1f" % g[7] for g in good))
    print("    자이로        %s 도/s" % " → ".join("%.1f" % g[4] for g in good))
    print("    바퀴          %s 도/s" % " → ".join("%.1f" % g[6] for g in good))
    print("    지령          %s 도/s" % " → ".join("%.1f" % g[3] for g in good))

    key = 1 if has_path else 4
    (lo, hi), rx, px, rc, rg, pr, rw, rm = max(good, key=lambda g: g[key])
    print()
    print("  가장 심한 구간: %.2f ~ %.2f m/s" % (lo, hi))
    if has_path:
        print("    경로에서 좌우로 rms %.0f mm, 최대 %.0f mm, 주기 %.1f 초"
              % (rx, px, pr))
    print("    지령 %.2f  →  바퀴 %.2f  →  자이로 %.2f  (지도 %.2f) 도/s"
          % (rc, rw, rg, rm))
    print()
    if rg < 1.5:
        print("  흔들림이라 부를 만한 크기가 아닙니다.")
    elif rw > rg * 0.6:
        print("  바퀴가 실제로 다르게 돌고 있습니다 -> 구동 쪽입니다.")
        print("  볼 곳: rpm_dither, 정수 rpm 양자화, 좌우 모터 속도제어,")
        print("         wheel_diameter_ratio.")
        if rc > rw * 0.6:
            print("  다만 그 바퀴 차이를 지령이 이미 시키고 있습니다.")
            print("  -> 위쪽(경로추종기 또는 조이스틱)에서 내려온 것입니다.")
        else:
            print("  지령은 그만큼 안 시켰습니다 -> 지령이 바퀴에 실리는")
            print("     과정에서 생깁니다(양자화·디더·모터 제어).")
    else:
        print("  바퀴는 고르게 도는데 몸이 틀어집니다 -> 바깥에서 밀립니다.")
        print("  캐스터 떨림, 바닥 기울기, 좌우 하중 불균형을 보세요.")
        print("  구동 지령을 아무리 다듬어도 안 낫습니다.")
    print("=" * 68)


if __name__ == "__main__":
    main()
