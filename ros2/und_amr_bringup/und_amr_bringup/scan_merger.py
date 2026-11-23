#!/usr/bin/env python3
"""라이다 2대(앞 왼쪽 · 뒤 오른쪽)를 360도 스캔 하나로 합친다 (2026-10-26).

  /scan_front (laser_front) + /scan_rear (laser_rear)  ->  /scan (laser)

'laser' 프레임은 구동축 바로 위, 스캔면 높이, 정면이 0도인 가상 라이다다(URDF).
SLAM · nav2 · api_node · cart_detector 는 예전처럼 /scan 하나와 그 frame_id 를 쓴다.

- 두 라이다 장착 위치 · 방향은 TF(URDF)에서 읽는다. 값은 tools/lidar_calib.py 로 쟀다.
- 두 스캔은 찍힌 시각이 최대 한 바퀴(0.067초) 다르다. 그 사이 로봇이 움직인 만큼은
  odom -> base_footprint TF 로 맞춘다 (0.4 m/s 면 27mm). 못 구하면 그대로 합친다.
- 차체 외곽(+여유) 안쪽 점, 각 라이다 바로 옆 점(거치대)은 버린다 — 자기 몸을
  장애물로 보면 nav2 가 출발을 못 하고 위치 추정이 차체 점에 끌린다(사용자 지적).
- 앞 스캔이 들어올 때마다 낸다. 뒤 스캔이 끊기면 앞 것만, 앞이 끊기면 뒤 것으로 낸다.
- 비스듬히 스친 점(그림자 · 가장자리 혼합 반사)을 버린다 — laser_filters ScanShadowsFilter 와
  같은 식. ⚠ 2026-10-26: 앞 라이다가 차체 앞면을 스치듯 보는 방향(로봇 오른쪽 앞 0.5m)에
  30장 중 3~26% 만 나타나는 점이 찍혀, RPP 가 '앞에 충돌' 148번으로 멈췄다(사용자: "쓰레기값
  때매 이동 못함"). 녹화한 스캔으로 5도 기준이 깜빡이는 점을 99~100% 지우고, 차체 밖 늘 보이는
  점은 9~11% (벽을 아주 비스듬히 본 가장자리) 만 잃었다.
- 각 라이다 바로 뒤쪽(센서 기준 180도 ± blind_half_deg) blind_range 안의 점을 버린다 — 차체를
  가로질러 반대편 옆을 보는 방향이다. ⚠ 2026-10-27 · 10-30 같은 점: 앞 라이다 센서각 +170~171도,
  거리 0.33~0.40m (로봇 오른쪽 앞, 차체 옆 3~9cm). 서 있어도 15~72% 찍혀 RPP 가 '앞에 충돌'로
  출발을 못 하거나 지도에 빨간 점으로 보였다(사용자: "이런식으로 이상한 장애물 점도 생김").
  body_margin 을 넓히자 점이 조금 바깥으로 옮겨 갔을 뿐이다. 이 방향 가까운 곳은 반대 모서리
  라이다가 바로 앞에서 본다. 먼 점(벽)은 그대로 둔다 — 차체 아래가 이 높이에서 뚫려 있어
  뒤 라이다가 차체 길이를 지나 4.5m 앞 벽까지 정상으로 본다(2026-10-30 확인).
"""
import math

import numpy as np
import rclpy
import tf2_ros
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def stamp_s(st):
    return st.sec + st.nanosec * 1e-9


class ScanMerger(Node):
    def __init__(self):
        super().__init__("scan_merger")
        g = self.declare_parameter
        g("front_topic", "/scan_front")
        g("rear_topic", "/scan_rear")
        g("out_topic", "/scan")
        g("out_frame", "laser")
        g("base_frame", "base_footprint")
        g("odom_frame", "odom")
        g("bins", 2400)                 # 0.15도 — 라이다 한 대와 같은 간격
        g("range_min", 0.05)
        g("range_max", 40.0)
        # 차체 외곽 (base_footprint 기준) + 여유. URDF base_len/base_wid, 구동축 = 정중앙.
        g("body_front", 0.43675)
        g("body_rear", 0.43675)
        g("body_half_width", 0.1644)
        # ⚠ 2026-10-27 14:25: 8시간 왕복 중 앞 라이다에 차체 오른쪽 옆 3cm(축 앞 0.36m,
        #   y -0.19~-0.20)에 늘 붙어 다니는 점이 생겨 RPP '앞에 충돌'로 출발 못 함
        #   (사용자: "오른쪽 앞에 점때매 출발 못해"). 0.02 -> 0.045. 차체 4.5cm 안은
        #   nav2 외곽(0.165)으로도 어차피 지나갈 수 없는 거리다.
        g("body_margin", 0.045)
        g("mount_radius", 0.08)        # 라이다 바로 옆 (거치대) 버림
        g("blind_half_deg", 15.0)       # 라이다 바로 뒤(센서 180도) ± 이만큼, blind_range 안 버림
        g("blind_range", 0.60)
        g("max_age", 0.20)              # 이보다 오래된 다른 쪽 스캔은 안 합친다
        g("shadow_min_angle", 5.0)      # 도. 이웃 점과 이은 선이 빔과 이보다 나란하면 버림
        g("shadow_window", 3)
        g("shadow_max_range", 5.0)      # 이 안쪽 점에만 (장애물 판단에 쓰이는 거리)
        p = lambda k: self.get_parameter(k).value  # noqa: E731
        self.front_topic, self.rear_topic = p("front_topic"), p("rear_topic")
        self.out_frame, self.base_frame, self.odom_frame = p("out_frame"), p("base_frame"), p("odom_frame")
        self.bins = int(p("bins"))
        self.rmin, self.rmax = float(p("range_min")), float(p("range_max"))
        m = float(p("body_margin"))
        self.box = (-(float(p("body_rear")) + m), float(p("body_front")) + m,
                    float(p("body_half_width")) + m)
        self.mount_r = float(p("mount_radius"))
        self.blind_half = math.radians(float(p("blind_half_deg")))
        self.blind_r = float(p("blind_range"))
        self.max_age = float(p("max_age"))
        self.sh_min = math.radians(float(p("shadow_min_angle")))
        self.sh_win = int(p("shadow_window"))
        self.sh_rmax = float(p("shadow_max_range"))

        self.tf = tf2_ros.Buffer(cache_time=Duration(seconds=5.0))
        tf2_ros.TransformListener(self.tf, self)
        self.ext = {}                   # frame -> (x, y, yaw) in out_frame (2D)
        self.last = {"front": None, "rear": None}
        self.pub = self.create_publisher(LaserScan, p("out_topic"), 10)
        self.create_subscription(LaserScan, self.front_topic,
                                 lambda m: self.on_scan("front", m), qos_profile_sensor_data)
        self.create_subscription(LaserScan, self.rear_topic,
                                 lambda m: self.on_scan("rear", m), qos_profile_sensor_data)
        self._warned = set()
        self.get_logger().info("%s + %s -> %s (%s)" % (self.front_topic, self.rear_topic,
                                                        p("out_topic"), self.out_frame))

    def warn_once(self, key, msg):
        if key not in self._warned:
            self._warned.add(key)
            self.get_logger().warn(msg)

    # ------------------------------------------------------------ 좌표
    def extrinsic(self, frame):
        """그 라이다 프레임의 out_frame 기준 2D 자세 (고정이라 한 번만 읽는다)."""
        if frame not in self.ext:
            try:
                t = self.tf.lookup_transform(self.out_frame, frame, Time())
            except tf2_ros.TransformException:
                return None
            self.ext[frame] = (t.transform.translation.x, t.transform.translation.y,
                               yaw_of(t.transform.rotation))
        return self.ext[frame]

    def base_pose(self, st):
        try:
            t = self.tf.lookup_transform(self.odom_frame, self.base_frame, Time.from_msg(st),
                                         timeout=Duration(seconds=0.0))
        except tf2_ros.TransformException:
            return None
        return (t.transform.translation.x, t.transform.translation.y, yaw_of(t.transform.rotation))

    def points(self, m):
        """스캔 -> out_frame 기준 점 (N, 2). 몸체 · 거치대 점은 뺀다."""
        ex = self.extrinsic(m.header.frame_id)
        if ex is None:
            self.warn_once("tf" + m.header.frame_id,
                           "%s -> %s TF 가 아직 없다" % (m.header.frame_id, self.out_frame))
            return None
        r = np.asarray(m.ranges, dtype=float)
        a = m.angle_min + np.arange(len(r)) * m.angle_increment
        ok = np.isfinite(r) & (r > max(self.rmin, m.range_min)) & (r < min(self.rmax, m.range_max))
        ok &= r > self.mount_r
        back = np.abs(np.angle(np.exp(1j * (a - math.pi))))   # 센서 180도에서 떨어진 각
        ok &= ~((back < self.blind_half) & (r < self.blind_r))
        ok &= ~self.shadows(np.where(ok, r, np.inf), m.angle_increment)
        r, a = r[ok], a[ok]
        lx, ly, lyaw = ex
        x = lx + r * np.cos(a + lyaw)
        y = ly + r * np.sin(a + lyaw)
        x0, x1, hw = self.box
        body = (x > x0) & (x < x1) & (np.abs(y) < hw)
        return np.stack([x[~body], y[~body]], 1)

    def shadows(self, r, inc):
        """비스듬히 스친 점 (그림자 · 혼합 반사). 이웃(±window 칸) 점과 이은 선이 빔 방향과
        shadow_min_angle 보다 나란하면 두 점 다 버린다. 360도 스캔이라 끝은 이어 붙인다."""
        bad = np.zeros(len(r), dtype=bool)
        fin = np.isfinite(r)
        rr = np.where(fin, r, 1.0e6)          # inf - inf 경고 없이
        for k in range(1, self.sh_win + 1):
            r2 = np.roll(rr, -k)
            ok = fin & np.roll(fin, -k) & ((rr < self.sh_rmax) | (r2 < self.sh_rmax))
            dth = k * inc
            ang = np.abs(np.arctan2(np.where(ok, r2, 0.0) * math.sin(dth),
                                    np.where(ok, rr - r2 * math.cos(dth), 1.0)))
            ang = np.minimum(ang, math.pi - ang)
            sh = ok & (ang < self.sh_min)
            bad |= sh
            bad |= np.roll(sh, k)
        return bad

    # ------------------------------------------------------------ 합치기
    def on_scan(self, which, m):
        self.last[which] = m
        other = self.last["rear" if which == "front" else "front"]
        if which == "rear" and self.last["front"] is not None and \
                stamp_s(m.header.stamp) - stamp_s(self.last["front"].header.stamp) < self.max_age:
            return                      # 앞 스캔이 살아 있으면 앞 스캔 때 낸다
        pts = self.points(m)
        if pts is None:
            return
        parts = [pts]
        if other is not None:
            age = stamp_s(m.header.stamp) - stamp_s(other.header.stamp)
            if abs(age) <= self.max_age:
                q = self.points(other)
                if q is not None:
                    parts.append(self.move(q, other.header.stamp, m.header.stamp))
            else:
                self.warn_once("stale" + which,
                               "%s 스캔이 %.2f초 늦다 — 한 대만으로 낸다" % (
                                   "뒤" if which == "front" else "앞", age))
        else:
            self.warn_once("none" + which, "%s 스캔이 아직 없다 — 한 대만으로 낸다" % (
                "뒤" if which == "front" else "앞"))
        self.publish(np.concatenate(parts, 0), m.header.stamp)

    def move(self, q, st_from, st_to):
        """st_from 에 찍은 점을 st_to 의 로봇 자리 기준으로 (그 사이 움직인 만큼)."""
        p0, p1 = self.base_pose(st_from), self.base_pose(st_to)
        if p0 is None or p1 is None:
            return q
        dx, dy = p0[0] - p1[0], p0[1] - p1[1]
        c1, s1 = math.cos(-p1[2]), math.sin(-p1[2])
        tx, ty = c1 * dx - s1 * dy, s1 * dx + c1 * dy
        dth = p0[2] - p1[2]
        c, s = math.cos(dth), math.sin(dth)
        return np.stack([tx + c * q[:, 0] - s * q[:, 1], ty + s * q[:, 0] + c * q[:, 1]], 1)

    def publish(self, pts, stamp):
        n = self.bins
        inc = 2.0 * math.pi / n
        out = np.full(n, np.inf, dtype=np.float32)
        if len(pts):
            r = np.hypot(pts[:, 0], pts[:, 1])
            a = np.arctan2(pts[:, 1], pts[:, 0])
            k = np.clip(((a + math.pi) / inc).astype(int), 0, n - 1)
            np.minimum.at(out, k, r.astype(np.float32))
        msg = LaserScan()
        msg.header.stamp = stamp
        msg.header.frame_id = self.out_frame
        msg.angle_min = -math.pi
        msg.angle_max = math.pi - inc
        msg.angle_increment = inc
        msg.time_increment = 0.0
        msg.scan_time = 1.0 / 15.0
        msg.range_min = self.rmin
        msg.range_max = self.rmax
        msg.ranges = out.tolist()
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = ScanMerger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
