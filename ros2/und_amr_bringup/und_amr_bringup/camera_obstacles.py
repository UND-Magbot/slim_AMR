#!/usr/bin/env python3
"""카메라(D435i 깊이)로 라이다가 못 보는 장애물을 찾아 스캔으로 낸다 (2026-10-27).

  /camera/camera/depth/image_rect_raw  ->  /camera/obstacle_scan (LaserScan, camera_link)

라이다 스캔면은 바닥 201.5mm 한 높이뿐이라 그보다 낮은 물건(상자 · 팔레트 · 발)과
그 위로만 튀어나온 것(책상 · 선반)은 못 본다(사용자: "카메라 인식으로도 장애물
인지되도록"). 카메라는 앞(+x) 끝 3cm 밖, 바닥 230mm, 정면을 본다.

- 바닥 위 floor_margin + floor_slope x 거리 보다 높고 max_height 보다 낮은 점을
  장애물로 본다. 바닥은 1m 안에서 ±5mm, 2~2.5m 에서 위로 40mm 까지 흩어졌다
  (2026-10-27 정지 프레임) — 그래서 멀수록 기준을 올리고 max_range 2.0m 까지만 쓴다.
  2m 밖은 깊이 오차가 커서(4.5m 에서 라이다와 15cm 차이) 라이다에 맡긴다.
- ⚠ 잡음 점 하나로 서지 않게(오늘 라이다 잡음 점으로 하루 종일 섰다):
  5cm 칸에 점이 min_points 개 넘고, 마지막 frames 장 중 hits 장 넘게 보인 칸만 낸다.
- 방향마다 가장 가까운 장애물 거리를 낸다. 장애물은 없는데 깊이는 보인 방향은 inf
  (nav2 가 그 방향을 비운다 — 치운 물건이 지도에 남지 않게), 깊이가 아예 없는
  방향(검은 물건 · 너무 가까움)은 nan(아무것도 안 바꾼다).
- 카메라가 끊기면 아무것도 안 낸다. nav2 는 라이다만으로 계속 간다.

⚠ 2026-10-30 보조 확인으로 좁힘 (사용자: "카메라는 그저 보조확인임 가까운 물체각 라이다가
감지못하는 위치의 장애물만 인식"). 2m 로 경로 계획 지도에도 넣었더니 렉1 윗판(높이 0.6~0.8m),
A 뒤 가구(0.4~0.6m) 같은 진짜 물체가 경로 옆 1~2m 에 찍혀 팽창 0.7m 로 경로가 휘었다
(사용자: "이상한 장애물 점때문에 경로가 이상해 지고있어"). 그래서
- max_range 2.0 -> 1.0 (카메라에서 1m = 로봇 중심에서 약 1.5m 안)
- 라이다(/scan)가 lidar_clear 안에 점을 가진 칸은 빼고 라이다가 못 보는 것만 낸다.
  라이다 스캔이 lidar_max_age 보다 오래되면 빼지 않는다(안전 쪽).
- nav2 는 주행 지도(local)에만 따로 층으로 넣는다(config/nav2.yaml camera_layer).

2026-11-11 넥 위 아래 카메라(D435F, 0.955m, 65.6도 숙임)로 옮김 (launch 파라미터).
숙인 카메라는 차체 앞부분과 넥까지 보므로 차체 상자(+body_margin) 안 점은 버린다.
⚠ 넥 앞면은 차체 폭 밖(왼쪽 y 0.10~0.28)까지 나와 영상 맨 아래 70줄에 높이 0.10~0.51m
  세로면으로 늘 찍혔다(base x 0.40~0.46 = 카메라 x 0.47 보다 뒤). 그래서 self_x_max 보다
  뒤, |y| < self_half_width 안도 버린다. 그 밖(로봇 앞)은 그대로 본다.
"""
import math
import time

import numpy as np
import rclpy
import tf2_ros
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, LaserScan


def quat_matrix(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


class CameraObstacles(Node):
    def __init__(self):
        super().__init__("camera_obstacles")
        g = self.declare_parameter
        g("depth_topic", "/camera/camera/depth/image_rect_raw")
        g("info_topic", "/camera/camera/depth/camera_info")
        g("out_topic", "/camera/obstacle_scan")
        g("base_frame", "base_footprint")
        g("scan_frame", "camera_link")
        g("every", 3)                 # 30Hz 중 3장에 1장 -> 10Hz
        g("decimate", 4)              # 640x480 -> 160x120
        g("min_depth", 0.15)          # D435 최소 거리 근처는 버림
        g("max_range", 1.0)
        g("floor_margin", 0.05)       # 바닥 위 이만큼 + floor_slope x 거리 위부터 장애물
        g("floor_slope", 0.02)
        g("max_height", 1.8)          # 렉을 들었을 때 높이까지
        g("cell", 0.05)
        g("min_points", 3)
        g("frames", 3)
        g("hits", 2)
        g("fov_deg", 88.0)
        g("bin_deg", 0.5)
        g("lidar_topic", "/scan")
        g("body_front", 0.43675)      # 차체 상자 (base_footprint 기준, scan_merger 와 같음)
        g("body_rear", 0.43675)
        g("body_half_width", 0.1644)
        g("body_margin", 0.05)
        g("self_x_max", 0.50)         # 이보다 뒤(base x) 이고
        g("self_half_width", 0.35)    # |y| 가 이 안이면 로봇(넥·차체) 으로 보고 버림
        g("lidar_clear", 0.10)        # 라이다 점이 이 안에 있는 칸은 라이다에 맡긴다. 0 = 끔
        g("lidar_max_age", 0.5)
        p = lambda k: self.get_parameter(k).value  # noqa: E731
        self.base_frame, self.scan_frame = p("base_frame"), p("scan_frame")
        self.every, self.dec = int(p("every")), int(p("decimate"))
        self.min_depth, self.max_range = float(p("min_depth")), float(p("max_range"))
        self.fl_m, self.fl_s = float(p("floor_margin")), float(p("floor_slope"))
        self.max_h, self.cell = float(p("max_height")), float(p("cell"))
        self.min_pts, self.nf, self.hits = int(p("min_points")), int(p("frames")), int(p("hits"))
        self.half = math.radians(float(p("fov_deg"))) / 2.0
        self.inc = math.radians(float(p("bin_deg")))
        self.nbins = int(round(2 * self.half / self.inc))
        self.lidar_clear, self.lidar_age = float(p("lidar_clear")), float(p("lidar_max_age"))
        m = float(p("body_margin"))
        self.body = (-float(p("body_rear")) - m, float(p("body_front")) + m,
                     float(p("body_half_width")) + m)
        self.self_x, self.self_hw = float(p("self_x_max")), float(p("self_half_width"))

        self.tf = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.tf, self)
        self.K = None
        self.ext = None               # (R, t) optical -> base, scan 원점 (x, y, yaw) in base
        self.rays = None
        self.n = 0
        self.hist = []                # 최근 frames 장의 칸 집합
        self.lidar = None             # (받은 시각, base 기준 점 (N, 2))
        self.lidar_ext = {}
        self.pub = self.create_publisher(LaserScan, p("out_topic"), 10)
        self.create_subscription(CameraInfo, p("info_topic"), self.on_info, qos_profile_sensor_data)
        self.create_subscription(Image, p("depth_topic"), self.on_depth, qos_profile_sensor_data)
        if self.lidar_clear > 0.0:
            self.create_subscription(LaserScan, p("lidar_topic"), self.on_lidar, qos_profile_sensor_data)
        self.get_logger().info("%s -> %s (%s, %.1fm 안)" % (p("depth_topic"), p("out_topic"),
                                                           self.scan_frame, self.max_range))

    def on_info(self, m):
        if self.K is None:
            self.K = (m.k[0], m.k[4], m.k[2], m.k[5])

    def on_lidar(self, m):
        f = m.header.frame_id
        if f not in self.lidar_ext:
            try:
                t = self.tf.lookup_transform(self.base_frame, f, Time())
            except tf2_ros.TransformException:
                return
            q = t.transform.rotation
            self.lidar_ext[f] = (t.transform.translation.x, t.transform.translation.y,
                                 math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))
        lx, ly, lyaw = self.lidar_ext[f]
        r = np.asarray(m.ranges, dtype=float)
        a = m.angle_min + np.arange(len(r)) * m.angle_increment + lyaw
        # 카메라 범위 + 여유 안의 점만 둔다
        ok = np.isfinite(r) & (r > m.range_min) & (r < self.max_range + 1.0)
        self.lidar = (time.monotonic(),
                      np.stack([lx + r[ok] * np.cos(a[ok]), ly + r[ok] * np.sin(a[ok])], 1))

    def lidar_sees(self, cells):
        """각 칸(번호) 중심 lidar_clear 안에 라이다 점이 있으면 True. 스캔이 오래되면 모두 False."""
        out = np.zeros(len(cells), dtype=bool)
        if self.lidar is None or not len(cells):
            return out
        t, L = self.lidar
        if time.monotonic() - t > self.lidar_age or not len(L):
            return out
        ctr = (np.asarray(cells, dtype=float) + 0.5) * self.cell
        for i, c in enumerate(ctr):
            d2 = (L[:, 0] - c[0]) ** 2 + (L[:, 1] - c[1]) ** 2
            out[i] = d2.min() < self.lidar_clear ** 2
        return out

    def extrinsic(self, optical):
        if self.ext is None:
            try:
                t = self.tf.lookup_transform(self.base_frame, optical, Time())
                s = self.tf.lookup_transform(self.base_frame, self.scan_frame, Time())
            except tf2_ros.TransformException:
                return None
            R = quat_matrix(t.transform.rotation)
            tr = t.transform.translation
            q = s.transform.rotation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            self.ext = (R, np.array([tr.x, tr.y, tr.z]),
                        (s.transform.translation.x, s.transform.translation.y, yaw))
        return self.ext

    # ------------------------------------------------------------ 점 -> 장애물 칸
    def points(self, m):
        """깊이 영상 -> base 좌표 점 (N, 3)."""
        fx, fy, cx, cy = self.K
        d = self.dec
        if m.encoding not in ("16UC1", "mono16"):
            return None
        z = np.frombuffer(m.data, dtype=np.uint16).reshape(m.height, m.width)[::d, ::d]
        if self.rays is None or self.rays[0].shape != z.shape:
            v, u = np.mgrid[0:m.height:d, 0:m.width:d]
            self.rays = ((u - cx) / fx, (v - cy) / fy)
        z = z.astype(np.float32) * 0.001
        ok = z > self.min_depth
        z = z[ok]
        P = np.stack([self.rays[0][ok] * z, self.rays[1][ok] * z, z], 1)
        R, t, _ = self.ext
        return P @ R.T + t

    def obstacle_cells(self, B, sx, sy):
        """장애물 점만 골라 칸 번호 -> (점 수) 를 낸다. 거리는 스캔 원점 기준."""
        dist = np.hypot(B[:, 0] - sx, B[:, 1] - sy)
        h = B[:, 2]
        ob = (dist < self.max_range) & (h > self.fl_m + self.fl_s * dist) & (h < self.max_h)
        x0, x1, hw = getattr(self, "body", (0.0, 0.0, 0.0))
        ob &= ~((B[:, 0] > x0) & (B[:, 0] < x1) & (np.abs(B[:, 1]) < hw))
        sx_max, shw = getattr(self, "self_x", -1e9), getattr(self, "self_hw", 0.0)
        ob &= ~((B[:, 0] < sx_max) & (np.abs(B[:, 1]) < shw))
        C = np.floor(B[ob, :2] / self.cell).astype(np.int64)
        if not len(C):
            return {}
        keys, cnt = np.unique(C, axis=0, return_counts=True)
        return {(int(k[0]), int(k[1])): int(c) for k, c in zip(keys, cnt) if c >= self.min_pts}

    def on_depth(self, m):
        # ⚠ 2026-11-11 D435F 노드가 같은 장을 다시 보낸다(5초 248장 중 고유 148장).
        #   두 번 세면 '최근 frames 장 중 hits 장' 잡음 거르기가 한 장만 보인 것도 낸다.
        st = (m.header.stamp.sec, m.header.stamp.nanosec)
        if st == getattr(self, "last_stamp", None):
            return
        self.last_stamp = st
        self.n += 1
        if self.n % self.every or self.K is None:
            return
        if self.extrinsic(m.header.frame_id) is None:
            return
        B = self.points(m)
        if B is None:
            return
        sx, sy, syaw = self.ext[2]
        cells = self.obstacle_cells(B, sx, sy)
        self.hist.append(set(cells))
        self.hist = self.hist[-self.nf:]
        ok = [c for c in cells if sum(c in h for h in self.hist) >= self.hits]
        if ok and self.lidar_clear > 0.0:
            seen = self.lidar_sees(ok)
            ok = [c for c, s in zip(ok, seen) if not s]

        out = np.full(self.nbins, np.nan, dtype=np.float32)
        # 깊이가 보인 방향은 일단 비었다(inf) — 2m 안 아무 점이나 (바닥 포함)
        a = np.arctan2(B[:, 1] - sy, B[:, 0] - sx) - syaw
        k = ((a + self.half) / self.inc).astype(int)
        seen = (k >= 0) & (k < self.nbins)
        out[np.unique(k[seen])] = np.inf
        if ok:
            ctr = (np.array(ok, dtype=float) + 0.5) * self.cell
            obs = np.full(self.nbins, np.inf, dtype=np.float32)
            # 칸이 걸친 방향 전부를 칸 네 모서리 중 가장 가까운 거리로 채운다. 모서리
            # 방향만 찍으면 1m 에서 5cm 칸이 3도씩 벌어져 그 사이 방향이 '비었음'이 된다.
            ang, rng = [], []
            for ox in (-0.5, 0.5):
                for oy in (-0.5, 0.5):
                    px = ctr[:, 0] + ox * self.cell - sx
                    py = ctr[:, 1] + oy * self.cell - sy
                    rng.append(np.hypot(px, py))
                    ang.append(np.arctan2(py, px) - syaw)
            k0 = np.floor((np.min(ang, 0) + self.half) / self.inc).astype(int)
            k1 = np.floor((np.max(ang, 0) + self.half) / self.inc).astype(int)
            r = np.min(rng, 0).astype(np.float32)
            for a0, a1, rr in zip(np.clip(k0, 0, self.nbins), np.clip(k1 + 1, 0, self.nbins), r):
                if a1 > a0:
                    np.minimum(obs[a0:a1], rr, out=obs[a0:a1])
            out = np.where(np.isfinite(obs), obs, out)

        msg = LaserScan()
        msg.header.stamp = m.header.stamp
        msg.header.frame_id = self.scan_frame
        msg.angle_min = -self.half
        msg.angle_increment = self.inc
        msg.angle_max = -self.half + self.inc * (self.nbins - 1)
        msg.time_increment = 0.0
        msg.scan_time = self.every / 30.0
        msg.range_min = 0.05
        # ⚠ 2026-11-11: range_max 를 max_range 와 같게 두었더니 nav2 가 inf(비었음)를
        #   range_max - 0.0001 로 바꿔 obstacle_max_range(1.0) 안이라 '장애물'로 찍었다 —
        #   빈 방향마다 카메라 1m 끝에 호가 생겨 RPP '앞에 충돌'로 이동이 취소됐다(사용자:
        #   "가다가 이동 취소는 왜되는거야"). 10/27~30 의 2m 설정에도 같은 호가 있었다.
        #   range_max 를 0.5m 더 길게 내 inf 는 표시 한계 밖 = 비우기만 한다.
        msg.range_max = self.max_range + 0.5
        msg.ranges = out.tolist()
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = CameraObstacles()
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
