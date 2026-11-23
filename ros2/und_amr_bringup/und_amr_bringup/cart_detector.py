#!/usr/bin/env python3
"""대차(렉) 검출 노드 — 라이다로 다리 네 개를 찾아 대차를 알아낸다.

설계 요점 (2026-08-26 실측을 근거로)

  * 다리 모양과 굵기는 보지 않는다.
    같은 대차의 네 다리도 캐스터가 향한 방향에 따라 32~82mm 로 제각각
    측정된다. 원형 맞춤을 쓰면 각진 다리를 놓치고, 굵기로 종류를 가르면
    같은 대차도 매번 다르게 판정된다. 쓰는 것은 '네 다리의 배치' 뿐이다.

  * 후보를 전부 다리로 가정하지 않는다.
    접지선이나 바닥 잡동사니가 다리로 잡히면, 전부를 다리로 보는 방식은
    크기가 안 맞아 인식 자체가 실패한다(현장에서 실제로 겪은 문제).
    그래서 후보 중 네 개를 고르는 조합을 모두 훑어 규격에 가장 잘 맞는
    조합을 찾는다. 실측에서 후보 26개(진짜 4 + 가짜 22) 중 진짜만 골라냈다.

  * 라이다는 다리의 '로봇 쪽 표면' 만 본다.
    덩어리 중심은 실제 다리 중심보다 앞으로 나와 있어 사각형이 실제보다
    크게 잡힌다. 폭의 절반만큼 뒤로 밀어 보정한다(오차 42mm → 20mm).

발행
    /carts        대차 목록 (JSON 문자열). 관리자 화면이 지도에 그린다
    /cart_markers RViz 표시용 마커
"""
import json
import math
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.duration import Duration
from sensor_msgs.msg import CameraInfo, Image, LaserScan
from std_msgs.msg import String as StringMsg
from tf2_ros import Buffer, TransformListener
import tf2_ros as _tf2
from visualization_msgs.msg import Marker, MarkerArray

from und_amr_bringup.cart_match import (cluster_scan, leg_candidates,
                                        match_carts, match_carts3)


class CartDetector(Node):

    def __init__(self):
        super().__init__("cart_detector")
        p = self.declare_parameters("", [
            ("max_range", 3.0),
            ("cluster_gap", 0.05),
            ("leg_width_min", 0.010),
            ("leg_width_max", 0.100),
            ("leg_min_points", 2),
            ("leg_radius", 0.020),         # 표면->중심 보정에 쓸 다리 반지름 [m]
            # 다리보다 넓은 덩어리의 양 끝 이 길이도 다리 후보로 [m]. 0 이면 끔.
            # 다리가 뒤 물체에 붙어 한 덩어리가 되면 통째로 버려졌다(2026-09-28).
            ("leg_end_seg", 0.04),
            ("min_score", 0.25),
            # 프레임 간 유지 - 점수가 잠깐 내려가도 직전에 확실히 잡힌
            # 자리 근처면 놓치지 않는다
            ("hold_sec", 1.0),             # 이 시간 안에 확실히 잡힌 적이 있으면
            ("hold_dist", 0.20),           # 그 자리에서 이만큼 안이면 [m]
            ("hold_score_ratio", 0.6),     # 점수를 이 배까지 봐준다
            ("publish_hz", 5.0),
            ("verify_camera", True),
            ("verify_col_height", 0.15),   # 다리가 세로로 이어져야 하는 최소 높이 [m]
            # 위쪽으로도 이만큼 훑는다. 라이다 평면 아래가 뚫린 렉이 있다.
            ("verify_col_up", 0.15),
            ("verify_depth_tol", 0.12),    # 다리 거리와 이만큼 안이면 같은 물체 [m]
            ("verify_min_ratio", 0.45),    # 기둥으로 인정할 최소 적중 비율
            ("verify_smooth", 5),          # 적중 비율을 이 프레임 수의 중앙값으로
            ("verify_occl_margin", 0.15),  # 이만큼 더 가까운 것이 앞에 있으면 가려진 것 [m]
            # 2026-11-13 넥 위 정면 카메라(1.0m): 기둥을 바닥 위 이 높이 구간에서 본다 [m]
            ("verify_h_lo", 0.35),
            ("verify_h_hi", 1.15),
            # 2026-11-13 라이다 다리와 카메라 기둥이 좌우로 20~45mm 어긋났다(장면마다 방향도
            #   바뀜). 투영 자리에서만 보면 상자가 기둥을 빗나가 '실패'가 된다. 좌우로 이만큼
            #   안에서 '그 거리로 세로로 이어진 기둥' 을 찾는다 [m].
            ("verify_search", 0.06),
            # 다리 셋 인식(넷째 계산)을 받아들이는 조건: 이 시간·거리 안에 넷으로 확실히
            #   잡힌 적이 있거나, 카메라가 기둥을 확인할 때 [s], [m]
            ("three_leg", True),
            ("three_hold_sec", 2.0),
            ("three_hold_dist", 0.25),
            ("lidar_z_ground", 0.2015),    # 라이다(laser 프레임) 높이 [m]
            ("carts", ["LG"]),
        ])
        g = self.get_parameter
        self.rmax = g("max_range").value
        self.gap = g("cluster_gap").value
        self.wmin = g("leg_width_min").value
        self.wmax = g("leg_width_max").value
        self.minpts = int(g("leg_min_points").value)
        self.leg_r = g("leg_radius").value
        self.end_seg = g("leg_end_seg").value or None
        self.min_score = g("min_score").value
        self.hold_sec = g("hold_sec").value
        self.hold_dist = g("hold_dist").value
        self.hold_ratio = g("hold_score_ratio").value
        # 대차별로 '마지막으로 확실히 잡힌' 시각과 자리
        self._anchor = {}

        # 대차 규격은 설정 파일에서 읽는다. 새 종류는 yaml 에 줄만 추가하면 된다.
        self.specs = {}
        for name in g("carts").value:
            self.declare_parameter(name + ".width", 0.0)
            self.declare_parameter(name + ".depth", 0.0)
            self.declare_parameter(name + ".tol", 0.06)
            w = g(name + ".width").value
            d = g(name + ".depth").value
            if w <= 0 or d <= 0:
                self.get_logger().warn("대차 '%s' 규격이 없어 건너뜁니다" % name)
                continue
            self.specs[name] = {"width": w, "depth": d,
                                "tol": g(name + ".tol").value}
        if not self.specs:
            self.get_logger().error("등록된 대차가 없습니다 (config/carts.yaml 확인)")
        else:
            self.get_logger().info(
                "대차 %d종 등록: %s" %
                (len(self.specs),
                 ", ".join("%s %.0fx%.0fmm" % (n, s["width"] * 1000,
                                               s["depth"] * 1000)
                           for n, s in self.specs.items())))

        self.verify = bool(g("verify_camera").value)
        self.v_h = g("verify_col_height").value
        self.v_up = g("verify_col_up").value
        self.v_tol = g("verify_depth_tol").value
        self.v_ratio = g("verify_min_ratio").value
        self.v_occl = g("verify_occl_margin").value
        self.v_smooth = max(1, int(g("verify_smooth").value))
        self.v_hlo, self.v_hhi = g("verify_h_lo").value, g("verify_h_hi").value
        self.lidar_zg = g("lidar_z_ground").value
        self.v_search = g("verify_search").value
        self.three = bool(g("three_leg").value)
        self.three_sec = g("three_hold_sec").value
        self.three_dist = g("three_hold_dist").value
        self._ratio_hist = {}          # 다리 자리별 최근 적중 비율

        self.scan = None
        self.depth = None
        self.cinfo = None
        self.create_subscription(LaserScan, "/scan", self._on_scan,
                                 qos_profile_sensor_data)
        # TF 는 카메라 검증과 도킹 어긋남 계산 둘 다 쓴다. 검증을 꺼도 필요하다.
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        if self.verify:
            # ⚠ 컬러 기준으로 정렬된 뎁스를 쓴다.
            #   원본 뎁스는 화각 80도(fx 384), 컬러는 56도(fx 607) 로 달라서,
            #   원본 뎁스로 계산한 픽셀을 컬러 영상에 그리면 바깥으로 퍼진다.
            #   aligned_depth_to_color 는 컬러와 화각·프레임이 같아 그대로 겹친다.
            self.create_subscription(
                Image, "/camera/camera/aligned_depth_to_color/image_raw",
                self._on_depth, qos_profile_sensor_data)
            self.create_subscription(
                CameraInfo, "/camera/camera/aligned_depth_to_color/camera_info",
                self._on_cinfo, qos_profile_sensor_data)
        self.pub = self.create_publisher(StringMsg, "/carts", 10)
        self.pub_al = self.create_publisher(StringMsg, "/cart_align", 10)
        self.pub_mk = self.create_publisher(MarkerArray, "/cart_markers", 10)
        hz = max(0.5, g("publish_hz").value)
        self.create_timer(1.0 / hz, self._tick)
        self._last_n = -1

    def _on_scan(self, m):
        self.scan = m

    def _on_depth(self, m):
        self.depth = m

    def _on_cinfo(self, m):
        self.cinfo = m

    # ---------------- 카메라 검증 ----------------
    #
    # 라이다는 325mm 높이의 수평 단면 하나만 본다. 그 높이에만 있는 물건
    # (상자 모서리, 파레트 귀퉁이, 늘어진 선)이 다리처럼 잡힐 수 있다.
    # 카메라는 세로 방향을 보므로 "그 자리에 기둥이 실제로 서 있는가" 를
    # 확인할 수 있다. 다리 모양은 보지 않는다 — 세로로 이어지는지만 본다.

    def _verify_with_camera(self, legs, laser_frame):
        """다리 자리 위로 기둥이 서 있는지 정면 카메라(컬러 정렬 깊이)로 본다.

        ⚠ 2026-11-13 다시 짬: 카메라가 넥 위(1.0m, 수평)로 올라가 라이다 높이(0.2m)의
          다리 지점은 컬러 화면(위아래 ±21.6도) 아래 밖이 됐다 — 예전 방식은 늘 '화각 밖'.
          그래서 다리 자리 위 바닥 위 verify_h_lo ~ verify_h_hi 기둥 구간을 화면에 찍어,
          그 세로 띠에 '그 거리의' 깊이가 이어지는지 본다. 크롬 기둥(ls_origin)은 1.4m 에서
          0.1~1.15m 가 촘촘히 잡혔다(같은 날 실측). 앞 선반 철망에 가려 더 가까운 것만 보이면
          '가려짐', 깊이가 거의 없으면 판정 안 함(먼 뒤 기둥은 흔하다).
        """
        d, ci = self.depth, self.cinfo
        if d is None or ci is None:
            return None, "뎁스 영상 없음", None
        try:
            tf = self.tf_buffer.lookup_transform(
                d.header.frame_id, laser_frame, rclpy.time.Time(),
                timeout=Duration(seconds=0.1))
        except _tf2.TransformException as e:
            return None, "TF 없음: %s" % str(e)[:40], None

        img = np.frombuffer(d.data, dtype=np.uint16).reshape(d.height, d.width)
        fx, fy = ci.k[0], ci.k[4]
        cx, cy = ci.k[2], ci.k[5]
        t = tf.transform.translation
        q = tf.transform.rotation
        xx, yy, zz, ww = q.x, q.y, q.z, q.w
        R = np.array([
            [1 - 2 * (yy * yy + zz * zz), 2 * (xx * yy - zz * ww), 2 * (xx * zz + yy * ww)],
            [2 * (xx * yy + zz * ww), 1 - 2 * (xx * xx + zz * zz), 2 * (yy * zz - xx * ww)],
            [2 * (xx * zz - yy * ww), 2 * (yy * zz + xx * ww), 1 - 2 * (xx * xx + yy * yy)]])
        T = np.array([t.x, t.y, t.z])
        heights = np.arange(self.v_hlo, self.v_hhi + 1e-6, 0.025)

        ok_cnt, checked, notes, uv = 0, 0, [], []
        for (lx, ly) in legs:
            rows = []                         # (v, u, Z) 화면 안에 드는 높이들
            for h in heights:
                P = R @ np.array([lx, ly, h - self.lidar_zg]) + T
                Z = float(P[2])
                if Z <= 0.2:
                    continue
                u = int(round(fx * P[0] / Z + cx))
                v = int(round(fy * P[1] / Z + cy))
                if 2 <= u < d.width - 2 and 2 <= v < d.height - 2:
                    rows.append((v, u, Z))
            if len(rows) < 4:
                uv.append(None)
                continue                      # 화각 밖
            Zm = float(np.median([r[2] for r in rows]))
            hw = max(2, min(8, int(round(fx * 0.02 / Zm))))   # 기둥 반지름 20mm + 여유
            # ⚠ 2026-11-13 좌우로 찾는다(verify_search). 투영 자리 ±hw 만 보면 라이다-카메라
            #   어긋남 28mm 가 0.55m 에서 31px 이라 상자가 기둥을 통째로 빗나갔다.
            #   높이마다 가로 한 줄을 떼어 '그 거리 적중' 표를 만들고, 기둥 폭(hw)으로
            #   번지게 한 뒤 세로로 가장 많이 이어진 열을 고른다. 선반 가로 철망은
            #   몇 줄만 차지해 세로 비율을 못 채운다.
            S = int(round(fx * getattr(self, "v_search", 0.06) / Zm))
            span = np.arange(-S - hw, S + hw + 1)
            H = np.zeros((len(rows), len(span)), dtype=bool)     # 그 거리 적중
            V = np.zeros((len(rows), len(span)), dtype=bool)     # 깊이 있음
            N = np.zeros((len(rows), len(span)), dtype=bool)     # 훨씬 가까운 것
            for ri, (v, u, Z) in enumerate(rows):
                cols = u + span
                okc = (cols >= 0) & (cols < d.width)
                band = img[max(0, v - 1):v + 2, np.clip(cols, 0, d.width - 1)].astype(np.float32) / 1000.0
                val = (band > 0.1) & okc[None, :]
                V[ri] = val.any(axis=0)
                H[ri] = (val & (np.abs(band - Z) < self.v_tol)).any(axis=0)
                N[ri] = (val & (band < Z - self.v_occl)).all(axis=0) & V[ri]
            k = 2 * hw + 1
            ker = np.ones(k)
            def _win(A):
                return np.convolve(A.astype(float), ker, "valid") > 0
            Hc = np.array([_win(r) for r in H])       # 열 = 가운데 오프셋 -S..S
            Vc = np.array([_win(r) for r in V])
            Nc = np.array([_win(r) & ~h for r, h in zip(N, Hc)])
            hits = Hc.sum(axis=0)
            offs = np.arange(-S, S + 1)
            best = int(np.lexsort((np.abs(offs), -hits))[0])
            du = int(offs[best])
            seen = int(Vc[:, best].sum())
            hit = int(hits[best])
            near = int(Nc[:, best].sum())
            vs = [r[0] for r in rows]
            box = {"u": int(round(np.median([r[1] for r in rows]))) + du, "v": int(min(vs)),
                   "z": round(Zm, 3), "h": int(max(vs) - min(vs)),
                   "du_mm": round(du * Zm / fx * 1000.0)}
            key = (round(lx, 1), round(ly, 1))
            if seen < max(3, len(rows) // 4):
                state, ratio = "nodepth", None
            elif near > hit and near >= seen // 2:
                state, ratio = "occl", None
            else:
                ratio = hit / seen
                hh = self._ratio_hist.setdefault(key, [])
                hh.append(ratio)
                if len(hh) > self.v_smooth:
                    del hh[0]
                ratio = float(np.median(hh))
                state = "ok" if ratio >= self.v_ratio else "fail"
            # ⚠ 2026-11-13 먼 뒤 기둥(2m, 철망 선반 너머)은 프레임마다 '가려짐' 과 '확인/실패'
            #   를 오가 화면 상자가 30초에 47~60번 깜빡였다(판정은 앞 기둥으로 늘 통과).
            #   기둥마다 최근 7번 중 가장 많은 상태로 낸다.
            sh = self.__dict__.setdefault("_state_hist", {}).setdefault(key, [])
            sh.append(state)
            if len(sh) > 7:
                del sh[0]
            state = max(set(sh), key=lambda k: (sh.count(k), k == sh[-1]))
            if state in ("nodepth", "occl"):
                box.update(ok=None, occl=True, why="깊이 없음" if state == "nodepth" else "가려짐")
                uv.append(box)
                continue                      # 판정 안 함
            hh = self._ratio_hist.get(key) or [0.0]
            ratio = float(np.median(hh))
            checked += 1
            if state == "ok":
                ok_cnt += 1
            notes.append(round(ratio, 2))
            box.update(ok=(state == "ok"), ratio=round(ratio, 2))
            uv.append(box)
        if checked == 0:
            return None, "가려지거나 화각 밖", (uv, d.width, d.height)
        return (ok_cnt, checked), notes, (uv, d.width, d.height)

    # ---------------- 도킹 어긋남 ----------------
    #
    # 도킹은 대차 중심과 로봇 구동축 중심의 어긋남을 0 으로 몰아가는 일이다.
    # 진입 제어는 구동륜 접지(하드웨어)가 먼저 해결돼야 붙일 수 있지만,
    # '얼마나 어긋났는가' 는 지금 재서 내보낼 수 있다. 제어기는 이 값을
    # 그대로 따라가면 된다.
    #
    # 값은 base_footprint 기준이다. 라이다 좌표를 그대로 쓰면 라이다 장착각
    # (2.1도) 만큼 계속 틀어진 채로 들어가게 된다.

    def _align(self, cart, laser_frame):
        if cart is None:
            return {"ready": False, "reason": "대차 없음"}
        try:
            tf = self.tf_buffer.lookup_transform(
                "base_footprint", laser_frame, rclpy.time.Time(),
                timeout=Duration(seconds=0.1))
        except _tf2.TransformException as e:
            return {"ready": False, "reason": "TF 없음: %s" % str(e)[:40]}
        t = tf.transform.translation
        q = tf.transform.rotation
        yaw0 = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                          1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        c, sn = math.cos(yaw0), math.sin(yaw0)
        dx = t.x + cart["x"] * c - cart["y"] * sn
        dy = t.y + cart["x"] * sn + cart["y"] * c
        # 대차의 방향은 90도 주기다(다리 넷으로는 앞뒤·좌우를 못 가린다).
        # 로봇이 대차와 나란해지려면 얼마나 더 돌아야 하는가를 -45~45도로
        # 접어서 낸다. 어느 변으로 진입할지는 이 값과 별개 문제다.
        yaw = cart["yaw"] + yaw0
        dyaw = (yaw + math.pi / 4) % (math.pi / 2) - math.pi / 4
        return {
            "ready": True, "reason": None,
            "cart": cart["cart"],
            "verified": cart["cam_ok"] is True,
            "forward": round(dx, 4),          # 앞뒤 어긋남 [m] (+ = 대차가 앞)
            "lateral": round(dy, 4),          # 좌우 어긋남 [m] (+ = 대차가 왼쪽)
            "yaw": round(dyaw, 4),            # 회전 어긋남 [rad]
            "yaw_deg": round(math.degrees(dyaw), 2),
            "dist": round(math.hypot(dx, dy), 4),
            "score": cart["score"],
        }

    def _tick(self):
        m = self.scan
        if m is None or not self.specs:
            return
        stamp = (m.header.stamp.sec, m.header.stamp.nanosec)
        if stamp == getattr(self, "_last_scan_stamp", None):
            return
        self._last_scan_stamp = stamp
        x, y, groups = cluster_scan(m.ranges, m.angle_min, m.angle_increment,
                                    rmax=self.rmax, gap=self.gap)
        cands = leg_candidates(x, y, groups, wmin=self.wmin, wmax=self.wmax,
                               min_pts=self.minpts, radius=self.leg_r,
                               end_seg=getattr(self, "end_seg", None))
        # 문턱보다 낮은 후보까지 일단 받아 두고, 유지 조건을 통과한 것만
        # 남긴다. 문턱 하나로 자르면 점수가 문턱에 걸친 대차가 프레임마다
        # 나타났다 사라진다(실측에서 초당 한 번꼴로 깜빡였다).
        found = match_carts(cands, self.specs,
                            min_score=self.min_score * self.hold_ratio)
        now = time.time()
        keep = []
        for f in found:
            if f["score"] >= self.min_score:
                keep.append(f)
                continue
            a = self._anchor.get(f["cart"])
            if (a and now - a[0] <= self.hold_sec
                    and math.hypot(f["x"] - a[1], f["y"] - a[2]) <= self.hold_dist):
                f["held"] = True          # 유지로 살린 것
                keep.append(f)
        # 기준점은 '확실히 잡힌' 것만 갱신한다. 유지된 것으로 갱신하면
        # 한 번 잘못 잡힌 자리가 계속 되살아난다.
        for f in keep:
            if not f.get("held"):
                self._anchor[f["cart"]] = (now, f["x"], f["y"])
        found = keep
        # 다리 셋 인식 (넷째는 계산). 넷으로 이미 잡힌 자리와 겹치면 안 낸다.
        #   가짜(방 안 잡동사니 셋이 우연히 규격 모서리를 이룸)가 장당 2개꼴로
        #   나오므로 둘 중 하나일 때만 받는다:
        #   (1) 최근 three_hold_sec 안에 넷으로 확실히 잡힌 자리 three_hold_dist 안
        #   (2) 카메라가 그 다리 위 기둥을 2개 이상 확인하고 실패가 없을 때 (아래)
        three_cam = []
        if getattr(self, "three", False):
            for f in match_carts3(cands, self.specs, min_score=self.min_score):
                if any(math.hypot(f["x"] - k["x"], f["y"] - k["y"]) < 0.25
                       for k in found):
                    continue
                a = self._anchor.get(f["cart"])
                if (a and now - a[0] <= getattr(self, "three_sec", 2.0)
                        and math.hypot(f["x"] - a[1], f["y"] - a[2])
                        <= getattr(self, "three_dist", 0.25)):
                    found.append(f)
                elif len(three_cam) < 3 and self.verify:
                    three_cam.append(f)
        if len(self._ratio_hist) > 400:
            self._ratio_hist.clear()      # 자리가 계속 바뀌면 쌓이기만 한다
        if len(self.__dict__.get("_state_hist", {})) > 400:
            self._state_hist.clear()

        out = []
        cam_only = {id(t) for t in three_cam}
        for f in found + three_cam:
            ver, note, cam = (None, None, None)
            if self.verify:
                # 카메라 투영에는 보정 전 '표면 점' 을 쓴다. 카메라도 다리의
                # 앞면을 보므로, 중심으로 보정한 점을 투영하면 실제 보이는
                # 위치보다 안쪽으로 찍힌다.
                surf = f.get("legs_raw") or f["legs"]
                ver, note, cam = self._verify_with_camera(
                    surf, m.header.frame_id or "laser")
            if id(f) in cam_only and not (ver and ver[0] >= 2 and ver[0] == ver[1]):
                continue
            if id(f) in cam_only and any(
                    math.hypot(f["x"] - o["x"], f["y"] - o["y"]) < 0.25 for o in out):
                continue
            # ⚠ 2026-11-13 진짜 ls_origin 다리 하나 + 그 옆 캐스터 판 끝(50mm 옆) + 잡동사니로
            #   und_1(722x685) 이 다리 셋 인식으로 30초에 38번 나왔다(카메라는 진짜 기둥
            #   둘을 보고 통과). 렉 두 개는 한 자리에 겹쳐 설 수 없다 — 먼저 낸 것(넷 인식,
            #   점수 순)과 중심 거리가 두 렉 짧은 변 반의 합(x0.9)보다 가까우면 버린다.
            #   나란히 붙은 진짜 렉끼리는 이보다 멀다.
            #   (판정은 아래, 카메라 확인까지 끝난 뒤 우선순위대로)
            out.append({
                "cart": f["cart"], "score": round(f["score"], 3),
                "x": round(f["x"], 4), "y": round(f["y"], 4),
                "yaw": round(f["yaw"], 4),
                "yaw_deg": round(math.degrees(f["yaw"]), 1),
                "a": round(f["a"], 4), "b": round(f["b"], 4),
                "err_mm": round(f["err"] * 1000, 1),
                # 점수가 문턱 아래인데 직전 자리로 살린 것
                "held": bool(f.get("held")),
                # 다리 셋으로 잡고 넷째를 계산한 것 (legs 의 마지막이 계산한 다리)
                "legs3": bool(f.get("inferred")),
                "legs": [[round(px, 4), round(py, 4)] for px, py in f["legs"]],
                "dist": round(math.hypot(f["x"], f["y"]), 3),
                # 카메라 검증: 다리 자리에 기둥이 세로로 서 있는가
                #   None = 확인 못 함(화각 밖·뎁스 없음), 그 경우 라이다만 믿는다
                # 화각에 든 다리 중 하나쯤은 가려지거나 잡음이 낄 수 있어
                # 전부를 요구하지 않는다. 둘 이상 확인되고, 실패가 하나
                # 이하일 때 통과로 본다. 하나도 못 봤으면 판정 보류(None).
                "cam_ok": (None if ver is None
                           else (ver[0] >= 2 and ver[1] - ver[0] <= 1)),
                "cam_legs": (None if ver is None else "%d/%d" % ver),
                "cam_note": note,
                # 카메라 영상 위에 다리를 그리기 위한 픽셀 좌표
                "cam_uv": (cam[0] if cam else None),
                "cam_size": ([cam[1], cam[2]] if cam else None),
            })
        # 어느 것으로 들어갈 것인가.
        #
        # 등록 대차가 늘어나면서, 방 안의 다른 물건 넷이 우연히 어떤 규격의
        # 사각형을 이뤄 함께 잡히는 일이 생겼다(실측: 진짜 48회에 가짜
        # S300 8회, LG2 1회). 가짜는 대개 화각 밖이라 카메라가 판정을
        # 보류한다. 그래서 '카메라로 확인된 것' 을 먼저 놓고, 같으면 가까운
        # 것을 고른다. 고른 하나에만 target 을 달아 화면과 도킹이 같은 것을
        # 보게 한다. 나머지도 지우지 않고 그대로 내보낸다 - 왜 그렇게
        # 골랐는지 사람이 볼 수 있어야 한다.
        pri = sorted(out, key=lambda c: (
            0 if c["cam_ok"] is True else 1 if c["cam_ok"] is None else 2,
            c["legs3"], -c["score"]))
        kept = []
        for c in pri:
            if any(math.hypot(c["x"] - o["x"], c["y"] - o["y"])
                   < 0.45 * (min(c["a"], c["b"]) + min(o["a"], o["b"]))
                   for o in kept):
                continue
            kept.append(c)
        out = kept
        out.sort(key=lambda c: (0 if c["cam_ok"] is True else
                                1 if c["cam_ok"] is None else 2, c["dist"]))
        target = None
        for c in out:
            if c["cam_ok"] is not False:
                c["target"] = True
                target = c["cart"]
                break
        for c in out:
            c.setdefault("target", False)

        frame = m.header.frame_id or "laser"
        al = self._align(next((c for c in out if c["target"]), None), frame)
        am = StringMsg()
        am.data = json.dumps(al, ensure_ascii=False)
        self.pub_al.publish(am)

        msg = StringMsg()
        msg.data = json.dumps({"frame": m.header.frame_id or "laser",
                               "stamp": {"sec": m.header.stamp.sec,
                                         "nanosec": m.header.stamp.nanosec},
                               "target": target,
                               "align": al,
                               "candidates": len(cands),
                               "carts": out}, ensure_ascii=False)
        self.pub.publish(msg)
        self._markers(out, m.header.frame_id or "laser")

        if len(out) != self._last_n:
            self._last_n = len(out)
            if out:
                f = out[0]
                extra = ("" if len(out) == 1
                         else " · 다른 후보 %d개" % (len(out) - 1))
                self.get_logger().info(
                    "대차 %s 인식 — 앞 %.2fm, 옆 %+.2fm, %+.0f°, 점수 %.2f, "
                    "카메라 %s (후보 %d개 중)%s" %
                    (f["cart"], f["x"], f["y"], f["yaw_deg"], f["score"],
                     "통과" if f["cam_ok"] else
                     "보류" if f["cam_ok"] is None else "실패",
                     len(cands), extra))
            else:
                self.get_logger().info("대차 없음 (후보 %d개)" % len(cands))

    def _markers(self, carts, frame):
        arr = MarkerArray()
        mk = Marker()
        mk.header.frame_id = frame
        mk.action = Marker.DELETEALL
        arr.markers.append(mk)
        for i, c in enumerate(carts):
            for j, (lx, ly) in enumerate(c["legs"]):
                m = Marker()
                m.header.frame_id = frame
                m.header.stamp = self.get_clock().now().to_msg()
                m.ns = "cart_legs"
                m.id = i * 10 + j
                m.type = Marker.CYLINDER
                m.action = Marker.ADD
                m.pose.position.x = float(lx)
                m.pose.position.y = float(ly)
                m.pose.orientation.w = 1.0
                m.scale.x = m.scale.y = 0.06
                m.scale.z = 0.02
                m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.82, 0.25, 0.9
                arr.markers.append(m)
        self.pub_mk.publish(arr)


def main():
    rclpy.init()
    node = CartDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
