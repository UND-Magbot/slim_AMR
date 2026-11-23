#!/usr/bin/env python3
"""직진 휨 검증 — /cmd_vel 로 직진시키고 라이다로 실제 회전량을 잰다.

좌우 바퀴 직경차 보정(wheel_diameter_ratio)이 효과가 있는지 확인하는 용도.
base_node 와 라이다가 떠 있어야 한다.

    ros2 run und_amr_bringup base_node --ros-args --params-file .../amr_params.yaml
    ros2 launch und_amr_bringup lidar.launch.py
    python3 tools/verify_straight.py --meters 2.0 --reps 2

⚠ 캐스터 4개 구성이라 정지 후 첫 출발에서 옆으로 밀린다.
  측정 전 같은 방향으로 짧게 굴려 캐스터를 정렬시킨다(--prealign).
"""
import argparse, math, statistics, sys, time
sys.path.insert(0, "/home/und_amr1/projects/UND_AMR/tools")
import numpy as np, rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger
from safe_stop import install_safe_stop
from diagnostic_msgs.msg import DiagnosticArray
from scan_match import match

ap = argparse.ArgumentParser()
ap.add_argument("--meters", type=float, default=2.0)
ap.add_argument("--speed", type=float, default=0.20)
ap.add_argument("--prealign", type=float, default=0.5)
ap.add_argument("--back-speed", type=float, default=0.07, dest="back_speed",
                help="복귀 후진 속도. 후진하면 캐스터가 선행이 되어 불안정해져 "
                     "옆으로 크게 차는데(9/10: 획획 3회, 이후 회차 전부 오염), "
                     "이 흔들림은 속도에 민감하므로 전진보다 느리게 간다")
ap.add_argument("--reps", type=int, default=2)
ap.add_argument("--no-return", action="store_true",
                help="측정 후 원위치로 후진하지 않는다 (공간을 계속 소모함)")
ap.add_argument("--scans", type=int, default=14)
a = ap.parse_args()

STOP_M = 0.40      # 진행 방향에 이보다 가까우면 즉시 정지
VEER_DEG = 12.0    # 직진 지령인데 이보다 틀어지면 즉시 정지


class Rig(Node):
    def __init__(s):
        super().__init__("verify_straight")
        s.scan = None; s.meta = None; s.odom = None; s.enc = {}
        s.abort = None; s.trace = []
        s.cmd = {}; s.act = {}
        s.create_subscription(LaserScan, "/scan", s._sc, qos_profile_sensor_data)
        s.create_subscription(Odometry, "/odom", s._od, 10)
        s.create_subscription(DiagnosticArray, "/diagnostics", s._dg, 10)
        s.pub = s.create_publisher(Twist, "/cmd_vel", 10)
        # 축이 해제(shaft free)된 상태면 지령을 보내도 로봇이 안 움직인다.
        # safe_stop 이 끝날 때 축을 풀어두므로 시작할 때마다 다시 켜야 한다.
        s.cli_en = s.create_client(Trigger, "/base_node/enable")

    def _sc(s, m): s.scan = m; s.meta = m

    def _dg(s, m):
        # 엔코더만으로는 '소프트웨어가 뭘 보냈는지' 를 알 수 없다.
        # cmd_rpm 은 드라이브가 받은 값을 되돌려주는 것이라, 이게 좌우
        # 같으면 지령 경로는 결백하다. 고장 순간에 이걸 남긴다.
        for st in m.status:
            if not st.name.startswith("motor/axis"): continue
            ax = int(st.name.split("axis")[-1])
            for kv in st.values:
                if kv.key == "position": s.enc[ax] = int(kv.value)
                elif kv.key == "cmd_rpm": s.cmd[ax] = int(kv.value)
                elif kv.key == "act_rpm": s.act[ax] = int(kv.value)
    def _od(s, m): s.odom = m

    def spin(s, secs):
        t0 = time.time()
        while time.time() - t0 < secs:
            rclpy.spin_once(s, timeout_sec=0.02)

    def grab(s, n):
        s.spin(1.0)                       # 큐에 남은 오래된 스캔 버리기
        acc = None; got = 0; t0 = time.time()
        while got < n and time.time() - t0 < 14:
            s.scan = None
            while s.scan is None and time.time() - t0 < 14:
                rclpy.spin_once(s, timeout_sec=0.05)
            m = s.scan
            if acc is None: acc = [[] for _ in m.ranges]
            for i, r in enumerate(m.ranges):
                if math.isfinite(r) and m.range_min < r < m.range_max: acc[i].append(r)
            got += 1
        return np.array([statistics.median(v) if len(v) >= 3 else np.nan for v in acc])

    def odom_xy(s):
        s.spin(0.3)
        p = s.odom.pose.pose.position
        q = s.odom.pose.pose.orientation
        th = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
        return p.x, p.y, th

    def drive(s, dist, speed, guard=True):
        """오도메트리로 |dist| 만큼 갈 때까지 주행. speed 가 음수면 후진.

        주행 중에도 계속 감시한다. 출발 전 한 번만 여유를 보면, 가다가
        휘어서 옆 물건에 부딪히는 것을 아무도 못 막는다(9/10 에 박았다).
        s.abort 에 중단 사유가 남는다."""
        dist = abs(dist)
        x0, y0, th0 = s.odom_xy()
        t = Twist(); t.linear.x = speed
        t0 = time.time()
        s.abort = None
        s.trace = []
        e_l0 = e_r0 = None
        # speed 가 음수(후진)일 때 dist/speed 가 음수가 되어 루프가 아예
        # 안 돌던 버그가 있었다. 반드시 절대값으로 시간제한을 잡는다.
        #
        # 시간 여유를 15초나 두면, 오도메트리가 덜 세는 상황에서 로봇이
        # 한참 더 간다(9/10: 1.5m 요청에 2.12m 주행, 전방여유 3.38->1.16m).
        # 정상이라면 dist/speed 면 끝나므로 50% 여유면 충분하다.
        lim = dist / abs(speed) * 1.5 + 3.0
        while time.time() - t0 < lim:
            s.pub.publish(t)
            rclpy.spin_once(s, timeout_sec=0.02)
            x, y, th = s.odom_xy()
            # 어떻게 틀어지는지 남긴다. 서서히 도는 것과 홱 튀는 것은
            # 원인이 다르고, 사후에 숫자가 없으면 구분할 수 없다.
            if 1 in s.enc and 2 in s.enc and s.enc[1] is not None:
                if e_l0 is None:
                    e_l0, e_r0 = s.enc[1], s.enc[2]
                s.trace.append((time.time() - t0,
                                math.hypot(x - x0, y - y0),
                                math.degrees(math.atan2(math.sin(th - th0),
                                                        math.cos(th - th0))),
                                s.enc[1] - e_l0, s.enc[2] - e_r0,
                                s.cmd.get(1), s.act.get(1),
                                s.cmd.get(2), s.act.get(2)))
            if math.hypot(x - x0, y - y0) >= dist: break
            if guard and s.scan is not None:
                rng = s.scan.ranges
                d = (s.clearance(rng, 45.0) if speed > 0
                     else s.rear_clear(rng, 45.0))
                if 0.0 < d < STOP_M:
                    s.abort = "진행 방향 %.2f m 앞에 장애물" % d
                    break
                # 직진 지령인데 크게 틀어지면 그대로 두면 안 된다.
                dth = abs(math.degrees(math.atan2(math.sin(th - th0),
                                                  math.cos(th - th0))))
                if dth > VEER_DEG:
                    s.abort = "직진 지령인데 %.1f 도 틀어짐" % dth
                    break
        else:
            # 루프가 break 없이 끝났다 = 시간 초과. 거리를 못 채운 것이다.
            if s.abort is None:
                s.abort = "시간 초과 (오도메트리가 거리를 못 채움)"
        s.pub.publish(Twist())
        for _ in range(10):
            s.pub.publish(Twist()); s.spin(0.05)
        s.spin(1.5)
        x, y, th = s.odom_xy()
        return math.hypot(x - x0, y - y0), th

    def rear_clear(s, rng, half_deg=35.0):
        """후방 최소거리 (원위치 복귀용)."""
        m = s.meta
        v = []
        for i, r in enumerate(rng):
            if not np.isfinite(r): continue
            d = math.degrees(m.angle_min + i * m.angle_increment)
            if abs(d) >= 180.0 - half_deg: v.append(r)
        return min(v) if v else 0.0

    def clearance(s, rng, half_deg=35.0):
        """전방 +-half_deg 섹터의 최소거리. 직진 테스트에는 진행 방향
        회랑만 보면 된다 (전 방향 최소값을 쓰면 뒤쪽 벽에 걸려 과하게 막힌다)."""
        m = s.meta
        v = []
        for i, r in enumerate(rng):
            if not np.isfinite(r): continue
            d = math.degrees(m.angle_min + i * m.angle_increment)
            if -half_deg <= d <= half_deg: v.append(r)
        return min(v) if v else 0.0


rclpy.init(); n = Rig()
install_safe_stop(n)

# 축 켜기 — 안 켜면 엔코더가 0 인 채로 전 회차가 지나간다(9/10 에 그렇게 됐다)
if not n.cli_en.wait_for_service(timeout_sec=5.0):
    print("!! /base_node/enable 서비스 없음 — base_node 가 떠 있는지 확인하세요")
    sys.exit(1)
n.cli_en.call_async(Trigger.Request())
n.spin(2.5)
if not any(d.get("enabled") for d in getattr(n, "diag", {}).values()) \
        and not n.enc:
    pass          # 진단을 아직 못 받았을 수 있으니 여기서 막지는 않는다
print("  축 활성화 요청 완료")
n.spin(2.0)
if n.odom is None or n.scan is None:
    print("!! /odom 또는 /scan 이 안 옵니다. base_node / lidar 확인"); sys.exit(1)

rows = []
try:
    for k in range(a.reps):
        if a.prealign > 0:
            n.drive(a.prealign, a.speed); n.spin(1.0)
        A = n.grab(a.scans); meta = n.meta
        clr = n.clearance(A)
        print("      전방 여유 %.2f m (필요 %.2f m)" % (clr, a.meters + 0.8))
        if clr < a.meters + 0.8:
            print("!! 전방 여유 부족 — 중단. 사람이 앞에 서 있지 않은지 확인하세요"); break
        e0 = dict(n.enc)
        od_d, th_before = n.drive(a.meters, a.speed)
        e1 = dict(n.enc)
        if n.abort:
            print("  [%d] !! 주행 중단 — %s (%.2f m 진행 후)"
                  % (k + 1, n.abort, od_d))
            tr = [r for r in n.trace if r[1] > 0.001]
            if tr:
                print("      틀어지는 과정")
                print("        거리     각도      축1 counts  cmd/act   "
                      "축2 counts  cmd/act")
                step = max(1, len(tr) // 16)
                prev = None
                for r in tr[::step]:
                    mk = ""
                    if prev is not None and abs(r[2] - prev) > 2.0:
                        mk = "  <-- %.1f deg" % (r[2] - prev)
                    print("        %5.3f m %+7.2f  %+8d  %4s/%-4s  %+8d  %4s/%-4s%s"
                          % (r[1], r[2], r[3], r[5], r[6], r[4], r[7], r[8], mk))
                    prev = r[2]
                # 지령이 좌우로 갈렸는지 한 줄로 판정한다
                bad = [r for r in tr if r[5] is not None and r[7] is not None
                       and abs(r[5] - r[7]) > 1]
                print()
                if bad:
                    print("      !! 좌우 지령이 다른 구간 %d 개 — 소프트웨어 문제"
                          % len(bad))
                    for r in bad[:5]:
                        print("         %5.3f m  축1 cmd %+d / 축2 cmd %+d"
                              % (r[1], r[5], r[7]))
                else:
                    print("      좌우 지령은 전 구간 동일 (소프트웨어는 같은 값을 보냄)")
            print("      주변을 정리하고 다시 실행하세요")
            break
        dc1 = e1.get(1, 0) - e0.get(1, 0)
        dc2 = e1.get(2, 0) - e0.get(2, 0)
        B = n.grab(a.scans)
        (tx, ty, th), info = match(A, B, meta, init=(od_d, 0.0, 0.0), rmax=10.0)
        dist = math.hypot(tx, ty)
        # ICP 는 초기값이 나쁘면 엉뚱한 최소점으로 간다. 초기값을 흔들어
        # 같은 값으로 모이는지 반드시 확인한다 (이 검사를 빠뜨려 15% 틀린
        # 값을 그대로 믿은 적이 있다).
        alts = []
        for f in (0.85, 0.95, 1.05, 1.15):
            (x2, y2, t2), _ = match(A, B, meta, init=(od_d * f, 0.0, 0.0), rmax=10.0)
            alts.append((math.hypot(x2, y2), math.degrees(t2)))
        dsp = max([dist] + [v[0] for v in alts]) - min([dist] + [v[0] for v in alts])
        asp = max([deg_ := math.degrees(th)] + [v[1] for v in alts]) - \
              min([deg_] + [v[1] for v in alts])
        # 수렴 안정성만으로는 부족하다. 모든 초기값이 "똑같이 틀린" 곳으로
        # 수렴하는 경우가 있어서, 정합 잔차(rms)도 함께 본다.
        # 정상 정합은 rms 0.01~0.05, 실패한 정합은 0.3 이상으로 확연히 갈린다.
        # 좌우 엔코더가 크게 벌어진 회차는 한쪽 구동륜이 헛돈 것이다.
        # 장착각 역산은 "원호를 따라 굴러갔다"를 전제로 하는데 미끄러지면
        # 그 전제가 깨져 값이 통째로 틀린다(9/10: 4.65% 회차가 -11.7도).
        skew_pct = (dc1 - dc2) / max(abs(dc1), abs(dc2), 1) * 100
        stable = (dsp < 0.01 and asp < 0.3 and info["rms"] < 0.10
                  and abs(skew_pct) < 2.0)
        if not stable:
            why = []
            if dsp >= 0.01 or asp >= 0.3:
                why.append("초기값 ±15%%에서 산포 %.3f m / %.2f deg" % (dsp, asp))
            if info["rms"] >= 0.10:
                why.append("정합 잔차 rms %.3f (정상 0.05 이하)" % info["rms"])
            if abs(skew_pct) >= 2.0:
                why.append("좌우 엔코더 %+.2f%% 벌어짐 (구동륜 헛돎, 2%% 이상)"
                           % skew_pct)
            print("      ⚠ 기각 — " + ", ".join(why))
        if od_d < 0.05 or (dc1 == 0 and dc2 == 0):
            print("  [%d] !! 로봇이 움직이지 않았습니다 (엔코더 좌 %+d / 우 %+d, "
                  "오도 %.4f m)" % (k + 1, dc1, dc2, od_d))
            print("      축이 꺼져 있거나(shaft free) 비상정지가 눌려 있는지 확인하세요:")
            print("        curl -s http://127.0.0.1:8090/chassis/status")
            break
        # 직진에서는 지면이 양쪽 바퀴를 묶어주므로 총합만 보면 0% 로 나온다.
        # 미끄러짐은 '순간적으로' 벌어졌다가 다시 묶이므로 구간 최대치를 본다.
        # 창을 짧게 잡으면 미끄러짐이 아니라 피드백 타이밍 잡음을 재게 된다.
        # base_node 가 20Hz 로 두 축을 읽는데 한 축이 이전 주기 값이면
        # 0.15m/s 에서 57 counts(7.5mm) 가 어긋나 보인다. 표본 간격이
        # 300 counts 면 그것만으로 19% 가 나온다(9/11 에 35.7% 로 오탐).
        # 창을 1200 counts(약 0.15m) 로 넓히면 타이밍분이 5% 아래로 내려가고,
        # 진짜 미끄러짐은 누적되므로 그대로 남는다.
        WIN = 1200
        peak, peak_at, peak_dth = 0.0, 0.0, 0.0
        tr = n.trace
        j = 0
        for i in range(1, len(tr)):
            a1 = tr[i][3] - tr[j][3]
            a2 = tr[i][4] - tr[j][4]
            b = max(abs(a1), abs(a2))
            if b < WIN:
                continue
            g = abs(a1 - a2) / b * 100
            if g > peak:
                peak, peak_at, peak_dth = g, tr[i][1], tr[i][2]
            j = i
        deg = math.degrees(th)
        per2m = deg / dist * 2.0
        # 라이다 장착각(lidar_yaw) 역산.
        #   라이다 프레임이 로봇 전진축에서 +a 만큼 (좌로) 틀어져 있으면,
        #   로봇이 th 만큼 휘며 X 를 갔을 때 라이다가 보는 가로변위는
        #       ty = X * sin(th/2 - a)
        #   이므로  a = th/2 - asin(ty/X).
        #   0 으로 두면 오도메트리와 스캔이 계속 어긋나 지도가 뒤틀린다.
        lyaw = th / 2.0 - math.asin(max(-1.0, min(1.0, ty / dist)))
        if stable:
            rows.append((od_d, dist, deg, per2m, info["rms"], ty, math.degrees(lyaw)))
        back = None
        skew = (dc1 - dc2) / max(abs(dc1), abs(dc2), 1) * 100
        print("  [%d] 엔코더 좌 %+d / 우 %+d (차 %+.2f%%) | 오도 %.4f m | 라이다 %.4f m (차 %+.1f mm) "
              "| 회전 %+.3f deg → %+.3f deg/2m | 가로 %+.1f mm | 장착각 %+.2f deg "
              "| rms %.4f %s"
              % (k + 1, dc1, dc2, skew, od_d, dist, (dist - od_d) * 1000, deg, per2m,
                 ty * 1000, math.degrees(lyaw), info["rms"], "OK" if stable else "기각"))
        print("      구간 최대 좌우차 %.1f%% (%.2f m 지점, 그때 각도 %+.2f deg)%s"
              % (peak, peak_at, peak_dth,
                 "   <-- 미끄러짐" if peak > 15 else ""))

        # 원위치 복귀 — 전진한 만큼(사전정렬 포함) 그대로 후진한다.
        # 이걸 안 하면 회차마다 공간을 계속 먹어 벽에 부딪힌다.
        if not a.no_return:
            C = n.grab(6)
            rc = n.rear_clear(C)
            need = od_d + a.prealign + 0.5
            if rc < need:
                print("      ⚠ 후방 여유 %.2f m < 필요 %.2f m — 복귀 생략" % (rc, need))
            else:
                back, _ = n.drive(od_d + a.prealign, -abs(a.back_speed))
                print("      복귀 %.3f m 후진 %.2f m/s (후방여유 %.2f m)%s"
                      % (back, a.back_speed, rc,
                         "  ⚠ " + n.abort if n.abort else ""))
                n.spin(1.0)
finally:
    for _ in range(10):
        n.pub.publish(Twist()); n.spin(0.05)

print()
print("=" * 70)
if rows:
    p = [r[3] for r in rows]
    e = [(r[1] - r[0]) / r[0] * 100 for r in rows if r[0] > 1e-6]
    ly = [r[6] for r in rows]
    print("  직진 2m당 휨: 평균 %+.3f deg  (보정 전 기준값 -0.81 deg)" % statistics.mean(p))
    if e:
        print("  오도메트리 거리 오차: 평균 %+.3f %%" % statistics.mean(e))
    print()
    m = statistics.mean(ly)
    sd = statistics.pstdev(ly) if len(ly) > 1 else 0.0
    print("  라이다 장착각(lidar_yaw): 평균 %+.3f deg  표준편차 %.3f  (%d 회)"
          % (m, sd, len(ly)))
    print("                            = %+.4f rad" % math.radians(m))
    print("  (전체 %d 회 중 %d 회 채택)" % (a.reps, len(ly)))
    if len(ly) < 3:
        print("  채택 회차가 %d 회뿐이다. --reps 를 늘려 3 회 이상 모을 것" % len(ly))
    elif sd > 0.5:
        print("  ⚠ 회차간 흩어짐이 %.2f deg 로 크다. 캐스터 정렬(--prealign)을"
              % sd)
        print("    늘리거나 거리를 늘려 다시 잴 것")
    else:
        print("  -> urdf/amr.urdf.xacro 의 lidar_yaw 를 이 rad 값으로")
else:
    print("  쓸 수 있는 회차가 없다 (전부 기각). rms 와 산포 경고를 확인할 것")
print("=" * 70)
rclpy.shutdown()
