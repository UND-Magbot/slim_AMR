#!/usr/bin/env python3
"""축별 지령 대비 실제 rpm 확인 — 한쪽 바퀴가 안 도는지 진단.

짧게(기본 1.5초) 저속 직진시키며 /diagnostics 의 축별 cmd_rpm / act_rpm /
엔코더 position 을 기록한다. 양축이 지령을 따라가면 모터는 정상이고,
한쪽만 뒤처지면 그쪽 구동계(모터/드라이브/기계 저항) 문제다.
"""
import argparse, math, statistics, sys, time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import Twist
from safe_stop import install_safe_stop
from sensor_msgs.msg import LaserScan

ap=argparse.ArgumentParser()
ap.add_argument("--secs",type=float,default=1.5)
ap.add_argument("--speed",type=float,default=0.15)
ap.add_argument("--spin",action="store_true",help="직진 대신 제자리 회전으로 시험")
a=ap.parse_args()

rclpy.init()
class R(Node):
    def __init__(s):
        super().__init__("motor_check"); s.d={}; s.scan=None; s.hist=[]
        s.create_subscription(DiagnosticArray,"/diagnostics",s._dg,10)
        s.create_subscription(LaserScan,"/scan",lambda m:setattr(s,'scan',m),
                              qos_profile_sensor_data)
        s.pub=s.create_publisher(Twist,"/cmd_vel",10)
    def _dg(s,m):
        cur={}
        for st in m.status:
            if not st.name.startswith("motor/axis"): continue
            ax=int(st.name.split("axis")[-1])
            cur[ax]={kv.key:kv.value for kv in st.values}
        if cur: s.d=cur; s.hist.append((time.time(),cur))
    def spin_for(s,t):
        t0=time.time()
        while time.time()-t0<t: rclpy.spin_once(s,timeout_sec=0.02)
    def front(s,half=35.0):
        m=s.scan
        if m is None: return None
        b=None
        for i,r in enumerate(m.ranges):
            if not math.isfinite(r) or not (m.range_min<r<m.range_max): continue
            d=math.degrees(m.angle_min+i*m.angle_increment)
            if -half<=d<=half and (b is None or r<b): b=r
        return b

n=R(); n.spin_for(2.5)
install_safe_stop(n)

# 축 켜기 — safe_stop 이 끝날 때 축을 풀어두므로 매번 다시 켜야 한다.
# 안 켜면 지령만 나가고 엔코더가 0 인 채로 시험이 끝난다(9/10).
from std_srvs.srv import Trigger as _Trg
_cli = n.create_client(_Trg, "/base_node/enable")
if _cli.wait_for_service(timeout_sec=5.0):
    _cli.call_async(_Trg.Request())
    n.spin_for(2.5)
    print("  축 활성화 요청 완료")
else:
    print("!! /base_node/enable 서비스 없음")
if not n.d: print("!! /diagnostics 미수신"); sys.exit(1)
need=a.secs*a.speed+0.8
f=n.front()
if not a.spin:
    if f is None or f<need:
        print("!! 전방 여유 %.2f m < 필요 %.2f m — 중단"%(f if f else -1,need)); sys.exit(1)
    print("전방 여유 %.2f m — 진행"%f)

print("\n초기 상태:")
for ax in sorted(n.d):
    v=n.d[ax]; print("  axis%d online=%s enabled=%s err=%s volt=%s pos=%s"
                     %(ax,v.get('online'),v.get('enabled'),v.get('error'),v.get('voltage'),v.get('position')))
p0={ax:int(n.d[ax]["position"]) for ax in n.d}
t=Twist()
if a.spin: t.angular.z=0.5
else:      t.linear.x=a.speed
print("\n>> %.1f초 %s"%(a.secs,"제자리 회전" if a.spin else "직진 %.2f m/s"%a.speed))
n.hist=[]
t0=time.time()
while time.time()-t0<a.secs:
    n.pub.publish(t); rclpy.spin_once(n,timeout_sec=0.02)
for _ in range(12): n.pub.publish(Twist()); n.spin_for(0.05)
n.spin_for(1.5)
p1={ax:int(n.d[ax]["position"]) for ax in n.d}

# --- 한쪽 축이 순간적으로 멈추는지 찾는다 -------------------------------
# 9/10: 주행 중 한쪽 바퀴가 안 움직여 로봇이 홱 돌아가는 것을 눈으로 확인.
# 평균만 보면 안 잡히므로 전 표본을 훑어 '지령은 있는데 실제가 0' 인 구간을 센다.
RAMP_S = 2.0      # 출발 램프. 이 구간은 실제 rpm 이 아직 올라오는 중이다.
                  # 0.8 로 뒀더니 1.38s 의 램프를 멈춤으로 오탐했다(9/11)
stalls = {1: [], 2: []}
glitch = []       # 제자리 회전인데 좌우 지령 부호가 같은 순간 = 시리얼 깨짐


def _rpm(v, key):
    """부호를 살려서 돌려준다. abs() 로 받으면 좌우 부호 비교가 무의미해진다
    (9/11: 그 탓에 '시리얼 깨짐' 을 823회 오탐했다)."""
    try:
        return int(v.get(key, 0) or 0)
    except (TypeError, ValueError):
        return None


for ts, cur in n.hist:
    if ts - t0 < RAMP_S:
        continue
    if 1 not in cur or 2 not in cur:
        continue
    got = {ax: (_rpm(cur[ax], "cmd_rpm"), _rpm(cur[ax], "act_rpm")) for ax in (1, 2)}
    if any(v[0] is None or v[1] is None for v in got.values()):
        continue
    for ax, other in ((1, 2), (2, 1)):
        c, ac = abs(got[ax][0]), abs(got[ax][1])
        oc, oac = abs(got[other][0]), abs(got[other][1])
        # 한쪽만 죽은 경우만 잡는다. 둘 다 0 이면 감속이나 램프다.
        # 반대쪽 기준을 절대값(5rpm)으로 두면, 지령이 낮을 때 놓친다.
        # (9/10: 반대쪽이 4rpm 이라 진짜 멈춤을 통과시켰다)
        # '반대쪽이 자기 지령의 절반은 따라가고 있는가' 로 본다.
        if c >= 3 and ac <= 1 and oc >= 3 and oac >= oc * 0.5:
            stalls[ax].append(ts - t0)
    # 지령 자체가 튀는 것도 본다. base_node 가 보낸 값과 다르면
    # 시리얼이 깨진 것이고, 그러면 act 도 믿을 수 없다.
    c1, c2 = got[1][0], got[2][0]
    if c1 and c2 and c1 * c2 > 0 and min(abs(c1), abs(c2)) >= 3:
        glitch.append((ts - t0, c1, c2))
print("\n멈춤 구간 검사 (한쪽만 지령 5rpm 이상인데 실제 1rpm 이하, 램프 %.1fs 제외)"
      % RAMP_S)
bad = False
for ax in (1, 2):
    v = stalls[ax]
    if v:
        bad = True
        print("  !! axis%d 에서 %d 회 — %s"
              % (ax, len(v), ", ".join("%.2fs" % x for x in v[:8])
                 + (" ..." if len(v) > 8 else "")))
    else:
        print("  axis%d 정상 (멈춤 없음)" % ax)
if a.spin and glitch:
    print("  !! 제자리 회전인데 좌우 지령 부호가 같은 순간 %d 회 — %s"
          % (len(glitch), ", ".join("%.2fs(%+d/%+d)" % g for g in glitch[:5])))
    print("     base_node 는 반대 부호를 보낸다. 이렇게 읽혔다면 시리얼 피드백이")
    print("     깨진 것이고, 그러면 act_rpm 도 그대로 믿을 수 없다.")
    bad = True
if bad:
    print("  -> 지령이 나가는데 축이 안 도는 구간이 있다. err 가 0 이면")
    print("     드라이브 알람이 아니라 배선(모터 상선·홀센서 커넥터) 접촉을")
    print("     먼저 의심할 것. 흔들면서 재현되는지 보면 갈린다.")

print("\n  시각   axis1 cmd/act    axis2 cmd/act")
keep=set(range(0,len(n.hist),max(1,len(n.hist)//12)))
for i,(ts,cur) in enumerate(n.hist):
    if i not in keep and not (1 in cur and 2 in cur and
        any(abs(int(cur[x].get("cmd_rpm",0) or 0))>=5 and
            abs(int(cur[x].get("act_rpm",0) or 0))<=1 for x in (1,2))):
        continue
    if 1 in cur and 2 in cur:
        print("  %5.2fs   %4s / %4s      %4s / %4s"
              %(ts-t0,cur[1].get('cmd_rpm'),cur[1].get('act_rpm'),
                cur[2].get('cmd_rpm'),cur[2].get('act_rpm')))
print("\n엔코더 변화:")
for ax in sorted(p1):
    print("  axis%d: %+d counts"%(ax,p1[ax]-p0[ax]))
if 1 in p1 and 2 in p1:
    d1,d2=p1[1]-p0[1],p1[2]-p0[2]
    if a.spin:
        # 부호만 보면 안 된다. 제자리 회전은 양쪽이 '같은 크기로' 반대여야
        # 하고, 크기가 벌어지면 그쪽 바퀴가 헛돈 것이다(9/10: 58% 차이를
        # 부호만 보고 정상으로 통과시켰다).
        if d1*d2 >= 0:
            print("  → !! 같은 부호다. 제자리 회전이 아니다")
        else:
            base = max(abs(d1), abs(d2))
            gap = abs(abs(d1)-abs(d2))/base*100 if base else 0.0
            more = 1 if abs(d1) > abs(d2) else 2
            print("  → 부호 반대 정상. 크기 차이 %.1f%% %s"
                  % (gap, "정상" if gap < 10 else "!! 비정상"))
            if gap >= 10:
                print("     axis%d 이 %.0f%% 더 돌았다. 제자리 회전은 양쪽이"
                      % (more, gap))
                print("     같은 거리를 굴러야 하므로, 더 돈 쪽이 지면을 놓치고")
                print("     헛돈 것이다. 그 바퀴의 접지하중을 의심할 것.")
                print("     (직진에서는 지면이 양쪽을 묶어주므로 잘 안 드러난다)")
    else:
        base=max(abs(d1),abs(d2))
        print("  → 직진은 좌우가 같아야 정상. 차이 %.1f%% %s"
              %(abs(d1-d2)/base*100 if base else 0,
                "정상" if base and abs(d1-d2)/base<0.05 else "!! 한쪽이 안 돌고 있음"))
print("\n최종 상태:")
for ax in sorted(n.d):
    v=n.d[ax]; print("  axis%d enabled=%s err=%s status=%s volt=%s"
                     %(ax,v.get('enabled'),v.get('error'),v.get('status'),v.get('voltage')))
rclpy.shutdown()
