#!/usr/bin/env python3
"""손으로 밀어보는 접지 시험 — 모터를 빼고 구동륜 접지만 본다.

축을 해제(free)한 상태에서 사람이 로봇을 똑바로 밀면, 두 구동륜이
모두 지면에 닿아 있는 한 좌우 엔코더는 거의 같은 카운트를 센다.
한쪽이 뜨거나 미끄러지면 그쪽만 카운트가 뒤처지거나 앞선다.

모터 지령이 개입하지 않으므로 "드라이브 문제"와 "접지 문제"를 분리할 수 있다.

    python3 tools/push_test.py --secs 25
그동안 로봇을 앞으로 2~3m 천천히 똑바로 밀어 주세요.
"""
import argparse, math, statistics, sys, time
import rclpy, yaml
from rclpy.node import Node
from diagnostic_msgs.msg import DiagnosticArray

ap=argparse.ArgumentParser()
ap.add_argument("--secs",type=float,default=25.0)
ap.add_argument("--params",default="/home/und_amr1/ros2_ws/src/und_amr_bringup/config/amr_params.yaml")
a=ap.parse_args()
P=yaml.safe_load(open(a.params))["base_node"]["ros__parameters"]
D,K=P["wheel_diameter"],P.get("wheel_diameter_ratio",1.0)
CPR=float(P["counts_per_wheel_rev"])
RL=D*(2*K/(1+K))/2.0; RR=D*(2/(1+K))/2.0

rclpy.init()
class R(Node):
    def __init__(s):
        super().__init__("push"); s.enc={}
        s.create_subscription(DiagnosticArray,"/diagnostics",s._dg,10)
    def _dg(s,m):
        for st in m.status:
            if not st.name.startswith("motor/axis"): continue
            ax=int(st.name.split("axis")[-1])
            for kv in st.values:
                if kv.key=="position": s.enc[ax]=int(kv.value)
    def sp(s,t):
        t0=time.time()
        while time.time()-t0<t: rclpy.spin_once(s,timeout_sec=0.02)

n=R(); n.sp(2.5)
if 1 not in n.enc or 2 not in n.enc:
    print("!! /diagnostics 미수신"); sys.exit(1)
print("="*62)
print("  축 해제 상태입니다. 지금부터 %.0f초 동안"%a.secs)
print("  로봇을 앞으로 2~3 m 천천히, 되도록 똑바로 밀어 주세요.")
print("="*62)
p0=dict(n.enc)
samples=[]
t0=time.time()
last=None
while time.time()-t0<a.secs:
    n.sp(0.25)
    cur=dict(n.enc)
    if 1 in cur and 2 in cur:
        d1=cur[1]-p0[1]; d2=cur[2]-p0[2]
        samples.append((time.time()-t0,d1,d2))
        if last is None or abs(d1-last)>400:
            last=d1
            sk=(d1-d2)/max(abs(d1),abs(d2),1)*100
            print("   %5.1fs  좌 %+7d  우 %+7d   차 %+6.2f%%"%(time.time()-t0,d1,d2,sk))
p1=dict(n.enc)
d1=p1[1]-p0[1]; d2=p1[2]-p0[2]
dl=d1/CPR*2*math.pi*RL; dr=d2/CPR*2*math.pi*RR
print()
print("="*62)
print("  총 이동  좌 %+.4f m (%+d counts) / 우 %+.4f m (%+d counts)"%(dl,d1,dr,d2))
if max(abs(d1),abs(d2))<500:
    print("  !! 거의 안 움직였습니다. 다시 시도해 주세요")
else:
    skew=(d1-d2)/max(abs(d1),abs(d2))*100
    print("  좌우 차이 %+.2f%%"%skew)
    print()
    if abs(skew)<1.0:
        print("  → 양쪽 구동륜 모두 접지 정상. 손으로 밀 때는 문제가 없다는 뜻이고,")
        print("     주행 중 벌어지는 것은 구동 토크가 걸릴 때만 생기는 현상이다.")
    else:
        print("  → 손으로 밀어도 %+.2f%% 벌어진다. 한쪽 구동륜이 접지를 잃고 있다."%skew)
        print("     %s 바퀴가 %s"%("좌" if abs(d1)>abs(d2) else "우",
              "더 돌고 있음(뜨거나 미끄러짐)"))
    # 구간별 변화도 본다
    if len(samples)>6:
        mid=[]
        for i in range(3,len(samples)):
            a1=samples[i][1]-samples[i-3][1]; a2=samples[i][2]-samples[i-3][2]
            if max(abs(a1),abs(a2))>150: mid.append((a1-a2)/max(abs(a1),abs(a2))*100)
        if mid:
            print()
            print("  구간별 좌우 차이: 평균 %+.2f%%  범위 %+.2f ~ %+.2f%%  (구간 %d개)"
                  %(statistics.mean(mid),min(mid),max(mid),len(mid)))
            print("  구간마다 크게 튀면 = 특정 지점에서만 접지를 잃는 것 (바닥 요철)")
print("="*62)
rclpy.shutdown()
