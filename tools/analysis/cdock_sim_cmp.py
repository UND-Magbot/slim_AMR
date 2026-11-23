"""충전소 후진 S자 시뮬 비교 (2026-11-16): 실제 w 지연 0.3s, 라이다 V 위치 잡음."""
import sys, math, random, types, pathlib, importlib.util
T=pathlib.Path.home()/"ros2_ws/src/und_amr_bringup/test"; sys.path.insert(0,str(T))
import conftest, test_charger_dock as tc
api=conftest._load("api_node")
AN=[v for v in vars(api).values() if isinstance(v,type) and hasattr(v,"_compose_map")][0]
def once(pivot, lat0, deg0, seed):
    random.seed(seed)
    n,s=tc.make(AN, start_off=(0,0,0))
    if not pivot: n.CD_PIVOT_YAW=math.radians(1000); n.CD_PIVOT_YAW0=math.radians(999)
    ux,uy=math.cos(s.syaw),math.sin(s.syaw); d=n.CHARGER_HALF_LEN+0.8
    s.r=[s.face[0]+d*ux-lat0*uy, s.face[1]+d*uy+lat0*ux, s.syaw+math.radians(deg0)]
    # 지연: 명령 w 를 0.3초 1차 지연, v 는 0.15초
    act=[0.0,0.0]
    def step(dt, s=s):
        v,w=s.tw
        act[0]+= (v-act[0])*dt/0.15; act[1]+=(w-act[1])*dt/0.30
        s.r[2]+=act[1]*dt; s.r[0]+=act[0]*math.cos(s.r[2])*dt; s.r[1]+=act[0]*math.sin(s.r[2])*dt
    s.step=step
    # 스테이션 자리 잡음 (스캔마다 좌우 ±8mm)
    orig=tc.Sim.scan
    def scan(self=s):
        f=self.face; e=random.gauss(0,0.006)
        self.face=(f[0]-e*uy, f[1]+e*ux); r=orig(self); self.face=f; return r
    s.scan=scan
    lats=[]; 
    real_step=s.step
    def st(dt):
        real_step(dt); g,l,_=n._cdock_err(tuple(s.r),(s.face[0],s.face[1],s.syaw)); lats.append(l)
    s.step=st
    n.cdock_start("충전소"); t=tc.run(n,s,max_t=60)
    g,l,dy=tc.contact_err(n,s)
    L=[x for x in lats]
    sg=[1 if x>0.003 else -1 if x<-0.003 else 0 for x in L]; sg=[x for x in sg if x]
    flips=sum(1 for a,b in zip(sg,sg[1:]) if a!=b)
    over=-min(L) if lat0>0 else max(L)
    return n.cdock["state"],t,l*1000,dy,flips,over*1000
for lat0,deg0 in [(0.025,16),(0.0,16),(0.03,-10),(0.02,3),(-0.05,12)]:
    for pv in (False,True):
        R=[once(pv,lat0,deg0,k) for k in range(10)]
        ok=sum(r[0]=="done" for r in R)
        f=lambda i: sum(abs(r[i]) for r in R)/len(R)
        print("시작 좌우 %+3.0fmm 방향 %+3.0f° %s: 성공 %d/10, 시간 %.1fs, 끝 좌우 |%.1f|mm 방향 |%.2f|°, 좌우 부호바뀜 %.1f회, 반대쪽 넘침 %.1fmm"%(
            lat0*1000,deg0,"제자리 먼저" if pv else "예전      ",ok,f(1),f(2),f(3),f(4),max(r[5] for r in R)))
