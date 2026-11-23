import csv,math,sys,datetime,numpy as np
def load(fn):
    R=list(csv.DictReader(open(fn)))
    od=np.array([(float(r["t"]),float(r["a"]),float(r["b"])) for r in R if r["src"]=="odom"])
    ps=np.array([(float(r["t"]),float(r["a"]),float(r["b"]),float(r["c"])) for r in R if r["src"]=="pose"])
    return od,ps
def segs(od):
    mv=od[(np.abs(od[:,1])>0.03)]
    out=[];s=p=None
    for t in mv[:,0]:
        if s is None or t-p>3:
            if s and p-s>8: out.append((s,p))
            s=t
        p=t
    if s and p-s>8: out.append((s,p))
    return out
def analyze(fn):
    od,ps=load(fn)
    print("==",fn)
    for a,b in segs(od):
        P=ps[(ps[:,0]>=a)&(ps[:,0]<=b)]
        dx=np.diff(P[:,1]);dy=np.diff(P[:,2]);th=P[:-1,3]
        lat=-np.sin(th)*dx+np.cos(th)*dy; fwd=np.cos(th)*dx+np.sin(th)*dy
        w=np.interp(P[:-1,0],od[:,0],od[:,2]); v=np.interp(P[:-1,0],od[:,0],od[:,1])
        k=(np.abs(w)<0.15)&(np.abs(v)>0.05)
        L=lat[k]
        # 직진 구간 지도 궤적의 좌우 흔들림: 5cm 간격 재표본, 2차 차분 부호 바뀜
        xy=P[:,1:3]; d=np.r_[0,np.cumsum(np.hypot(*np.diff(xy,axis=0).T))]
        if d[-1]<1.0: continue
        s=np.arange(0,d[-1],0.05); X=np.interp(s,d,xy[:,0]); Y=np.interp(s,d,xy[:,1])
        h=np.unwrap(np.arctan2(np.diff(Y),np.diff(X))); c=np.diff(h)/0.05
        # 평활 후 부호 바뀜(곡률 0.15/m 넘는 것만)
        sig=np.sign(c)*(np.abs(c)>0.15); sig=sig[sig!=0]; flips=int(np.sum(sig[1:]!=sig[:-1]))
        print("%s %4.1fs 거리 %.1fm | 직진 중 0.05s당 옆 이동 최대 %.1fmm, 2mm넘는 튐 %d회 | 지도궤적 꺾임방향 바뀜 %d회, 곡률 표준편차 %.2f/m"%(
            datetime.datetime.fromtimestamp(a).strftime("%H:%M:%S"),b-a,d[-1],np.abs(L).max()*1000 if len(L) else 0,int((np.abs(L)>0.002).sum()),flips,np.std(c)))
for fn in sys.argv[1:]: analyze(fn)
