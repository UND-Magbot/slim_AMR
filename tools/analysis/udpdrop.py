import time,sys
def snap():
    d={}
    for f in ('/proc/net/udp','/proc/net/udp6'):
        for line in open(f).readlines()[1:]:
            c=line.split(); d[(f,c[1],c[9])]=int(c[12])
    s=open('/proc/net/snmp').read().split('\n'); u=[l for l in s if l.startswith('Udp:')]; h=u[0].split(); v=u[1].split()
    return d,int(v[h.index('RcvbufErrors')]),int(v[h.index('InDatagrams')])
a,ea,ia=snap(); T=float(sys.argv[1]); time.sleep(T); b,eb,ib=snap()
print('전체: 받은 %d개/초, 버림 %d개/초'%((ib-ia)/T,(eb-ea)/T))
inc=sorted(((b[k]-a.get(k,0),int(k[1].split(':')[1],16)) for k in b if b[k]-a.get(k,0)>0),reverse=True)[:10]
print('포트별 버림 증가:',inc)
