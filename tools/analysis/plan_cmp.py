import math, rclpy, numpy as np, io, json, urllib.request
from rclpy.action import ActionClient
from nav2_msgs.action import ComputePathToPose
from PIL import Image
rclpy.init(); n=rclpy.create_node("pcmp"); c=ActionClient(n,ComputePathToPose,"/compute_path_to_pose"); c.wait_for_server(timeout_sec=5)
st=json.load(urllib.request.urlopen("http://localhost:8090/mapping/status",timeout=3))["map"]
a=np.asarray(Image.open(io.BytesIO(urllib.request.urlopen("http://localhost:8090/map.png",timeout=3).read())).convert("L")); H=a.shape[0]
res=st["resolution"]; ox,oy=st["origin"]["x"],st["origin"]["y"]; ys,xs=np.nonzero(a<100); P=np.c_[ox+xs*res, oy+(H-1-ys)*res]
pois=json.load(urllib.request.urlopen("http://localhost:8090/pois",timeout=3))["pois"]
goals={}
for p in pois:
    so=p.get("standoff") or 0
    if p["kind"]=="point": goals[p["name"]]=(p["x"],p["y"],p["yaw"])
    elif p["kind"]=="cart": goals[p["name"]]=(p["x"]+so*math.cos(p["yaw"]),p["y"]+so*math.sin(p["yaw"]),p["yaw"]+math.pi)
    else: goals[p["name"]]=(p["x"]+so*math.cos(p["yaw"]),p["y"]+so*math.sin(p["yaw"]),p["yaw"])
for name,(x,y,yaw) in goals.items():
    g=ComputePathToPose.Goal(); g.goal.header.frame_id="map"; g.goal.pose.position.x=x; g.goal.pose.position.y=y
    g.goal.pose.orientation.z=math.sin(yaw/2); g.goal.pose.orientation.w=math.cos(yaw/2); g.planner_id="GridBased"; g.use_start=False
    f=c.send_goal_async(g); rclpy.spin_until_future_complete(n,f,timeout_sec=5); h=f.result()
    r=h.get_result_async(); rclpy.spin_until_future_complete(n,r,timeout_sec=10); rs=r.result()
    Q=np.array([[q.pose.position.x,q.pose.position.y] for q in rs.result.path.poses]) if rs and rs.status==4 else np.zeros((0,2))
    if len(Q)==0: print(name,"경로 없음"); continue
    d=np.sqrt(((Q[:,None,:]-P[None,::2,:])**2).sum(-1)).min(1)
    L=np.sum(np.hypot(*np.diff(Q,axis=0).T)); straight=math.hypot(*(Q[-1]-Q[0]))
    # 직선에서 벗어난 최대
    u=(Q[-1]-Q[0])/max(straight,1e-6); dev=np.abs((Q-Q[0])[:,0]*u[1]-(Q-Q[0])[:,1]*u[0]).max()
    print("%-8s 길이 %.2fm(직선 %.2f) 장애물까지 최소 %.2fm  꺾임점 %d"%(name,L,straight,d.min(), len(Q)))
