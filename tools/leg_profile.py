"""다리 자리의 세로 뎁스 단면을 찍는다. 높이별로 뭐가 보이는지 본다."""
import json, math, urllib.request
import numpy as np, rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo

CAM_H, LASER_H = 0.185, 0.296          # TF 실측
CAM_DX = 0.466                          # 라이다 -> 카메라 앞뒤 간격

class P(Node):
    def __init__(self):
        super().__init__("legprofile")
        self.img = None; self.k = None
        self.create_subscription(Image,
            "/camera/camera/aligned_depth_to_color/image_raw", self.on_img, 1)
        self.create_subscription(CameraInfo,
            "/camera/camera/aligned_depth_to_color/camera_info", self.on_k, 1)
    def on_img(self, m):
        self.img = np.frombuffer(m.data, np.uint16).reshape(m.height, m.width)
    def on_k(self, m): self.k = m.k

rclpy.init(); n = P()
for _ in range(200):
    rclpy.spin_once(n, timeout_sec=0.05)
    if n.img is not None and n.k is not None: break
fx, cx, fy, cy = n.k[0], n.k[2], n.k[4], n.k[5]
print("내부값: fx=%.1f fy=%.1f cx=%.1f cy=%.1f  영상 %dx%d"
      % (fx, fy, cx, cy, n.img.shape[1], n.img.shape[0]))
import time
c = None
for _ in range(40):                      # 인식이 깜빡여서 몇 초 기다린다
    with urllib.request.urlopen("http://127.0.0.1:8090/carts", timeout=5) as f:
        d = json.load(f)
    cs = d.get("carts") or []
    if cs:
        c = cs[0]; break
    rclpy.spin_once(n, timeout_sec=0.05); time.sleep(0.2)
if c is None:
    print("대차가 안 보입니다 (후보 %s)" % d.get("candidates"))
    print("렉이 로봇 '앞' 에 보이는 자리에 세우고 다시 돌려 주세요.")
    raise SystemExit
if c["x"] < 0.5:
    print("!! 렉이 로봇 뒤(x=%.2f)에 있습니다. 카메라는 앞만 봅니다." % c["x"])
    print("   렉을 정면으로 보게 세우고 다시 돌려 주세요.")
    raise SystemExit
print("대차 %s  점수 %.2f  판정 %s  적중비 %s\n" % (c["cart"], c["score"], c["cam_ok"], c["cam_note"]))
img = n.img.astype(np.float32) / 1000.0
for i, (lx, ly) in enumerate(c.get("legs_raw") or c["legs"]):
    Z = lx - CAM_DX                      # 카메라 앞 거리
    X = -ly
    if Z <= 0.15: print("다리%d  카메라 뒤" % (i+1)); continue
    u = int(round(fx * X / Z + cx))
    hw = max(1, min(6, int(round(fx * 0.020 * 0.6 / Z))))
    u0, u1 = max(0, u-hw), min(img.shape[1], u+hw+1)
    print("다리%d  라이다 x=%.2f y=%+.2f  카메라 z=%.2f  u=%d (띠 %d~%d)"
          % (i+1, lx, ly, Z, u, u0, u1))
    if not (0 <= u < img.shape[1]): print("   화면 밖\n"); continue
    print("    바닥높이   뎁스중앙값   기대 %.2f   판정" % Z)
    for h_mm in range(450, -30, -30):     # 450mm 에서 0 까지
        hh = h_mm/1000.0
        v = int(round(fy * (CAM_H - hh) / Z + cy))
        if not (0 <= v < img.shape[0]):
            print("     %4d mm   화면 밖" % h_mm); continue
        st = img[max(0,v-1):v+2, u0:u1]
        seen = st[st > 0.1]
        if seen.size == 0:
            print("     %4d mm   (뎁스 없음)" % h_mm); continue
        m = float(np.median(seen))
        print("     %4d mm   %.3f m    %s" % (h_mm, m,
              "맞음" if abs(m-Z) < 0.06 else "다름(%+.0fmm)" % ((m-Z)*1000)))
    print()
