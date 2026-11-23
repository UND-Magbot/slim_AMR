#!/usr/bin/env python3
"""
LDS-50C-E-12 실시간 스캔 뷰어 (표준 라이브러리만 사용)

라이다는 전원만으로는 데이터를 보내지 않는다. 서비스 포트로 'LSTARH' 를
보내야 스트리밍이 시작되고, 1초마다 keepalive(0x4b41)를 보내야 유지된다.

사용법:
    python3 tools/lidar_view.py
    브라우저에서 http://localhost:8081

    python3 tools/lidar_view.py --http-port 9001 --lidar-ip 192.168.158.98
"""

import argparse
import json
import math
import random
import socket
import struct
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

LIDAR_IP = "192.168.158.98"
CTRL_PORT = 6543
DATA_PORT = 6668

PKT_MAGIC = 0xFAC7
HDR_FMT = "<HHHHIIIII"
HDR_LEN = struct.calcsize(HDR_FMT)   # 28


# --------------------------------------------------------------------------
# 프로토콜
# --------------------------------------------------------------------------

def stm32crc(words):
    crc32 = 0xFFFFFFFF
    poly = 0x04C11DB7
    for data in words:
        xbit = 1 << 31
        for _ in range(32):
            crc32 = ((crc32 << 1) & 0xFFFFFFFF) ^ poly if crc32 & 0x80000000 \
                else (crc32 << 1) & 0xFFFFFFFF
            if data & xbit:
                crc32 ^= poly
            xbit >>= 1
    return crc32 & 0xFFFFFFFF


def build_cmd(cmd_id, payload=b""):
    n = ((len(payload) + 3) >> 2) * 4
    payload = payload.ljust(n, b"\0")
    hdr = struct.pack("<HHHH", 0x484C, cmd_id,
                      random.randint(0, 0xFFFF), n)
    body = hdr + payload
    words = [int.from_bytes(body[i:i + 4], "little")
             for i in range(0, len(body), 4)]
    return body + stm32crc(words[:n // 4 + 2]).to_bytes(4, "little")


# --------------------------------------------------------------------------
# 수집
# --------------------------------------------------------------------------

class Scanner:
    def __init__(self, lidar_ip):
        self.lidar_ip = lidar_ip
        self.lock = threading.Lock()
        self.scan = []          # [[angle_deg, dist_mm, strength], ...]
        self.stats = {}
        self.running = False
        self._acc = {}
        self._last_ang = -1.0
        self._revs = 0
        self._t0 = time.monotonic()
        self._pkts = 0
        self._bytes = 0

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(("0.0.0.0", DATA_PORT))
        self.sock.settimeout(0.5)

    # 명령은 반드시 데이터 소켓으로 보낸다(응답이 이 포트로 돌아옴)
    def _cmd(self, cmd_id, payload=b""):
        self.sock.sendto(build_cmd(cmd_id, payload), (self.lidar_ip, CTRL_PORT))

    def start(self):
        self._cmd(0x0043, b"LSTARH")
        self.running = True

    def stop(self):
        self._cmd(0x0043, b"LSTOPH")
        self.running = False

    def keepalive(self):
        now = time.time()
        world_clock = int((now % 3600) * 1000) & 0xFFFFFFFF
        self._cmd(0x4B41, struct.pack("<II", world_clock, 0))

    def _publish(self):
        pts = [[a, d, s] for a, (d, s) in sorted(self._acc.items())]
        valid = [p for p in pts if p[1] > 0]
        with self.lock:
            self.scan = pts
            el = max(1e-6, time.monotonic() - self._t0)
            self.stats = {
                "points": len(pts),
                "valid": len(valid),
                "valid_pct": (100.0 * len(valid) / len(pts)) if pts else 0.0,
                "min_mm": min((p[1] for p in valid), default=0),
                "max_mm": max((p[1] for p in valid), default=0),
                "revs": self._revs,
                "hz": self._revs / el,
                "pkt_s": self._pkts / el,
                "kbps": self._bytes / el / 1024,
                "running": self.running,
            }
        self._acc = {}

    def reader(self):
        while True:
            try:
                data, _ = self.sock.recvfrom(65535)
            except socket.timeout:
                continue
            except OSError:
                continue
            self._pkts += 1
            self._bytes += len(data)
            if len(data) < HDR_LEN:
                continue

            code, count, _whole, _off, beg, _end, flags, _ts, _dev = \
                struct.unpack_from(HDR_FMT, data, 0)
            if code != PKT_MAGIC or count == 0:
                continue

            need = HDR_LEN + 5 * count + 2
            if len(data) < need:
                continue

            o = HDR_LEN
            dist = struct.unpack_from("<%dH" % count, data, o); o += 2 * count
            ang = struct.unpack_from("<%dH" % count, data, o); o += 2 * count
            has_intensity = bool(flags & 0x02)
            stg = struct.unpack_from("<%dB" % count, data, o) if has_intensity \
                else (0,) * count

            for i in range(count):
                a = ((beg + ang[i]) / 1000.0) % 360.0
                # 각도가 되감기면 한 바퀴 완료
                if a + 30.0 < self._last_ang:
                    self._revs += 1
                    self._publish()
                self._last_ang = a
                self._acc[round(a, 2)] = (dist[i], stg[i])

    def snapshot(self):
        with self.lock:
            return {"scan": self.scan, "stats": self.stats}


# --------------------------------------------------------------------------
# 웹
# --------------------------------------------------------------------------

PAGE = r"""<!doctype html>
<html lang="ko"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>LDS-50C-E 스캔 뷰어</title>
<style>
:root{--bg:#0e1015;--panel:#181b23;--line:#2c313d;--fg:#e6e8ef;--dim:#98a0b0;
  --accent:#4da3ff;--ok:#3ddc97;--warn:#ffb454;
  font-family:system-ui,-apple-system,"Segoe UI",Roboto,"Noto Sans KR",sans-serif}
*{box-sizing:border-box}
body{margin:0;background:var(--bg);color:var(--fg)}
header{background:var(--panel);border-bottom:1px solid var(--line);padding:10px 16px;
  display:flex;gap:12px;align-items:center;flex-wrap:wrap}
h1{font-size:16px;margin:0;font-weight:650}
button{background:#232733;color:var(--fg);border:1px solid var(--line);border-radius:8px;
  padding:8px 14px;font-size:14px;cursor:pointer}
button:hover{border-color:var(--accent)}
.badge{font-size:12px;padding:3px 9px;border-radius:999px;background:#232733;
  color:var(--dim);border:1px solid var(--line)}
.badge.on{color:var(--ok);border-color:#2b6b52}
main{display:grid;grid-template-columns:1fr 260px;gap:14px;padding:14px;
  max-width:1200px;margin:0 auto}
#wrap{background:var(--panel);border:1px solid var(--line);border-radius:12px;padding:10px}
canvas{width:100%;height:auto;display:block;border-radius:8px;background:#0a0c10}
aside{background:var(--panel);border:1px solid var(--line);border-radius:12px;padding:14px}
h2{font-size:12px;margin:0 0 10px;color:var(--dim);text-transform:uppercase;
  letter-spacing:.06em}
table{width:100%;border-collapse:collapse;font-variant-numeric:tabular-nums}
td{padding:5px 0;font-size:13.5px;border-bottom:1px solid var(--line)}
td:last-child{text-align:right;font-weight:650}
label{display:block;color:var(--dim);font-size:12px;margin:14px 0 6px}
input[type=range]{width:100%;accent-color:var(--accent)}
@media(max-width:860px){main{grid-template-columns:1fr}}
</style></head><body>
<header>
  <h1>LDS-50C-E 스캔 뷰어</h1>
  <span class="badge" id="st">연결 확인 중</span>
  <button onclick="cmd('start')">시작</button>
  <button onclick="cmd('stop')">정지</button>
</header>
<main>
  <div id="wrap"><canvas id="c" width="760" height="760"></canvas></div>
  <aside>
    <h2>측정</h2>
    <table id="tb"></table>
    <label>표시 범위 <span id="rl">8</span> m</label>
    <input type="range" id="rng" min="1" max="40" value="8" step="1" oninput="draw()">
    <label>점 크기 <span id="pl">2</span> px</label>
    <input type="range" id="psz" min="1" max="5" value="2" step="1" oninput="draw()">
  </aside>
</main>
<script>
let scan=[], stats={};
const c=document.getElementById('c'), g=c.getContext('2d');

async function cmd(w){ try{ await fetch('/'+w,{method:'POST'}); }catch(e){} }

function draw(){
  const R=+document.getElementById('rng').value;
  const PS=+document.getElementById('psz').value;
  document.getElementById('rl').textContent=R;
  document.getElementById('pl').textContent=PS;
  const W=c.width, H=c.height, cx=W/2, cy=H/2, sc=(Math.min(W,H)/2-24)/(R*1000);

  g.fillStyle='#0a0c10'; g.fillRect(0,0,W,H);

  // 거리 링
  g.strokeStyle='#242a36'; g.fillStyle='#5c6577'; g.font='11px ui-monospace,monospace';
  const step = R<=5?1: R<=12?2: R<=25?5:10;
  for(let r=step;r<=R;r+=step){
    g.beginPath(); g.arc(cx,cy,r*1000*sc,0,Math.PI*2); g.stroke();
    g.fillText(r+'m', cx+4, cy-r*1000*sc-3);
  }
  // 십자선 + 전방 표시
  g.strokeStyle='#242a36'; g.beginPath();
  g.moveTo(cx,0);g.lineTo(cx,H);g.moveTo(0,cy);g.lineTo(W,cy); g.stroke();
  g.fillStyle='#4da3ff'; g.font='12px system-ui';
  g.fillText('0° (전방)', cx+6, 16);

  // 점군: 0° = 위쪽, 각도 증가 = 시계방향
  for(const p of scan){
    const d=p[1]; if(!d) continue;
    const a=(p[0]-90)*Math.PI/180;
    const x=cx+Math.cos(a)*d*sc, y=cy+Math.sin(a)*d*sc;
    if(x<0||y<0||x>W||y>H) continue;
    const t=Math.min(1,p[2]/160);
    g.fillStyle='rgb('+Math.round(80+175*t)+','+Math.round(200+40*t)+',255)';
    g.fillRect(x-PS/2,y-PS/2,PS,PS);
  }
  // 원점
  g.fillStyle='#ff5c5c'; g.beginPath(); g.arc(cx,cy,4,0,Math.PI*2); g.fill();
}

function row(k,v){ return '<tr><td>'+k+'</td><td>'+v+'</td></tr>'; }

async function poll(){
  try{
    const r=await fetch('/scan'); const j=await r.json();
    scan=j.scan; stats=j.stats||{};
    const s=stats;
    document.getElementById('tb').innerHTML =
      row('점 수', s.points??'-') +
      row('유효', (s.valid??'-')+' ('+(s.valid_pct??0).toFixed(0)+'%)') +
      row('최소 거리', s.min_mm? (s.min_mm/1000).toFixed(3)+' m':'-') +
      row('최대 거리', s.max_mm? (s.max_mm/1000).toFixed(3)+' m':'-') +
      row('회전수', s.revs??'-') +
      row('스캔 주파수', (s.hz??0).toFixed(2)+' Hz') +
      row('패킷율', (s.pkt_s??0).toFixed(0)+' /s') +
      row('데이터율', (s.kbps??0).toFixed(1)+' KB/s');
    const st=document.getElementById('st');
    const live=(s.pkt_s||0)>1;
    st.textContent = live?'수신 중':'데이터 없음';
    st.className='badge '+(live?'on':'');
    draw();
  }catch(e){
    document.getElementById('st').textContent='서버 연결 끊김';
  }
}
setInterval(poll,150); poll();
</script></body></html>
"""


class Handler(BaseHTTPRequestHandler):
    scanner = None

    def log_message(self, *a):
        pass

    def _send(self, code, body, ctype="application/json"):
        data = body if isinstance(body, bytes) else body.encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(data)

    def do_GET(self):
        if self.path == "/":
            self._send(200, PAGE, "text/html; charset=utf-8")
        elif self.path == "/scan":
            self._send(200, json.dumps(self.scanner.snapshot()))
        else:
            self._send(404, "{}")

    def do_POST(self):
        if self.path == "/start":
            self.scanner.start()
        elif self.path == "/stop":
            self.scanner.stop()
        self._send(200, "{}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--lidar-ip", default=LIDAR_IP)
    ap.add_argument("--http-port", type=int, default=8081)
    args = ap.parse_args()

    try:
        sc = Scanner(args.lidar_ip)
    except OSError as e:
        sys.exit("UDP %d 바인드 실패: %s\n  lidar_probe.py 나 ROS 드라이버가 "
                 "떠 있으면 먼저 종료하세요." % (DATA_PORT, e))

    threading.Thread(target=sc.reader, daemon=True).start()

    def alive_loop():
        while True:
            time.sleep(1.0)
            if sc.running:
                try:
                    sc.keepalive()
                except OSError:
                    pass
    threading.Thread(target=alive_loop, daemon=True).start()

    sc.start()
    Handler.scanner = sc
    srv = ThreadingHTTPServer(("0.0.0.0", args.http_port), Handler)
    print("라이다  : %s  (제어 %d / 데이터 %d)" % (args.lidar_ip, CTRL_PORT, DATA_PORT))
    print("뷰어    : http://localhost:%d" % args.http_port)
    print("종료    : Ctrl+C")
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        print("\n스트리밍 정지 후 종료합니다.")
        try:
            sc.stop()
        except OSError:
            pass


if __name__ == "__main__":
    main()
