#!/usr/bin/env python3
"""
UND_AMR 통합 테스트 콘솔 — 모터 / 라이다 / 카메라 탭

    모터   : STM32 USART1 콘솔 (/dev/ttyUSB0, 115200)
    라이다 : LDS-50C-E-12 (UDP, 192.168.158.98)
    카메라 : Intel RealSense D435i (pyrealsense2 필요)

카메라 외에는 표준 라이브러리만 쓴다. 일부 장치가 없어도 나머지는 정상 동작한다.

사용법:
    python3 tools/amr_web.py
    브라우저에서 http://localhost:8080

    python3 tools/amr_web.py --serial /dev/ttyACM0 --no-lidar
    python3 tools/amr_web.py --http-port 9000
"""

import argparse
import fcntl
import json
import os
import random
import re
import select
import socket
import struct
import sys
import termios
import threading
import time
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

# ==========================================================================
# 모터 — 시리얼
# ==========================================================================



def crc_line(line):
    """줄 끝에 CRC-16/CCITT-FALSE 확인값 ' %XXXX' 를 붙인다.

    2026-09-30 펌웨어부터 확인값 없는 주행 명령(v,s,w,b,t)은 버린다.
    base_node 의 SerialPort.with_crc 와 같은 식."""
    line = line.rstrip("\r\n")
    if not line.strip():
        return line
    crc = 0xFFFF
    for b in line.encode():
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return "%s %%%04X" % (line, crc)

class Serial:
    def __init__(self, path, baud=115200):
        self.path = path
        self.baud = baud
        self.fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)

        # 배타 모드: picocom 등이 같은 포트를 열어 바이트를 훔쳐가는 사고 차단
        self.exclusive = True
        try:
            fcntl.ioctl(self.fd, termios.TIOCEXCL)
        except OSError:
            self.exclusive = False

        self._speed = getattr(termios, "B%d" % baud)
        cc = list(termios.tcgetattr(self.fd)[6])
        cc[termios.VMIN] = 0
        cc[termios.VTIME] = 0
        self._attrs = [0, 0, termios.CS8 | termios.CREAD | termios.CLOCAL, 0,
                       self._speed, self._speed, cc]
        termios.tcsetattr(self.fd, termios.TCSANOW, self._attrs)
        termios.tcflush(self.fd, termios.TCIOFLUSH)
        self._wlock = threading.Lock()

    def ensure_config(self):
        """picocom 은 종료할 때 termios 를 복원해 우리 설정(115200)을 지운다."""
        try:
            cur = termios.tcgetattr(self.fd)
        except OSError:
            return False
        if cur[4] != self._speed or cur[5] != self._speed:
            termios.tcsetattr(self.fd, termios.TCSANOW, self._attrs)
            return True
        return False

    def read(self, timeout=0.2):
        r, _, _ = select.select([self.fd], [], [], timeout)
        if not r:
            return b""
        try:
            return os.read(self.fd, 4096)
        except OSError:
            return b""

    def write_line(self, line):
        data = (crc_line(line) + "\r\n").encode("utf-8", "ignore")
        with self._wlock:
            os.write(self.fd, data)


FEEDBACK_RE = re.compile(
    r"axis(?P<axis>\d)\s+node(?P<node>\d)\s+(?P<online>\S+)\s+(?P<enabled>\S+)\s+"
    r"cmd=\s*(?P<cmd>-?\d+)\s*rpm\s+act=\s*(?P<act>-?\d+)\s*rpm\s+"
    r"pos=\s*(?P<pos>-?\d+)\s+status=0x(?P<status>[0-9A-Fa-f]+)\s+"
    r"err=0x(?P<err>[0-9A-Fa-f]+)\s+(?P<volt>-?\d+)V"
)

MOTION_CMDS = ("s", "b", "t")


class MotorHub:
    def __init__(self, serial, watchdog_s=3.0):
        self.serial = serial
        self.watchdog_s = watchdog_s
        self.lock = threading.Lock()
        self.log = deque(maxlen=1000)
        self.seq = 0
        self.axes = {}
        self.last_beat = time.time()
        self.motion_active = False
        self.stopped_by_watchdog = False
        self._partial = ""

    def _push(self, text, kind="rx"):
        with self.lock:
            self.seq += 1
            self.log.append({"seq": self.seq, "kind": kind, "text": text,
                             "t": time.strftime("%H:%M:%S")})

    def since(self, seq):
        with self.lock:
            return [e for e in self.log if e["seq"] > seq], self.seq, dict(self.axes)

    def reader(self):
        next_check = 0.0
        while True:
            now = time.monotonic()
            if now >= next_check:
                next_check = now + 1.0
                if self.serial.ensure_config():
                    self._push("!! 포트 설정이 변경되어 %d bps 로 복원했습니다"
                               % self.serial.baud, kind="warn")
            chunk = self.serial.read()
            if not chunk:
                continue
            self._partial += chunk.decode("utf-8", "replace")
            while True:
                idx = self._partial.find("\n")
                if idx < 0:
                    break
                line = self._partial[:idx].replace("\r", "").rstrip()
                self._partial = self._partial[idx + 1:]
                if line:
                    self._push(line)
                    self._parse(line)
            if len(self._partial) > 400:
                self._push(self._partial.rstrip())
                self._partial = ""

    def _parse(self, line):
        m = FEEDBACK_RE.search(line)
        if not m:
            return
        d = m.groupdict()
        with self.lock:
            self.axes[d["axis"]] = {
                "node": int(d["node"]),
                "online": d["online"] == "online",
                "enabled": d["enabled"] == "enabled",
                "cmd": int(d["cmd"]), "act": int(d["act"]), "pos": int(d["pos"]),
                "status": int(d["status"], 16), "err": int(d["err"], 16),
                "volt": int(d["volt"]), "t": time.time(),
            }

    def send(self, line):
        line = line.strip()
        if not line:
            return
        head = line.split()[0][:1].lower()
        if head in MOTION_CMDS:
            nums = re.findall(r"-?\d+", line)
            self.motion_active = bool(nums) and any(int(n) != 0 for n in nums[-1:])
        elif head in ("x", "z"):
            self.motion_active = False
        self.stopped_by_watchdog = False
        self._push(line, kind="tx")
        self.serial.write_line(line)

    def beat(self):
        self.last_beat = time.time()

    def watchdog(self):
        while True:
            time.sleep(0.5)
            if not self.motion_active or self.stopped_by_watchdog:
                continue
            if (time.time() - self.last_beat) > self.watchdog_s:
                self.stopped_by_watchdog = True
                self.motion_active = False
                self._push("!! 브라우저 연결 끊김 - 자동 정지 (x)", kind="warn")
                self.serial.write_line("x")


# ==========================================================================
# 라이다 — UDP
# ==========================================================================

CTRL_PORT = 6543
DATA_PORT = 6668
PKT_MAGIC = 0xFAC7
HDR_FMT = "<HHHHIIIII"
HDR_LEN = struct.calcsize(HDR_FMT)


def stm32crc(words):
    crc32, poly = 0xFFFFFFFF, 0x04C11DB7
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
    """CmdHeader{0x484C, cmd, sn, len} + payload(4정렬) + STM32 CRC32."""
    n = ((len(payload) + 3) >> 2) * 4
    payload = payload.ljust(n, b"\0")
    body = struct.pack("<HHHH", 0x484C, cmd_id, random.randint(0, 0xFFFF), n) + payload
    words = [int.from_bytes(body[i:i + 4], "little") for i in range(0, len(body), 4)]
    return body + stm32crc(words[:n // 4 + 2]).to_bytes(4, "little")


class Scanner:
    """라이다는 전원만으로는 송신하지 않는다. 'LSTARH' 를 받아야 시작한다."""

    def __init__(self, lidar_ip):
        self.lidar_ip = lidar_ip
        self.lock = threading.Lock()
        self.scan = []
        self.stats = {}
        self.running = False
        self._acc = {}
        self._last_ang = -1.0
        self._revs = self._pkts = self._bytes = 0
        self._t0 = time.monotonic()

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(("0.0.0.0", DATA_PORT))
        self.sock.settimeout(0.5)

    def _cmd(self, cmd_id, payload=b""):
        # 명령은 반드시 데이터 소켓으로 보낸다(응답이 이 포트로 돌아옴)
        self.sock.sendto(build_cmd(cmd_id, payload), (self.lidar_ip, CTRL_PORT))

    def start(self):
        self._cmd(0x0043, b"LSTARH")
        self.running = True

    def stop(self):
        self._cmd(0x0043, b"LSTOPH")
        self.running = False

    def keepalive(self):
        self._cmd(0x4B41, struct.pack("<II", int((time.time() % 3600) * 1000), 0))

    def _publish(self):
        pts = [[a, d, s] for a, (d, s) in sorted(self._acc.items())]
        valid = [p for p in pts if p[1] > 0]
        el = max(1e-6, time.monotonic() - self._t0)
        with self.lock:
            self.scan = pts
            self.stats = {
                "points": len(pts), "valid": len(valid),
                "valid_pct": (100.0 * len(valid) / len(pts)) if pts else 0.0,
                "min_mm": min((p[1] for p in valid), default=0),
                "max_mm": max((p[1] for p in valid), default=0),
                "revs": self._revs, "hz": self._revs / el,
                "pkt_s": self._pkts / el, "kbps": self._bytes / el / 1024,
                "running": self.running,
            }
        self._acc = {}

    def reader(self):
        while True:
            try:
                data, _ = self.sock.recvfrom(65535)
            except (socket.timeout, OSError):
                continue
            self._pkts += 1
            self._bytes += len(data)
            if len(data) < HDR_LEN:
                continue
            code, count, _w, _o, beg, _e, flags, _ts, _dev = \
                struct.unpack_from(HDR_FMT, data, 0)
            if code != PKT_MAGIC or count == 0 or len(data) < HDR_LEN + 5 * count + 2:
                continue

            o = HDR_LEN
            dist = struct.unpack_from("<%dH" % count, data, o); o += 2 * count
            ang = struct.unpack_from("<%dH" % count, data, o); o += 2 * count
            stg = struct.unpack_from("<%dB" % count, data, o) if flags & 0x02 \
                else (0,) * count

            for i in range(count):
                a = ((beg + ang[i]) / 1000.0) % 360.0
                if a + 30.0 < self._last_ang:      # 각도 되감김 = 한 바퀴 완료
                    self._revs += 1
                    self._publish()
                self._last_ang = a
                self._acc[round(a, 2)] = (dist[i], stg[i])

    def snapshot(self):
        with self.lock:
            return {"scan": self.scan, "stats": self.stats}


# ==========================================================================
# 카메라 — Intel RealSense D435i
# ==========================================================================

try:
    import pyrealsense2 as rs
    import numpy as np
    from PIL import Image
    import io as _io
    HAS_RS = True
except ImportError:
    HAS_RS = False


class Camera:
    """컬러/뎁스를 계속 받아 최신 JPEG 을 들고 있는다.

    파이썬에서 640x480 두 장을 30fps 로 인코딩하면 CPU 를 다 먹으므로
    캡처는 30fps 로 두고 JPEG 인코딩만 max_fps 로 낮춘다."""

    def __init__(self, width=640, height=480, fps=30, quality=70, max_fps=15):
        self.w, self.h = width, height
        self.quality = quality
        self.min_dt = 1.0 / max_fps
        self.lock = threading.Lock()
        self.jpeg = {"color": None, "depth": None}
        self.depth_m = None          # float32 미터 단위
        self.info = {}
        self._frames = 0
        self._t0 = time.monotonic()

        self.pipe = rs.pipeline()
        cfg = rs.config()
        cfg.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)
        cfg.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
        prof = self.pipe.start(cfg)
        dev = prof.get_device()
        self.depth_scale = dev.first_depth_sensor().get_depth_scale()
        self.name = dev.get_info(rs.camera_info.name)
        self.serial = dev.get_info(rs.camera_info.serial_number)
        self.colorizer = rs.colorizer()
        self.align = rs.align(rs.stream.color)

    @staticmethod
    def _encode(arr, quality):
        buf = _io.BytesIO()
        Image.fromarray(arr).save(buf, format="JPEG", quality=quality)
        return buf.getvalue()

    def reader(self):
        last = 0.0
        while True:
            try:
                fs = self.align.process(self.pipe.wait_for_frames())
            except RuntimeError:
                time.sleep(0.1)
                continue
            cf, df = fs.get_color_frame(), fs.get_depth_frame()
            if not cf or not df:
                continue
            self._frames += 1

            now = time.monotonic()
            if now - last < self.min_dt:
                continue
            last = now

            color = np.asanyarray(cf.get_data())[:, :, ::-1]      # BGR -> RGB
            depth_raw = np.asanyarray(df.get_data())
            depth_vis = np.asanyarray(self.colorizer.colorize(df).get_data())

            cj = self._encode(color, self.quality)
            dj = self._encode(depth_vis, self.quality)
            dm = depth_raw.astype("float32") * self.depth_scale
            valid = dm[dm > 0]
            el = max(1e-6, now - self._t0)

            with self.lock:
                self.jpeg["color"] = cj
                self.jpeg["depth"] = dj
                self.depth_m = dm
                self.info = {
                    "name": self.name, "serial": self.serial,
                    "size": "%dx%d" % (self.w, self.h),
                    "fps": self._frames / el,
                    "valid_pct": 100.0 * valid.size / dm.size if dm.size else 0.0,
                    "min_m": float(valid.min()) if valid.size else 0.0,
                    "max_m": float(valid.max()) if valid.size else 0.0,
                    "center_m": float(dm[self.h // 2, self.w // 2]),
                }

    def get_jpeg(self, kind):
        with self.lock:
            return self.jpeg.get(kind)

    def depth_at(self, x, y):
        with self.lock:
            if self.depth_m is None:
                return None
            if not (0 <= x < self.w and 0 <= y < self.h):
                return None
            return float(self.depth_m[y, x])

    def snapshot(self):
        with self.lock:
            return dict(self.info)


# ==========================================================================
# 웹
# ==========================================================================

PAGE = r"""<!doctype html>
<html lang="ko"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>UND_AMR 테스트 콘솔</title>
<style>
:root{--bg:#12141a;--panel:#1b1e26;--panel2:#232733;--line:#333846;--fg:#e6e8ef;
  --dim:#9aa1b1;--accent:#4da3ff;--ok:#3ddc97;--warn:#ffb454;--bad:#ff5c5c;
  font-family:system-ui,-apple-system,"Segoe UI",Roboto,"Noto Sans KR",sans-serif}
*{box-sizing:border-box}
body{margin:0;background:var(--bg);color:var(--fg);font-size:15px}
header{position:sticky;top:0;z-index:10;background:var(--panel);
  border-bottom:1px solid var(--line);padding:10px 16px;
  display:flex;align-items:center;gap:12px;flex-wrap:wrap}
h1{font-size:16px;margin:0;font-weight:650;letter-spacing:-.01em}
.tabs{display:flex;gap:6px;margin-left:8px}
.tab{background:transparent;border:1px solid transparent;color:var(--dim);
  padding:7px 16px;border-radius:8px;cursor:pointer;font-size:14px;font-weight:600}
.tab:hover{color:var(--fg)}
.tab.sel{background:var(--panel2);border-color:var(--line);color:var(--fg)}
.badge{font-size:12px;padding:3px 9px;border-radius:999px;background:var(--panel2);
  color:var(--dim);border:1px solid var(--line)}
.badge.on{color:var(--ok);border-color:#2b6b52}
.badge.off{color:var(--bad);border-color:#6b2b2b}
.stop{margin-left:auto;background:var(--bad);color:#fff;border:0;border-radius:10px;
  padding:11px 24px;font-size:16px;font-weight:800;cursor:pointer}
.stop:active{transform:translateY(1px)}
main{max-width:1200px;margin:0 auto;padding:16px}
.pane{display:none} .pane.sel{display:grid;gap:14px;grid-template-columns:1fr 1fr}
section{background:var(--panel);border:1px solid var(--line);border-radius:12px;padding:14px}
section.wide{grid-column:1/-1}
h2{font-size:12px;margin:0 0 12px;color:var(--dim);font-weight:600;
  text-transform:uppercase;letter-spacing:.06em}
button{background:var(--panel2);color:var(--fg);border:1px solid var(--line);
  border-radius:8px;padding:9px 14px;font-size:14px;cursor:pointer}
button:hover{border-color:var(--accent)}
button:active{transform:translateY(1px)}
.row{display:flex;gap:8px;flex-wrap:wrap;align-items:center}
.slider{display:grid;grid-template-columns:52px 1fr 74px;gap:10px;align-items:center;margin:10px 0}
.slider label{color:var(--dim);font-size:13px}
input[type=range]{width:100%;accent-color:var(--accent)}
.val{font-variant-numeric:tabular-nums;text-align:right;font-weight:650}
table{width:100%;border-collapse:collapse;font-variant-numeric:tabular-nums}
th,td{text-align:right;padding:6px 8px;border-bottom:1px solid var(--line);font-size:14px}
th{color:var(--dim);font-weight:600;font-size:12px}
th:first-child,td:first-child{text-align:left}
.pill{font-size:11px;padding:2px 7px;border-radius:999px;background:var(--panel2)}
.pill.ok{color:var(--ok)} .pill.bad{color:var(--bad)} .pill.dim{color:var(--dim)}
#log{background:#0d0f14;border:1px solid var(--line);border-radius:8px;padding:10px;
  height:260px;overflow-y:auto;font-family:ui-monospace,Menlo,monospace;
  font-size:12.5px;line-height:1.55;white-space:pre-wrap;word-break:break-all}
.l-tx{color:var(--accent)} .l-warn{color:var(--warn);font-weight:600}
.l-err{color:var(--bad)} .l-ok{color:var(--ok)}
#cmd{flex:1;background:#0d0f14;color:var(--fg);border:1px solid var(--line);
  border-radius:8px;padding:9px 12px;font-family:ui-monospace,monospace;font-size:13px}
.hint{color:var(--dim);font-size:12px;margin-top:8px;line-height:1.6}
canvas{width:100%;height:auto;display:block;border-radius:8px;background:#0a0c10}\n.cam{width:100%;height:auto;display:block;border-radius:8px;background:#0a0c10;\n  border:1px solid var(--line)}\n#imgDepth{cursor:crosshair}
.err{color:var(--bad);padding:24px;text-align:center}
@media(max-width:860px){.pane.sel{grid-template-columns:1fr}}
</style></head><body>

<header>
  <h1>UND_AMR</h1>
  <div class="tabs">
    <button class="tab sel" id="tab-motor" onclick="tab('motor')">모터</button>
    <button class="tab" id="tab-lidar" onclick="tab('lidar')">라이다</button>
    <button class="tab" id="tab-cam" onclick="tab('cam')">카메라</button>
  </div>
  <span class="badge" id="stMotor">모터 -</span>
  <span class="badge" id="stLidar">라이다 -</span>
  <span class="badge" id="stCam">카메라 -</span>
  <button class="stop" onclick="stop()">■ 정지 (Space)</button>
</header>

<main>
<div class="pane sel" id="pane-motor">
  <section>
    <h2>드라이브 준비</h2>
    <div class="row">
      <button onclick="send('i')">초기화 (양축)</button>
      <button onclick="send('i 1')">축1만</button>
      <button onclick="send('i 2')">축2만</button>
      <button onclick="send('a')">알람 클리어</button>
      <button onclick="send('z')">축 프리</button>
      <button onclick="send('f')">피드백</button>
    </div>
    <div class="row" style="margin-top:10px">
      <button onclick="send('d')">방향 확인</button>
      <button onclick="send('d 1 -1')">축1 반전</button>
      <button onclick="send('d 2 -1')">축2 반전</button>
      <button onclick="send('c')">통신두절정지 설정</button>
      <button onclick="send('boot')">DFU 진입</button>
    </div>
    <p class="hint">부호는 <b>+ = 로봇 전진</b> 기준입니다. 초기화(<code>i</code>) 전에는 속도 지령이 거부됩니다.</p>
  </section>

  <section>
    <h2>주행</h2>
    <div class="slider"><label>직진</label>
      <input type="range" id="lin" min="-150" max="150" value="0" step="5" oninput="onDrive()">
      <span class="val" id="linV">0 rpm</span></div>
    <div class="slider"><label>회전</label>
      <input type="range" id="ang" min="-150" max="150" value="0" step="5" oninput="onDrive()">
      <span class="val" id="angV">0 rpm</span></div>
    <div class="row">
      <button onclick="preset(30,0)">직진 30</button>
      <button onclick="preset(-30,0)">후진 30</button>
      <button onclick="preset(0,30)">우회전</button>
      <button onclick="preset(0,-30)">좌회전</button>
      <button onclick="preset(0,0)">중립</button>
    </div>
    <p class="hint">축1 = 직진+회전, 축2 = 직진−회전 으로 계산해 전송합니다.</p>
  </section>

  <section>
    <h2>축 개별 제어</h2>
    <div class="slider"><label>축 1</label>
      <input type="range" id="a1" min="-150" max="150" value="0" step="5" oninput="onAxis(1)">
      <span class="val" id="a1V">0 rpm</span></div>
    <div class="slider"><label>축 2</label>
      <input type="range" id="a2" min="-150" max="150" value="0" step="5" oninput="onAxis(2)">
      <span class="val" id="a2V">0 rpm</span></div>
  </section>

  <section>
    <h2>피드백</h2>
    <table id="fb"><thead><tr><th>축</th><th>지령</th><th>실측</th><th>위치</th>
      <th>전압</th><th>상태</th></tr></thead>
      <tbody><tr><td colspan="6" style="text-align:left;color:var(--dim)">대기 중</td></tr></tbody></table>
  </section>

  <section class="wide">
    <h2>콘솔</h2>
    <div id="log"></div>
    <div class="row" style="margin-top:10px">
      <input id="cmd" placeholder="명령 입력 후 Enter  (h = 도움말)"
             onkeydown="if(event.key==='Enter'){send(this.value);this.value=''}">
      <button onclick="send('h')">도움말</button>
      <button onclick="document.getElementById('log').textContent=''">지우기</button>
    </div>
  </section>
</div>

<div class="pane" id="pane-lidar">
  <section class="wide" style="grid-column:1/2">
    <canvas id="c" width="760" height="760"></canvas>
  </section>
  <section>
    <h2>측정</h2>
    <table id="lt"></table>
    <div class="row" style="margin-top:12px">
      <button onclick="lidar('start')">시작</button>
      <button onclick="lidar('stop')">정지</button>
    </div>
    <div class="slider" style="grid-template-columns:60px 1fr 54px">
      <label>범위</label>
      <input type="range" id="rng" min="1" max="40" value="8" step="1" oninput="drawScan()">
      <span class="val" id="rl">8 m</span></div>
    <div class="slider" style="grid-template-columns:60px 1fr 54px">
      <label>점 크기</label>
      <input type="range" id="psz" min="1" max="5" value="2" step="1" oninput="drawScan()">
      <span class="val" id="pl">2 px</span></div>
    <p class="hint">0° = 위쪽(라이다 전방), 각도 증가 = 시계방향. 색이 밝을수록 반사 강도가 큽니다.</p>
  </section>
</div>
<div class="pane" id="pane-cam">
  <section>
    <h2>RGB</h2>
    <img id="imgColor" class="cam" alt="color">
  </section>
  <section>
    <h2>Depth <span style="color:var(--dim);font-weight:400">— 클릭하면 그 지점 거리</span></h2>
    <img id="imgDepth" class="cam" alt="depth" onclick="probe(event)">
    <div id="probeOut" class="hint" style="font-size:14px;color:var(--fg)">
      이미지를 클릭해 거리를 측정하세요.</div>
  </section>
  <section class="wide">
    <h2>카메라 상태</h2>
    <table id="ct"></table>
    <p class="hint">뎁스는 컬러 프레임에 정합(align)되어 있어 같은 픽셀이 같은 지점을 가리킵니다.
      색은 가까울수록 파랑, 멀수록 빨강입니다.</p>
  </section>
</div>
</main>

<script>
let seq=0, scan=[], curTab='motor';
const c=document.getElementById('c'), g=c.getContext('2d');

function tab(n){
  curTab=n;
  for(const t of ['motor','lidar','cam']){
    document.getElementById('pane-'+t).className = 'pane'+(t===n?' sel':'');
    document.getElementById('tab-'+t).className  = 'tab'+(t===n?' sel':'');
  }
  if(n==='lidar') drawScan();
  if(n==='cam') startCam();
}

let camOn=false;
function startCam(){
  if(camOn) return; camOn=true;
  document.getElementById('imgColor').src='/cam/color.mjpg?'+Date.now();
  document.getElementById('imgDepth').src='/cam/depth.mjpg?'+Date.now();
}

async function probe(ev){
  const img=ev.target, r=img.getBoundingClientRect();
  const x=Math.round((ev.clientX-r.left)/r.width*img.naturalWidth);
  const y=Math.round((ev.clientY-r.top)/r.height*img.naturalHeight);
  try{
    const j=await (await fetch('/cam/depth_at?x='+x+'&y='+y)).json();
    document.getElementById('probeOut').textContent =
      j.m>0 ? `( ${x} , ${y} ) → ${j.m.toFixed(3)} m` : `( ${x} , ${y} ) → 측정 불가 (뎁스 없음)`;
  }catch(e){}
}

async function send(line){
  if(!line) return;
  try{ await fetch('/cmd',{method:'POST',headers:{'Content-Type':'application/json'},
    body:JSON.stringify({line})}); }catch(e){}
}
async function lidar(w){ try{ await fetch('/lidar/'+w,{method:'POST'}); }catch(e){} }

function stop(){
  ['lin','ang','a1','a2'].forEach(id=>document.getElementById(id).value=0);
  paint(); send('x');
}
function paint(){
  for(const [i,u] of [['lin','rpm'],['ang','rpm'],['a1','rpm'],['a2','rpm']])
    document.getElementById(i+'V').textContent=document.getElementById(i).value+' '+u;
}
let dT=null;
function onDrive(){ paint(); clearTimeout(dT); dT=setTimeout(()=>{
  const v=+document.getElementById('lin').value, w=+document.getElementById('ang').value;
  send('s 1 '+(v+w)); setTimeout(()=>send('s 2 '+(v-w)),40); },70); }
let aT={};
function onAxis(n){ paint(); clearTimeout(aT[n]); aT[n]=setTimeout(()=>{
  send('s '+n+' '+document.getElementById('a'+n).value); },70); }
function preset(v,w){ document.getElementById('lin').value=v;
  document.getElementById('ang').value=w; onDrive(); }

function cls(t){
  if(/FAIL|abort|no response|OFFLINE|ALARM/i.test(t)) return 'l-err';
  if(/^!!/.test(t)) return 'l-warn';
  if(/PASS|init OK|ALIVE/i.test(t)) return 'l-ok';
  return '';
}
function badge(id,txt,ok){ const e=document.getElementById(id);
  e.textContent=txt; e.className='badge '+(ok===true?'on':ok===false?'off':''); }

function drawScan(){
  const R=+document.getElementById('rng').value, PS=+document.getElementById('psz').value;
  document.getElementById('rl').textContent=R+' m';
  document.getElementById('pl').textContent=PS+' px';
  const W=c.width,H=c.height,cx=W/2,cy=H/2,sc=(Math.min(W,H)/2-24)/(R*1000);
  g.fillStyle='#0a0c10'; g.fillRect(0,0,W,H);
  g.strokeStyle='#242a36'; g.fillStyle='#5c6577'; g.font='11px ui-monospace,monospace';
  const step=R<=5?1:R<=12?2:R<=25?5:10;
  for(let r=step;r<=R;r+=step){ g.beginPath(); g.arc(cx,cy,r*1000*sc,0,7); g.stroke();
    g.fillText(r+'m',cx+4,cy-r*1000*sc-3); }
  g.strokeStyle='#242a36'; g.beginPath();
  g.moveTo(cx,0);g.lineTo(cx,H);g.moveTo(0,cy);g.lineTo(W,cy); g.stroke();
  g.fillStyle='#4da3ff'; g.font='12px system-ui'; g.fillText('0° (전방)',cx+6,16);
  for(const p of scan){ const d=p[1]; if(!d) continue;
    const a=(p[0]-90)*Math.PI/180, x=cx+Math.cos(a)*d*sc, y=cy+Math.sin(a)*d*sc;
    if(x<0||y<0||x>W||y>H) continue;
    const t=Math.min(1,p[2]/160);
    g.fillStyle='rgb('+Math.round(80+175*t)+','+Math.round(200+40*t)+',255)';
    g.fillRect(x-PS/2,y-PS/2,PS,PS); }
  g.fillStyle='#ff5c5c'; g.beginPath(); g.arc(cx,cy,4,0,7); g.fill();
}
function row(k,v){ return '<tr><td>'+k+'</td><td>'+v+'</td></tr>'; }

async function poll(){
  try{
    const j = await (await fetch('/poll?seq='+seq)).json();

    if(j.motor){
      seq=j.motor.seq;
      const log=document.getElementById('log');
      const bot = log.scrollTop+log.clientHeight >= log.scrollHeight-30;
      for(const e of j.motor.lines){
        const d=document.createElement('div');
        d.className = e.kind==='tx'?'l-tx':e.kind==='warn'?'l-warn':cls(e.text);
        d.textContent=(e.kind==='tx'?'> ':'')+e.text; log.appendChild(d); }
      while(log.childElementCount>800) log.removeChild(log.firstChild);
      if(bot) log.scrollTop=log.scrollHeight;
      const ax=j.motor.axes, ks=Object.keys(ax).sort();
      if(ks.length) document.querySelector('#fb tbody').innerHTML = ks.map(k=>{
        const a=ax[k];
        const st = !a.online?'<span class="pill bad">오프라인</span>'
          : a.err?'<span class="pill bad">ERR 0x'+a.err.toString(16).padStart(4,'0')+'</span>'
          : a.enabled?'<span class="pill ok">사용</span>':'<span class="pill dim">프리</span>';
        return `<tr><td>축 ${k}</td><td>${a.cmd}</td><td>${a.act}</td><td>${a.pos}</td>
          <td>${a.volt} V</td><td>${st}</td></tr>`; }).join('');
      badge('stMotor', j.motor.port, true);
    } else badge('stMotor','모터 없음', false);

    if(j.lidar){
      scan=j.lidar.scan; const s=j.lidar.stats||{};
      document.getElementById('lt').innerHTML =
        row('점 수',s.points??'-')+row('유효',(s.valid??'-')+' ('+(s.valid_pct??0).toFixed(0)+'%)')+
        row('최소 거리',s.min_mm?(s.min_mm/1000).toFixed(3)+' m':'-')+
        row('최대 거리',s.max_mm?(s.max_mm/1000).toFixed(3)+' m':'-')+
        row('회전수',s.revs??'-')+row('스캔 주파수',(s.hz??0).toFixed(2)+' Hz')+
        row('패킷율',(s.pkt_s??0).toFixed(0)+' /s')+row('데이터율',(s.kbps??0).toFixed(1)+' KB/s');
      badge('stLidar',(s.pkt_s||0)>1?'라이다 수신 중':'라이다 무신호',(s.pkt_s||0)>1);
      if(curTab==='lidar') drawScan();
    } else badge('stLidar','라이다 없음', false);

    if(j.cam){
      const k=j.cam;
      document.getElementById('ct').innerHTML =
        row('모델',k.name??'-')+row('시리얼',k.serial??'-')+row('해상도',k.size??'-')+
        row('캡처 FPS',(k.fps??0).toFixed(1))+
        row('유효 뎁스',(k.valid_pct??0).toFixed(1)+' %')+
        row('거리 범위',(k.min_m??0).toFixed(3)+' ~ '+(k.max_m??0).toFixed(3)+' m')+
        row('중앙 거리',(k.center_m??0).toFixed(3)+' m');
      badge('stCam','카메라 '+(k.fps??0).toFixed(0)+' fps', (k.fps||0)>1);
    } else badge('stCam','카메라 없음', false);
  }catch(e){}
}
setInterval(poll,200);
setInterval(()=>fetch('/beat').catch(()=>{}),1000);
setInterval(()=>send('f'),700);
document.addEventListener('keydown',e=>{
  if(e.target.id==='cmd') return;
  if(e.code==='Space'||e.code==='Escape'){ e.preventDefault(); stop(); }});
paint(); poll();
</script></body></html>
"""


class Handler(BaseHTTPRequestHandler):
    motor = None
    scanner = None
    camera = None
    port_name = ""

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
        if self.path == "/" or self.path.startswith("/index"):
            self._send(200, PAGE, "text/html; charset=utf-8")
            return
        if self.path.startswith("/poll"):
            try:
                seq = int(self.path.split("seq=")[1].split("&")[0])
            except (IndexError, ValueError):
                seq = 0
            out = {"motor": None, "lidar": None}
            if self.motor:
                lines, newseq, axes = self.motor.since(seq)
                out["motor"] = {"seq": newseq, "lines": lines, "axes": axes,
                                "port": self.port_name}
            if self.scanner:
                out["lidar"] = self.scanner.snapshot()
            out["cam"] = self.camera.snapshot() if self.camera else None
            self._send(200, json.dumps(out))
            return
        if self.path.startswith("/cam/") and self.camera:
            if "depth_at" in self.path:
                q = self.path.split("?", 1)[1] if "?" in self.path else ""
                kv = dict(p.split("=") for p in q.split("&") if "=" in p)
                try:
                    m = self.camera.depth_at(int(kv.get("x", -1)), int(kv.get("y", -1)))
                except ValueError:
                    m = None
                self._send(200, json.dumps({"m": m if m else 0.0}))
                return
            kind = "depth" if self.path.startswith("/cam/depth") else "color"
            self._stream(kind)
            return
        if self.path == "/beat":
            if self.motor:
                self.motor.beat()
            self._send(200, "{}")
            return
        self._send(404, "{}")

    def _stream(self, kind):
        """multipart/x-mixed-replace 로 MJPEG 송출. <img src> 에 그대로 물린다."""
        self.send_response(200)
        self.send_header("Content-Type",
                         "multipart/x-mixed-replace; boundary=frame")
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        last = None
        try:
            while True:
                jpg = self.camera.get_jpeg(kind)
                if jpg is None or jpg is last:
                    time.sleep(0.02)
                    continue
                last = jpg
                self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n"
                                 b"Content-Length: " + str(len(jpg)).encode() +
                                 b"\r\n\r\n" + jpg + b"\r\n")
        except (BrokenPipeError, ConnectionResetError, OSError):
            pass          # 브라우저가 탭을 닫거나 새로고침한 경우

    def do_POST(self):
        if self.path == "/cmd" and self.motor:
            n = int(self.headers.get("Content-Length", 0))
            try:
                payload = json.loads(self.rfile.read(n) or b"{}")
            except json.JSONDecodeError:
                payload = {}
            self.motor.beat()
            self.motor.send(str(payload.get("line", "")))
        elif self.path == "/lidar/start" and self.scanner:
            self.scanner.start()
        elif self.path == "/lidar/stop" and self.scanner:
            self.scanner.stop()
        self._send(200, "{}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--serial", default="/dev/ttyUSB0")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--lidar-ip", default="192.168.158.98")
    ap.add_argument("--http-port", type=int, default=8080)
    ap.add_argument("--watchdog", type=float, default=3.0)
    ap.add_argument("--no-motor", action="store_true")
    ap.add_argument("--no-lidar", action="store_true")
    ap.add_argument("--no-camera", action="store_true")
    ap.add_argument("--cam-fps", type=int, default=15,
                    help="JPEG 인코딩 상한 fps (캡처는 30fps 유지)")
    args = ap.parse_args()

    motor = scanner = None

    if not args.no_motor:
        try:
            ser = Serial(args.serial, args.baud)
            motor = MotorHub(ser, args.watchdog if args.watchdog > 0 else 1e9)
            threading.Thread(target=motor.reader, daemon=True).start()
            threading.Thread(target=motor.watchdog, daemon=True).start()
            print("모터   : %s @ %d%s" % (args.serial, args.baud,
                  "" if ser.exclusive else "  (배타 모드 실패)"))
        except OSError as e:
            hint = " — 다른 프로그램이 쓰는 중" if getattr(e, "errno", 0) == 16 else ""
            print("모터   : 사용 불가 (%s%s)" % (e, hint))

    if not args.no_lidar:
        try:
            scanner = Scanner(args.lidar_ip)
            threading.Thread(target=scanner.reader, daemon=True).start()

            def alive():
                while True:
                    time.sleep(1.0)
                    if scanner.running:
                        try:
                            scanner.keepalive()
                        except OSError:
                            pass
            threading.Thread(target=alive, daemon=True).start()
            scanner.start()
            print("라이다 : %s  (제어 %d / 데이터 %d)"
                  % (args.lidar_ip, CTRL_PORT, DATA_PORT))
        except OSError as e:
            print("라이다 : 사용 불가 (UDP %d: %s)" % (DATA_PORT, e))

    camera = None
    if not args.no_camera:
        if not HAS_RS:
            print("카메라 : pyrealsense2 미설치  "
                  "(python3 -m pip install --user pyrealsense2 numpy)")
        else:
            try:
                camera = Camera(max_fps=args.cam_fps)
                threading.Thread(target=camera.reader, daemon=True).start()
                print("카메라 : %s  SN=%s" % (camera.name, camera.serial))
            except RuntimeError as e:
                print("카메라 : 사용 불가 (%s)" % e)

    if motor is None and scanner is None and camera is None:
        sys.exit("모터/라이다/카메라를 하나도 열지 못했습니다. 연결 상태를 확인하세요.")

    Handler.motor = motor
    Handler.scanner = scanner
    Handler.camera = camera
    Handler.port_name = "%s @ %d" % (args.serial, args.baud)
    srv = ThreadingHTTPServer(("0.0.0.0", args.http_port), Handler)
    print("웹 UI  : http://localhost:%d" % args.http_port)
    print("종료   : Ctrl+C")
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        print("\n정지 명령 전송 후 종료합니다.")
        if motor:
            try:
                motor.serial.write_line("x")
            except OSError:
                pass
        if scanner:
            try:
                scanner.stop()
            except OSError:
                pass
        time.sleep(0.2)


if __name__ == "__main__":
    main()
