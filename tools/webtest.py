#!/usr/bin/env python3
"""
UND_AMR 모터 테스트 웹 UI

STM32 의 USART1 콘솔(/dev/ttyUSB0, 115200)을 웹 브라우저에서 조작한다.
표준 라이브러리만 사용하므로 별도 설치가 필요 없다.

사용법:
    python3 tools/webtest.py                       # 기본 /dev/ttyUSB0
    python3 tools/webtest.py --port /dev/ttyACM0
    python3 tools/webtest.py --http-port 8080

주의: picocom 등 다른 프로그램이 같은 포트를 열고 있으면 안 된다.
"""

import argparse
import fcntl
import json
import os
import re
import select
import sys
import termios
import threading
import time
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

# --------------------------------------------------------------------------
# 시리얼 (stdlib termios)
# --------------------------------------------------------------------------


class Serial:
    def __init__(self, path, baud=115200):
        self.path = path
        self.baud = baud
        self.fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)

        # 배타 모드: 이후 다른 프로그램(picocom 등)의 open() 이 EBUSY 로 실패한다.
        # 같은 포트를 두 곳에서 열어 바이트를 서로 훔쳐가는 사고를 원천 차단.
        self.exclusive = True
        try:
            fcntl.ioctl(self.fd, termios.TIOCEXCL)
        except OSError:
            self.exclusive = False

        self._speed = getattr(termios, "B%d" % baud)
        cc = list(termios.tcgetattr(self.fd)[6])
        cc[termios.VMIN] = 0
        cc[termios.VTIME] = 0
        # raw 8N1, no flow control, no modem control
        self._attrs = [0, 0, termios.CS8 | termios.CREAD | termios.CLOCAL, 0,
                       self._speed, self._speed, cc]
        self._apply()
        termios.tcflush(self.fd, termios.TCIOFLUSH)
        self._wlock = threading.Lock()

    def _apply(self):
        termios.tcsetattr(self.fd, termios.TCSANOW, self._attrs)

    def ensure_config(self):
        """다른 프로그램이 포트 설정을 바꿔놓았으면 되돌린다.

        picocom 은 종료할 때 termios 를 원래대로 복원한다(noreset=no 가 기본).
        그 바람에 우리가 맞춰둔 115200 이 9600 으로 돌아가 수신이 죽는다.
        """
        try:
            cur = termios.tcgetattr(self.fd)
        except OSError:
            return False
        if cur[4] != self._speed or cur[5] != self._speed:
            self._apply()
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
        data = (line.rstrip("\r\n") + "\r\n").encode("utf-8", "ignore")
        with self._wlock:
            os.write(self.fd, data)

    def close(self):
        try:
            os.close(self.fd)
        except OSError:
            pass


# --------------------------------------------------------------------------
# 상태 관리
# --------------------------------------------------------------------------

# 예:
# axis1 node1 online  enabled  cmd=  30 rpm  act=  29 rpm  pos=    -12345  status=0x0000 err=0x0000  24V
FEEDBACK_RE = re.compile(
    r"axis(?P<axis>\d)\s+node(?P<node>\d)\s+(?P<online>\S+)\s+(?P<enabled>\S+)\s+"
    r"cmd=\s*(?P<cmd>-?\d+)\s*rpm\s+act=\s*(?P<act>-?\d+)\s*rpm\s+"
    r"pos=\s*(?P<pos>-?\d+)\s+status=0x(?P<status>[0-9A-Fa-f]+)\s+"
    r"err=0x(?P<err>[0-9A-Fa-f]+)\s+(?P<volt>-?\d+)V"
)

# 모터를 실제로 움직이는 명령 (워치독 대상)
MOTION_CMDS = ("s", "b", "t")


class Hub:
    """시리얼 <-> 웹 클라이언트 중계 + 안전 워치독."""

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

    # -- 로그 ------------------------------------------------------------
    def _push(self, text, kind="rx"):
        with self.lock:
            self.seq += 1
            self.log.append({"seq": self.seq, "kind": kind, "text": text,
                             "t": time.strftime("%H:%M:%S")})

    def since(self, seq):
        with self.lock:
            return [e for e in self.log if e["seq"] > seq], self.seq, dict(self.axes)

    # -- 시리얼 수신 ------------------------------------------------------
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
            # 프롬프트처럼 개행이 없는 잔여분이 길어지면 그대로 흘려보낸다
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
                "cmd": int(d["cmd"]),
                "act": int(d["act"]),
                "pos": int(d["pos"]),
                "status": int(d["status"], 16),
                "err": int(d["err"], 16),
                "volt": int(d["volt"]),
                "t": time.time(),
            }

    # -- 명령 -------------------------------------------------------------
    def send(self, line):
        line = line.strip()
        if not line:
            return
        head = line.split()[0][:1].lower()
        if head in MOTION_CMDS:
            # 0 rpm 지령은 정지로 본다
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
        """브라우저와 연결이 끊긴 채 모터가 돌고 있으면 정지시킨다."""
        while True:
            time.sleep(0.5)
            if not self.motion_active or self.stopped_by_watchdog:
                continue
            if (time.time() - self.last_beat) > self.watchdog_s:
                self.stopped_by_watchdog = True
                self.motion_active = False
                self._push("!! 브라우저 연결 끊김 - 자동 정지 (x)", kind="warn")
                self.serial.write_line("x")


# --------------------------------------------------------------------------
# HTTP
# --------------------------------------------------------------------------

PAGE = r"""<!doctype html>
<html lang="ko"><head>
<meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>UND_AMR 모터 테스트</title>
<style>
:root{
  --bg:#12141a; --panel:#1b1e26; --panel2:#232733; --line:#333846;
  --fg:#e6e8ef; --dim:#9aa1b1; --accent:#4da3ff; --ok:#3ddc97;
  --warn:#ffb454; --bad:#ff5c5c;
  font-family:system-ui,-apple-system,"Segoe UI",Roboto,"Noto Sans KR",sans-serif;
}
*{box-sizing:border-box}
body{margin:0;background:var(--bg);color:var(--fg);font-size:15px}
header{position:sticky;top:0;z-index:10;background:var(--panel);
  border-bottom:1px solid var(--line);padding:10px 16px;
  display:flex;align-items:center;gap:14px;flex-wrap:wrap}
h1{font-size:17px;margin:0;font-weight:650;letter-spacing:-.01em}
.badge{font-size:12px;padding:3px 9px;border-radius:999px;background:var(--panel2);
  color:var(--dim);border:1px solid var(--line)}
.badge.on{color:var(--ok);border-color:#2b6b52}
.badge.off{color:var(--bad);border-color:#6b2b2b}
.stop{margin-left:auto;background:var(--bad);color:#fff;border:0;border-radius:10px;
  padding:12px 26px;font-size:17px;font-weight:800;cursor:pointer;letter-spacing:.02em}
.stop:active{transform:translateY(1px)}
main{max-width:1100px;margin:0 auto;padding:16px;display:grid;gap:14px;
  grid-template-columns:1fr 1fr}
section{background:var(--panel);border:1px solid var(--line);border-radius:12px;padding:14px}
section.wide{grid-column:1/-1}
h2{font-size:13px;margin:0 0 12px;color:var(--dim);font-weight:600;
  text-transform:uppercase;letter-spacing:.06em}
button{background:var(--panel2);color:var(--fg);border:1px solid var(--line);
  border-radius:8px;padding:9px 14px;font-size:14px;cursor:pointer}
button:hover{border-color:var(--accent)}
button:active{transform:translateY(1px)}
.row{display:flex;gap:8px;flex-wrap:wrap;align-items:center}
.slider{display:grid;grid-template-columns:52px 1fr 74px;gap:10px;align-items:center;
  margin:10px 0}
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
  height:280px;overflow-y:auto;font-family:ui-monospace,"JetBrains Mono",Menlo,monospace;
  font-size:12.5px;line-height:1.55;white-space:pre-wrap;word-break:break-all}
.l-tx{color:var(--accent)} .l-warn{color:var(--warn);font-weight:600}
.l-err{color:var(--bad)} .l-ok{color:var(--ok)}
#cmd{flex:1;background:#0d0f14;color:var(--fg);border:1px solid var(--line);
  border-radius:8px;padding:9px 12px;font-family:ui-monospace,monospace;font-size:13px}
.hint{color:var(--dim);font-size:12px;margin-top:8px;line-height:1.6}
@media(max-width:820px){main{grid-template-columns:1fr}}
</style></head><body>

<header>
  <h1>UND_AMR 모터 테스트</h1>
  <span class="badge" id="port">-</span>
  <span class="badge" id="link">연결 확인 중</span>
  <button class="stop" onclick="stop()">■ 정지 (Space)</button>
</header>

<main>
  <section>
    <h2>드라이브 준비</h2>
    <div class="row">
      <button onclick="send('i')">초기화 (양축)</button>
      <button onclick="send('i 1')">축1만</button>
      <button onclick="send('i 2')">축2만</button>
      <button onclick="send('a')">알람 클리어</button>
      <button onclick="send('z')">축 프리</button>
      <button onclick="send('f')">피드백 갱신</button>
    </div>
    <div class="row" style="margin-top:10px">
      <button onclick="send('d')">방향 확인</button>
      <button onclick="send('d 1 -1')">축1 반전</button>
      <button onclick="send('d 2 -1')">축2 반전</button>
      <button onclick="send('d 1 1')">축1 정</button>
      <button onclick="send('d 2 1')">축2 정</button>
    </div>
    <p class="hint">부호는 <b>+ = 로봇 전진</b> 기준입니다. 한쪽 바퀴가 반대로 돌면 그 축을 반전하세요.</p>
  </section>

  <section>
    <h2>주행</h2>
    <div class="slider">
      <label>직진</label>
      <input type="range" id="lin" min="-150" max="150" value="0" step="5"
             oninput="onDrive()" onchange="onDrive()">
      <span class="val" id="linV">0 rpm</span>
    </div>
    <div class="slider">
      <label>회전</label>
      <input type="range" id="ang" min="-150" max="150" value="0" step="5"
             oninput="onDrive()" onchange="onDrive()">
      <span class="val" id="angV">0 rpm</span>
    </div>
    <div class="row">
      <button onclick="preset(30,0)">직진 30</button>
      <button onclick="preset(-30,0)">후진 30</button>
      <button onclick="preset(0,30)">우회전</button>
      <button onclick="preset(0,-30)">좌회전</button>
      <button onclick="preset(0,0)">중립</button>
    </div>
    <p class="hint">축1 = 직진 + 회전, 축2 = 직진 − 회전 으로 계산해 전송합니다.
      슬라이더를 놓으면 그 값이 유지되므로 멈추려면 정지 버튼을 누르세요.</p>
  </section>

  <section>
    <h2>축 개별 제어</h2>
    <div class="slider">
      <label>축 1</label>
      <input type="range" id="a1" min="-150" max="150" value="0" step="5"
             oninput="onAxis(1)" onchange="onAxis(1)">
      <span class="val" id="a1V">0 rpm</span>
    </div>
    <div class="slider">
      <label>축 2</label>
      <input type="range" id="a2" min="-150" max="150" value="0" step="5"
             oninput="onAxis(2)" onchange="onAxis(2)">
      <span class="val" id="a2V">0 rpm</span>
    </div>
    <p class="hint">한 축씩 돌려서 회전 방향을 확인할 때 사용합니다.</p>
  </section>

  <section>
    <h2>피드백</h2>
    <table id="fb"><thead><tr>
      <th>축</th><th>지령</th><th>실측</th><th>위치</th><th>전압</th><th>상태</th>
    </tr></thead><tbody>
      <tr><td>1</td><td colspan="5" style="text-align:left;color:var(--dim)">대기 중</td></tr>
    </tbody></table>
    <p class="hint">0.7초마다 자동 갱신합니다.</p>
  </section>

  <section class="wide">
    <h2>콘솔</h2>
    <div id="log"></div>
    <div class="row" style="margin-top:10px">
      <input id="cmd" placeholder="명령 직접 입력 후 Enter  (h = 도움말)"
             onkeydown="if(event.key==='Enter'){send(this.value);this.value=''}">
      <button onclick="send('h')">도움말</button>
      <button onclick="document.getElementById('log').textContent=''">지우기</button>
    </div>
  </section>
</main>

<script>
let seq = 0, lastOk = 0;

async function send(line){
  if(!line) return;
  try{
    await fetch('/cmd',{method:'POST',headers:{'Content-Type':'application/json'},
      body:JSON.stringify({line})});
  }catch(e){ setLink(false); }
}

function stop(){
  ['lin','ang','a1','a2'].forEach(id=>{
    const el=document.getElementById(id); el.value=0;
  });
  paintSliders();
  send('x');
}

function paintSliders(){
  document.getElementById('linV').textContent = document.getElementById('lin').value+' rpm';
  document.getElementById('angV').textContent = document.getElementById('ang').value+' rpm';
  document.getElementById('a1V').textContent  = document.getElementById('a1').value+' rpm';
  document.getElementById('a2V').textContent  = document.getElementById('a2').value+' rpm';
}

let driveTimer=null;
function onDrive(){
  paintSliders();
  clearTimeout(driveTimer);
  driveTimer=setTimeout(()=>{
    const v=+document.getElementById('lin').value, w=+document.getElementById('ang').value;
    send('s 1 '+(v+w)); setTimeout(()=>send('s 2 '+(v-w)),40);
  },70);
}

let axisTimer={};
function onAxis(n){
  paintSliders();
  clearTimeout(axisTimer[n]);
  axisTimer[n]=setTimeout(()=>{
    send('s '+n+' '+document.getElementById('a'+n).value);
  },70);
}

function preset(v,w){
  document.getElementById('lin').value=v;
  document.getElementById('ang').value=w;
  onDrive();
}

function setLink(ok){
  const el=document.getElementById('link');
  el.textContent = ok ? '연결됨' : '연결 끊김';
  el.className = 'badge ' + (ok?'on':'off');
}

function cls(t){
  if(/FAIL|abort|no response|OFFLINE|ALARM/i.test(t)) return 'l-err';
  if(/^!!/.test(t)) return 'l-warn';
  if(/PASS|init OK|ALIVE/i.test(t)) return 'l-ok';
  return '';
}

function renderFb(axes){
  const tb=document.querySelector('#fb tbody');
  const keys=Object.keys(axes).sort();
  if(!keys.length) return;
  tb.innerHTML = keys.map(k=>{
    const a=axes[k];
    const bad = a.err!==0 || !a.online;
    const state = !a.online ? '<span class="pill bad">오프라인</span>'
      : a.err!==0 ? '<span class="pill bad">ERR 0x'+a.err.toString(16).padStart(4,'0')+'</span>'
      : a.enabled ? '<span class="pill ok">사용</span>'
      : '<span class="pill dim">프리</span>';
    return `<tr><td>축 ${k}</td><td>${a.cmd}</td><td>${a.act}</td>
      <td>${a.pos}</td><td>${a.volt} V</td><td>${state}</td></tr>`;
  }).join('');
}

async function poll(){
  try{
    const r = await fetch('/poll?seq='+seq);
    const j = await r.json();
    seq = j.seq;
    if(j.lines.length){
      const log=document.getElementById('log');
      const atBottom = log.scrollTop+log.clientHeight >= log.scrollHeight-30;
      for(const e of j.lines){
        const d=document.createElement('div');
        d.className = e.kind==='tx' ? 'l-tx' : e.kind==='warn' ? 'l-warn' : cls(e.text);
        d.textContent = (e.kind==='tx'?'> ':'') + e.text;
        log.appendChild(d);
      }
      while(log.childElementCount>800) log.removeChild(log.firstChild);
      if(atBottom) log.scrollTop = log.scrollHeight;
    }
    renderFb(j.axes);
    document.getElementById('port').textContent = j.port;
    setLink(true); lastOk=Date.now();
  }catch(e){
    if(Date.now()-lastOk > 2000) setLink(false);
  }
}

setInterval(poll, 300);
setInterval(()=>fetch('/beat').catch(()=>{}), 1000);
setInterval(()=>send('f'), 700);

document.addEventListener('keydown', e=>{
  if(e.target.tagName==='INPUT' && e.target.id==='cmd') return;
  if(e.code==='Space' || e.code==='Escape'){ e.preventDefault(); stop(); }
});

paintSliders(); poll();
</script>
</body></html>
"""


class Handler(BaseHTTPRequestHandler):
    hub = None
    port_name = ""

    def log_message(self, *a):
        pass  # 요청 로그 억제

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
            lines, newseq, axes = self.hub.since(seq)
            self._send(200, json.dumps(
                {"seq": newseq, "lines": lines, "axes": axes,
                 "port": self.port_name}))
            return
        if self.path == "/beat":
            self.hub.beat()
            self._send(200, "{}")
            return
        self._send(404, "{}")

    def do_POST(self):
        if self.path != "/cmd":
            self._send(404, "{}")
            return
        n = int(self.headers.get("Content-Length", 0))
        try:
            payload = json.loads(self.rfile.read(n) or b"{}")
        except json.JSONDecodeError:
            payload = {}
        self.hub.beat()
        self.hub.send(str(payload.get("line", "")))
        self._send(200, "{}")


# --------------------------------------------------------------------------


def check_busy(path):
    lock = "/var/lock/LCK.." + os.path.basename(path)
    if os.path.exists(lock):
        return "잠금 파일 %s 존재" % lock
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyUSB0")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--http-port", type=int, default=8080)
    ap.add_argument("--watchdog", type=float, default=3.0,
                    help="브라우저 무응답 시 자동 정지까지의 시간(초), 0이면 해제")
    args = ap.parse_args()

    busy = check_busy(args.port)
    if busy:
        print("경고: %s 를 다른 프로그램이 쓰고 있을 수 있습니다 (%s)." % (args.port, busy))
        print("      picocom 등을 먼저 종료하세요 (Ctrl+A, Ctrl+X).")

    try:
        ser = Serial(args.port, args.baud)
    except PermissionError:
        sys.exit("권한 없음: %s\n  newgrp dialout 또는 sudo 로 실행하세요." % args.port)
    except OSError as e:
        if getattr(e, "errno", None) == 16:  # EBUSY
            sys.exit("포트 사용 중: %s\n"
                     "  다른 webtest.py 나 시리얼 프로그램을 먼저 종료하세요." % args.port)
        sys.exit("열기 실패: %s (%s)" % (args.port, e))

    hub = Hub(ser, watchdog_s=args.watchdog if args.watchdog > 0 else 1e9)
    threading.Thread(target=hub.reader, daemon=True).start()
    threading.Thread(target=hub.watchdog, daemon=True).start()

    Handler.hub = hub
    Handler.port_name = "%s @ %d" % (args.port, args.baud)
    srv = ThreadingHTTPServer(("0.0.0.0", args.http_port), Handler)

    print("시리얼 : %s @ %d%s" % (args.port, args.baud,
          "" if ser.exclusive else "  (배타 모드 실패 - 다른 프로그램과 충돌 주의)"))
    print("웹 UI  : http://localhost:%d" % args.http_port)
    if args.watchdog > 0:
        print("워치독 : 브라우저 무응답 %.1f초 시 자동 정지" % args.watchdog)
    print("종료   : Ctrl+C")
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        print("\n정지 명령 전송 후 종료합니다.")
        try:
            ser.write_line("x")
            time.sleep(0.2)
        finally:
            ser.close()


if __name__ == "__main__":
    main()
