#!/usr/bin/env python3
"""
바퀴 캘리브레이션 — 엔코더 카운트/회전 실측

정속 구간만 재기 때문에 가감속(램프) 오차가 들어가지 않는다.
    기동 → 안정화 대기 → pos 기록 → T초 정속 → pos 기록 → 정지

⚠ 반드시 바퀴를 공중에 띄운 상태에서 실행할 것.

사용법:
    python3 tools/calib_wheel.py                 # 두 축 순서대로
    python3 tools/calib_wheel.py --axis 1
    python3 tools/calib_wheel.py --rpm 60 --seconds 30
"""

import argparse
import os
import re
import select
import sys
import termios
import threading
import time

FEEDBACK_RE = re.compile(
    r"axis(?P<axis>\d)\s+node\d\s+(?P<online>\S+)\s+(?P<enabled>\S+)\s+"
    r"cmd=\s*(?P<cmd>-?\d+)\s*rpm\s+act=\s*(?P<act>-?\d+)\s*rpm\s+"
    r"pos=\s*(?P<pos>-?\d+)\s+status=0x(?P<status>[0-9A-Fa-f]+)\s+"
    r"err=0x(?P<err>[0-9A-Fa-f]+)"
)



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

class Link:
    def __init__(self, path, baud=115200):
        self.fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        speed = getattr(termios, "B%d" % baud)
        cc = list(termios.tcgetattr(self.fd)[6])
        cc[termios.VMIN] = 0
        cc[termios.VTIME] = 0
        termios.tcsetattr(
            self.fd, termios.TCSANOW,
            [0, 0, termios.CS8 | termios.CREAD | termios.CLOCAL, 0, speed, speed, cc])
        termios.tcflush(self.fd, termios.TCIOFLUSH)

        self.state = {}          # axis -> (t, pos, act, err)
        self.lines = []
        self._buf = ""
        self._run = True
        threading.Thread(target=self._reader, daemon=True).start()
        threading.Thread(target=self._poller, daemon=True).start()

    def _reader(self):
        while self._run:
            r, _, _ = select.select([self.fd], [], [], 0.1)
            if not r:
                continue
            try:
                self._buf += os.read(self.fd, 4096).decode("utf-8", "replace")
            except OSError:
                continue
            while "\n" in self._buf:
                line, self._buf = self._buf.split("\n", 1)
                line = line.strip()
                if not line:
                    continue
                self.lines.append(line)
                m = FEEDBACK_RE.search(line)
                if m:
                    self.state[int(m.group("axis"))] = (
                        time.monotonic(), int(m.group("pos")),
                        int(m.group("act")), int(m.group("err"), 16))
            if len(self._buf) > 512:
                self._buf = ""

    def _poller(self):
        while self._run:
            self.send("f")
            time.sleep(0.1)

    def send(self, line):
        os.write(self.fd, (crc_line(line) + "\r\n").encode())

    def sample(self, axis, timeout=2.0):
        """가장 최근 피드백을 기다렸다 돌려준다."""
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            s = self.state.get(axis)
            if s and s[0] > t0 - 0.3:
                return s
            time.sleep(0.05)
        return None

    def close(self):
        self._run = False
        try:
            self.send("x")
            time.sleep(0.2)
            os.close(self.fd)
        except OSError:
            pass


def calibrate(link, axis, rpm, seconds, settle):
    print("\n--- 축 %d : %d rpm, 정속 %d초 ---" % (axis, rpm, seconds))

    link.send("i %d" % axis)
    time.sleep(1.5)

    s = link.sample(axis)
    if s is None:
        print("  ❌ 피드백 없음 — 초기화 실패")
        return None
    if s[3] != 0:
        print("  ❌ 알람 0x%04X — 먼저 해소하세요" % s[3])
        return None

    link.send("s %d %d" % (axis, rpm))
    print("  기동... %d초 안정화 대기" % settle)
    time.sleep(settle)

    s1 = link.sample(axis)
    if s1 is None:
        print("  ❌ 피드백 유실")
        link.send("x")
        return None
    t1, p1, act1 = s1[0], s1[1], s1[2]
    print("  시작  pos=%-10d act=%d rpm" % (p1, act1))

    acts = []
    t_end = time.monotonic() + seconds
    while time.monotonic() < t_end:
        time.sleep(0.25)
        st = link.state.get(axis)
        if st:
            acts.append(st[2])
        left = t_end - time.monotonic()
        print("\r  측정 중... %4.1f초 남음" % max(0, left), end="", flush=True)
    print()

    s2 = link.sample(axis)
    t2, p2, act2 = s2[0], s2[1], s2[2]
    link.send("s %d 0" % axis)
    print("  끝    pos=%-10d act=%d rpm" % (p2, act2))

    dt = t2 - t1
    dpos = p2 - p1
    act_avg = sum(acts) / len(acts) if acts else 0.0

    # 지령 rpm 과 실측 rpm 두 기준으로 각각 환산해 비교한다
    rev_cmd = rpm / 60.0 * dt
    rev_act = act_avg / 60.0 * dt
    cpr_cmd = dpos / rev_cmd if rev_cmd else 0.0
    cpr_act = dpos / rev_act if rev_act else 0.0

    print("  경과 %.2f초, Δpos = %d" % (dt, dpos))
    print("  실측 rpm 평균 %.1f  (지령 %d, 오차 %.1f%%)"
          % (act_avg, rpm, 100 * (act_avg - rpm) / rpm if rpm else 0))
    print("  → counts/회전 = %.1f  (지령 기준)" % cpr_cmd)
    print("  → counts/회전 = %.1f  (실측 rpm 기준)" % cpr_act)
    return {"axis": axis, "dpos": dpos, "dt": dt, "act_avg": act_avg,
            "cpr_cmd": cpr_cmd, "cpr_act": cpr_act}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyUSB0")
    ap.add_argument("--axis", type=int, choices=[1, 2], default=0,
                    help="지정하지 않으면 두 축 모두")
    ap.add_argument("--rpm", type=int, default=60)
    ap.add_argument("--seconds", type=int, default=30, help="정속 측정 시간")
    ap.add_argument("--settle", type=int, default=4, help="기동 후 안정화 대기")
    ap.add_argument("--yes", action="store_true", help="확인 질문 건너뛰기")
    args = ap.parse_args()

    print("=" * 58)
    print(" 바퀴 캘리브레이션 — 바퀴가 공중에 떠 있어야 합니다")
    print("=" * 58)
    if not args.yes:
        try:
            if input(" 바퀴를 띄우셨습니까? [y/N] ").strip().lower() != "y":
                sys.exit(" 중단했습니다.")
        except EOFError:
            sys.exit(" 대화형 실행이 필요합니다 (--yes 로 건너뛸 수 있음).")

    try:
        link = Link(args.port)
    except OSError as e:
        sys.exit(" 시리얼 열기 실패: %s\n   amr_web.py 등을 먼저 종료하세요." % e)

    axes = [args.axis] if args.axis else [1, 2]
    results = []
    try:
        for ax in axes:
            r = calibrate(link, ax, args.rpm, args.seconds, args.settle)
            if r:
                results.append(r)
            time.sleep(1.5)
    except KeyboardInterrupt:
        print("\n 중단")
    finally:
        link.close()

    if not results:
        return
    print("\n" + "=" * 58)
    print(" 결과")
    for r in results:
        print("   축%d : counts/회전 = %.1f   (실측rpm %.1f)"
              % (r["axis"], r["cpr_cmd"], r["act_avg"]))
    if len(results) == 2:
        a, b = results[0]["cpr_cmd"], results[1]["cpr_cmd"]
        if a and b:
            print("   두 축 차이 : %.2f %%" % abs(100 * (a - b) / a))
    print("=" * 58)
    print(" 바퀴가 실제로 분당 몇 바퀴 돌았는지도 눈으로 확인해주세요.")
    print(" 지령 rpm 과 같으면 s<rpm> 은 바퀴 rpm, 다르면 감속기가 있습니다.")


if __name__ == "__main__":
    main()
