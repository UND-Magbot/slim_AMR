#!/usr/bin/env python3
"""
LDS-50C-E-12 라이다 연결 확인 도구 (표준 라이브러리만 사용)

출고 기본값
    라이다 IP     192.168.158.98
    업로드 대상   192.168.158.15   (= PC IP 여야 데이터가 옴)
    점군 UDP      6668
    설정/제어     6543

사용법:
    python3 tools/lidar_probe.py            # 링크·IP·핑·UDP 수신 전체 점검
    python3 tools/lidar_probe.py --seconds 10
"""

import argparse
import random
import socket
import subprocess
import sys
import time

LIDAR_IP = "192.168.158.98"
HOST_IP = "192.168.158.15"
DATA_PORT = 6668
CTRL_PORT = 6543


def sh(cmd):
    try:
        return subprocess.run(cmd, shell=True, capture_output=True,
                              text=True, timeout=15).stdout.strip()
    except subprocess.SubprocessError:
        return ""


def check_link():
    print("[1] 네트워크 인터페이스")
    found = None
    for line in sh("ip -br addr").splitlines():
        parts = line.split()
        if not parts or parts[0] == "lo":
            continue
        name = parts[0]
        carrier = sh("cat /sys/class/net/%s/carrier 2>/dev/null" % name)
        speed = sh("cat /sys/class/net/%s/speed 2>/dev/null" % name)
        addrs = " ".join(p for p in parts[2:] if "." in p)
        mark = ""
        if HOST_IP in addrs:
            mark = "  <-- 라이다 대역 설정됨"
            found = name
        print("    %-10s %-8s carrier=%s speed=%sMb/s  %s%s"
              % (name, parts[1], carrier or "?", speed or "?", addrs or "-", mark))
    if not found:
        print("    ❌ %s/24 를 가진 인터페이스가 없습니다." % HOST_IP)
        print("       sudo ip addr add %s/24 dev <인터페이스>" % HOST_IP)
        return False
    return True


def check_ping():
    print("\n[2] 라이다 핑 (%s)" % LIDAR_IP)
    out = sh("ping -c 3 -W 1 %s" % LIDAR_IP)
    ok = "3 received" in out or " 0% packet loss" in out
    for line in out.splitlines()[-3:]:
        print("    " + line)
    print("    ->", "응답 있음" if ok else "❌ 무응답")
    return ok


def stm32crc(words):
    """lanhai-driver 의 stm32crc() 를 그대로 옮긴 것 (poly 0x04C11DB7)."""
    crc32 = 0xFFFFFFFF
    poly = 0x04C11DB7
    for data in words:
        xbit = 1 << 31
        for _ in range(32):
            if crc32 & 0x80000000:
                crc32 = ((crc32 << 1) & 0xFFFFFFFF) ^ poly
            else:
                crc32 = (crc32 << 1) & 0xFFFFFFFF
            if data & xbit:
                crc32 ^= poly
            xbit >>= 1
    return crc32 & 0xFFFFFFFF


def build_cmd(cmd_id, payload=b""):
    """CmdHeader{sign=0x484c, cmd, sn, len} + payload(4바이트 정렬) + crc32.

    라이다는 전원만 넣으면 회전은 하지만 데이터는 쏘지 않는다.
    'LSTARH' 를 서비스 포트(6543)로 보내야 스트리밍이 시작된다.
    """
    n = ((len(payload) + 3) >> 2) * 4
    payload = payload.ljust(n, b"\0")
    sn = random.randint(0, 0xFFFF)
    hdr = (0x484C).to_bytes(2, "little") + cmd_id.to_bytes(2, "little") + \
          sn.to_bytes(2, "little") + n.to_bytes(2, "little")
    body = hdr + payload
    words = [int.from_bytes(body[i:i + 4], "little") for i in range(0, len(body), 4)]
    crc = stm32crc(words[:n // 4 + 2])
    return body + crc.to_bytes(4, "little"), sn


def decode_heartbeat(d):
    """매뉴얼 7.4 네트워크 심장박동 패킷 (sign='LiDA')."""
    if len(d) < 92 or d[:4] != b"LiDA":
        return None
    ip = lambda o: ".".join(str(x) for x in d[o:o + 4])
    u16 = lambda o: int.from_bytes(d[o:o + 2], "little")
    return {
        "dev_sn": d[16:36].split(b"\0")[0].decode("ascii", "replace"),
        "dev_type": d[36:52].split(b"\0")[0].decode("ascii", "replace"),
        "ip": ip(60), "mask": ip(64), "gateway": ip(68),
        "remote_ip": ip(72), "remote_udp": u16(76), "port": u16(78),
        "status": u16(80), "rpm": u16(82) / 10.0, "freq": u16(84) / 100.0,
        "cpu_temp": u16(88) / 10.0, "input_volt": u16(90) / 1000.0,
        "alarm": d[92:108].hex() if len(d) >= 108 else "",
    }


def check_udp(seconds, send_start=True):
    print("\n[3] UDP 수신  점군 %d / 하트비트 %d  (%d초)"
          % (DATA_PORT, CTRL_PORT, seconds))

    socks = {}
    for port, label in ((DATA_PORT, "점군"), (CTRL_PORT, "하트비트")):
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            s.bind(("0.0.0.0", port))
            s.setblocking(False)
            socks[s] = (port, label)
        except OSError as e:
            print("    ❌ %d 바인드 실패: %s" % (port, e))
            s.close()
    if not socks:
        return False

    import select as _sel

    # lanhai-driver 와 동일하게: local_port(6668) 에 바인드한 소켓 '그 자체로'
    # 명령을 보낸다. 별도 소켓으로 보내면 ACK 가 임시 포트로 돌아가 안 잡힌다.
    data_sock = next((s for s, (p, _) in socks.items() if p == DATA_PORT), None)
    if send_start and data_sock is not None:
        pkt, sn = build_cmd(0x0043, b"LSTARH")
        data_sock.sendto(pkt, (LIDAR_IP, CTRL_PORT))
        print("    -> 'LSTARH' from :%d -> %s:%d  sn=%d  [%s]"
              % (DATA_PORT, LIDAR_IP, CTRL_PORT, sn, pkt.hex(" ")))

    stat = {p: [0, 0, set()] for p, _ in socks.values()}
    hb = None
    t0 = time.time()
    while time.time() - t0 < seconds:
        r, _, _ = _sel.select(list(socks), [], [], 0.3)
        for s in r:
            port, _label = socks[s]
            try:
                data, addr = s.recvfrom(65535)
            except OSError:
                continue
            stat[port][0] += 1
            stat[port][1] += len(data)
            stat[port][2].add(addr[0])
            if hb is None:
                hb = decode_heartbeat(data)
    for s in socks:
        s.close()

    dur = time.time() - t0
    for port, label in sorted((v for v in socks.values()), key=lambda x: x[0]):
        n, total, senders = stat[port]
        if n:
            print("    ✅ %-5s %5d 패킷  %.0f pkt/s  %.1f KB/s  from %s"
                  % (label, n, n / dur, total / dur / 1024, ", ".join(sorted(senders))))
        else:
            print("    ❌ %-5s 패킷 없음" % label)

    if hb:
        print("\n    --- 라이다가 보고한 자기 설정 (하트비트) ---")
        print("      모델/시리얼 : %s / %s" % (hb["dev_type"], hb["dev_sn"]))
        print("      자기 IP     : %s / %s  gw %s" % (hb["ip"], hb["mask"], hb["gateway"]))
        print("      업로드 대상  : %s : %d      <<< 여기가 핵심"
              % (hb["remote_ip"], hb["remote_udp"]))
        print("      서비스 포트  : %d" % hb["port"])
        print("      회전수/주파수: %.1f rpm / %.2f Hz" % (hb["rpm"], hb["freq"]))
        print("      CPU 온도    : %.1f C     입력전압: %.2f V"
              % (hb["cpu_temp"], hb["input_volt"]))
        print("      상태/알람   : 0x%04X / %s" % (hb["status"], hb["alarm"] or "-"))
        if hb["remote_ip"] != HOST_IP:
            print("\n      ⚠ 업로드 대상이 %s 라서 이 PC(%s)로 안 옵니다."
                  % (hb["remote_ip"], HOST_IP))
            print("        PaceCatView 로 remote_ip 를 %s 로 바꾸거나," % HOST_IP)
            print("        PC IP 를 %s 로 맞추세요." % hb["remote_ip"])

    got = stat[DATA_PORT][0] > 0
    if not got:
        print("\n    점군이 안 옵니다. 확인 순서:")
        print("      1) 방화벽:  sudo ufw allow from 192.168.158.0/24")
        print("      2) 위 하트비트의 '업로드 대상' 이 %s 인지" % HOST_IP)
        print("      3) 하트비트조차 없으면 방화벽이 전부 막고 있을 가능성이 큼")
    return got


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=int, default=5)
    ap.add_argument("--no-start", action="store_true",
                    help="LSTARH 시작 명령을 보내지 않는다")
    args = ap.parse_args()

    print("=" * 62)
    print(" LDS-50C-E-12 연결 점검")
    print("=" * 62)
    link = check_link()
    ping = check_ping() if link else False
    udp = check_udp(args.seconds, not args.no_start) if link else False

    print("\n" + "=" * 62)
    print(" 결과:  링크/IP %s   핑 %s   점군 %s"
          % ("OK" if link else "NG", "OK" if ping else "NG", "OK" if udp else "NG"))
    if link and ping and udp:
        print(" 라이다 정상 동작 중입니다. ROS 드라이버 연결 단계로 진행 가능합니다.")
    print("=" * 62)
    return 0 if (link and ping and udp) else 1


if __name__ == "__main__":
    sys.exit(main())
