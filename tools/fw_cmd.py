#!/usr/bin/env python3
"""STM32 콘솔에 명령을 보내고, 그 뒤 bringup.log 에 찍힌 펌웨어 줄을 보여 준다.

    python3 tools/fw_cmd.py q                   드라이브 제한·이득 조회
    python3 tools/fw_cmd.py "k 0 70 64" sleep:0.5 q
base_node 가 포트를 쥐고 있어도 쓰기는 된다(읽기는 base_node 가 로그로 남긴다).
[DRIVE]/[CAN2]/[BAT]/[UART1]/[LIFT] 줄만 base_node 가 로그에 남긴다.

    python3 tools/fw_cmd.py "l up"              리프팅 올림 (PC0 HIGH -> 올림 시간 뒤 LOW)
    python3 tools/fw_cmd.py "l down"            리프팅 내림 (PE1 HIGH -> 내림 시간 뒤 LOW)
    python3 tools/fw_cmd.py "l stop"            리프팅 즉시 정지 (둘 다 LOW)
    python3 tools/fw_cmd.py "l t 10000 10000"   올림/내림 시간 ms (재부팅하면 기본 10000)
    python3 tools/fw_cmd.py l                   리프팅 상태

명령마다 CRC-16 확인값(' %XXXX')을 붙여 보낸다. 펌웨어는 주행·리프팅 명령(v s w b t l)을
확인값 없이는 받지 않는다(2026-10-07).
"""
import os
import sys
import time

LOG = os.path.expanduser("~/projects/UND_AMR/logs/bringup.log")


def crc_line(line):
    """줄 끝에 CRC-16/CCITT-FALSE ' %XXXX' (base_node SerialPort.with_crc 와 같은 식)."""
    crc = 0xFFFF
    for b in line.encode():
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return "%s %%%04X" % (line, crc)


def main():
    with open(LOG, "rb") as f:
        n0 = sum(1 for _ in f)
    fd = os.open("/dev/ttyUSB0", os.O_WRONLY | os.O_NOCTTY)
    for c in sys.argv[1:]:
        if c.startswith("sleep:"):
            time.sleep(float(c[6:]))
            continue
        os.write(fd, ("\r\n" + crc_line(c) + "\r\n").encode())
        time.sleep(0.4)
    os.close(fd)
    time.sleep(1.5)
    with open(LOG, "rb") as f:
        lines = f.read().decode("utf-8", "replace").splitlines()[n0:]
    for ln in lines:
        if "펌웨어: " in ln:
            print(ln.split("펌웨어: ", 1)[1].replace("> ", ""))


if __name__ == "__main__":
    main()
