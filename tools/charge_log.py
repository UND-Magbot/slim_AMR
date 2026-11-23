#!/usr/bin/env python3
"""충전 기록 (2026-11-13): /battery_detail 을 N초마다 CSV 로. 끝은 시간 한도 또는 SOC 100 + 전류 작아짐.

    python3 tools/charge_log.py logs/charge_1113.csv [간격초=10] [최대시간=6]
    python3 tools/charge_log.py --report logs/charge_1113.csv
"""
import csv, json, sys, time


def report(path):
    rows = list(csv.DictReader(open(path)))
    if len(rows) < 2:
        print("기록이 부족합니다"); return
    t = [float(r["t"]) for r in rows]; soc = [float(r["soc"]) for r in rows]
    ah = [float(r["remaining_ah"]) for r in rows]; cur = [float(r["current"]) for r in rows]
    v = [float(r["voltage"]) for r in rows]
    h = (t[-1] - t[0]) / 3600
    print("기록 %.1f분 (%d줄): SOC %.0f%% -> %.0f%%, 남은 용량 %.2f -> %.2f Ah, 전압 %.2f -> %.2f V"
          % (h * 60, len(rows), soc[0], soc[-1], ah[0], ah[-1], v[0], v[-1]))
    print("전류 평균 %.2f A (처음 %.2f, 마지막 %.2f)" % (sum(cur) / len(cur), cur[0], cur[-1]))
    if h > 0:
        rate_soc = (soc[-1] - soc[0]) / h; rate_ah = (ah[-1] - ah[0]) / h
        print("속도: 10분에 SOC %.1f%%p, %.2f Ah (시간당 %.1f%%p)" % (rate_soc / 6, rate_ah / 6, rate_soc))
        # 최근 20분 기준으로 남은 시간 (끝으로 갈수록 느려지므로 최근이 더 맞다)
        k = next((i for i in range(len(t)) if t[-1] - t[i] <= 1200), 0)
        hr = (t[-1] - t[k]) / 3600
        r2 = (soc[-1] - soc[k]) / hr if hr > 0.05 else rate_soc
        if r2 > 0 and soc[-1] < 100:
            print("최근 %.0f분 속도 시간당 %.1f%%p -> 100%%까지 약 %.0f분 (끝 무렵 정전압 구간은 더 느려진다)"
                  % (hr * 60, r2, (100 - soc[-1]) / r2 * 60))
    # 10분 구간별
    print("10분 구간:")
    i0 = 0
    for i in range(1, len(t)):
        if t[i] - t[i0] >= 600 or i == len(t) - 1:
            print("  %s~%s  SOC %+.0f%%p  %+.2f Ah  전류 %.2f A"
                  % (time.strftime("%H:%M", time.localtime(t[i0])), time.strftime("%H:%M", time.localtime(t[i])),
                     soc[i] - soc[i0], ah[i] - ah[i0], sum(cur[i0:i + 1]) / (i - i0 + 1)))
            i0 = i


def main():
    if sys.argv[1] == "--report":
        report(sys.argv[2]); return
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    path = sys.argv[1]; dt = float(sys.argv[2]) if len(sys.argv) > 2 else 10.0
    tmax = float(sys.argv[3]) * 3600 if len(sys.argv) > 3 else 6 * 3600
    last = {}

    class P(Node):
        def __init__(s):
            super().__init__("charge_log")
            s.create_subscription(String, "/battery_detail", lambda m: last.update(json.loads(m.data)), 10)
    rclpy.init(); n = P(); t0 = time.time(); nxt = t0
    f = open(path, "a", newline=""); w = csv.writer(f)
    if f.tell() == 0:
        w.writerow(["t", "soc", "current", "voltage", "remaining_ah"])
    full_n = 0
    while time.time() - t0 < tmax:
        rclpy.spin_once(n, timeout_sec=0.2)
        if time.time() >= nxt and last.get("ok"):
            nxt += dt
            w.writerow([round(time.time(), 1), last.get("soc"), last.get("current"), last.get("voltage"),
                        last.get("remaining_ah")]); f.flush()
            full_n = full_n + 1 if (last.get("soc", 0) >= 100 and abs(last.get("current", 9)) < 0.5) else 0
            if full_n >= 30:
                break


if __name__ == "__main__":
    main()
