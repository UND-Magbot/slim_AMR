#!/usr/bin/env python3
"""렉 도킹 반복 검증 — [렉 나오기] 와 [도킹] 을 번갈아 돌리고 회차별로 정리한다.

    python3 tools/dock_repeat.py 렉2            5회
    python3 tools/dock_repeat.py 렉2 -n 10      10회

로봇이 렉 밑에 있으면 먼저 빠져나오고, 밖에 있으면 바로 도킹부터 한다.
실패·취소가 나면 그 자리에서 멈춘다(다음 회차로 넘어가지 않는다).

⚠ 접촉 여부는 로그로 알 수 없다. 계산상 모서리가 여유 안이어도 실제로
  닿은 적이 있다(2026-09-23, 09-28). 반드시 옆에서 보고 [정지] 를 누를
  준비를 할 것. 결과표의 '접촉' 칸은 사람이 채운다.

값은 모두 bringup.log 의 api_node 줄에서 뽑는다. 회차마다 로그 위치를
기억해 두고 그 뒤에 새로 찍힌 줄만 본다.
"""
import argparse
import json
import re
import sys
import time
import urllib.request

BASE = "http://localhost:8090"
LOG = "/home/und_amr1/projects/UND_AMR/logs/bringup.log"
BUSY = ("moving", "looking", "staging", "entering", "preflip", "exiting",
        "exit_fix")
IN_M = 720.0     # api_node JOB_IN_M — 남은 거리가 이보다 짧으면 다리 사이


def api(path, body=None):
    data = None if body is None else json.dumps(body).encode()
    req = urllib.request.Request(BASE + path, data=data,
                                 headers={"Content-Type": "application/json"})
    try:
        with urllib.request.urlopen(req, timeout=5) as r:
            return json.loads(r.read())
    except urllib.error.HTTPError as e:
        return json.loads(e.read())


def log_pos():
    with open(LOG, "rb") as f:
        f.seek(0, 2)
        return f.tell()


def log_since(pos):
    with open(LOG, "rb") as f:
        f.seek(pos)
        return f.read().decode("utf-8", "replace").splitlines()


def wait(limit):
    """작업이 끝날 때까지 기다린다. (상태, 메시지)"""
    t0 = time.time()
    while time.time() - t0 < limit:
        time.sleep(0.5)
        st = api("/job/status")
        if st.get("state") not in BUSY:
            return st.get("state"), st.get("message", "")
    api("/job/cancel", {})
    return "timeout", "시간 초과"


NUM = r"([+-]?\d+(?:\.\d+)?)"


def parse_dock(lines):
    out = {"확인좌우": None, "다시서기": 0, "돌아섬좌우": None,
           "정렬끝좌우": None, "다리사이최대모서리": None,
           "최종좌우": None, "최종방향": None, "최종모서리": None}
    worst = None
    for ln in lines:
        if "[api_node]" not in ln:
            continue
        m = re.search(r"대차 확인: .*좌우 " + NUM + "mm", ln)
        if m:
            out["확인좌우"] = float(m.group(1))
        if "실제 축 위로 설 자리를 옮긴다" in ln:
            out["다시서기"] += 1
        m = re.search(r"캐스터 정렬 끝 — 렉 축 좌우 " + NUM + "mm", ln)
        if m:
            out["정렬끝좌우"] = float(m.group(1))
        m = re.search(r"진입 라이다: 축 거리 " + NUM + r"mm 좌우 " + NUM
                      + r"mm 방향 " + NUM + r"도 모서리 " + NUM, ln)
        if m and out["돌아섬좌우"] is None:
            out["돌아섬좌우"] = float(m.group(2))
        # 다리 사이 최대 모서리 — 작업 기준과 라이다 기준 중 나쁜 쪽
        m = re.search(r"진입 남은 +" + NUM + r"mm .*모서리 +" + NUM, ln)
        if m and float(m.group(1)) < IN_M:
            worst = max(worst or 0.0, float(m.group(2)))
        m = re.search(r"진입 라이다: 축 거리 " + NUM + r"mm .*모서리 " + NUM
                      + "mm", ln)
        if m and float(m.group(1)) < IN_M:
            worst = max(worst or 0.0, float(m.group(2)))
        m = re.search(r"진입 끝\((작업|라이다) 기준\): 렉 축 좌우 " + NUM
                      + r"mm .*?방향 " + NUM + r"도  모서리 " + NUM, ln)
        if m:
            # 둘 다 찍히면 뒤(라이다, 더 나쁜 쪽)가 남는다 — api_node 와 같다
            out["최종좌우"] = float(m.group(2))
            out["최종방향"] = float(m.group(3))
            out["최종모서리"] = float(m.group(4))
    out["다리사이최대모서리"] = worst
    # S자 지표 — 렉 밖(남은 거리 > 다리 사이) 구간의 방향 흔들림 폭과 부호가
    # 바뀐 횟수(±0.3도 밖만 셈). 2026-09-29 영상 지적 "S자로 휘어져 들어감"
    ys = []
    for ln in lines:
        m = re.search(r"진입 남은 +" + NUM + r"mm +좌우 +" + NUM + r" +방향 +"
                      + NUM + "도", ln)
        if m and float(m.group(1)) > IN_M:
            ys.append(float(m.group(3)))
    flips, last = 0, 0
    for y in ys:
        sg = 1 if y > 0.3 else (-1 if y < -0.3 else 0)
        if sg and last and sg != last:
            flips += 1
        if sg:
            last = sg
    out["렉밖방향폭"] = (max(ys) - min(ys)) if ys else None
    out["부호바뀜"] = flips if ys else None
    return out


def parse_exit(lines):
    out = {"나오기최대좌우": None, "나오기최대모서리": None}
    for ln in lines:
        m = re.search(r"나오기 남은 +" + NUM + r"mm +축 좌우 +" + NUM
                      + r"mm .*모서리 +" + NUM, ln)
        if m:
            e, c = abs(float(m.group(2))), float(m.group(3))
            out["나오기최대좌우"] = max(out["나오기최대좌우"] or 0.0, e)
            out["나오기최대모서리"] = max(out["나오기최대모서리"] or 0.0, c)
    return out


def fmt(v, unit=""):
    return "  -" if v is None else "%+.0f%s" % (v, unit)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument("poi", help="도킹할 대차 POI 이름 (예: 렉2)")
    ap.add_argument("-n", type=int, default=5, help="도킹 횟수 (기본 5)")
    a = ap.parse_args()

    rows = []
    for i in range(a.n):
        # 1) 렉 밑이면 빠져나온다. 첫 회차는 직전 작업이 실패로 끝나 렉에
        #    걸쳐 있을 수도 있으므로 늘 먼저 시도한다 — 렉 밑이 아니면
        #    api_node 가 거절하고, 그때는 바로 도킹으로 간다.
        if True:
            pos = log_pos()
            r = api("/job/exit", {})
            if not r.get("ok"):
                print("[%d] 나오기 안 함: %s" % (i + 1, r.get("message")))
                msg = r.get("message") or ""
                # 렉 밑이 아니면(첫 회차) 바로 도킹으로 간다. 문구는 2026-09-29
                # 나오기 기준 변경으로 '렉 밑에서 렉을 확인하지 못해' 로 바뀌었다.
                if i > 0 or not ("렉 밑에 있지 않습니다" in msg
                                 or "렉 밑에서 렉을 확인하지 못해" in msg):
                    break
            else:
                st, msg = wait(60)
                ex = parse_exit(log_since(pos))
                print("[%d] 나오기 %s — 최대 좌우 %s mm, 최대 모서리 %s mm"
                      % (i + 1, msg, fmt(ex["나오기최대좌우"]),
                         fmt(ex["나오기최대모서리"])))
                if st != "done":
                    break
                time.sleep(1.0)
        # 2) 도킹
        pos = log_pos()
        t0 = time.time()
        r = api("/job/start", {"poi": a.poi})
        if not r.get("ok"):
            print("[%d] 도킹 거절: %s" % (i + 1, r.get("message")))
            break
        st, msg = wait(240)
        d = parse_dock(log_since(pos))
        d["결과"] = "성공" if st == "done" else "%s: %s" % (st, msg)
        d["시간"] = time.time() - t0
        rows.append(d)
        print("[%d] 도킹 %s (%.0f초) — 다리 사이 최대 모서리 %s mm, 최종 좌우 %s mm"
              % (i + 1, d["결과"], d["시간"], fmt(d["다리사이최대모서리"]),
                 fmt(d["최종좌우"])))
        if st != "done":
            break
        time.sleep(1.0)

    if not rows:
        return 1
    print("\n회차 | 확인좌우 | 다시서기 | 진입시작좌우 | 정렬끝좌우 | 다리사이최대모서리"
          " | 최종좌우 | 최종방향 | 최종모서리 | 렉밖방향폭 | 부호바뀜 | 결과 | 접촉")
    for k, d in enumerate(rows, 1):
        print("%4d | %8s | %8d | %12s | %10s | %18s | %8s | %8s | %10s | %10s | %8s | %s |"
              % (k, fmt(d["확인좌우"]), d["다시서기"], fmt(d["돌아섬좌우"]),
                 fmt(d["정렬끝좌우"]), fmt(d["다리사이최대모서리"]),
                 fmt(d["최종좌우"]),
                 "  -" if d["최종방향"] is None else "%+.1f" % d["최종방향"],
                 fmt(d["최종모서리"]),
                 "  -" if d.get("렉밖방향폭") is None else "%.1f도" % d["렉밖방향폭"],
                 "  -" if d.get("부호바뀜") is None else "%d" % d["부호바뀜"],
                 d["결과"]))
    return 0


if __name__ == "__main__":
    sys.exit(main())
