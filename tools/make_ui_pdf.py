#!/usr/bin/env python3
"""
UND_AMR 운용 콘솔 화면 설명서 PDF 생성

    python3 tools/make_ui_pdf.py
    python3 tools/make_ui_pdf.py --shots ~/amr_shots --out docs/UND_AMR_UI.pdf

스크린샷은 tools 의 촬영 절차로 미리 만들어 둔 것을 쓴다.
(firefox --headless --screenshot "http://localhost:8090/?shot=4000#<섹션>")
"""

import argparse
import os
import sys
from datetime import date

from reportlab.lib.colors import HexColor
from reportlab.lib.pagesizes import A4, landscape
from reportlab.lib.utils import ImageReader
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.cidfonts import UnicodeCIDFont
from reportlab.pdfbase.ttfonts import TTFont
from reportlab.pdfgen import canvas as pdfcanvas

# --------------------------------------------------------------------------

# 시스템의 Noto CJK 는 CFF(OpenType/PostScript) 방식이라 reportlab 의
# TTFont 로는 못 읽는다. TrueType(.ttf) 한글 폰트가 있으면 그걸 쓰고,
# 없으면 reportlab 내장 CID 폰트로 떨어진다(폰트 파일 설치 불필요).
TTF_CANDIDATES = [
    "/usr/share/fonts/truetype/nanum/NanumGothic.ttf",
    "/usr/share/fonts/truetype/nanum/NanumBarunGothic.ttf",
]
CID_FALLBACK = "HYGothic-Medium"
FONT_NAME = "KR"

BG = HexColor("#0f1116")
FG = HexColor("#e8eaf0")
DIM = HexColor("#98a0b0")
ACCENT = HexColor("#4da3ff")
OK = HexColor("#3ddc97")
WARN = HexColor("#ffb454")
LINE = HexColor("#2a3140")

# (파일명, 제목, 상태, 설명 줄들)
PAGES = [
    ("01_dash_auto", "대시보드 - 자동 모드", "ok", [
        "메인 화면. 지도 위에 로봇 위치와 라이다 스캔을 실시간으로 겹쳐 보여준다.",
        "- 지도 : SLAM 이 만든 점유격자. 흰색=자유공간, 검정=장애물, 회색=미탐색",
        "- 로봇 : 초록 화살표가 현재 위치와 바라보는 방향",
        "- 라이다 : 거리별 색 구분 (0.5m 미만 빨강 / 1m 미만 주황 / 그 외 파랑)",
        "- 상단 토글로 라이다-로봇추적-이동궤적 표시를 켜고 끌 수 있고,",
        "  드래그로 화면 이동, 휠로 확대/축소, [맞춤] 으로 전체 보기",
        "- 자동 모드에서는 조이스틱이 숨겨지고 수동 조종이 차단된다",
    ]),
    ("02_dash_manual", "대시보드 - 수동 모드", "ok", [
        "[수동] 을 선택하면 조이스틱이 나타나 직접 조종할 수 있다.",
        "- 스틱을 미는 방향과 크기가 그대로 속도가 된다 (위=전진, 좌우=회전)",
        "- 현재 지령값을 m/s - rad/s 로 실시간 표시",
        "- 손을 떼면 중앙 복귀 + 즉시 정지. 키보드 W A S D 도 사용 가능",
        "",
        "안전장치 3중 구성",
        "  1) 조종 중에는 0.15초마다 명령을 갱신, 떼는 순간 정지 명령 전송",
        "  2) 명령이 0.5초 끊기면 구동부가 스스로 정지",
        "  3) 자동 모드이거나 비상정지 상태면 조종 자체가 거부됨",
    ]),
    ("03_motor", "모터", "ok", [
        "축별 상태를 표로 보여준다. 지령 rpm, 실측 rpm, 엔코더 위치, 전압, 알람.",
        "- 축1 = CAN 노드 1, 축2 = CAN 노드 2 (L2DB4830 듀얼 서보 드라이버)",
        "- 알람은 코드가 아니라 사람이 읽는 문장으로 표시",
        "  (예: 모터 미연결, 모선 저전압, 과부하)",
        "- 엔코더 4096 counts/회전, 감속비 1.0 직결 - 실측으로 확정한 값",
    ]),
    ("04_battery", "배터리", "ok", [
        "모터 드라이버가 보고하는 DC 모선 전압을 표시한다.",
        "- 전용 BMS 가 없어 잔량(%) 은 알 수 없고 전압만 확인 가능",
        "- 22V 미만 경고, 20V 미만 위험으로 색이 바뀐다",
        "- 정확한 잔량이 필요하면 BMS 를 달고 그 값을 올려야 한다",
    ]),
    ("05_lidar", "라이다", "ok", [
        "LDS-50C-E-12 의 원본 스캔을 로봇 중심으로 표시한다.",
        "지도 없이 주변 장애물만 확인할 때 쓴다.",
        "- 15Hz, 회전당 2400점, 각분해능 0.15도",
        "- 표시 범위를 1-30m 로 조절 가능",
        "- 전원만 넣으면 회전은 하지만 데이터는 나오지 않는다.",
        "  시작 명령(LSTARH)을 보내야 스트리밍이 시작된다",
    ]),
    ("06_camera", "카메라", "ok", [
        "RealSense D435i 컬러 영상을 실시간 스트리밍한다.",
        "- 뎁스는 컬러 프레임에 정합(align)되어 같은 픽셀이 같은 지점을 가리킨다",
        "- 내장 IMU(200Hz)는 휠 오도메트리와 융합해 주행 방향 오차를 보정한다",
        "- 뎁스 영상 표시는 아직 API 에 추가하지 않았다",
    ]),
    ("07_maps", "맵", "ok", [
        "저장된 지도 목록. 아직 주행 매핑을 하지 않아 비어 있다.",
        "- 지도는 -/ros2_ws/maps 에 저장된다",
        "- 로봇이 움직여야 지도가 넓어진다. 정지 상태에서는",
        "  주변 한 스캔 분량만 만들어진다",
    ]),
    ("08_mapping", "매핑", "todo", [
        "미구현. 화면에 무엇이 필요한지 안내만 표시된다.",
        "- 지도 작성 시작/정지/저장을 웹에서 실행하려면",
        "  SLAM 노드의 서비스를 API 에 연결해야 한다",
        "- 로봇 조립 후 주행하며 지도를 만드는 단계에서 구현 예정",
    ]),
    ("09_moves", "이동 작업", "todo", [
        "미구현. 지도상의 목표 지점을 지정해 자율 이동시키는 기능.",
        "- 자율주행 스택(nav2) 연동이 필요하다",
        "- 지도가 있어야 의미가 있어 매핑 다음 단계",
    ]),
    ("10_nodes", "노드", "ok", [
        "실행 중인 ROS 노드 목록. 어떤 기능이 살아 있는지 한눈에 확인한다.",
        "- 구동부-라이다-카메라-SLAM-API 노드가 모두 보여야 정상",
    ]),
    ("11_topics", "토픽", "ok", [
        "발행 중인 ROS 토픽과 자료형 목록.",
        "- /scan(라이다), /odom(주행거리), /map(지도),",
        "  /camera/...(영상), /diagnostics(진단) 등",
        "- 데이터가 흐르지 않을 때 어디서 끊겼는지 추적하는 데 쓴다",
    ]),
    ("12_log", "로그", "ok", [
        "시스템 로그를 실시간으로 보여준다. 레벨별 필터 제공.",
        "- INFO / WARN / ERROR 로 걸러서 볼 수 있다",
        "- 터미널에 접속하지 않고도 이상 여부를 확인할 수 있다",
    ]),
    ("13_params", "파라미터", "todo", [
        "미구현. 차체 치수-속도 제한 등을 웹에서 수정하는 기능.",
        "- 현재는 설정 파일(config/amr_params.yaml)을 직접 편집한다",
        "- 윤거-바퀴직경은 조립 후 주행 보정으로 확정할 값",
    ]),
    ("14_services", "서비스", "ok", [
        "로봇 제어 명령 목록과 실행 버튼.",
        "- 즉시 정지, 모터 사용, 축 해제, 알람 클리어",
        "- 웹이 시리얼을 직접 열지 않고 ROS 서비스로 호출한다.",
        "  같은 명령을 상위 시스템에서도 그대로 쓸 수 있다",
    ]),
    ("15_device", "장비 정보", "ok", [
        "로봇의 하드웨어-소프트웨어 사양.",
        "- 제어기 STM32H743, 모터 드라이버 L2DB4830 (CAN 500kbit/s)",
        "- 라이다 LDS-50C-E-12, 카메라 RealSense D435i",
        "- 상위 시스템이 로봇 종류를 자동 인식하는 데도 쓰인다",
    ]),
]


def register_font():
    for path in TTF_CANDIDATES:
        if not os.path.exists(path):
            continue
        try:
            pdfmetrics.registerFont(TTFont("KR", path))
            return "TrueType %s" % os.path.basename(path)
        except Exception:                                    # noqa: BLE001
            continue
    try:
        pdfmetrics.registerFont(UnicodeCIDFont(CID_FALLBACK))
        pdfmetrics.registerFontFamily("KR", normal=CID_FALLBACK)
        # 이름만 KR 로 맞춰 두면 이후 코드가 그대로 동작한다
        pdfmetrics.registerFont(UnicodeCIDFont(CID_FALLBACK))
        global FONT_NAME
        FONT_NAME = CID_FALLBACK
        return "CID %s (내장)" % CID_FALLBACK
    except Exception as e:                                   # noqa: BLE001
        print("폰트 등록 실패: %s" % e)
        return None


def draw_cover(c, W, H, n_pages, doc_date):
    c.setFillColor(BG)
    c.rect(0, 0, W, H, fill=1, stroke=0)
    c.setFillColor(ACCENT)
    c.rect(0, H - 6, W, 6, fill=1, stroke=0)

    c.setFillColor(FG)
    c.setFont(FONT_NAME, 34)
    c.drawString(56, H - 130, "UND_AMR 운용 콘솔")
    c.setFont(FONT_NAME, 19)
    c.setFillColor(DIM)
    c.drawString(56, H - 166, "화면 설명서")

    c.setStrokeColor(LINE)
    c.line(56, H - 196, W - 56, H - 196)

    c.setFont(FONT_NAME, 12)
    c.setFillColor(DIM)
    y = H - 232
    for label, value in [
        ("작성일", doc_date),
        ("대상", "2륜 차동구동 AMR - STM32 구동부 + 2D 라이다 + 뎁스 카메라"),
        ("접속", "http://<로봇 IP>:8090"),
        ("구성", "%d개 화면 (구현 %d / 미구현 %d)"
         % (n_pages,
            sum(1 for p in PAGES if p[2] == "ok"),
            sum(1 for p in PAGES if p[2] == "todo"))),
    ]:
        c.setFillColor(DIM)
        c.drawString(56, y, label)
        c.setFillColor(FG)
        c.drawString(150, y, value)
        y -= 26

    y -= 18
    c.setFillColor(DIM)
    c.setFont(FONT_NAME, 11)
    for line in [
        "이 문서는 로봇 운용 화면을 페이지별로 정리한 것이다.",
        "구현이 끝난 화면은 실제 동작 중인 데이터를 캡처했고,",
        "아직 구현하지 않은 화면은 무엇이 필요한지 표시해 두었다.",
    ]:
        c.drawString(56, y, line)
        y -= 19

    c.setFont(FONT_NAME, 9)
    c.setFillColor(HexColor("#5c6577"))
    c.drawString(56, 40, "UND_AMR | Robot Admin")
    c.showPage()


def draw_page(c, W, H, idx, total, shot, title, state, lines):
    c.setFillColor(BG)
    c.rect(0, 0, W, H, fill=1, stroke=0)

    # 머리말
    c.setFillColor(FG)
    c.setFont(FONT_NAME, 20)
    c.drawString(46, H - 52, title)

    badge = "구현" if state == "ok" else "미구현"
    col = OK if state == "ok" else WARN
    tw = pdfmetrics.stringWidth(badge, FONT_NAME, 10) + 18
    bx = 46 + pdfmetrics.stringWidth(title, FONT_NAME, 20) + 14
    c.setFillColor(col)
    c.roundRect(bx, H - 54, tw, 19, 9, fill=1, stroke=0)
    c.setFillColor(BG)
    c.setFont(FONT_NAME, 10)
    c.drawCentredString(bx + tw / 2, H - 48, badge)

    c.setFillColor(HexColor("#5c6577"))
    c.setFont(FONT_NAME, 10)
    c.drawRightString(W - 46, H - 50, "%d / %d" % (idx, total))

    c.setStrokeColor(LINE)
    c.line(46, H - 66, W - 46, H - 66)

    # 설명
    c.setFont(FONT_NAME, 9.5)
    y = H - 84
    for ln in lines:
        if not ln:
            y -= 5
            continue
        c.setFillColor(DIM if ln.startswith(("-", " ")) else FG)
        c.drawString(46, y, ln)
        y -= 13.5

    # 이미지
    if os.path.exists(shot):
        img = ImageReader(shot)
        iw, ih = img.getSize()
        avail_w = W - 72
        avail_h = y - 34
        k = min(avail_w / iw, avail_h / ih)
        dw, dh = iw * k, ih * k
        x = (W - dw) / 2
        yy = 28 + (avail_h - dh) / 2
        c.setStrokeColor(LINE)
        c.rect(x - 1, yy - 1, dw + 2, dh + 2, fill=0, stroke=1)
        c.drawImage(img, x, yy, dw, dh, mask="auto")
    else:
        c.setFillColor(WARN)
        c.drawString(46, y - 20, "(스크린샷 없음: %s)" % os.path.basename(shot))

    c.showPage()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--shots", default=os.path.expanduser("~/amr_shots"))
    ap.add_argument("--date", default=date.today().isoformat(),
                    help="표지에 찍을 작성일 (기본: 오늘)")
    ap.add_argument("--out", default=os.path.expanduser(
        "~/projects/UND_ARM/docs/UND_AMR_UI_설명서.pdf"))
    args = ap.parse_args()

    used = register_font()
    if not used:
        sys.exit("한글 폰트를 찾지 못했습니다. sudo apt install fonts-nanum")
    print("폰트     : %s" % used)

    os.makedirs(os.path.dirname(args.out), exist_ok=True)
    W, H = landscape(A4)
    c = pdfcanvas.Canvas(args.out, pagesize=(W, H))
    c.setTitle("UND_AMR 운용 콘솔 화면 설명서")

    draw_cover(c, W, H, len(PAGES), args.date)
    missing = []
    for i, (name, title, state, lines) in enumerate(PAGES, 1):
        shot = os.path.join(args.shots, name + ".png")
        if not os.path.exists(shot):
            missing.append(name)
        draw_page(c, W, H, i, len(PAGES), shot, title, state, lines)
    c.save()

    print("생성 완료 : %s" % args.out)
    print("  페이지  : %d (표지 1 + 화면 %d)" % (len(PAGES) + 1, len(PAGES)))
    print("  크기    : %.1f MB" % (os.path.getsize(args.out) / 1024 / 1024))
    if missing:
        print("  ⚠ 누락된 스크린샷: %s" % ", ".join(missing))


if __name__ == "__main__":
    main()
