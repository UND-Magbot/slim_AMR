#!/usr/bin/env python3
"""아티팩트 HTML 을 인쇄용으로 감싸 PDF 로 뽑는다.

아티팩트는 <html><head><body> 없이 본문만 들고 있고, 발행 시점에 감싸진다.
여기서는 그 껍데기를 직접 씌우고 인쇄용 규칙을 얹는다.
  - 배경을 흰색으로 고정 (다크 테마로 뽑히면 잉크만 먹는다)
  - 표·그림이 페이지 경계에서 잘리지 않게
  - 웹폰트를 다 받은 뒤에 인쇄
"""
import sys, pathlib
from playwright.sync_api import sync_playwright

src = pathlib.Path(sys.argv[1]).resolve(); out = pathlib.Path(sys.argv[2]).resolve()
body = src.read_text(encoding="utf-8")

PRINT_CSS = """
<style>
@page{ size:A4; margin:16mm 14mm; }
html{ -webkit-print-color-adjust:exact; print-color-adjust:exact; }
/* 인쇄는 항상 밝은 테마로. 다크로 뽑으면 배경만 시커멓게 나온다 */
:root{
  --bg:#ffffff; --panel:#ffffff; --ink:#12171c; --muted:#5a6670;
  --line:#d5dbd6; --line-soft:#ecefec;
  --ok:#1a875b; --warn:#9c6d15; --bad:#ad4630; --accent:#1a875b;
}
body{ background:#fff; }
.wrap{ max-width:100%; padding:0; }
h1{ font-size:30px; }
h2{ font-size:20px; margin-top:26px; padding-top:16px;
    break-after:avoid; page-break-after:avoid; }
h3{ break-after:avoid; page-break-after:avoid; }
figure{ break-inside:avoid; page-break-inside:avoid; padding:14px 12px 10px; }
table{ break-inside:avoid; page-break-inside:avoid; }
tr{ break-inside:avoid; }
.note{ break-inside:avoid; }
p,li{ orphans:3; widows:3; }
/* 세로로 긴 흐름도는 한 장에 담기게 줄인다 */
figure:first-of-type svg{ max-height:238mm; }
</style>
"""

# 아티팩트 본문을 그대로 body 에 넣고, 인쇄용 CSS 를 뒤에 붙여 덮어쓴다.
# title/link/style 을 head 로 옮기려고 잘라내다가 CSS 본문이 새어 나와
# 그대로 인쇄된 적이 있다. 브라우저는 body 안의 style/link 도 정상 적용한다.
html = ("<!doctype html><html lang=\"ko\"><head><meta charset=\"utf-8\">"
        "<meta name=\"viewport\" content=\"width=900\"></head><body>"
        + body + PRINT_CSS + "</body></html>")

tmp = src.with_suffix(".print.html")
tmp.write_text(html, encoding="utf-8")

with sync_playwright() as p:
    b = p.chromium.launch()
    pg = b.new_page(viewport={"width": 900, "height": 1200})
    pg.goto(tmp.as_uri(), wait_until="networkidle")
    pg.emulate_media(media="print", color_scheme="light")
    pg.evaluate("document.fonts.ready")
    pg.wait_for_timeout(1200)
    pg.pdf(path=str(out), format="A4", print_background=True,
           margin={"top": "16mm", "bottom": "16mm",
                   "left": "14mm", "right": "14mm"})
    b.close()
print("생성: %s (%.0f KB)" % (out, out.stat().st_size / 1024))
