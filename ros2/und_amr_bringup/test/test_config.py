"""설정 파일을 글로 고치는 부분의 회귀 시험.

yaml 로 읽고 다시 쓰면 이 파일들의 설명 주석이 전부 날아간다.
새 대차를 넣는 사람이 읽어야 할 내용이라 글로 다룬다 — 그래서 시험이 필요하다.
"""
import threading

import pytest

SAMPLE = '''# 대차(렉) 규격 등록표
#
#   width  : 좌우 다리 중심 간 거리 [m]

cart_detector:
  ros__parameters:
    max_range: 3.0          # 이 거리 안에서만 찾는다 [m]
    carts: ["LG", "LG2"]
    LG:
      width: 0.70
      depth: 0.50
      tol: 0.06
      note: "2026-07 실측"
    # LG2 는 캐스터가 GDS-100 이다
    LG2:
      width: 0.64
      depth: 0.545
      tol: 0.06
      note: "2026-08 실측"
'''


def carts_line(text):
    for ln in text.splitlines():
        if ln.strip().startswith("carts:"):
            return ln.strip()
    return None


def test_새_대차를_넣으면_목록과_블록이_함께_생긴다(ApiNode):
    out = ApiNode._yaml_put_cart(SAMPLE, "S300", 0.707, 0.688, 0.06, "실측")
    assert '"S300"' in carts_line(out)
    assert "    S300:" in out
    assert "width: 0.707" in out and "depth: 0.688" in out


def test_이미_있는_대차는_값만_바뀐다(ApiNode):
    out = ApiNode._yaml_put_cart(SAMPLE, "LG2", 0.650, 0.574, 0.04, "다시 실측")
    assert carts_line(out).count("LG2") == 1, "목록에 두 번 들어갔다"
    assert out.count("    LG2:") == 1, "블록이 두 개가 됐다"
    assert "width: 0.650" in out
    assert "0.64" not in out.split("    LG2:")[1][:120], "옛 값이 남았다"
    assert "    LG:" in out and "width: 0.70" in out, "다른 대차를 건드렸다"


def test_주석이_살아남는다(ApiNode):
    out = ApiNode._yaml_put_cart(SAMPLE, "S300", 0.707, 0.688, 0.06, "실측")
    assert "# 대차(렉) 규격 등록표" in out
    assert "width  : 좌우 다리 중심 간 거리" in out
    assert "# 이 거리 안에서만 찾는다" in out


def test_지우면_목록과_블록이_함께_빠진다(ApiNode):
    out, gone = ApiNode._yaml_drop_cart(SAMPLE, "LG2")
    assert gone
    assert "LG2" not in carts_line(out)
    assert "    LG2:" not in out
    assert "# LG2 는 캐스터가" not in out, "블록 위 설명 주석이 남았다"
    assert "    LG:" in out, "다른 대차까지 지웠다"


def test_없는_대차를_지우면_알려준다(ApiNode):
    out, gone = ApiNode._yaml_drop_cart(SAMPLE, "없는대차")
    assert not gone


def test_넣고_지우면_제자리로(ApiNode):
    a = ApiNode._yaml_put_cart(SAMPLE, "TMP", 0.5, 0.4, 0.05, "x")
    b, gone = ApiNode._yaml_drop_cart(a, "TMP")
    assert gone
    assert carts_line(b) == carts_line(SAMPLE)
    assert "TMP" not in b


# ---------------------------------------------------------------- 고정 IP 검사

@pytest.fixture
def node(ApiNode):
    return object.__new__(ApiNode)


def test_자동이면_주소를_비운다(node):
    args, err = node._ip_args({"mode": "auto"})
    assert err is None
    assert args[args.index("ipv4.method") + 1] == "auto"


def test_고정_IP_를_인자로_옮긴다(node):
    args, err = node._ip_args({"mode": "manual", "address": "192.168.30.50",
                               "prefix": 24, "gateway": "192.168.30.1",
                               "dns": "8.8.8.8, 1.1.1.1"})
    assert err is None
    assert args[args.index("ipv4.addresses") + 1] == "192.168.30.50/24"
    assert args[args.index("ipv4.gateway") + 1] == "192.168.30.1"
    assert "8.8.8.8" in args[args.index("ipv4.dns") + 1]


@pytest.mark.parametrize("bad,왜", [
    ({"mode": "manual", "address": "192.168.30", "prefix": 24}, "자리가 모자람"),
    ({"mode": "manual", "address": "192.168.30.999", "prefix": 24}, "255 초과"),
    ({"mode": "manual", "address": "192.168.30.5", "prefix": 99}, "넷마스크 범위"),
    ({"mode": "manual", "address": "192.168.30.5", "prefix": "스물넷"}, "숫자가 아님"),
    ({"mode": "manual", "address": "192.168.30.5", "prefix": 24,
      "gateway": "192.168"}, "게이트웨이 형식"),
    ({"mode": "manual", "address": "192.168.30.5", "prefix": 24,
      "dns": "8.8.8"}, "DNS 형식"),
])
def test_잘못된_고정_IP_는_적용_전에_막는다(node, bad, 왜):
    """현장에서 오타 하나로 로봇이 고립되면 안 된다."""
    args, err = node._ip_args(bad)
    assert args is None and err, "통과시켰다 (%s)" % 왜


# ---------------------------------------------------------------- nmcli 출력 자르기

def test_nmcli_출력을_규칙대로_자른다(ApiNode):
    """nmcli -t 는 ':' 를 구분자로 쓰고 값 안의 ':' 는 백슬래시로 피해 둔다."""
    assert ApiNode._tsplit("a:b:c") == ["a", "b", "c"]
    assert ApiNode._tsplit(r"my\:ssid:80211") == ["my:ssid", "80211"]
    assert ApiNode._tsplit("") == [""]
    assert ApiNode._tsplit("a::c") == ["a", "", "c"]
