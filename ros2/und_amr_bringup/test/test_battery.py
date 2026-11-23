"""배터리(타보스 BMS, CAN2) 값 처리 시험 (2026-09-29)."""
import threading
import time
from collections import deque


def _node(ApiNode):
    n = ApiNode.__new__(ApiNode)
    n.lock = threading.Lock()
    n.motors = {"axis1": {"voltage": "28"}, "axis2": {"voltage": "28"}}
    n.bat = None
    n.bat_t = 0.0
    n.bat_hist = deque(maxlen=n.BAT_HIST_MAX)
    return n


BAT = {"ok": True, "voltage": 27.91, "current": -1.0, "soc": 76, "soh": 98,
       "remaining_ah": 18.9, "energy_wh": 478.8, "temperature": 24.7,
       "time_to_full_min": 0, "time_to_empty_min": 1134, "cycles": 0,
       "status": 0, "flags": [], "age_ms": 200, "design_ah": 25.0}


def test_fresh_bms_value_is_used(ApiNode):
    n = _node(ApiNode)
    n.bat, n.bat_t = dict(BAT), time.time()
    s = n.battery_state()
    assert s["source"] == "bms" and s["soc"] == 76
    assert s["state"] == "discharging"
    assert abs(s["power_w"] - (-27.9)) < 0.05
    assert s["bus_voltage"] == 28.0


def test_stale_bms_falls_back_to_motor_bus(ApiNode):
    n = _node(ApiNode)
    n.bat, n.bat_t = dict(BAT), time.time() - 10
    s = n.battery_state()
    assert s["source"] == "motor driver DC bus" and s["voltage"] == 28.0
    assert not s["ok"] and s["bms_error"]


def test_bms_error_is_reported(ApiNode):
    n = _node(ApiNode)
    n.bat, n.bat_t = {"ok": False, "error": "배터리 응답 없음"}, time.time()
    s = n.battery_state()
    assert s["bms_error"] == "배터리 응답 없음"


def test_history_is_thinned_to_plot_limit(ApiNode):
    n = _node(ApiNode)
    now = int(time.time())
    for k in range(2000):                     # 10초 간격, 약 5.5시간
        n.bat_hist.append([now - (2000 - k) * 10, 60, 27.0, -1.0, 25.0])
    h = n.battery_history(6)
    assert 0 < len(h["t"]) <= n.BAT_PLOT_MAX
    assert h["soc"][0] == 60 and h["step_s"] == 30     # 3개씩 묶음
    # 요청 창 밖은 빠진다
    assert 179 <= len(n.battery_history(0.5)["t"]) <= 180   # 경계 한 점은 시각 반올림에 따라


def test_firmware_line_parses(api):
    import importlib.util
    import pathlib
    import sys
    src = pathlib.Path(api.__file__).with_name("base_node.py")
    spec = importlib.util.spec_from_file_location("base_node", src)
    bn = importlib.util.module_from_spec(spec)
    sys.modules["base_node"] = bn
    spec.loader.exec_module(bn)
    m = bn.BAT_RE.search("> [BAT] v=2791 i=-100 st=0012 soc=76 soh=98 rc=1890 "
                         "ae=4788 t=-15 ttf=0 tte=1134 cyc=3 age=210")
    assert m and m.group("i") == "-100" and m.group("t") == "-15"
    st = int(m.group("st"), 16)
    assert [n for b, n in enumerate(bn.BAT_FLAGS) if st >> b & 1] == \
        ["저전압", "고온"]


def test_line_crc_matches_firmware(api):
    # 2026-09-30: base_node 의 CRC-16 이 펌웨어(L2DB_Crc16, CCITT-FALSE)와 같아야 한다.
    import importlib.util, pathlib, sys
    src = pathlib.Path(api.__file__).with_name("base_node.py")
    spec = importlib.util.spec_from_file_location("base_node_crc", src)
    bn = importlib.util.module_from_spec(spec)
    sys.modules["base_node_crc"] = bn
    spec.loader.exec_module(bn)
    assert bn.SerialPort.crc16(b"123456789") == 0x29B1       # 표준 확인값
    sp = bn.SerialPort.__new__(bn.SerialPort)
    assert sp.with_crc("w 1325 -1325") == "w 1325 -1325 %8FDF"
