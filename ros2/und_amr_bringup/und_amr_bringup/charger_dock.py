"""충전 스테이션 후진 도킹 (2026-11-12, ApiNode 에 섞어 쓴다).

사용자: "도킹에서 충전소하면 도킹까지 로직이 안되어있어?" / "딱접촉하면 멈추는거라
일단 거기까지만 짜고 나중에 전류 들어오면 정차하는것도 추가로 넣자".

  moving   nav2 로 대기 자리(충전소 POI 에서 바라보는 쪽으로 standoff)까지 간다.
  aligning 서서 V 홈(charger_detect)으로 실제 스테이션을 잰다. POI 에서 CD_POI_NEAR
           넘게 떨어진 V 는 다른 물체로 본다. 스테이션 축에서 좌우 CD_LAT_OK · 방향
           CD_YAW_OK 넘게 어긋나면 maneuver 로 제자리에서 맞추고 다시 잰다(최대
           CD_REALIGN_MAX 번).
  maneuver 지도(SLAM)를 안 쓰고 odom + 방금 본 스테이션으로: 좌우가 어긋났으면 축에서
           CD_SIDE_M 더 먼 축 위 점을 향해 돌고 → 거기까지 곧게 → 스테이션 반대쪽으로
           돈다. 방향만이면 제자리에서 돈다.
  ⚠ 2026-11-12: 처음엔 어긋나면 실제 스테이션 기준 대기 자리를 지도 좌표로 nav2 에 다시
    보냈다. 이 스테이션 근처는 SLAM 이 30cm 안팎 흔들려(POI 와 294~356mm), nav2 는 지도
    목표에 2mm 로 섰는데 실제 축에서는 또 좌우 33mm · 8.6도 어긋나 실패했다(사용자: "기존
    poi랑 조금 떨어져도 도킹이 되어야해 정확한 위치에서 할순 없어"). 그래서 V 가 한 번
    보이면 그다음은 지도를 안 쓴다. 시작할 때 이미 V 가 보이면 nav2 이동도 건너뛴다.
  backing  /cmd_vel_noradius 로 직접 후진한다 (충돌감시를 거치지 않는다 — 스테이션이
           곧 정지 구역에 들어오므로). 매 스캔 V 로 스테이션 자리를 조금씩 보정하며
           축을 따라간다. 스테이션이 차체 여유(4.5cm) 안에 들어오면 라이다에서 빠지므로
           CD_BLIND_GAP 부터는 마지막 값으로 곧게 간다.
  done     충전 전류(BMS +CD_CHARGE_A 넘게)가 들어오면 바로 선다 (2026-11-13, 사용자: "배터리에
           전류도 들어가서 충전되면 멈추는 방식으로 바꿔도 되지 않을까?"). 계산한 접촉
           자리(틈 0) 또는 바퀴가 막혔는데 아직 전류가 없으면 CD_PUSH_V 로 CD_PUSH_M 더
           밀어 보고, 접촉 뒤 CD_CHARGE_WAIT_S 안에 전류가 없으면 실패. BMS 값을 못 받으면
           예전처럼 접촉 자리에서 끝낸다.

나오기 (2026-11-12, 사용자: "도킹에서 나갈때는 마찬가지로 0.5m는 앞으로 나가고 출발해야함"):
  충전소에 붙어 있을 때(라이다에 V 홈이 바로 뒤 틈 CD_DOCKED_GAP 안에 보이면 — 또는 방금
  도킹을 마친 자리, 충전소 POI 도킹 자세 CD_DOCKED_NEAR · CD_DOCKED_YAW 안) nav_goto 가 불리면,
  ⚠ 처음엔 SLAM 자세로만 봤는데, 붙어 있는 로봇을 SLAM 이 스테이션 안쪽 21cm 로 보고 있어
    나오기를 건너뛰었다. nav2 는 뒤에 붙은 스테이션이 충돌감시 정지 구역(뒤 6cm)에 들어
    있어 꼼짝 못 했다(2026-11-12). 붙어 있으면 라이다에 V 가 틈 25mm 로 늘 보인다. 목표 번호는 그대로 둔 채 먼저 앞으로 CD_LEAVE_M
  곧게 나가고(방향 붙잡기) 그다음 원래 출발(_nav_dispatch)로 넘긴다. 제자리에서 돌면 뒤
  모서리가 스테이션을 긁는다. 앞 통로에 장애물이 있으면 서서 기다린다.

빠르게 (2026-11-13, 사용자: "충전소 도킹하러갈때 너무 오래걸려 ... 후진하면서 맞춰서 들어가는데
  우린 정확한 위치에서 완전 일직선 후진"): 55초 중 정밀 도착(mm 기어가기 · 방향 보정) 16초,
  제자리 맞추기 10초였다. 그래서
  - 대기 자리 CD_HANDOFF_M · CD_HANDOFF_YAW 안에 들어오고 V 가 보이면 nav2 도착을 안 기다리고
    끊고 넘겨받는다.
  - 좌우 CD_LAT_OK(15cm) · 방향 CD_YAW_OK(20도) 안이면 서서 맞추지 않고 후진하면서 곡선으로
    맞춘다(최대 CD_HEAD_MAX 기울임). 그 밖일 때만 제자리 맞추기.
  - 후진 0.12 m/s, 마지막 CD_SLOW_GAP 만 천천히.
  - 목표 방향과 CD_PIVOT_YAW 넘게 어긋나면 후진을 멈추고 그 자리에서 먼저 돈다 (2026-11-16 S자).

안전은 여기서 직접 진다: 처음 틈 + CD_OVER 넘게 가면, CD_BACK_S 넘게 걸리면,
축에서 CD_LAT_BAD 넘게 벌어지면, 스캔이 CD_SCAN_STALE_S 넘게 끊기면 선다.
로봇 뒤 통로(차체 폭)에 스테이션 앞면보다 CD_GAP_CLEAR 넘게 가까운 점이 CD_BLOCK_PTS 개
넘게 보이면(사람 · 물건) 그 자리에 서서 기다리고, CD_BLOCK_S 넘게 안 치워지면 실패.
[정지] · [비상정지] · 수동 전환은 stop_now() 에서 cdock_cancel() 로 끊는다.
"""
import math
import time

from geometry_msgs.msg import Twist

from und_amr_bringup.charger_detect import detect_scan


def _wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class ChargerDockMixin:
    CHARGER_HALF_LEN = 0.43675  # 로봇 중심 -> 뒤끝 (URDF base_len/2)
    CD_HZ = 20.0
    CD_BUSY = ("moving", "aligning", "maneuver", "backing")
    CD_ALIGN_N = 5            # 정렬 판단에 쓸 인식 수
    CD_ALIGN_S = 4.0          # 그 안에 못 모으면 실패 [s]
    CD_POI_NEAR = 1.00        # 인식한 스테이션이 POI 에서 이보다 멀면 다른 물체 [m] (SLAM 흔들림 30cm+)
    CD_POI_YAW = math.radians(30)
    CD_POI_GATE = 0.45        # 이보다 POI 에서 먼 인식은 버린다 [m] (2026-11-18)
    CD_LAT_OK = 0.15          # 이 안이면 서서 맞추지 않고 후진하며 맞춘다 [m]
    CD_YAW_OK = math.radians(20.0)
    CD_HANDOFF_M = 0.30       # 대기 자리 이 안 · 방향 CD_HANDOFF_YAW 안이면 nav2 를 끊고 넘겨받는다
    CD_HANDOFF_YAW = math.radians(25)
    CD_STILL_V = 0.02         # 정렬용 인식은 이보다 느릴 때만 모은다 [m/s]
    CD_STILL_W = 0.05
    CD_REALIGN_MAX = 3        # 제자리 맞추기 횟수
    CD_SEE_MIN = 0.30         # 시작할 때 이 틈 ~ CD_SEE_MAX 안에 V 가 보이면 nav2 이동 생략 [m]
    CD_SEE_MAX = 2.00
    # ⚠ 2026-11-18 왕복 10회: 대기 자리에서 -155도 방향 맞추기를 돌다가 25도 남은 곳에서 넘겨받아 서고, 인식한 뒤
    #   다시 -23도 돌았다(회전을 두 번에 나눠 돌고 사이에 섬). 대기 자리에 닿아 방향 맞추기를 시작하는 순간
    #   넘겨받아, POI 기준 중간 지점(틈 CD_MID_GAP) 쪽으로 한 번에 등을 돌린다. 중간 지점에서 V 홈을 재서 맞춘다
    #   (POI 와 실제 스테이션 차이 6~27mm, 최대 67mm). False 면 예전처럼 방향 맞춘 뒤 인식.
    CD_POI_FIRST = True
    # ⚠ 2026-11-18 POI 우선 10회: 중간 지점에서 매번 축 좌우 -12~-29mm · 방향 -5~-6도 -> 제자리 다시 돌기 1~2번(약 4.7초),
    #   끝 좌우 평균 -5mm (예전 -2mm). POI 는 맞다(도킹 끝 지도 자세와 옆 -6mm · 방향 -0.3도). 큰 회전 뒤 바퀴 기록만 믿고
    #   '점' 으로 후진해서 생긴 것. 중간 지점까지 후진하면서 등 뒤 V 홈을 라이다로 계속 재고(CD_AXIS_BLEND),
    #   좌우를 CD_AXIS_L 거리 안에 지우며 축에 올라탄다(_cdock_axis_head, 방향 상한 CD_AXIS_HEAD).
    CD_AXIS_BACK = True
    CD_AXIS_L = 0.15
    CD_AXIS_HEAD = math.radians(12.0)
    CD_AXIS_BLEND = 0.3
    CD_TURN_W_BIG = 0.70      # 등 돌리기처럼 큰 회전 최대 [rad/s] (0.45 -> 0.70 2026-11-18, 충전소엔 렉을 안 싣는다 - 사용자)
    CD_TURN_ACC = 0.6         # 제자리 회전 올리기 [rad/s^2] (갑자기 0.3 으로 뛰지 않게)
    # ⚠ 2026-11-18 왕복 10회: 대기 자리 10cm 앞에서 nav2 를 끊은 뒤 남은 약 8cm 를 매번 1.6~2.6초 기어 들어갔다
    #   (끝 13~19mm 남음). 도킹이 중간 지점까지 축을 다시 잡으므로 기어 들어가지 않고 바로 등 돌리기.
    CD_SKIP_CREEP = True
    CD_CREEP_SHORT = 0.06     # [m]
    CD_APPROACH_YAW_OK = math.radians(8.0)   # 대기 자리 방향 허용 (도킹이 다시 재고 돌므로 대충, 2026-11-18)
    CD_SIDE_M = 0.30          # 좌우 맞추기: 축 위 이만큼 더 먼 점으로 비스듬히 나간다 [m]
    CD_MIN_GAP = 0.55         # 맞춘 뒤 틈이 이보다 작으면 그만큼 더 멀리 나간다 [m]
    CD_TURN_W = 0.30          # 제자리 회전 최대 [rad/s]
    CD_TURN_TOL = math.radians(0.7)
    CD_MOVE_V = 0.08
    CD_MAN_S = 40.0
    CD_V_FAST = 0.12          # 후진 속도 [m/s]
    CD_V_TURN = 0.08          # 좌우를 크게 맞추는 동안 [m/s]
    CD_V_SLOW = 0.04          # 마지막 구간 [m/s]
    CD_SLOW_GAP = 0.15        # 틈이 이보다 작으면 천천히 [m]
    CD_BLIND_GAP = 0.12       # 틈이 이보다 작으면 인식 보정을 멈추고 곧게 [m]
    CD_STOP_GAP = 0.003       # 이만큼 남으면 선다 (0.03 m/s 감속 여유) [m]
    # ⚠ 2026-11-13 실기 S자: 좌우 -58mm 에서 목표 방향 -16도를 잡았는데 실제 회전이 명령보다
    #   0.6~0.8초 늦어 -18도까지 넘쳤다 되돌아왔다. 고정 K_LAT(=좌우를 20cm 앞에서 지움) 대신
    #   '남은 틈의 CD_LOOK_K 배' 앞에서 지운다 — 멀면 완만하게, 가까우면 세게.
    CD_LOOK_K = 0.45
    CD_LOOK_MIN = 0.20        # [m] (= 예전 K_LAT 5)
    CD_LOOK_MAX = 0.50        # [m]
    # 2026-11-13 '틈 30cm 부터 인식 안 섞기' 를 넣었다가 되돌림: 도킹 사진에서 로봇이 충전기
    #   중심보다 오른쪽 14~18mm — 가까이서 잰 라이다 -10mm 가 맞았고 먼 거리 측정이 틀렸다.
    CD_FREEZE_GAP = 0.0       # [m] 0 = 끝까지(CD_BLIND_GAP 까지) 섞는다
    CD_K_LAT = 5.0            # 좌우 1m 당 목표 방향 [rad] (상한 CD_HEAD_MAX). 시뮬 3 -> 좌우 6~16mm 남음, 5 -> 1mm
    CD_HEAD_MAX = math.radians(25)
    CD_K_YAW = 1.5            # 방향 오차 -> 회전 [1/s]
    CD_W_MAX = 0.25           # [rad/s]
    # ⚠ 2026-11-16 S자 진입 (사용자: "각도때문인듯"): 방향 16도 틀어진 채 0.08m/s 로 후진을 시작해
    #   처음 1초 최대로 꺾고 넘어가 반대로 꺾었다 (좌우 +28 -> +2 -> +14 -> -11mm, 방향 +17 -> -4도).
    #   후진하며 맞추기(11/13 빠르게)는 그대로 두고, 목표 방향과 CD_PIVOT_YAW 넘게 어긋나면 그
    #   자리에서 먼저 돌고 CD_PIVOT_YAW0 안으로 오면 속도를 서서히 붙인다. 다시 재지 않는다.
    #   모의(회전 지연 0.4초 · 흔들림 0.04 rad/s)에서 6도로 두면 1초 늘고 흔들림은 그대로여서
    #   크게 틀어졌을 때만 쓴다.
    CD_PIVOT_YAW = math.radians(12.0)
    CD_PIVOT_YAW0 = math.radians(8.0)
    # ⚠ 2026-11-18 사용자: "충전소 들어갈 때 중앙에 안 맞는 거랑 휘청거리는 거 해결하자". 도킹 7번 모두 후진 시작
    #   때 방향이 15도 안팎 틀어져 있었고(nav2 가 대기 자리로 비스듬히 옴) 후진하며 곡선으로 고치다 S자, 끝은
    #   늘 좌우 -9~-14mm (마지막 20cm 는 0.04 m/s 로 아주 완만하게만 고침). 이제 '서서 맞추고 곧게 후진' 두 번:
    #     대기 자리 -> 축 위 틈 CD_MID_GAP 지점을 등지고 제자리 회전 -> 거기까지 곧게 후진 -> 서서 다시 잼(가까워
    #     정확) -> 축 방향으로 제자리 회전(CD_FINAL_YAW 넘으면) -> 마지막 CD_MID_GAP 곧게 후진.
    CD_TWO_LEG = True
    # 2026-11-18 0.30 -> 0.45: 왕복 5번 모두 중간 지점에서 다시 재면 이미 -9~-20mm (멀리서 잰 값의 치우침), 마지막
    #   0.3m 중 안 보이는 12cm 를 빼면 18cm 뿐이라 기울였다 펴며 그대로 남았다(끝 -11~-23mm).
    CD_MID_GAP = 0.45         # 중간 지점: 뒤끝~스테이션 앞면 [m]
    CD_MID_TOL = 0.08         # 틈이 CD_MID_GAP 보다 이만큼 넘게 남았으면 중간 지점까지 먼저 간다
    # ⚠ 2026-11-18 사용자: "들어가다 갑자기 빨라지는 느낌, 다 똑같은 속도로 어느 정도 천천히 들어가야 안 틀어질 것".
    #   0.12 -> 멈춤 -> 0.12 -> 0.04 로 바뀌던 것을 두 단계 모두 CD_DOCK_V 한 가지로, 접촉 직전 CD_DOCK_END_GAP 만 살짝 줄인다.
    CD_DOCK_V = 0.06
    CD_DOCK_END_GAP = 0.04
    CD_DOCK_ACC = 0.10        # 서서 출발할 때 속도를 올리는 기울기 [m/s^2]
    CD_MID_V = CD_DOCK_V      # 중간 지점까지 곧게 후진 [m/s]
    # ⚠ 2026-11-18 1.5 -> 2.0: 중간 지점에서 +2.2도 다시 돈 직후 후진을 시작하면 명령과 반대로 0.8초 동안 같은 쪽으로
    #   더 돌았다(w실제 +0.03~+0.04, 방향 폭 4~5도, 10회 중 3번 - 모두 다시 돈 회차). 안 돈 회차는 폭 0.5~1.6도.
    #   축 따라 후진 뒤 중간 방향은 20회 -0.7~-2.9도. 3.0 이면 실기는 안 돌지만 흔들림 센 모의에서 끝 방향 최대 5.1도(2.0 은 4.4).
    # ⚠ 2026-11-18 한쪽 바퀴 세우고 돌기(bturn, 기어 틈 가설)로 바꿔도 그대로 +3도 더 돌았다(10회 중 2번) - 가설 틀림.
    #   그때 바퀴(w실제)는 곧게, 자이로 방향만 +3도 -> 차체가 미끄러지며 돎. 다시 돈 각도(+2.2~2.5도)만큼 같은 쪽으로 더 감.
    #   작은 제자리 회전은 하지 않는다: 실기 40회 중간 방향 최대 -2.9도 -> 3.0 이면 안 돈다. 끝 방향은 실기 40회 모두 0.8도 안.
    CD_PIVOT_BACK = False
    CD_PIVOT_B = 0.0895
    CD_FINAL_YAW = math.radians(3.0)  # 이 안이면 중간 지점에서 돌지 않는다 (나머지는 마지막 후진이 맞춤)
    CD_MID_TURN_TOL = math.radians(3.0)  # 중간 지점 쪽으로 등 돌리기 - 후진하며 계속 다시 겨누므로 대충
    CD_PIVOTS_MAX = 2
    CD_FINAL_HEAD = math.radians(4.0)   # 마지막 곧게에서 좌우를 지우려 기울 수 있는 최대
    # ⚠ 2026-11-18 사용자: '보정하고 멈췄다가 출발할때 많이 틀어지는듯' - 마지막 후진 출발 0.7초 동안 로봇은 안 돌았는데
    #   (w실제 0) 스캔마다 섞은 스테이션 방향이 -2.7도 바뀌어 w명령 -0.10 으로 실제 -3도 돌았다 되돌아옴(방향 폭 5~6도).
    #   방향은 중간 지점에서 서서 5장 평균한 값으로 고정하고, 좌우 위치만 달리며 고친다.
    CD_MID_YAW_HOLD = True
    CD_FREEZE_AFTER_MID = False         # 마지막 곧게는 중간 지점에서 서서 잰 값만 쓴다 (2026-11-18 측정 치우침 보정 뒤 켤 것)
    # 2026-11-18 2.0 -> 0: 접촉 순간 2.3도 기울어 들어갔다(예전 충전 잘 되던 도킹은 0.3~1.3도). 마지막 구간은 방향만.
    CD_BLIND_HEAD = 0.0
    CD_BLEND = 0.3            # 새 인식을 섞는 비율 (CD_AVG 꺼졌을 때)
    CD_AVG = False            # 본 것 전부를 가까울수록 크게 평균 (2026-11-16 모의에서 차이 없어 꺼 둠)
    CD_ANCHOR = 0.05          # 정렬 때 값에서 이보다 멀리 끌려가지 않는다 [m]
    CD_LAT_BAD = 0.06         # 틈 CD_LAT_BAD_GAP 안에서 축 좌우가 이보다 벌어지면 중단 [m]
    CD_LAT_BAD_GAP = 0.30
    CD_LAT_BAD_FAR = 0.25     # 그 밖에서는 이만큼
    CD_OVER = 0.03            # 처음 틈보다 이만큼 더 가면 중단 [m]
    CD_BACK_S = 40.0
    CD_SCAN_STALE_S = 0.5
    CD_STALL_S = 0.8          # 틈 CD_STALL_GAP 안에서 이만큼 안 나가면 닿은 것
    CD_STALL_GAP = 0.04
    CD_STALL_V = 0.005
    CD_CHARGE_A = 1.0         # BMS 전류(+ = 충전, 로봇 소비를 뺀 값)가 이보다 크면 충전 중 [A]
    # ⚠ 2026-11-18 배터리 93% 에서 도킹 2번이 '접촉했는데 전류 없음' 으로 실패했는데, 실제로는 충전기가 넣고 있었다:
    #   서 있을 때 로봇 소비가 -0.9~-1.9A 인데 접촉 뒤 -0.3~0.0A (충전기 약 1~1.6A, 거의 다 차서 약하게).
    #   서 있으면 이 값보다 높을 때 충전 중으로 본다.
    CD_CHARGE_STILL_A = -0.6
    # 2026-11-18 배터리 92~93% 에서 접촉 뒤 -0.66 / -1.13A 로 실패 처리(통과는 -0.3~-0.5A). 거의 찬 배터리는 충전기가 거의
    #   안 넣거나 끊는 게 정상 - 이 이상이면 접촉 위치 도달로 완료.
    CD_FULL_SOC = 90.0
    CD_CHARGE_FRESH_S = 2.5   # BMS 는 1초마다 읽는다. 이보다 오래된 값은 안 믿는다
    CD_PUSH_M = 0.02          # 접촉 자리에서 전류가 없으면 이만큼 더 밀어 본다 [m]
    CD_PUSH_V = 0.015
    CD_CHARGE_WAIT_S = 6.0    # 접촉 뒤 이 안에 전류가 없으면 실패
    CD_BLOCK_HW = 0.19        # 뒤 통로 반폭 [m] (차체 0.164 + 2.6cm)
    CD_GAP_CLEAR = 0.04       # 스테이션 앞면보다 이만큼 가까우면 다른 물체 [m]
    CD_BLOCK_PTS = 5
    CD_BLOCK_S = 15.0
    CD_LEAVE_M = 0.50         # 나올 때 곧게 앞으로 [m]
    CD_LEAVE_V = 0.08
    # ⚠ 2026-11-17 사용자: "충전스테이션 나올 때는 휘청거리는 게 계속 눈에 보임". 10:11 기록: 나오기 0.12m
    #   지점(0.07~0.08 m/s)에서 차체가 +0.18 rad/s 로 틀어졌고(후진으로 들어갔다 앞으로 나오는 방향 전환),
    #   방향 붙잡기(이득 1.5)가 넘쳐 ±0.15 rad/s · 2.5초 주기로 4초 넘게 흔들렸다. 방향 전환 뒤 첫
    #   CD_LEAVE_SLOW_M 은 CD_LEAVE_SLOW_V 로, 방향 붙잡기는 CD_LEAVE_K + 자이로 감쇠 CD_LEAVE_KD.
    CD_LEAVE_SLOW_M = 0.20
    CD_LEAVE_SLOW_V = 0.04
    CD_LEAVE_K = 1.0
    CD_LEAVE_KD = 0.6
    CD_LEAVE_S = 30.0
    CD_DOCKED_NEAR = 0.10     # 충전소 POI 도킹 자세에서 이 안이면 붙어 있는 것 [m]
    CD_DOCKED_GAP = 0.08      # 라이다로 본 스테이션 앞면이 뒤끝에서 이 안 [m]
    CD_DOCKED_LAT = 0.08      # 그리고 좌우 이 안
    CD_DOCKED_YAW = math.radians(15)
    CD_FRONT_CLEAR = 0.30     # 나올 때 차체 앞끝에서 이만큼 · 차체 폭 안 점을 본다 [m]

    # ⚠ 2026-11-18 인식 치우침 보정 (사용자: "스테이션 왼쪽에 치우쳐서 삐딱하게 도킹됨"). 사용자가 정확히 붙여 둔(충전되는)
    #   자세에서 IMU 로 방향을 붙잡고 0.05 m/s 로 1m 곧게 빼며 307번 잰 결과(logs/charger_calib.py): 라이다는 늘
    #   로봇이 8~15mm 오른쪽(-)에 있다고 보았고, 가까울수록 더 치우쳤다(측정 흔들림 ±0.7mm). 방향도 0.2m 안에서 +1.2도.
    #   뒤 라이다가 차체 오른쪽 뒤에 있어 V 홈을 비스듬히 보는 탓으로 본다. 틈 별 (좌우 [m], 방향 [도]) 치우침을 뺀다.
    CD_BIAS_GAP = (0.05, 0.15, 0.25, 0.35, 0.45, 0.55, 0.65, 0.75, 0.85, 0.95)
    CD_BIAS_LAT = (-0.0147, -0.0123, -0.0115, -0.0093, -0.0080, -0.0077, -0.0093, -0.0081, -0.0094, -0.0080)
    CD_BIAS_YAW_DEG = (1.22, 1.18, 0.28, -0.40, -0.16, -0.39, -0.33, -0.54, -0.22, -0.30)
    CD_BIAS_ON = True

    def _cdock_detect(self, m, las):
        """스테이션 인식 + 거리별 치우침 보정 (base 기준 dict 또는 None)."""
        c = detect_scan(m.ranges, m.angle_min, m.angle_increment, las)
        if c is None or not self.CD_BIAS_ON:
            return c
        import numpy as _np
        gap = -self.CHARGER_HALF_LEN - c["x"]
        c = dict(c)
        c["y"] -= float(_np.interp(gap, self.CD_BIAS_GAP, self.CD_BIAS_LAT))
        c["yaw"] = _wrap(c["yaw"] - math.radians(float(_np.interp(gap, self.CD_BIAS_GAP, self.CD_BIAS_YAW_DEG))))
        return c

    # ------------------------------------------------------------ 시작 · 끝
    def cdock_start(self, name, from_route=False):
        if self.estop:
            return False, "비상정지 중입니다"
        if self.control_mode != "auto":
            return False, "자동 모드에서만 도킹할 수 있습니다"
        if self._lift_moving():
            return False, "리프트가 움직이는 중입니다"
        if getattr(self, "job", {}).get("state") in self.JOB_BUSY:
            return False, "렉 작업 중입니다"
        if getattr(self, "route", {}).get("state") in self.ROUTE_BUSY and not from_route:
            # 2026-11-18 왕복이 충전소 끝점으로 부를 때는 거절하지 않는다 (처음에 이것에 걸려 왕복이 멈췄다)
            return False, "왕복 중입니다. 왕복을 멈춘 뒤 하세요"
        if self.cdock.get("state") in self.CD_BUSY:
            return False, "이미 충전소 도킹 중입니다"
        p = self.poi_find(name)
        if p is None:
            return False, "'%s' 가 없습니다" % name
        if p.get("kind") != "charger":
            return False, "'%s' 는 충전소 POI 가 아닙니다" % name
        # ⚠ 상태는 한 번에 정한다. 'moving' 으로 두었다 바꾸면 그 사이 도킹 타이머(다른
        #   스레드)가 끼어들어 아직 없는 이동 번호를 읽었다(2026-11-13).
        if self._cdock_visible_now():
            self.cdock = {"state": "aligning", "poi": p["name"], "started": time.time(),
                          "realign": 0, "t0": time.time(), "seen": [], "message": "충전소 인식 중"}
            self.get_logger().info("충전소 도킹 시작: %s (이미 보임 — 대기 자리 이동 생략)" % p["name"])
            return True, "충전소 도킹을 시작합니다 — 충전소가 보여 바로 맞춥니다"
        self.cdock = {"state": "starting", "poi": p["name"], "message": "",
                      "started": time.time(), "realign": 0}
        ok, msg = self._cdock_go(p)
        if not ok:
            self.cdock.update(state="failed", message=msg)
            return False, msg
        self.get_logger().info("충전소 도킹 시작: %s" % p["name"])
        return True, "충전소 도킹을 시작합니다 — 먼저 대기 자리로 갑니다"

    def _cdock_visible_now(self):
        """최신 스캔에 스테이션 V 가 CD_SEE_MIN ~ CD_SEE_MAX 틈으로 보이는가."""
        with self.lock:
            m = self.scan
        if m is None:
            return False
        las = self._poi_laser(m.header.frame_id) or (0.0, 0.0, 0.0)
        c = self._cdock_detect(m, las)
        if c is None:
            return False
        d = math.hypot(c["x"], c["y"]) - self.CHARGER_HALF_LEN
        return self.CD_SEE_MIN < d < self.CD_SEE_MAX

    def _cdock_go(self, p, pose=None):
        ax, ay, ayaw = pose or self.charger_approach(p)
        if pose is None and self.CD_SKIP_CREEP:
            # 기어 들어가기를 안 하면 nav2 를 끊은 자리(목표 약 6cm 앞)에 선다 - 그만큼 목표를 스테이션 쪽으로 당긴다
            ax -= self.CD_CREEP_SHORT * math.cos(ayaw)
            ay -= self.CD_CREEP_SHORT * math.sin(ayaw)
        # ⚠ 2026-11-18 사용자: 도킹 "시간이 너무 오래걸려". 대기 자리에서 정밀 도착(옆 40mm 맞추려 -67도 돌고
        #   다시 기어 들어감) + 2도까지 방향 맞추기로 약 15초. 도킹이 V 홈을 다시 재서 중간 지점 · 축을 잡으므로
        #   대기 자리는 대충이면 된다: 자리 맞추기 없이, 방향은 CD_APPROACH_YAW_OK 안에서 한 번만.
        ok, msg = self.nav_goto(ax, ay, ayaw, depart=True,
                                yaw_ok=self.CD_APPROACH_YAW_OK, turn_max=1)
        self._nav_precise = False
        if ok:
            self.cdock.update(state="moving", seq=self.nav_seq, target=(ax, ay, ayaw),
                              message="%s 대기 자리로 이동 중" % p["name"])
        return ok, msg

    def cdock_cancel(self, why="충전소 도킹을 멈췄습니다", stop=True):
        st = self.cdock.get("state")
        if st not in self.CD_BUSY:
            return False, "진행 중인 충전소 도킹이 없습니다"
        self.cdock.update(state="canceled", message=why)
        if st in ("backing", "maneuver"):
            self._pub_zero()
        elif stop and st == "moving":
            self.nav_cancel()
        self.get_logger().info("충전소 도킹 종료: %s" % why)
        return True, why

    def _cdock_fail(self, why):
        self._pub_zero()
        self.cdock.update(state="failed", message=why)
        self.get_logger().warn("충전소 도킹 실패: %s" % why)

    def cdock_status(self):
        hide = ("seq", "est", "est0", "seen", "scan_t", "scan_rx", "stall_t", "last_xy", "start",
                "scan_m", "block_t", "done_odom", "steps", "contact_t", "push_from", "trace_t",
                "scan_hdr", "acc")
        d = {k: v for k, v in self.cdock.items() if k not in hide}
        d["busy"] = d.get("state") in self.CD_BUSY
        return d

    def _cdock_charging(self):
        """충전 전류가 들어오나: True / False, BMS 값이 없거나 오래됐으면 None."""
        with self.lock:
            d, t = getattr(self, "bat", None), getattr(self, "bat_t", 0.0)
        if not d or not d.get("ok") or time.time() - t > self.CD_CHARGE_FRESH_S:
            return None
        try:
            thr = self.CD_CHARGE_STILL_A if self._cdock_still() else self.CD_CHARGE_A
            return float(d.get("current")) > thr
        except (TypeError, ValueError):
            return None

    # ------------------------------------------------------------ 나오기
    def _cdock_leave_needed(self):
        lv = getattr(self, "cleave", {})
        if lv.get("state") == "leaving":
            return True                       # 나오는 중에 목표가 바뀌면 새 목표로 이어 간다
        if self._cdock_docked_lidar():
            return True
        rm = self.tf_pose()
        if rm is None:
            return False
        for p in getattr(self, "pois", []):
            if p.get("kind") != "charger":
                continue
            if (math.hypot(rm["x"] - p["x"], rm["y"] - p["y"]) < self.CD_DOCKED_NEAR
                    and abs(_wrap(rm["yaw"] - p["yaw"])) < self.CD_DOCKED_YAW):
                return True
        cd = getattr(self, "cdock", {})
        ro = self.tf_pose("odom")
        if cd.get("state") == "done" and cd.get("done_odom") and ro is not None:
            d = cd["done_odom"]
            return math.hypot(ro["x"] - d[0], ro["y"] - d[1]) < self.CD_DOCKED_NEAR
        return False

    def _cdock_docked_lidar(self):
        """최신 스캔에서 스테이션 V 가 로봇 바로 뒤에 붙어 보이는가."""
        with self.lock:
            m = self.scan
        if m is None:
            return False
        las = self._poi_laser(m.header.frame_id) or (0.0, 0.0, 0.0)
        c = self._cdock_detect(m, las)
        if c is None:
            return False
        gap = -self.CHARGER_HALF_LEN - c["x"]
        return (-0.02 < gap < self.CD_DOCKED_GAP and abs(c["y"]) < self.CD_DOCKED_LAT
                and abs(_wrap(c["yaw"])) < self.CD_DOCKED_YAW)

    def _cdock_leave_start(self, g, seq, x, y, yaw, depart):
        lv = getattr(self, "cleave", {})
        if lv.get("state") == "leaving":
            lv["pending"] = (g, seq, x, y, yaw, depart)
            self.get_logger().info("충전소 나오는 중 — 목표만 바꿈 (%.2f, %.2f)" % (x, y))
            return
        ro = self.tf_pose("odom")
        self.cleave = {"state": "leaving", "pending": (g, seq, x, y, yaw, depart),
                       "start": (ro["x"], ro["y"]) if ro else None,
                       "yaw0": ro["yaw"] if ro else None, "t0": time.time(), "block_t": None}
        self.nav_msg = "충전소에서 나오는 중"
        self.get_logger().info("충전소에서 %.1fm 곧게 나간 뒤 (%.2f, %.2f) 로 출발"
                               % (self.CD_LEAVE_M, x, y))

    def cleave_cancel(self):
        lv = getattr(self, "cleave", {})
        if lv.get("state") != "leaving":
            return False
        lv["state"] = "canceled"
        self._pub_zero()
        self.get_logger().info("충전소 나오기 멈춤")
        return True

    def _cleave_tick(self):
        lv = self.cleave
        g, seq, x, y, yaw, depart = lv["pending"]
        if getattr(self, "nav_seq", None) != seq:
            # 나오는 사이 새 목표가 왔는데 여기로 안 왔다 = 다른 길로 끊겼다
            lv["state"] = "canceled"
            self._pub_zero()
            return
        if self.estop:
            lv["state"] = "canceled"
            self._pub_zero()
            return
        ro = self.tf_pose("odom")
        if ro is None or lv["start"] is None:
            lv["state"] = "failed"
            self._pub_zero()
            self.nav_state, self.nav_msg = "failed", "충전소에서 나오지 못했습니다 (위치 모름)"
            return
        gone = math.hypot(ro["x"] - lv["start"][0], ro["y"] - lv["start"][1])
        now = time.time()
        if gone >= self.CD_LEAVE_M - 0.005:
            lv["state"] = "done"
            self._pub_zero()
            self.get_logger().info("충전소에서 %.0fmm 나옴 — 목표로 출발 (경로가 옆·뒤면 서서 돎)" % (gone * 1000))
            self._dispatch_after_out(g, seq, x, y, yaw, depart)
            return
        if now - lv["t0"] > self.CD_LEAVE_S:
            lv["state"] = "failed"
            self._pub_zero()
            self.nav_state, self.nav_msg = "failed", "충전소에서 나오지 못했습니다 (시간 초과)"
            self.get_logger().warn(self.nav_msg)
            return
        if self._cdock_front_blocked() >= self.CD_BLOCK_PTS:
            lv["block_t"] = lv["block_t"] or now
            lv["t0"] += 1.0 / self.CD_HZ
            self.nav_msg = "충전소에서 나오는 중 — 앞에 장애물, 기다림"
            self._pub_zero()
            return
        lv["block_t"] = None
        self.nav_msg = "충전소에서 나오는 중"
        o = getattr(self, "odom", None)
        w_act = o.twist.twist.angular.z if o is not None else 0.0
        w = max(-self.CD_W_MAX, min(self.CD_W_MAX, self.CD_LEAVE_K * _wrap(lv["yaw0"] - ro["yaw"])
                                    - self.CD_LEAVE_KD * w_act))
        v = min(self.CD_LEAVE_V, 0.03 + gone * 0.5, max(0.02, (self.CD_LEAVE_M - gone) * 1.5))
        if gone < self.CD_LEAVE_SLOW_M:
            v = min(v, self.CD_LEAVE_SLOW_V)
        t = Twist()
        t.linear.x = float(v)
        t.angular.z = float(w)
        self.pub_twist_free.publish(t)

    def _cdock_front_blocked(self):
        """차체 앞끝에서 CD_FRONT_CLEAR 안, 차체 폭 안 라이다 점 수 (최신 스캔)."""
        with self.lock:
            m = self.scan
        if m is None:
            return 0
        import numpy as np
        las = self._poi_laser(m.header.frame_id) or (0.0, 0.0, 0.0)
        r = np.asarray(m.ranges, float)
        a = m.angle_min + np.arange(len(r)) * m.angle_increment + las[2]
        ok = np.isfinite(r)
        x = las[0] + r[ok] * np.cos(a[ok])
        y = las[1] + r[ok] * np.sin(a[ok])
        x0 = self.CHARGER_HALF_LEN
        return int(((x > x0) & (x < x0 + self.CD_FRONT_CLEAR)
                    & (np.abs(y) < self.CD_BLOCK_HW)).sum())

    # ------------------------------------------------------------ 주기
    def _cdock_tick(self):
        if getattr(self, "cleave", {}).get("state") == "leaving":
            self._cleave_tick()
        cd = self.cdock
        st = cd.get("state")
        if st not in self.CD_BUSY:
            return
        if self.estop or self.control_mode != "auto":
            self.cdock_cancel("비상정지 또는 수동 모드로 도킹을 멈췄습니다")
            return
        if st == "moving":
            if getattr(self, "nav_seq", None) != cd.get("seq"):
                self.cdock_cancel("다른 이동 명령이 들어와 도킹을 멈췄습니다", stop=False)
            elif self.nav_state == "done":
                cd.update(state="aligning", t0=time.time(), seen=[],
                          message="충전소 인식 중")
            elif (self.CD_POI_FIRST and getattr(self, "_turning", False) and self._cdock_still()
                  and self._cdock_poi_first()):
                pass
            elif self.nav_state == "active" and self._cdock_handoff_ready():
                cd.update(state="aligning", t0=time.time(), seen=[],
                          message="충전소 보임 — 넘겨받아 인식 중")
                self.get_logger().info("충전소 대기 자리 근처 — nav2 를 끊고 넘겨받음")
                self.nav_cancel()
            elif self.nav_state in ("failed", "canceled"):
                self.cdock.update(state="failed" if self.nav_state == "failed" else "canceled",
                                  message="대기 자리로 가지 못했습니다 (%s)" % self.nav_msg)
        elif st == "aligning":
            self._cdock_aligning()
        elif st == "maneuver":
            self._cdock_maneuver()
        elif st == "backing":
            self._cdock_backing()

    def _cdock_poi_first(self):
        """대기 자리에 닿아 방향 맞추기를 시작했다: POI 기준 중간 지점으로 한 번에 등 돌려 후진. 했으면 True."""
        cd = self.cdock
        tg = cd.get("target")
        rm, ro = self.tf_pose(), self.tf_pose("odom")
        p = self.poi_find(cd["poi"])
        if tg is None or rm is None or ro is None or p is None:
            return False
        if math.hypot(rm["x"] - tg[0], rm["y"] - tg[1]) > self.CD_HANDOFF_M:
            return False
        if abs(_wrap(rm["yaw"] - tg[2])) <= self.CD_HANDOFF_YAW:
            return False                     # 거의 맞다 -> 예전처럼 V 홈을 보고 넘겨받는다
        hl = self.CHARGER_HALF_LEN
        fx, fy = p["x"] - hl * math.cos(p["yaw"]), p["y"] - hl * math.sin(p["yaw"])
        # 지도 -> odom (지금 로봇 자세 두 개로 구한다)
        dyaw = _wrap(ro["yaw"] - rm["yaw"])
        c, s_ = math.cos(dyaw), math.sin(dyaw)
        dx, dy = fx - rm["x"], fy - rm["y"]
        est = (ro["x"] + c * dx - s_ * dy, ro["y"] + s_ * dx + c * dy, _wrap(p["yaw"] + dyaw))
        along = hl + self.CD_MID_GAP
        tx, ty = est[0] + along * math.cos(est[2]), est[1] + along * math.sin(est[2])
        rear = _wrap(math.atan2(ty - ro["y"], tx - ro["x"]) + math.pi)
        steps = [("turn", rear, self.CD_MID_TURN_TOL), ("back", (tx, ty))]
        if self.CD_AXIS_BACK:
            g0, lat0, _ = self._cdock_err((ro["x"], ro["y"], ro["yaw"]), est)
            h0 = self._cdock_axis_head(lat0, g0 - self.CD_MID_GAP)
            rear = _wrap(est[2] + h0)
            steps = [("turn", rear, self.CD_MID_TURN_TOL), ("axis", None)]
        cd.update(state="maneuver", mid_done=True, est_last=est, steps=steps, step_i=0,
                  t0=time.time(), message="충전소 축 %.0fcm 지점까지 곧게 후진" % (self.CD_MID_GAP * 100))
        self.nav_cancel()                    # 방향 맞추기를 끊는다
        self.get_logger().info("충전소: 대기 자리 도착 — POI 기준 틈 %.0fmm 지점으로 한 번에 등 돌리기 %+.1f도 후 후진"
                               % (self.CD_MID_GAP * 1000, math.degrees(_wrap(rear - ro["yaw"]))))
        return True

    def _cdock_axis_head(self, lat, rem):
        """중간 지점까지 축에 올라타는 방향 (축 기준) [rad] - 좌우를 CD_AXIS_L 거리 안에 지운다.
        (2026-11-18 원호 tan(방향/2)=좌우/남은거리 도 모의했지만 지연 있는 모의에서 축을 더 넘어가(평균 7mm vs 2mm) 안 씀)"""
        return max(-self.CD_AXIS_HEAD, min(self.CD_AXIS_HEAD, math.atan(lat / self.CD_AXIS_L)))

    def _cdock_handoff_ready(self):
        tg = self.cdock.get("target")
        rm = self.tf_pose()
        if tg is None or rm is None:
            return False
        if math.hypot(rm["x"] - tg[0], rm["y"] - tg[1]) > self.CD_HANDOFF_M \
                or abs(_wrap(rm["yaw"] - tg[2])) > self.CD_HANDOFF_YAW:
            return False
        return self._cdock_visible_now()

    def _cdock_still(self):
        o = getattr(self, "odom", None)
        if o is None:
            return True
        return (abs(o.twist.twist.linear.x) < self.CD_STILL_V
                and abs(o.twist.twist.angular.z) < self.CD_STILL_W)

    def _cdock_scan(self):
        """새 스캔에서 인식한 스테이션 (base 기준 dict) 과 그 스캔 시각. 새 스캔이 없으면 (None, None)."""
        with self.lock:
            m = self.scan
        if m is None:
            return None, None
        stamp = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        if stamp == self.cdock.get("scan_t"):
            return None, None
        self.cdock["scan_t"] = stamp
        self.cdock["scan_rx"] = time.time()
        las = self._poi_laser(m.header.frame_id) or (0.0, 0.0, 0.0)
        self.cdock["scan_m"] = (m, las)
        self.cdock["scan_hdr"] = m.header.stamp
        return self._cdock_detect(m, las), stamp

    def _cdock_blocked(self, gap):
        """로봇 뒤 통로에 스테이션보다 가까운 점 수 (마지막 스캔, base 기준)."""
        sm = self.cdock.get("scan_m")
        if sm is None:
            return 0
        m, las = sm
        import numpy as np
        r = np.asarray(m.ranges, float)
        a = m.angle_min + np.arange(len(r)) * m.angle_increment + las[2]
        ok = np.isfinite(r)
        x = las[0] + r[ok] * np.cos(a[ok])
        y = las[1] + r[ok] * np.sin(a[ok])
        far = -(self.CHARGER_HALF_LEN + gap - self.CD_GAP_CLEAR)
        return int(((x < -self.CHARGER_HALF_LEN) & (x > far) & (np.abs(y) < self.CD_BLOCK_HW)).sum())

    @staticmethod
    def _cdock_to_frame(robot, c):
        """base 기준 인식 c 를 robot=(x, y, yaw) 프레임(map/odom)의 (x, y, yaw) 로."""
        cs, sn = math.cos(robot[2]), math.sin(robot[2])
        return (robot[0] + cs * c["x"] - sn * c["y"], robot[1] + sn * c["x"] + cs * c["y"],
                _wrap(robot[2] + c["yaw"]))

    @staticmethod
    def _cdock_err(robot, station):
        """robot, station (같은 프레임). (틈, 축 좌우, 방향 오차).
        틈 = 로봇 뒤끝에서 앞면까지 [m], 좌우 = 축 왼쪽(+) [m], 방향 = 로봇 - 스테이션 바깥 방향."""
        ux, uy = math.cos(station[2]), math.sin(station[2])
        dx, dy = robot[0] - station[0], robot[1] - station[1]
        along = dx * ux + dy * uy
        lat = -dx * uy + dy * ux
        return along - ChargerDockMixin.CHARGER_HALF_LEN, lat, _wrap(robot[2] - station[2])

    def _cdock_aligning(self):
        cd = self.cdock
        c, _ = self._cdock_scan()
        if not self._cdock_still():
            cd["seen"] = []                    # 넘겨받은 직후 아직 서는 중
            cd["t0"] = time.time()
            self._pub_zero()
            return
        if c is not None:
            # ⚠ 2026-11-18 POI 에서 926mm 떨어진 다른 V 모양을 충전소로 잡아 거기 맞추러 나갔다 돌아왔다(51초).
            #   POI 에서 CD_POI_GATE 넘게 떨어진 인식은 그 스캔만 버린다 (진짜는 POI 와 8~47mm).
            rm0 = self.tf_pose()
            p0 = self.poi_find(cd["poi"]) if rm0 is not None else None
            if p0 is not None:
                hl = self.CHARGER_HALF_LEN
                sm0 = self._cdock_to_frame((rm0["x"], rm0["y"], rm0["yaw"]), c)
                off0 = math.hypot(sm0[0] - (p0["x"] - hl * math.cos(p0["yaw"])),
                                  sm0[1] - (p0["y"] - hl * math.sin(p0["yaw"])))
                if off0 > self.CD_POI_GATE:
                    cd["gated"] = cd.get("gated", 0) + 1
                    if cd["gated"] == 1:
                        self.get_logger().info("충전소 인식: POI 와 %.0fmm 떨어진 V 모양은 다른 물체로 보고 버림" % (off0 * 1000))
                    c = None
        if c is not None:
            cd["seen"].append(c)
        if time.time() - cd["t0"] > self.CD_ALIGN_S:
            self._cdock_fail("충전소를 찾지 못했습니다 (%d번 인식%s)" % (
                len(cd["seen"]), ", POI 와 먼 V 모양 %d번 버림" % cd["gated"] if cd.get("gated") else ""))
            return
        if len(cd["seen"]) < self.CD_ALIGN_N:
            return
        rm, ro = self.tf_pose(), self.tf_pose("odom")
        if rm is None or ro is None:
            self._cdock_fail("로봇 위치를 모릅니다")
            return
        S = cd["seen"]
        mid = sorted(S, key=lambda q: q["x"])[len(S) // 2]
        avg = {"x": sum(q["x"] for q in S) / len(S), "y": sum(q["y"] for q in S) / len(S),
               "yaw": math.atan2(sum(math.sin(q["yaw"]) for q in S),
                                 sum(math.cos(q["yaw"]) for q in S))}
        if math.hypot(mid["x"] - avg["x"], mid["y"] - avg["y"]) > 0.02:
            avg = mid
        sm = self._cdock_to_frame((rm["x"], rm["y"], rm["yaw"]), avg)
        p = self.poi_find(cd["poi"])
        if p is None:
            self._cdock_fail("충전소 POI 가 없어졌습니다")
            return
        hl = self.CHARGER_HALF_LEN
        pf = (p["x"] - hl * math.cos(p["yaw"]), p["y"] - hl * math.sin(p["yaw"]))
        off = math.hypot(sm[0] - pf[0], sm[1] - pf[1])
        if off > self.CD_POI_NEAR or abs(_wrap(sm[2] - p["yaw"])) > self.CD_POI_YAW:
            self._cdock_fail("보이는 스테이션이 POI 와 %.0fmm · %.0f도 달라 들어가지 않습니다"
                             % (off * 1000, math.degrees(_wrap(sm[2] - p["yaw"]))))
            return
        gap, lat, dyaw = self._cdock_err((rm["x"], rm["y"], rm["yaw"]), sm)
        self.get_logger().info("충전소 인식: 틈 %.0fmm 축 좌우 %+.0fmm 방향 %+.1f도 (POI 와 %.0fmm)"
                               % (gap * 1000, lat * 1000, math.degrees(dyaw), off * 1000))
        est = self._cdock_to_frame((ro["x"], ro["y"], ro["yaw"]), avg)
        cd["est_last"] = est
        too_close = gap < self.CD_MIN_GAP - 0.05 and not (self.CD_TWO_LEG and cd.get("mid_done"))
        if abs(lat) > self.CD_LAT_OK or abs(dyaw) > self.CD_YAW_OK or too_close:
            if cd["realign"] >= self.CD_REALIGN_MAX:
                self._cdock_fail("제자리에서 %d번 맞춰도 축에서 좌우 %.0fmm · 방향 %.1f도 어긋납니다"
                                 % (cd["realign"], lat * 1000, math.degrees(dyaw)))
                return
            cd["realign"] += 1
            self._cdock_plan((ro["x"], ro["y"], ro["yaw"]), est, gap, lat)
            return
        if self.CD_TWO_LEG:
            if gap > self.CD_MID_GAP + self.CD_MID_TOL and not cd.get("mid_done"):
                ux, uy = math.cos(est[2]), math.sin(est[2])
                along = self.CHARGER_HALF_LEN + self.CD_MID_GAP
                tx, ty = est[0] + along * ux, est[1] + along * uy
                rear = _wrap(math.atan2(ty - ro["y"], tx - ro["x"]) + math.pi)   # 뒤로 가므로 등을 돌린다
                cd["mid_done"] = True
                cd.update(state="maneuver", steps=[("turn", rear, self.CD_MID_TURN_TOL), ("back", (tx, ty))], step_i=0,
                          t0=time.time(), message="충전소 축 %.0fcm 지점까지 곧게 후진" % (self.CD_MID_GAP * 100))
                self.get_logger().info("충전소: 축 위 틈 %.0fmm 지점까지 곧게 (등 돌리기 %+.1f도)"
                                       % (self.CD_MID_GAP * 1000, math.degrees(_wrap(rear - ro["yaw"]))))
                return
            if abs(dyaw) > self.CD_FINAL_YAW and cd.get("pivots", 0) < self.CD_PIVOTS_MAX:
                cd["pivots"] = cd.get("pivots", 0) + 1
                cd.update(state="maneuver", steps=[("bturn" if self.CD_PIVOT_BACK else "turn", est[2])], step_i=0, t0=time.time(),
                          message="충전소 축 방향으로 맞추는 중")
                self.get_logger().info("충전소: 틈 %.0fmm 에서 축 방향으로 %+.1f도 맞춘 뒤 곧게 (좌우 %+.0fmm)"
                                       % (gap * 1000, -math.degrees(dyaw), lat * 1000))
                return
        cd.update(state="backing", est=est, est0=est, gap0=gap + abs(lat), t0=time.time(), acc=[0.0] * 5,
                  start=(ro["x"], ro["y"]), stall_t=None, last_xy=(ro["x"], ro["y"]),
                  message="후진 진입 중")

    def _cdock_plan(self, robot, est, gap, lat):
        """odom 기준 제자리 맞추기 계획: [(turn, yaw) | (drive, (x, y))]."""
        ux, uy = math.cos(est[2]), math.sin(est[2])
        steps = []
        if abs(lat) > self.CD_LAT_OK or gap < self.CD_MIN_GAP - 0.05:
            along = self.CHARGER_HALF_LEN + max(gap, self.CD_MIN_GAP) + self.CD_SIDE_M
            tx, ty = est[0] + along * ux, est[1] + along * uy
            steps += [("turn", math.atan2(ty - robot[1], tx - robot[0])), ("drive", (tx, ty))]
        steps.append(("turn", est[2]))
        self.cdock.update(state="maneuver", steps=steps, step_i=0, t0=time.time(),
                          message="제자리에서 충전소 축에 맞추는 중 (%d회)" % self.cdock["realign"])
        self.get_logger().info("충전소 맞추기 %d회: 틈 %.0fmm 좌우 %+.0fmm — %s"
                               % (self.cdock["realign"], gap * 1000, lat * 1000,
                                  " → ".join("돌기" if k == "turn" else "곧게" for k, _ in steps)))

    def _cdock_maneuver(self):
        cd = self.cdock
        if time.time() - cd["t0"] > self.CD_MAN_S:
            return self._cdock_fail("제자리 맞추기 시간이 초과되었습니다")
        ro = self.tf_pose("odom")
        if ro is None:
            return self._cdock_fail("로봇 위치를 모릅니다")
        if cd["step_i"] >= len(cd["steps"]):
            self._pub_zero()
            cd.update(state="aligning", t0=time.time(), seen=[], message="충전소 다시 인식 중")
            return
        step = cd["steps"][cd["step_i"]]
        kind, tgt = step[0], step[1]
        tol = step[2] if len(step) > 2 else self.CD_TURN_TOL
        t = Twist()
        if kind in ("turn", "bturn"):
            e = _wrap(tgt - ro["yaw"])
            if abs(e) < tol:
                cd["step_i"] += 1
                cd.pop("turn_t0", None)
                self._pub_zero()
                return
            # 2026-11-18 큰 회전은 CD_TURN_W_BIG 까지, 처음엔 CD_TURN_ACC 로 올린다
            t0 = cd.setdefault("turn_t0", time.time())
            cap = min(self.CD_TURN_W_BIG, 0.08 + (time.time() - t0) * self.CD_TURN_ACC)
            w = max(-cap, min(cap, 1.5 * e))
            if abs(w) < 0.06:
                w = math.copysign(0.06, e)
            t.angular.z = float(w)
            if kind == "bturn":
                # 한쪽 바퀴는 세우고 다른 쪽만 뒤로 - 두 바퀴 모두 마지막 방향이 '뒤' 라 이어지는 후진에 기어 틈이 없다
                t.linear.x = -abs(float(w)) * self.CD_PIVOT_B
        elif kind == "back":
            # 등 뒤 점까지 곧게 후진 - 등을 그 점에 계속 겨눈다(가까우면 방향만 붙잡기)
            dx, dy = tgt[0] - ro["x"], tgt[1] - ro["y"]
            c, s_ = math.cos(ro["yaw"]), math.sin(ro["yaw"])
            behind = -(c * dx + s_ * dy)
            if behind < 0.005:
                cd["step_i"] += 1
                cd.pop("back_yaw", None)
                cd.pop("back_t0", None)
                self._pub_zero()
                return
            est = cd.get("est_last")
            gap = self._cdock_err((ro["x"], ro["y"], ro["yaw"]), est)[0] if est else 1.0
            if self._cdock_blocked(gap) >= self.CD_BLOCK_PTS:
                cd["t0"] += 1.0 / self.CD_HZ
                cd["message"] = "뒤에 장애물 — 치울 때까지 기다림"
                self._pub_zero()
                return
            if "back_yaw" not in cd:
                cd["back_yaw"] = ro["yaw"]
                cd["back_t0"] = time.time()
            want = (_wrap(math.atan2(dy, dx) + math.pi) if behind > 0.08 else cd["back_yaw"])
            o = getattr(self, "odom", None)
            w_act = o.twist.twist.angular.z if o is not None else 0.0
            t.linear.x = -float(min(self.CD_MID_V, max(0.02, behind * 1.2), 0.02 + (time.time() - cd["back_t0"]) * self.CD_DOCK_ACC))
            t.angular.z = float(max(-self.CD_W_MAX, min(self.CD_W_MAX, self.CD_LEAVE_K * _wrap(want - ro["yaw"])
                                                       - self.CD_LEAVE_KD * w_act)))
        elif kind == "axis":
            # 2026-11-18 중간 지점(틈 CD_MID_GAP)까지 축에 올라타며 후진 - 등 뒤 V 홈을 계속 재서 est_last 를 고친다
            robot = (ro["x"], ro["y"], ro["yaw"])
            est = cd["est_last"]
            e0 = cd.setdefault("axis_e0", est)
            c, _ = self._cdock_scan()
            if c is not None:
                rs = robot
                tfn = getattr(self, "_tf_nowait", None)
                hdr = cd.get("scan_hdr")
                if tfn is not None and hdr is not None:
                    p_ = tfn("odom", "base_footprint", hdr)
                    if p_ is not None:
                        rs = (p_["x"], p_["y"], p_["yaw"])
                sc = self._cdock_to_frame(rs, c)
                if math.hypot(sc[0] - e0[0], sc[1] - e0[1]) < 3 * self.CD_ANCHOR:
                    k = self.CD_AXIS_BLEND
                    est = (est[0] + k * (sc[0] - est[0]), est[1] + k * (sc[1] - est[1]),
                           _wrap(est[2] + k * _wrap(sc[2] - est[2])))
                    cd["est_last"] = est
                    cd["axis_seen"] = cd.get("axis_seen", 0) + 1
            gap, lat, dyaw = self._cdock_err(robot, est)
            rem = gap - self.CD_MID_GAP
            if rem < 0.005:
                cd["step_i"] += 1
                self.get_logger().info("충전소: 축 따라 중간 지점 도착 — 좌우 %+.0fmm 방향 %+.1f도 (라이다 %d번 반영)"
                                       % (lat * 1000, math.degrees(dyaw), cd.get("axis_seen", 0)))
                for k_ in ("axis_e0", "axis_t0", "axis_seen"):
                    cd.pop(k_, None)
                self._pub_zero()
                return
            if self._cdock_blocked(gap) >= self.CD_BLOCK_PTS:
                cd["t0"] += 1.0 / self.CD_HZ
                cd["message"] = "뒤에 장애물 — 치울 때까지 기다림"
                self._pub_zero()
                return
            t0 = cd.setdefault("axis_t0", time.time())
            head = self._cdock_axis_head(lat, rem)
            o = getattr(self, "odom", None)
            w_act = o.twist.twist.angular.z if o is not None else 0.0
            t.linear.x = -float(min(self.CD_MID_V, max(0.02, rem * 1.2), 0.02 + (time.time() - t0) * self.CD_DOCK_ACC))
            t.angular.z = float(max(-self.CD_W_MAX, min(self.CD_W_MAX, self.CD_K_YAW * (head - dyaw)
                                                       - self.CD_LEAVE_KD * w_act)))
        else:
            dx, dy = tgt[0] - ro["x"], tgt[1] - ro["y"]
            c, s_ = math.cos(ro["yaw"]), math.sin(ro["yaw"])
            ahead = c * dx + s_ * dy
            if ahead < 0.005:
                cd["step_i"] += 1
                self._pub_zero()
                return
            if self._cdock_front_blocked() >= self.CD_BLOCK_PTS:
                cd["t0"] += 1.0 / self.CD_HZ
                cd["message"] = "앞에 장애물 — 치울 때까지 기다림"
                self._pub_zero()
                return
            side = -s_ * dx + c * dy
            t.linear.x = float(min(self.CD_MOVE_V, max(0.02, ahead * 1.2)))
            t.angular.z = float(max(-self.CD_W_MAX, min(self.CD_W_MAX,
                                                       2.0 * math.atan2(side, max(ahead, 0.05)))))
        self.pub_twist_free.publish(t)

    def _cdock_backing(self):
        cd = self.cdock
        ro = self.tf_pose("odom")
        if ro is None:
            self._cdock_fail("로봇 위치를 모릅니다")
            return
        robot = (ro["x"], ro["y"], ro["yaw"])
        gap, lat, dyaw = self._cdock_err(robot, cd["est"])
        c, _ = self._cdock_scan()
        raw_lat = None
        if c is not None:
            # ⚠ 2026-11-16 S자: 스캔은 0.1~0.15초 전 자세로 찍혔다. 지금 자세로 옮기면 도는
            #   동안(0.25 rad/s) 1~2도 · 0.8m 앞에서 좌우 1~3cm 가 틀어져, 라이다 좌우가
            #   스캔마다 ±10mm 흔들렸고 로봇이 그 잡음을 따라 꺾었다. 찍은 순간 자세로 옮긴다.
            rs = robot
            tfn = getattr(self, "_tf_nowait", None)
            hdr = cd.get("scan_hdr")
            if tfn is not None and hdr is not None:
                p_ = tfn("odom", "base_footprint", hdr)
                if p_ is not None:
                    rs = (p_["x"], p_["y"], p_["yaw"])
            s_c = self._cdock_to_frame(rs, c)
            raw_lat = self._cdock_err(robot, s_c)[1]
        # ⚠ 2026-11-18 두 단계 마지막 곧게에서는 중간 지점에서 서서 잰 값을 고정한다. 달리며 잰 좌우가 거리마다 바뀌었다
        #   (0.45m -14~-17mm, 0.25~0.36m -2~-5mm, 0.15~0.23m -12~-17mm) - 0.3m 근처 값에 속아 보정을 늦췄다가 -10mm 로 접촉.
        frozen = self.CD_TWO_LEG and self.CD_FREEZE_AFTER_MID and cd.get("mid_done")
        if c is not None and not frozen and gap > max(self.CD_BLIND_GAP, self.CD_FREEZE_GAP):
            s = s_c
            e0 = cd["est0"]
            if math.hypot(s[0] - e0[0], s[1] - e0[1]) < 3 * self.CD_ANCHOR:
                if self.CD_AVG:
                    # 스테이션은 odom 에서 멈춰 있다 — 지금까지 본 것을 가까울수록 크게 평균
                    wgt = 1.0 / max(gap, 0.15) ** 2
                    acc = cd.setdefault("acc", [0.0] * 5)
                    if acc[0] == 0.0:
                        w0 = self.CD_ALIGN_N / max(cd.get("gap0", gap), 0.15) ** 2
                        acc[:] = [w0, w0 * e0[0], w0 * e0[1],
                                  w0 * math.sin(e0[2]), w0 * math.cos(e0[2])]
                    acc[0] += wgt
                    acc[1] += wgt * s[0]
                    acc[2] += wgt * s[1]
                    acc[3] += wgt * math.sin(s[2])
                    acc[4] += wgt * math.cos(s[2])
                    nx, ny = acc[1] / acc[0], acc[2] / acc[0]
                    nyaw = math.atan2(acc[3], acc[4])
                else:
                    k = self.CD_BLEND
                    nx = cd["est"][0] + k * (s[0] - cd["est"][0])
                    ny = cd["est"][1] + k * (s[1] - cd["est"][1])
                    nyaw = _wrap(cd["est"][2] + k * _wrap(s[2] - cd["est"][2]))
                d = math.hypot(nx - e0[0], ny - e0[1])
                if d > self.CD_ANCHOR:
                    nx = e0[0] + (nx - e0[0]) * self.CD_ANCHOR / d
                    ny = e0[1] + (ny - e0[1]) * self.CD_ANCHOR / d
                if self.CD_TWO_LEG and self.CD_MID_YAW_HOLD and cd.get("mid_done"):
                    nyaw = cd["est"][2]
                cd["est"] = (nx, ny, nyaw)
                gap, lat, dyaw = self._cdock_err(robot, cd["est"])
        gone = math.hypot(robot[0] - cd["start"][0], robot[1] - cd["start"][1])
        cd.update(gap=round(gap, 4), lat=round(lat, 4), yaw_err=round(math.degrees(dyaw), 2),
                  gone=round(gone, 3),
                  message="후진 진입 중 — 남은 %.0fmm, 좌우 %+.0fmm" % (gap * 1000, lat * 1000))
        now = time.time()
        chg = self._cdock_charging()
        if chg:
            return self._cdock_done("충전 전류 확인 (+%.1fA)" % float(self.bat.get("current", 0)))
        if cd.get("contact_t") is not None:
            return self._cdock_contact(robot, gone, dyaw, chg, now)
        if gap <= self.CD_STOP_GAP:
            return self._cdock_touch(gone, chg, now, "접촉 위치 도달")
        if gone > cd["gap0"] + self.CD_OVER:
            return self._cdock_fail("예상 거리를 %.0fmm 넘겨 멈췄습니다"
                                    % ((gone - cd["gap0"]) * 1000))
        if now - cd["t0"] > self.CD_BACK_S:
            return self._cdock_fail("도킹 시간이 초과되었습니다")
        if abs(lat) > (self.CD_LAT_BAD if gap < self.CD_LAT_BAD_GAP else self.CD_LAT_BAD_FAR):
            return self._cdock_fail("진입 중 축에서 좌우로 %.0fmm 벌어져 멈췄습니다" % (lat * 1000))
        if gap > self.CD_BLIND_GAP and now - cd.get("scan_rx", now) > self.CD_SCAN_STALE_S:
            return self._cdock_fail("라이다 스캔이 끊겨 멈췄습니다")
        # 바퀴가 막혀 안 나가면 (마지막 몇 cm) 닿은 것
        moved = math.hypot(robot[0] - cd["last_xy"][0], robot[1] - cd["last_xy"][1])
        cd["last_xy"] = (robot[0], robot[1])
        if gap < self.CD_STALL_GAP and moved < self.CD_STALL_V / self.CD_HZ:
            cd["stall_t"] = cd["stall_t"] or now
            if now - cd["stall_t"] > self.CD_STALL_S:
                return self._cdock_touch(gone, chg, now, "바퀴가 막힘 = 접촉")
        else:
            cd["stall_t"] = None
        if gap > self.CD_GAP_CLEAR and self._cdock_blocked(gap) >= self.CD_BLOCK_PTS:
            cd["block_t"] = cd.get("block_t") or now
            if now - cd["block_t"] > self.CD_BLOCK_S:
                return self._cdock_fail("뒤에 장애물이 %.0f초 넘게 있어 멈췄습니다" % self.CD_BLOCK_S)
            cd["message"] = "뒤에 장애물 — 치울 때까지 기다림 (남은 %.0fmm)" % (gap * 1000)
            cd["t0"] += 1.0 / self.CD_HZ          # 기다리는 동안은 시간 한도에 안 센다
            self._pub_zero()
            return
        cd["block_t"] = None
        if self.CD_TWO_LEG and cd.get("mid_done"):
            # 두 단계 마지막 곧게: 한 가지 속도, 접촉 직전만 CD_V_SLOW 로 서서히, 출발은 CD_DOCK_ACC 로 서서히
            v = self.CD_DOCK_V
            if gap < self.CD_DOCK_END_GAP:
                v = self.CD_V_SLOW + (self.CD_DOCK_V - self.CD_V_SLOW) * max(0.0, gap) / self.CD_DOCK_END_GAP
            v = min(v, 0.02 + (time.time() - cd["t0"]) * self.CD_DOCK_ACC)
        else:
            v = (self.CD_V_SLOW if gap < self.CD_SLOW_GAP else
                 self.CD_V_FAST if abs(lat) < 0.04 and abs(dyaw) < math.radians(6) else self.CD_V_TURN)
        # 뒤로 가며 축 좌우를 지운다: 좌우 e 면 방향을 +atan(K e) 로 (후진이라 부호 그대로)
        if self.CD_TWO_LEG and cd.get("mid_done"):
            # 두 단계의 마지막 곧게: 도킹 끝 중심점을 겨누고(좌우 lat 를 남은 틈 안에 지움) 자이로로 흔들림을 누른다.
            #   라이다에 안 보이는 마지막 CD_BLIND_GAP 도 바퀴·IMU 로 계속 맞추되 CD_BLIND_HEAD 까지만 기운다.
            cap = self.CD_BLIND_HEAD if gap < self.CD_BLIND_GAP else self.CD_FINAL_HEAD
            head = max(-cap, min(cap, math.atan(lat / max(gap, 0.06))))
            err = head - dyaw
            o = getattr(self, "odom", None)
            w_act = o.twist.twist.angular.z if o is not None else 0.0
            w = max(-self.CD_W_MAX, min(self.CD_W_MAX, self.CD_K_YAW * err - self.CD_LEAVE_KD * w_act))
        else:
            look = max(self.CD_LOOK_MIN, min(self.CD_LOOK_MAX, self.CD_LOOK_K * gap))
            head = 0.0 if gap < self.CD_BLIND_GAP else max(
                -self.CD_HEAD_MAX, min(self.CD_HEAD_MAX, math.atan(lat / look)))
            err = head - dyaw
            w = max(-self.CD_W_MAX, min(self.CD_W_MAX, self.CD_K_YAW * err))
        if gap > self.CD_BLIND_GAP:
            r = (self.CD_PIVOT_YAW - abs(err)) / (self.CD_PIVOT_YAW - self.CD_PIVOT_YAW0)
            v *= max(0.0, min(1.0, r))
            if v == 0.0 and abs(w) < 0.06:
                w = math.copysign(0.06, err)     # 제자리 회전은 너무 느리면 안 돈다
        t = Twist()
        t.linear.x = -float(v)
        t.angular.z = float(w)
        self.pub_twist_free.publish(t)
        # 후진 기록 (2026-11-13, 사용자: "뒤로들어갈때 s자로 들어가는건 왜그런거야?" — 시뮬레이션
        # (지연 0.25초 · 스캔 지연 0.15초)으로는 재현이 안 돼 실제 값을 남긴다)
        if now - cd.get("trace_t", 0.0) >= 0.25:
            cd["trace_t"] = now
            o = getattr(self, "odom", None)
            w_act = o.twist.twist.angular.z if o is not None else float("nan")
            self.get_logger().info(
                "  충전소 후진 %4.1fs 틈 %4.0f 좌우 %+4.0f 방향 %+5.1f 목표방향 %+5.1f | v %.2f w명령 %+.3f w실제 %+.3f | 라이다 좌우 %s"
                % (now - cd["t0"], gap * 1000, lat * 1000, math.degrees(dyaw), math.degrees(head),
                   v, w, w_act, "-" if raw_lat is None else "%+.0f" % (raw_lat * 1000)))

    def _cdock_touch(self, gone, chg, now, why):
        """접촉 자리. BMS 를 모르면 끝, 알면 전류를 기다리며 조금 더 민다."""
        if chg is None:
            return self._cdock_done(why + ", 배터리 값 없음")
        cd = self.cdock
        cd.update(contact_t=now, push_from=gone, contact_why=why, stall_t=None,
                  message="접촉 — 충전 전류 기다리는 중")
        self.get_logger().info("충전소 %s — 충전 전류 기다림" % why)
        self._pub_zero()

    def _cdock_contact(self, robot, gone, dyaw, chg, now):
        cd = self.cdock
        if now - cd["contact_t"] > self.CD_CHARGE_WAIT_S:
            b = self.bat or {}
            cur, soc = b.get("current"), b.get("soc")
            try:
                full = soc is not None and float(soc) >= self.CD_FULL_SOC
            except (TypeError, ValueError):
                full = False
            if full:
                return self._cdock_done("접촉 (배터리 %s%% — 거의 차서 충전 전류 약함 %s A)" % (soc, cur))
            return self._cdock_fail("접촉했는데 %.0f초 동안 충전 전류가 안 들어옵니다 (지금 %s A, %s%%)"
                                    % (self.CD_CHARGE_WAIT_S, cur, soc))
        moved = math.hypot(robot[0] - cd["last_xy"][0], robot[1] - cd["last_xy"][1])
        cd["last_xy"] = (robot[0], robot[1])
        pushing = gone - cd["push_from"] < self.CD_PUSH_M
        if pushing and moved < self.CD_STALL_V / self.CD_HZ and now - cd["contact_t"] > self.CD_STALL_S:
            pushing = False                     # 막혔다 — 더 밀지 않고 기다린다
        if not pushing:
            self._pub_zero()
            return
        t = Twist()
        t.linear.x = -float(self.CD_PUSH_V)
        t.angular.z = float(max(-self.CD_W_MAX, min(self.CD_W_MAX, -self.CD_K_YAW * dyaw)))
        self.pub_twist_free.publish(t)

    def _cdock_done(self, why):
        self._pub_zero()
        cd = self.cdock
        ro = self.tf_pose("odom")
        cd["done_odom"] = (ro["x"], ro["y"]) if ro else None
        cd.update(state="done", message="충전소 도킹 완료 (%s) — 남은 틈 %.0fmm, 좌우 %+.0fmm"
                  % (why, cd.get("gap", 0) * 1000, cd.get("lat", 0) * 1000))
        self.get_logger().info("충전소 도킹 완료: %s, 틈 %.0fmm 좌우 %+.0fmm 방향 %+.1f도 (%.1f초)"
                               % (why, cd.get("gap", 0) * 1000, cd.get("lat", 0) * 1000,
                                  cd.get("yaw_err", 0.0), time.time() - cd["t0"]))
