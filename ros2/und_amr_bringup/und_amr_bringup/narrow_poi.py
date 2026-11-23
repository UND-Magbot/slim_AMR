"""좁은 곳 POI - 축을 따라 곧게 들어가고, 곧게 후진해서 나온다 (2026-11-16).

사용자: "우리가 주행하는 곳이 대부분 좁아서 좁은 환경에서 움직일 수 있어야 함",
"중앙으로 들어가는 게 아니고 한쪽으로 치우쳐서 들어감", 들어가는 방식 1번(앞으로).

왜 nav2 에만 맡기지 않나
  - 경로 계획은 라이다(높이 20cm)만 본다. 통로를 좁히는 물건이 라이다 높이 밖에 있으면
    (test2: 라이다로는 왼쪽이 30~40cm 비어 보였지만 실제로는 양쪽 5cm) 비어 보이는 쪽으로
    붙어 들어간다. 사람이 찍은 POI 축이 진짜 가운데다.
  - 좁은 통로 안에서는 제자리 회전을 못 한다(test2: 중심~장애물 0.30m, 필요 약 0.50m).

동작
  들어가기: POI 축 좌우 NP_SIDE_M 안에 지도 장애물이 있으면 '좁은 POI'.
    nav2 로 POI 앞 NP_PRE_M (축 위, POI 방향) 까지 -> 거기서 방향까지 맞춘 뒤
    축을 따라 곧게 NP_V 로 들어간다. 앞(라이다 + 카메라)이 막히면 서서 기다린다.
  나오기: 좁은 POI 에 서 있는데 다음 목표로 돌 자리가 없으면, 축을 따라 곧게 후진해
    앞 자리까지 나온 뒤 평소처럼 출발한다.
"""
import math

from geometry_msgs.msg import Twist

try:
    import numpy as np
except ImportError:                                          # pragma: no cover
    np = None


def _wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class NarrowPoiMixin:
    NP_SIDE_M = 0.40          # POI 축 좌우 이 안(차체 중심 기준)에 장애물이 있으면 좁은 곳 [m]
    NP_PRE_M = 1.00           # 축을 따라 곧게 갈 거리 = 앞 자리에서 POI 까지 [m]
    NP_PRE_MIN = 0.60         # 앞 자리가 막혔으면 이만큼까지 줄여 본다 [m]
    NP_V = 0.10               # 곧게 들어가기 · 나오기 속도 [m/s]
    NP_V_END = 0.04           # 마지막 NP_SLOW_M 구간 [m/s]
    NP_SLOW_M = 0.20
    # 2026-11-16 0.50 / 1.0 로 바꿔 봤다가 되돌림: 반응 지연 모의에서 0.30 / 1.5 도 S자 없이
    #   0.6mm 로 맞았고 0.50 / 1.0 은 끝에 3.6mm 남았다. 실기 S자는 출발 캐스터 차임으로 본다(아래).
    NP_LOOK = 0.30            # 좌우를 이 거리 앞에서 지운다 [m]
    NP_K_YAW = 1.5
    # 출발 첫 NP_FLIP_M 은 NP_FLIP_V 로 기어가며 방향만 붙잡는다 — 앞 자리에서 돌아선 직후
    #   틀어진 캐스터가 풀리며 차체를 민다(렉 진입 JOB_PREALIGN 과 같은 이유).
    NP_FLIP_M = 0.10
    NP_FLIP_V = 0.04
    NP_W_MAX = 0.25
    NP_LAG_S = 0.35           # 지령을 끊어도 더 가는 시간 (멈춤 미리 끊기) [s]
    NP_FRONT_M = 0.20         # 차체 앞(뒤) 이 안에 점이 있으면 막힘 [m]
    NP_WAIT_S = 15.0
    NP_MAX_S = 40.0
    NP_HZ = 20.0
    # 반대로 들어가 안에서 돌 계획일 때, 앞 자리에서 미리 보는 회전 여유 [m]. 도는 동안 회전
    #   중심이 1~2cm 밀린다(2026-11-16 폭 95cm 통로: 3cm 여유로 '돌 수 있음' 이었다가 53도에서 막힘).
    #   2026-11-16 0.05 -> 0.03: test3(통로 1.04m, 가운데에서 가장 가까운 점 0.510m)는 3cm 로 0점,
    #   5cm 로 73점 — 돌 수 있는 곳을 못 돈다고 봤다. 도는 중 검사(SPIN_PAD)와 같게.
    #   2026-11-16 0.03 -> 0.01: 벽이 중심에서 0.485m(차체 각진 모서리 모델 0.467m -> 1.8cm)인 곳에서
    #   38도 돌다 섰는데, 수동으로 돌리면 충분히 돈다(사용자). 실제 모서리가 모델보다 안쪽(깎인 모서리)
    #   이다. 좁은 POI 안 회전(입구 판단 + 도는 중 검사)만 1cm. 일반 회전은 SPIN_PAD 3cm 그대로.
    NP_TURN_PAD = 0.01
    NP_PRE_PAD = 0.0          # 입구 앞 사전 판단 여유 [m] (라이다만)
    NP_TURN_W = 0.45          # 좁은 POI 안에서 돌 때 최대 회전 속도 [rad/s] (평소 0.50). 느릴수록 바퀴 출렁임 비율이 커 끊겨 보였다

    # ⚠ 2026-11-16 모든 지점 POI 를 '직선으로 들어가고 직선으로 나오기' (사용자: "amr 이 대부분 직선
    #   그니깐 깔끔하게 들어가고 나가고 움직이는게 좋을꺼같은데", "출발할때 직진하고 목적지로 회전").
    #   전에는 nav2 로 POI 에 바로 가서, 좌우가 16~30mm 남으면 그 자리에서 -71~-82도 돌고 기어 들어갔다가
    #   다시 돌아섰다(매번 6~8초, 끊겨 보임). 이제
    #     도착: nav2 로 POI 앞 SP_PRE_M (축 위) -> POI 점을 바라보며 직선 SP_V -> 도착 방향으로 제자리 회전
    #     출발: 앞으로 SP_OUT_M_FWD 빠져나온 뒤 평소 출발(회전 -> nav2). 앞이 막혔으면 뒤로 (2026-11-23 SP_OUT_FWD_ONLY)
    #   좁은 POI(NP_*)는 예전 그대로.
    SP_ENABLE = True
    SP_PRE_M = 0.60           # 직선으로 들어갈 거리 [m]
    SP_PRE_MIN = 0.40
    SP_V = 0.20               # 직선 들어가기 · 나오기 속도 [m/s]
    # 끝점을 바라보는 직선: 좌우를 '남은 거리 x SP_LOOK_K' 앞에서 지운다. 지연 0.3초 모의, 앞 자리
    #   좌우 1.5~3cm · 0~3도: 1.0 -> 끝 3~6mm, 0.6 -> 1.7~3.6mm (최대 방향 4.7도, 시간 같음).
    SP_LOOK_K = 0.8           # 2026-11-19 0.6 -> 0.8 (SP_LOOK_MIN 0.15 와 같이; 겨누는 거리가 서서히 줄게)
    # ⚠ 2026-11-17 왕복 11번 중 9번이 들어가는 방향으로 POI 15~20mm 앞에서 섰다. 0.04 m/s 끝에서
    #   NP_LAG_S(0.35초 = 14mm) 만큼 미리 끊었는데, 실측(2026-09-29 0.03 m/s)은 끊은 뒤 3~4mm 만 더 간다.
    SP_LAG_S = 0.12
    # ⚠ 2026-11-18 POI 1 도착 약 80회: 앞뒤 늘 +5~+10mm(평균 +8) 지나침. 11/17 1<->2(반대 진입)도 들어가는 방향으로 6mm.
    #   진입 방향과 상관없이 그만큼 일찍 끊는다 (일반 POI 직선 도착만).
    SP_STOP_EARLY = 0.007     # [m]
    # ⚠ 2026-11-18 같은 80회: 좌우 늘 +0~+20mm(평균 +9), 끝 방향 -1~-4도 = 축으로 다가오는 중에 도착(제어 값으로도 그대로).
    #   마지막 구간 겨누는 거리 하한이 NP_LOOK 0.30m 라 끝점보다 먼 곳을 보고 들어갔다. 일반 POI 는 하한을 줄여 끝점을 겨눈다
    #   모의(앞 자리 시작 3가지 · 지연 넣은 넘겨받기 6가지): 0.30/0.6 끝 좌우 최대 4.9mm -> 0.15/0.8 최대 2.0mm,
    #   최대 방향 3.9 -> 4.5도, 축 넘어감 0.1도. 0.15/0.6 은 최대 방향 5.1도.
    SP_LOOK_MIN = 0.15        # [m]
    # ⚠ 2026-11-20 빠져나오기 끝 쏠림(평균 -0.9도)은 좌우 +2mm 를 겨누는 거리 0.15m 로 지우느라 꺾던 것.
    #   빠져나오기는 곧게 나오는 것이 목적이라 끝에서 몇 mm 를 맞출 필요가 없다 - 열린 POI 빠져나오기만 이 아래로 안 줄인다.
    SP_OUT_LOOK_MIN = 0.40    # [m]
    # ⚠ 2026-11-19 1<->2 왕복 21번: 진입 5~6초 동안 축이 odom 에서 매번 같은 쪽으로 18~39mm 옮겨졌다(큰 회전 중 SLAM 이
    #   안 고치다가 진입 중 몰아서 고침). 겨누기만으로는 움직이는 축을 늘 뒤따라 끝 좌우 +7~+11mm. 모의로 재현(축 25mm 이동 ->
    #   8~10mm). 축이 로봇 자리에서 옆으로 움직이는 속도를 재서(시간상수 SP_AXIS_TAU) 그만큼 미리 옆으로 따라간다(상한 SP_AXIS_FF).
    #   상한 6도면 좌우 1~3mm 지만 끝 0.04 m/s 에서 차체가 6~8도 비스듬 -> 2도: 좌우 3.6~4.5mm, 끝 방향 -3.4도(앞먹임 없을 때 -3.2도).
    #   ⚠ SLAM 은 제자리 회전 중 위치를 안 낸다(11/19 기록: 회전 27초 0번, 주행 0.78번/초) - 원인 조사는 별도.
    #   실제 반복정밀도(tools/repeat_probe.py, 스캔끼리 ICP) 10회씩 비교: 2도 켬 좌우 ±4.7/±3.1mm vs 끔 ±5.5/±5.0mm,
    #   앞뒤 ±4.8/±5.6 vs ±4.3/±4.9 - 차이가 표본 잡음 수준인데 끝 방향은 -7~+6도로 흔들림. SLAM 과 실제 차이가 8~17mm 로
    #   훨씬 커서 꺼 둔다(되돌리려면 2.0). 정밀도는 POI 주변 위치추정(지도) 쪽이 먼저.
    SP_AXIS_FF = math.radians(0.0)
    SP_AXIS_TAU = 0.5         # [s]
    # ⚠ 2026-11-17 왕복 31번: 직선 끝에서 odom 축 기준 좌우 ±3mm 인데 지도 기준은 늘 -15mm(180도 회전 전후
    #   변화는 -1mm 뿐). 축을 시작 때 한 번만 odom 으로 바꿔 두어 들어가는 5~6초 동안 들어온 SLAM 보정
    #   만큼 비켜 섰다. map_odom_smoother 로 보정이 부드러우니 들어가는 동안 축을 지도 기준으로 계속 다시 맞춘다.
    SP_REANCHOR = True
    SP_PRE_YAW_OK = math.radians(4)   # 앞 자리 방향 맞추기 합격 (직선이 나머지를 맞춘다; 2~3도 맞추느라 2~4번 돌았다)
    # ⚠ 2026-11-19 POI 빠져나오기 76회: 제자리 회전(180도) 뒤 4초 넘게 서 있다 출발해도, 출발 0.2~0.9초에
    #   차체가 '직전 회전 방향' 으로 약 0.05 rad/s 더 돈다(58/60 회 부호 일치, 방향 +1.6도까지). 지령을
    #   반대로 내도 0.4~0.5초 늦게 따라와 막지 못하고, 바로잡다 넘쳐 ±2도로 2.2초 주기 흔들림이 이어졌다.
    #   직전 회전 방향을 알고 있으니 출발 첫 SP_KICK_T 초 동안 반대로 SP_KICK_W 를 미리 더한다.
    SP_KICK_W = 0.04          # [rad/s]  0 이면 끔
    SP_KICK_T = 0.8           # [s]
    # ⚠ 2026-11-19 미리 돌림으로 첫 비틀림은 절반이 됐지만 뒤 흔들림(-1.6 -> +2.3도)은 그대로였다. 붙잡기가
    #   끝나는 0.06m 에서 속도 상한 0.05 가 풀려 0.20 으로 한 번에 뛰던 때와 겹친다(출발 오름 0.03+0.2t 는 그때
    #   이미 0.27). 붙잡기가 끝난 뒤부터 SP_RAMP_A 로 서서히 올린다. 0 이면 예전처럼.
    #   -> 1↔2 10회: 뒤 흔들림 2.36 -> 2.31도, 폭 4.13 -> 3.96 (잡음 안) 효과 없어 끔.
    SP_RAMP_A = 0.0           # [m/s^2]  (시험값 0.15 = 0.05 -> 0.20 을 1초에)
    # ⚠ 2026-11-19 비틀림은 두 번 온다: 출발 0.2~0.9초에 직전 회전 쪽(+0.7도), 이어서 0.07~0.2m 간 곳에서
    #   반대쪽(-1.5도). 둘째는 지령이 거의 0 인데 생기고, 속도를 늦추면 늦게 나타나 거리에 묶여 있다.
    #   둘째도 간 거리 SP_KICK2_M 구간에서 직전 회전과 같은 쪽으로 SP_KICK2_W 를 미리 더한다(0.4초 지연만큼 앞당김).
    SP_KICK2_W = 0.04         # [rad/s]  0 이면 끔
    SP_KICK2_M = (0.04, 0.17) # [m]
    SP_FLIP_M = 0.06          # 출발 첫 구간 방향만 붙잡기 [m]
    SP_FLIP_V = 0.05
    SP_OUT_M = 0.50           # 출발 때 직선으로 빠져나오는 거리 [m]
    # ⚠ 2026-11-20 후진으로 빠져나온 뒤에는 늘 크게 돈다(목적지가 뒤). 렉이 없으면(리프트 bottom) 0.3m 만
    #   나오고(돌 자리가 없으면 sp_out_extend_plan 이 0.3m 씩 더 나간다), 끝 감속 구간도 짧게 한다.
    SP_OUT_M_NORACK = 0.30    # [m]  후진 빠져나오기 · 렉 없음
    SP_OUT_SLOW_M = 0.08      # [m]  빠져나오기 끝 감속 구간 (평소 NP_SLOW_M 0.20)
    # ⚠ 2026-11-23 사용자: "굳이 뒤로 나올 필요 있나, 항상 앞으로 나오고 거리를 짧게. 뒤로는 별로" — 목적지가 뒤에
    #   있어도 앞으로 SP_OUT_M_FWD 만 빠져나온 뒤 돈다. 뒤로는 앞이 막혔을 때만. 돌 자리가 부족하면
    #   sp_out_extend_plan 이 SP_OUT_STEP 씩 더 나간다(렉 밑에서 나올 때도).
    SP_OUT_FWD_ONLY = True
    SP_OUT_M_FWD = 0.30       # [m]
    SP_MIN_GOAL_M = 0.40      # 로봇이 POI 앞 자리보다 이만큼 더 멀어야 직선 도착을 쓴다 [m]
    SP_OUT_MIN_GOAL_M = 1.00  # 다음 목표가 이보다 가까우면 빠져나오기 없이 평소 출발 [m]
    # ⚠ 2026-11-16 사용자: "완전 부드럽게 끊기지 않게". 앞 자리에서 nav2 가 서고 -> 방향 맞추고 -> 다시
    #   출발하면 한 번 끊긴다. nav2 가 앞 자리 SP_HANDOFF_M 안에 오고 방향이 축과 SP_HANDOFF_YAW,
    #   좌우가 SP_HANDOFF_LAT 안이면 서지 않고 그 속도 그대로 직선 들어가기로 넘겨받는다
    #   (충전소 도킹 넘겨받기와 같은 방식). 아니면 예전처럼 서서 방향을 맞춘다.
    # ⚠ 2026-11-17 '좁은 곳/넓은 곳' 판정 없이 (사용자: "좁은곳 넓은곳 판정없이 다 해결할 방법은?").
    #   지도 기준 자동 판정이 넓은 곳(test1 · 1, 가장 가까운 점 9m)을 좁은 곳으로 잡아 POI 1m 앞 축 위로
    #   돌아갔다 들어왔다(사용자: "poi 를 왜 지나갔다 들어오는거야"). 이제
    #   - 출발 전에 nav2 에 POI 까지 경로를 묻고, 경로가 마지막 SP_PRE_M 동안 들어오는 방향을 본다.
    #     POI 축(앞 또는 뒤)과 SP_FULL_DEG 안이면 SP_PRE_M 직선, SP_ZERO_DEG 넘게 옆이면 직선 없이
    #     POI 로 바로(지나쳐 오지 않음), 사이는 비례. 좁은 통로는 경로가 저절로 축으로 들어온다.
    #   - 직선 속도는 양옆 여유(라이다 · 카메라)가 SP_SIDE_SLOW 안이면 SP_V_TIGHT.
    #   - 빠져나오기는 회전 공간이 생길 때까지 SP_OUT_STEP 씩 SP_OUT_MAX 까지.
    #   웹 '좁은 곳' 표시는 강제로 남긴다(NP_* 1m · 0.10 m/s).
    SP_FULL_DEG = 15.0
    SP_ZERO_DEG = 60.0
    SP_LINE_MIN = 0.15        # 직선이 이보다 짧으면 POI 로 바로 [m]
    SP_PLAN_TIMEOUT_S = 2.5
    SP_SIDE_SLOW = 0.10       # 차체 옆 여유가 이 안이면 느리게 [m]
    SP_V_TIGHT = 0.10
    SP_OUT_MAX = 1.50
    SP_OUT_STEP = 0.30
    SP_HANDOFF_M = 0.35
    # 2026-11-17 12 -> 25도, 4 -> 6cm: 왕복 11번 모두 앞 자리에서 서서 20~31도 돌았다(경로가 축과 7~10도로
    #   들어오고 nav2 가 앞 자리 점으로 꺾어 들어와 각이 벌어짐). 남은 각은 직선 앞부분에서 완만하게 지운다.
    #   단, 축에서 멀어지는 쪽을 보고 있으면(+22도에 축 왼쪽 5cm) 모의에서 반대로 15도 넘어갔다(S자) ->
    #   '가야 할 방향'(끝점을 바라보는 방향)과 SP_HANDOFF_ERR 안일 때만 넘겨받는다 = 축으로 다가오는 중.
    SP_HANDOFF_YAW = math.radians(25)
    # 2026-11-17 0.07 -> 0.10: 16:31 왕복에서 POI 2 쪽은 매번 축 왼쪽 73~91mm 에서 -10~-13도로 다가와(축 쪽으로
    #   다가오는 각, 가야 할 방향과 차이 약 3도) 좌우 기준에만 걸려 섰다.
    SP_HANDOFF_LAT = 0.10
    SP_HANDOFF_LAT_PAR = 0.05 # 이보다 옆이면 축 쪽으로 다가오는 각일 때만
    # 2026-11-17 12 -> 15도: 왕복 10번 모두 넘겨받지 못함(실제는 축 왼쪽 약 5cm 에서 -15~-25도로 다가옴, 차이
    #   10~20도). 모의(지연 0.45~0.6초): 차이 10도 넘침 0, 15도 1~3도, 20도 4~7도, 23도 9~12도(S자).
    SP_HANDOFF_ERR = math.radians(15)
    SP_TURN_V = 0.12          # 방향 오차가 SP_TURN_DEG 넘는 동안 속도 상한 [m/s]
    # ⚠ 2026-11-23 사용자: "휘청거림은 있으면 안 됨, 안정적 주행". 50Hz 기록(진동 제외 0.3~3Hz): 넘겨받기 31번이
    #   모두 0.42~0.53 m/s 로 달리다 축 좌우 2~10cm · 방향 -1~-16도에서 일어났고, 넘겨받는 순간 속도 상한이 바로
    #   SP_V(오차 12도 넘으면 SP_TURN_V)로 떨어져 0.5 -> 0.1 m/s 급제동 + 회전 지령 0 -> 최대로 튀어 차체가 15도
    #   휙 돌았다 6도 되돌아왔다(충전소 -> POI 1 매번). 고침:
    #   - 앞 자리 SP_HANDOFF_M + SP_APPR_RAMP_M 안에서 nav2 속도 상한을 S자로 SP_APPR_V 까지 미리 낮춘다
    #   - 달리며 넘겨받은 직선은 감속 SP_DEC_A, 회전 지령 변화 SP_W_ACC 로 제한
    # ⚠ 2026-11-23 위 조치 뒤에도 nav2 가 앞 자리로 비스듬히 들어와(충전소 -> POI 1 매번 축 좌우 9~16cm · 방향
    #   -13~-19도) 넘겨받은 직선이 그만큼 꺾어야 했다. nav2 를 먼저 축 위 경유점(앞 자리에서 SP_VIA_M 더 바깥)으로
    #   보내고, SP_VIA_SWITCH_M 안에 오면 서지 않고 목표를 앞 자리로 바꾼다 -> 마지막 구간 경로가 축과 겹친다.
    #   돌아가지 않게: 경로가 이미 축 쪽(SP_FULL_DEG 안)으로 들어오고 로봇이 경유점보다 축 바깥쪽에 있을 때만,
    #   경유점~앞 자리 사이 차체가 지도에 걸리지 않을 때만.
    # ⚠ 같은 날 실주행(3+3회): 넘겨받을 때는 축 좌우 0mm · -2도로 반듯했지만, nav2 가 경유점 -> 앞 자리 0.6m 안에서
    #   10cm · 13도를 세게 바로잡아 그 구간 흔들림이 최악 1초 8.9 deg/s (경유점 없이 5.7) -> 끔. 경로 자체가 도착 방향을
    #   지키는 플래너(Smac Hybrid/Lattice) 없이 점 하나 더 찍는 것으로는 꺾는 곳만 옮겨진다.
    SP_VIA_ON = False
    SP_VIA_M = 0.60           # 앞 자리에서 더 바깥 [m]
    SP_VIA_MIN_M = 0.30       # 막히면 이만큼까지 줄여 본다 [m]
    SP_VIA_SWITCH_M = 0.40    # 경유점 이 안에 오면 앞 자리로 바꾼다 [m]
    SP_APPR_V = 0.20          # 넘겨받을 때 속도 [m/s] (= SP_V)
    SP_APPR_RAMP_M = 0.50     # 이 거리 동안 서서히 줄인다 [m] (1.0 은 구간당 약 1.5초 늘어 시험, 2026-11-23)
    SP_DEC_A = 0.25           # [m/s^2]
    SP_W_ACC = 0.25           # [rad/s^2]  (0.80 은 실주행에서 여전히 15도 휙 + 6도 되돌림; 0.35 -> 0.25 같은 날 모의)
    # 실측(13:39): 지령 -> 실제 회전 0.4초 늦고, 실제가 지령보다 25% 더 돌고, 지령을 내린 뒤에도 0.3초 더 돈다.
    #   달리며 넘겨받은 직선은 회전을 SP_W_MAX_HO 로 묶고, 지금 회전 속도로 SP_YAW_PRED_S 뒤 방향을 내다보고 겨눈다.
    SP_W_MAX_HO = 0.122       # [rad/s] 7도/s (9도/s 실주행 최악 1초 5.5 deg/s; 모의 최대 회전 -15~22%, 끝 정확도 같음)
    SP_YAW_PRED_S = 0.30      # [s]
    SP_LOOK_K_HO = 0.8        # 달리며 넘겨받은 직선의 겨누는 거리 계수 (평소 SP_LOOK_K)
    SP_TURN_DEG = 12.0

    # ------------------------------------------------------------ 판단
    def _np_is_narrow(self, p):
        """POI 축 좌우 NP_SIDE_M 안, 앞 자리~POI 구간에 지도 장애물이 있나."""
        if np is None:
            return False
        with self.lock:
            png, meta = self.map_png, dict(self.map_meta)
        if png is None or not meta.get("available"):
            return False
        got = self._map_grid(png, meta)
        if got is None:
            return False
        _lik, res, ox, oy, H, _free, _likc, occ = got
        c, s = math.cos(p["yaw"]), math.sin(p["yaw"])
        for a in np.arange(-self.NP_PRE_M, 0.3001, res):
            for b in np.arange(-self.NP_SIDE_M, self.NP_SIDE_M + 1e-9, res):
                x = p["x"] + a * c - b * s
                y = p["y"] + a * s + b * c
                gx = int((x - ox) / res)
                gy = int(H - 1 - (y - oy) / res)
                if 0 <= gy < occ.shape[0] and 0 <= gx < occ.shape[1] and occ[gy, gx]:
                    return True
        return False

    def _np_pre_pose(self, p, d_max=None, d_min=None):
        """앞 자리 (축 위, POI 방향). 차체(+2cm)가 지도 장애물에 걸리면 줄여 본다. 없으면 None."""
        with self.lock:
            png, meta = self.map_png, dict(self.map_meta)
        got = self._map_grid(png, meta) if png is not None else None
        c, s = math.cos(p["yaw"]), math.sin(p["yaw"])
        d = self.NP_PRE_M if d_max is None else d_max
        d_min = self.NP_PRE_MIN if d_min is None else d_min
        while d >= d_min - 1e-9:
            x, y = p["x"] - d * c, p["y"] - d * s
            if got is None or not self._np_body_hits(got, x, y, p["yaw"]):
                return (x, y, p["yaw"], d)
            d -= 0.1
        return None

    def _np_body_hits(self, got, x, y, yaw, pad=0.02):
        _lik, res, ox, oy, H, _free, _likc, occ = got
        c, s = math.cos(yaw), math.sin(yaw)
        L, W = self.JOB_HALF_L + pad, self.JOB_HALF_W + pad
        for a in np.arange(-L, L + 1e-9, res / 2):
            for b in np.arange(-W, W + 1e-9, res / 2):
                gx = int((x + a * c - b * s - ox) / res)
                gy = int(H - 1 - (y + a * s + b * c - oy) / res)
                if 0 <= gy < occ.shape[0] and 0 <= gx < occ.shape[1] and occ[gy, gx]:
                    return True
        return False

    # ------------------------------------------------------------ 들어가기
    def np_goto(self, p):
        """좁은 POI 면 앞 자리로 보내고 True. 직선 도착(SP_ENABLE)이면 일반 POI 도 앞 자리로.
        아니면 False (평소대로)."""
        if not p.get("narrow"):
            return False                     # 2026-11-17 자동 판정 없음 - 웹 '좁은 곳' 표시만
        # ⚠ 2026-11-16 앞 자리는 로봇이 오는 쪽(가까운 쪽)에 잡는다. 늘 'POI 방향 뒤쪽' 에 잡았더니
        #   반대편에서 오면 POI 를 지나쳐 갔다가 돌아서 왔다(사용자: "한참 지나갔다가 돌아서 와").
        #   들어간 방향이 도착 방향과 반대면, 그 자리에서 돌 공간이 있을 때만 돈다(도착 회전 검사).
        cur = self.tf_pose()
        cands = []
        for flip in (False, True):
            q = dict(p, yaw=_wrap(p["yaw"] + (math.pi if flip else 0.0)))
            pre = self._np_pre_pose(q)
            if pre is None:
                continue
            d = math.hypot(pre[0] - cur["x"], pre[1] - cur["y"]) if cur else (1.0 if flip else 0.0)
            cands.append((d, flip, q, pre))
        if not cands:
            self.get_logger().warn("좁은 POI %s: 앞 자리가 양쪽 다 막혀 평소대로 갑니다" % p["name"])
            return False
        _, flip, entry, pre = min(cands, key=lambda c: c[0])
        ok, msg = self.nav_goto(pre[0], pre[1], pre[2], depart=True, yaw_ok=self.SP_PRE_YAW_OK)
        if not ok:
            return False
        self._nav_precise = False            # 앞 자리에서는 방향만 (2026-11-17: -81도 돌고 기어들던 것)
        self._np = {"state": "pre", "poi": dict(p), "entry": entry, "pre": pre,
                    "seq": getattr(self, "nav_seq", None), "mode": "narrow"}
        self.get_logger().info("좁은 POI %s: 축 위 %.1fm 앞 자리 (%.2f, %.2f) 까지 간 뒤 곧게 들어감%s"
                               % (p["name"], pre[3], pre[0], pre[1],
                                  " (도착 방향과 반대로 들어가 안에서 돌 수 있으면 돎)" if flip else ""))
        return True

    def sp_request(self, p):
        """지점 POI 로: nav2 에 경로를 먼저 물어 들어오는 방향을 보고 _sp_decide. (ok, msg)."""
        from nav2_msgs.action import ComputePathToPose
        cli = getattr(self, "plan_cli", None)
        tok = object()
        self._sp_req = tok
        if cli is None or not cli.server_is_ready():
            return self._sp_decide(p, None, "경로 계획기 없음")
        g = ComputePathToPose.Goal()
        g.goal.header.frame_id = "map"
        g.goal.pose.position.x, g.goal.pose.position.y = float(p["x"]), float(p["y"])
        g.goal.pose.orientation.z = math.sin(p["yaw"] / 2.0)
        g.goal.pose.orientation.w = math.cos(p["yaw"] / 2.0)
        g.planner_id = "GridBased"
        g.use_start = False
        self._sp_wait = {"tok": tok, "p": dict(p), "t0": 0.0, "goal_f": cli.send_goal_async(g), "res_f": None,
                         "seq0": getattr(self, "nav_seq", None)}
        self.nav_state, self.nav_msg = "sending", "경로 확인 중"
        old = getattr(self, "_sp_timer", None)
        if old is not None:
            self._sp_timer_stop()
        self._sp_timer = self.create_timer(0.05, self._sp_plan_tick)
        return True, "'%s' 로 갑니다" % p["name"]

    def _sp_timer_stop(self):
        t = getattr(self, "_sp_timer", None)
        self._sp_timer = None
        if t is not None:
            try:
                t.cancel()
                self.destroy_timer(t)
            except Exception:                                # noqa: BLE001
                pass

    def sp_cancel(self):
        self._sp_req = None
        self._sp_timer_stop()

    def _sp_plan_tick(self):
        w = getattr(self, "_sp_wait", None)
        if not w or w["tok"] is not getattr(self, "_sp_req", None):
            self._sp_timer_stop()
            return
        w["t0"] += 0.05
        path = None
        why = None
        try:
            if w["res_f"] is None and w["goal_f"].done():
                gh = w["goal_f"].result()
                if gh is None or not gh.accepted:
                    why = "경로 요청 거절"
                else:
                    w["res_f"] = gh.get_result_async()
            if w["res_f"] is not None and w["res_f"].done():
                r = w["res_f"].result()
                if r is not None and r.status == 4 and r.result.path.poses:
                    path = [(q.pose.position.x, q.pose.position.y) for q in r.result.path.poses]
                else:
                    why = "경로 없음"
        except Exception as e:                               # noqa: BLE001
            why = "경로 오류 %s" % e
        if path is None and why is None and w["t0"] < self.SP_PLAN_TIMEOUT_S:
            return
        self._sp_timer_stop()
        self._sp_wait = None
        self._sp_req = None
        ok, msg = self._sp_decide(w["p"], path, why or ("경로 확인 시간 초과" if path is None else None))
        if hasattr(self, "_seq_chain"):
            self._seq_chain(w.get("seq0"))
        if not ok:
            self.nav_state, self.nav_msg = "failed", msg

    @staticmethod
    def _sp_approach_yaw(path, back=0.6):
        """경로가 끝점으로 들어오는 방향 (마지막 back m). 모르면 None."""
        if not path or len(path) < 2:
            return None
        ex, ey = path[-1]
        acc = 0.0
        px, py = ex, ey
        for x, y in reversed(path[:-1]):
            acc += math.hypot(px - x, py - y)
            px, py = x, y
            if acc >= back:
                break
        if math.hypot(ex - px, ey - py) < 0.05:
            return None
        return math.atan2(ey - py, ex - px)

    def _sp_line_len(self, diff):
        """들어오는 방향과 축의 각 -> 직선 길이."""
        full, zero = math.radians(self.SP_FULL_DEG), math.radians(self.SP_ZERO_DEG)
        k = (zero - diff) / (zero - full)
        return self.SP_PRE_M * max(0.0, min(1.0, k))

    def _sp_decide(self, p, path, why=None):
        """경로가 들어오는 방향으로 직선 길이를 정해 앞 자리로, 짧으면 POI 로 바로. (ok, msg)."""
        cur = self.tf_pose()
        a = self._sp_approach_yaw(path, self.SP_PRE_M)
        if a is None and cur is not None:
            a = math.atan2(p["y"] - cur["y"], p["x"] - cur["x"])      # 경로를 모르면 로봇 -> POI
        direct = None
        if a is None:
            direct = "방향 모름"
        else:
            d_f = abs(_wrap(a - p["yaw"]))
            d_r = abs(_wrap(a - p["yaw"] - math.pi))
            flip = d_r < d_f
            diff = min(d_f, d_r)
            L = self._sp_line_len(diff)
            entry = dict(p, yaw=_wrap(p["yaw"] + (math.pi if flip else 0.0)))
            pre = None
            if L >= self.SP_LINE_MIN:
                pre = self._np_pre_pose(entry, L, self.SP_LINE_MIN)
            if cur is not None and pre is not None and math.hypot(p["x"] - cur["x"], p["y"] - cur["y"]) < pre[3] + 0.3:
                pre = None                   # 이미 가깝다
            if pre is None:
                direct = "들어오는 방향이 축과 %.0f도" % math.degrees(diff) if L < self.SP_LINE_MIN else "앞 자리 없음"
        if direct is not None:
            self.get_logger().info("POI %s: %s — 바로 감%s" % (p["name"], direct, (" (%s)" % why) if why else ""))
            return self.nav_goto(p["x"], p["y"], p["yaw"], depart=True, poi=p)
        via = self._sp_via_pose(entry, pre, diff, cur)
        tgt = via if via is not None else pre
        ok, msg = self.nav_goto(tgt[0], tgt[1], tgt[2], depart=True, yaw_ok=self.SP_PRE_YAW_OK)
        if not ok:
            return ok, msg
        self._nav_precise = False            # 앞 자리에서는 방향만 맞춘다 (자리 맞추기는 직선 끝에서)
        self._np = {"state": "via" if via is not None else "pre", "poi": dict(p), "entry": entry, "pre": pre,
                    "via": via, "seq": getattr(self, "nav_seq", None), "mode": "open"}
        if via is not None:
            self.get_logger().info("POI %s: 축 위 경유점 %.2fm (%.2f, %.2f) 를 지나 앞 자리로"
                                   % (p["name"], via[3], via[0], via[1]))
        self.get_logger().info("POI %s: 경로가 축과 %.0f도로 들어옴 — 앞 %.2fm (%.2f, %.2f) 에서 직선으로%s"
                               % (p["name"], math.degrees(diff), pre[3], pre[0], pre[1],
                                  " (반대로 들어가 그 자리에서 돎)" if flip else ""))
        return True, "'%s' 로 갑니다 (마지막 %.1fm 직선)" % (p["name"], pre[3])

    def _sp_via_pose(self, entry, pre, diff, cur):
        """축 위 경유점 (앞 자리에서 SP_VIA_M 바깥). 쓸 수 없으면 None."""
        if not self.SP_VIA_ON or cur is None or diff > math.radians(self.SP_FULL_DEG):
            return None
        c, s_ = math.cos(entry["yaw"]), math.sin(entry["yaw"])
        along_r = (cur["x"] - entry["x"]) * c + (cur["y"] - entry["y"]) * s_     # 로봇 (POI 기준, 바깥이 -)
        with self.lock:
            png, meta = self.map_png, dict(self.map_meta)
        got = self._map_grid(png, meta) if png is not None else None
        d = pre[3] + self.SP_VIA_M
        while d >= pre[3] + self.SP_VIA_MIN_M - 1e-9:
            if along_r < -(d + 0.30):                    # 로봇이 경유점보다 바깥에 있어야 돌아가지 않는다
                ok = True
                if got is not None:
                    k = pre[3]
                    while k <= d + 1e-9:
                        if self._np_body_hits(got, entry["x"] - k * c, entry["y"] - k * s_, entry["yaw"]):
                            ok = False
                            break
                        k += 0.1
                if ok:
                    return (entry["x"] - d * c, entry["y"] - d * s_, entry["yaw"], d)
            d -= 0.1
        return None

    def _sp_via_switch(self, st, cur):
        """경유점 근처 - 서지 않고 nav2 목표를 앞 자리로 바꾼다."""
        pre, entry = st["pre"], st.get("entry") or st["poi"]
        c, s_ = math.cos(entry["yaw"]), math.sin(entry["yaw"])
        dx, dy = cur["x"] - entry["x"], cur["y"] - entry["y"]
        lat = -dx * s_ + dy * c
        old = getattr(self, "nav_seq", 0)
        self.nav_seq = old + 1
        st["seq"] = self.nav_seq
        st["state"] = "pre"
        if hasattr(self, "_seq_chain"):
            self._seq_chain(old)
        if getattr(self, "_slow_seq", None) == old:
            self._slow_seq = self.nav_seq            # 출발 저속이 아직 돌고 있으면 이어 간다
        self.nav_target = (pre[0], pre[1], pre[2])
        self.get_logger().info("POI %s: 경유점 지남 — 축 좌우 %+.0fmm 방향 %+.1f도, 서지 않고 앞 자리로"
                               % (st["poi"]["name"], lat * 1000, math.degrees(_wrap(cur["yaw"] - entry["yaw"]))))
        self._nav_send(self._nav_goal_msg(pre[0], pre[1], pre[2]), self.nav_seq)

    def np_handoff(self):
        """nav2 가 앞 자리로 가는 중 - 축에 이미 맞았으면 끊고 직선 들어가기로. 넘겨받았으면 True."""
        st = getattr(self, "_np", None)
        if (not st or st.get("state") not in ("pre", "via") or st.get("mode") != "open"
                or st.get("seq") != getattr(self, "nav_seq", None)):
            return False
        entry, poi = st.get("entry") or st["poi"], st["poi"]
        cur = self.tf_pose()
        if cur is None:
            return False
        pre = st["pre"]
        self._appr_update(math.hypot(cur["x"] - pre[0], cur["y"] - pre[1]))
        if st["state"] == "via":
            via = st["via"]
            if math.hypot(cur["x"] - via[0], cur["y"] - via[1]) <= self.SP_VIA_SWITCH_M:
                self._sp_via_switch(st, cur)
            return False
        if math.hypot(cur["x"] - pre[0], cur["y"] - pre[1]) > self.SP_HANDOFF_M:
            return False
        c, s_ = math.cos(entry["yaw"]), math.sin(entry["yaw"])
        dx, dy = cur["x"] - entry["x"], cur["y"] - entry["y"]
        along, lat = dx * c + dy * s_, -dx * s_ + dy * c
        yerr = _wrap(cur["yaw"] - entry["yaw"])
        want = math.atan2(-lat, max(self.NP_LOOK, self.SP_LOOK_K * -along))   # 직선 구간이 처음 낼 방향
        why = None
        if along > -0.15:
            why = "앞 자리를 지남"
        elif abs(yerr) > self.SP_HANDOFF_YAW:
            why = "방향 %+.0f도" % math.degrees(yerr)
        elif abs(lat) > self.SP_HANDOFF_LAT:
            why = "축 좌우 %+.0fmm" % (lat * 1000)
        elif abs(lat) > self.SP_HANDOFF_LAT_PAR and yerr * lat >= 0.0:
            # 5cm 넘게 옆인데 축과 나란하거나 멀어지는 쪽 - 넘겨받으면 차선 바꾸기처럼 S자 (모의 0도·8cm: -12도 꺾었다 되돌아옴)
            why = "축 좌우 %+.0fmm 인데 축 쪽으로 안 다가옴" % (lat * 1000)
        elif abs(_wrap(want - yerr)) > self.SP_HANDOFF_ERR:
            why = "가야 할 방향과 %+.0f도" % math.degrees(_wrap(want - yerr))   # 축에서 멀어지거나 너무 가파름
        if why is None and abs(_wrap(entry["yaw"] - poi["yaw"])) > math.pi / 2 and not st.get("checked"):
            # 반대로 들어가 그 자리에서 돌 계획 - 넘겨받기 전에 회전 공간을 지금 본다 (2026-11-17: 1 · 2 가
            #   둘 다 반대 진입이라 넘겨받기가 한 번도 안 됐다). 모르거나 부족하면 앞 자리에 서서 예전대로.
            # '모름'(달리는 중 라이다 점을 못 옮김 - 16:31 왕복 POI 1 쪽 11번 모두)은 앞 자리에 서서 들어갈 때처럼
            #   그대로 넘겨받는다. 실제로 돌 때 실시간 검사(_final_turn)가 다시 보고 막히면 되돌아 선다.
            room = self._np_turn_room(poi, entry)
            if room is False:
                why = "POI 에서 돌 공간 부족"
        if why is not None:
            if not st.get("ho_logged"):
                st["ho_logged"] = True
                self.get_logger().info("POI %s: 앞 자리 %.0fmm 전 넘겨받지 않음 (%s; 방향 %+.1f도, 축 좌우 %+.0fmm) — 서서 돌고 들어감"
                                       % (poi["name"], math.hypot(cur["x"] - pre[0], cur["y"] - pre[1]) * 1000, why,
                                          math.degrees(yerr), lat * 1000))
            return False
        o = getattr(self, "odom", None)
        v0 = abs(o.twist.twist.linear.x) if o is not None else 0.0
        # 지난 nav2 목표의 결과는 버린다 (번호를 올린다)
        old = getattr(self, "nav_seq", 0)
        self.nav_seq = old + 1
        st["seq"] = self.nav_seq
        if hasattr(self, "_seq_chain"):
            self._seq_chain(old)
        gh = getattr(self, "nav_goal", None)
        if gh is not None:
            try:
                gh.cancel_goal_async()
            except Exception:                                # noqa: BLE001
                pass
        self.get_logger().info("POI %s: 앞 자리 %.0fmm 전, 축 좌우 %+.0fmm 방향 %+.1f도 - 서지 않고 직선으로 넘겨받음 (%.2f m/s)"
                               % (poi["name"], math.hypot(cur["x"] - pre[0], cur["y"] - pre[1]) * 1000,
                                  lat * 1000, math.degrees(_wrap(cur["yaw"] - entry["yaw"])), v0))
        st["checked"] = True
        self._appr_clear()
        # ⚠ 2026-11-17 state 를 'pre' 로 두었더니 다음 주기(0.05초)마다 또 넘겨받아 직선 진입이 0 m/s 에서
        #   계속 새로 시작했다(15:42 왕복, 넘겨받음 18,235번, 로봇은 앞 자리에서 묶임). 넘겨받으면 'in'.
        st["state"] = "in"
        self._np_start(direction=+1, along_target=0.0, p=entry, then="arrive", mode="open", v0=v0)
        self.nav_state, self.nav_msg = "active", "POI 로 직선으로 들어가는 중"
        return True

    def _appr_update(self, d_pre):
        """앞 자리로 다가오는 nav2 속도 상한 (S자). 멀면 상한 없음."""
        top = (self._speed_eff() if hasattr(self, "nav_speed") and hasattr(self, "_speed_eff")
               else getattr(self, "NAV_SPEED_DEFAULT", 0.5))
        x = (d_pre - self.SP_HANDOFF_M) / max(self.SP_APPR_RAMP_M, 1e-6)
        if x >= 1.0 or top <= self.SP_APPR_V:
            if getattr(self, "_appr_lim", None) is not None:
                self._appr_clear()
            return
        x = max(0.0, x)
        f = x * x * (3.0 - 2.0 * x)
        lim = self.SP_APPR_V + (top - self.SP_APPR_V) * f
        first = getattr(self, "_appr_lim", None) is None
        self._appr_lim = lim
        if first:
            self.get_logger().info("POI 앞 자리 %.0fmm 전 — %.2f m/s 까지 서서히 줄임" % (d_pre * 1000, self.SP_APPR_V))
        if getattr(self, "_slow_timer", None) is None:
            self._pub_speed_limit(0.0)          # _pub_speed_limit 이 _appr_lim 을 건다 (출발 중이면 _slow_tick 이)

    def _appr_clear(self):
        if getattr(self, "_appr_lim", None) is None:
            return
        self._appr_lim = None
        if getattr(self, "_slow_timer", None) is None:
            self._pub_speed_limit(0.0)

    def np_on_arrival(self):
        """앞 자리에 도착(방향까지)했으면 곧게 들어가기 시작하고 True."""
        st = getattr(self, "_np", None)
        if not st or st.get("state") not in ("pre", "via") or st.get("seq") != getattr(self, "nav_seq", None):
            return False                   # 'via' = 경유점에 서 버렸다 (바꾸기 전에 nav2 가 끝냄) -> 거기서 직선으로
        self._appr_clear()
        p = st.get("entry") or st["poi"]
        poi = st["poi"]
        if abs(_wrap(p["yaw"] - poi["yaw"])) > math.pi / 2 and not st.get("checked"):
            # 반대로 들어가 안에서 돌 계획 -> 지금(입구 앞) 라이다·카메라로 그 자리 회전 공간을 본다
            room = self._np_turn_room(poi, p)
            if room is False:
                far = (self._np_pre_pose(poi, self.SP_PRE_M, self.SP_PRE_MIN) if st.get("mode") == "open"
                       else self._np_pre_pose(poi))
                if far is not None:
                    seq0 = getattr(self, "nav_seq", None)
                    ok, _ = self.nav_goto(far[0], far[1], far[2], depart=True, yaw_ok=self.SP_PRE_YAW_OK)
                    if ok and hasattr(self, "_seq_chain"):
                        self._seq_chain(seq0)
                    if ok:
                        if st.get("mode") == "open":
                            self._nav_precise = False
                        self._np = {"state": "pre", "poi": dict(poi), "entry": dict(poi),
                                    "pre": far, "seq": getattr(self, "nav_seq", None), "checked": True,
                                    "mode": st.get("mode", "narrow")}
                        self.get_logger().warn("좁은 POI %s: 안에서 돌 공간이 부족해 반대편 앞 자리 (%.2f, %.2f)"
                                               " 에서 도착 방향 그대로 들어감" % (poi["name"], far[0], far[1]))
                        return True
                st["no_turn"] = True
                self.get_logger().warn("좁은 POI %s: 안에서 돌 공간이 부족하고 반대편 앞 자리도 없어 "
                                       "들어온 방향으로 섭니다" % poi["name"])
        cur = self.tf_pose()
        if cur is not None and abs(_wrap(cur["yaw"] - p["yaw"])) > math.radians(20):
            self._np = None
            self.nav_state = "failed"
            self.nav_msg = "앞 자리에서 방향을 못 맞춰 좁은 곳에 들어가지 않았습니다 (%.0f도)" % math.degrees(
                _wrap(cur["yaw"] - p["yaw"]))
            self.get_logger().warn(self.nav_msg)
            return True
        self._np_start(direction=+1, along_target=0.0, p=p, then="arrive",
                       mode=st.get("mode", "narrow"))
        self.nav_state, self.nav_msg = "active", ("좁은 곳으로 곧게 들어가는 중" if st.get("mode") != "open"
                                                  else "POI 로 직선으로 들어가는 중")
        return True

    def _np_lidar_pts(self):
        """지금 라이다 점 (로봇 기준, N x 2) 또는 None."""
        lk = getattr(self, "lock", None)
        if lk is None:
            return None
        with lk:
            m = self.scan
        if m is None:
            return None
        got = self._scan_xy(m, False)
        return None if got is None else np.stack(got, 1)

    def _np_turn_room(self, poi, entry):
        """POI 자리에서 entry 방향 -> POI 방향으로 돌 공간이 있나 (지금 센서 기준). 모르면 None.

        ⚠ 2026-11-16 입구 1m 앞에서는 라이다만, 여유 NP_PRE_PAD(0) 로 너그럽게 본다. 정면 카메라는
          이 거리에서 통로 안을 비스듬히 봐 몇 cm 흔들려, 같은 자리를 한 번은 '돎' 한 번은 '부족' 으로
          판단해 반대편으로 돌아갔다(사용자: "왜 또 지나침"). 실제로 돌 때는 라이다 + 카메라 · 여유
          NP_TURN_PAD 로 꼼꼼히 보고, 막히면 들어온 방향으로 되돌아 선다.
        """
        cur = self.tf_pose()
        if cur is None or np is None:
            return None
        P = self._np_lidar_pts()
        if P is None or len(P) == 0:
            return None
        x, y = P[:, 0], P[:, 1]
        c, s = math.cos(cur["yaw"]), math.sin(cur["yaw"])
        wx, wy = cur["x"] + x * c - y * s, cur["y"] + x * s + y * c
        c2, s2 = math.cos(-entry["yaw"]), math.sin(-entry["yaw"])
        dx, dy = wx - poi["x"], wy - poi["y"]
        rel = np.stack([dx * c2 - dy * s2, dx * s2 + dy * c2], 1)
        rel = rel[np.hypot(rel[:, 0], rel[:, 1]) < self.SPIN_RMAX]
        turn = _wrap(poi["yaw"] - entry["yaw"])
        best = None
        for ang in (turn, turn - math.copysign(2.0 * math.pi, turn)):
            h = self._sweep_hits(rel, ang, pad=self.NP_PRE_PAD)
            best = h if best is None else min(best, h)
        dmin = float(np.hypot(rel[:, 0], rel[:, 1]).min()) if len(rel) else 9.0
        self.get_logger().info("좁은 POI %s 회전 공간 사전 판단(라이다, 여유 %.0fcm): 걸리는 점 %d, "
                               "POI 중심~가장 가까운 점 %.3fm (차체 외접 0.467m)"
                               % (poi["name"], self.NP_PRE_PAD * 100, best, dmin))
        return best < self.SPIN_PTS

    # ------------------------------------------------------------ 나오기
    def np_back_out_needed(self):
        """지금 좁은 POI 안(축 위, 같은 방향)에 서 있나. 그 POI 를 낸다."""
        last = getattr(self, "_np_last", None)
        cur = self.tf_pose()
        if not last or cur is None:
            return None
        for flip in (0.0, math.pi):
            yaw = _wrap(last["yaw"] + flip)
            c, s = math.cos(yaw), math.sin(yaw)
            dx, dy = cur["x"] - last["x"], cur["y"] - last["y"]
            along, side = dx * c + dy * s, -dx * s + dy * c
            if (-0.5 < along < 0.3 and abs(side) < 0.15
                    and abs(_wrap(cur["yaw"] - yaw)) < math.radians(25)):
                return dict(last, yaw=yaw)
        return None

    def sp_out_plan(self, gx, gy):
        """직선 도착으로 선 POI 에서 출발할 때 직선으로 빠져나올 계획 (p, 방향, 거리) 또는 None."""
        if not self.SP_ENABLE:
            return None
        cur = self.tf_pose()
        if cur is None:
            return None
        # 2026-11-17 판정 없이: 지점 POI 위(0.15m 안)에 서 있으면 어떤 POI 든 직선으로 빠져나온다
        last = None
        lk = getattr(self, "lock", None)
        pois = []
        if lk is not None:
            with lk:
                pois = [dict(q) for q in getattr(self, "pois", []) if q.get("kind", "point") == "point"]
        for q in pois:
            if math.hypot(cur["x"] - q["x"], cur["y"] - q["y"]) <= 0.15:
                last = q
                break
        if last is None:
            return None
        if math.hypot(gx - cur["x"], gy - cur["y"]) < self.SP_OUT_MIN_GOAL_M:
            return None
        b = _wrap(math.atan2(gy - cur["y"], gx - cur["x"]) - cur["yaw"])
        p = dict(last, x=cur["x"], y=cur["y"], yaw=cur["yaw"])   # 지금 몸 방향 축
        with self.lock:
            png, meta = self.map_png, dict(self.map_meta)
        got = self._map_grid(png, meta) if png is not None and np is not None else None
        order = ((+1, -1) if (self.SP_OUT_FWD_ONLY or abs(b) <= math.pi / 2) else (-1, +1))
        if got is not None and self._np_body_hits(got, cur["x"], cur["y"], cur["yaw"]):
            # ⚠ 2026-11-17 POI 1: 서 있는 자리부터 저장 지도 장애물과 겹쳤다(지도가 예전 것). 지도로 보면 앞뒤가
            #   다 막혀 빠져나오기를 건너뛰었고 nav2 도 '출발 위치가 장애물' 로 실패했다. 실시간 센서만 본다.
            self.get_logger().warn("POI %s: 지금 자리가 저장 지도 장애물과 겹침(지도가 실제와 다름) — 빠져나오기는 "
                                   "라이다 · 카메라로만 판단" % last.get("name", "?"))
            got = None
        for d in order:
            if d > 0 and self.SP_OUT_FWD_ONLY:
                dist = self.SP_OUT_M_FWD
            else:
                dist = (self.SP_OUT_M_NORACK if (d < 0 and getattr(self, "lift", None) == "bottom")
                        else self.SP_OUT_M)
            ex = cur["x"] + d * dist * math.cos(cur["yaw"])
            ey = cur["y"] + d * dist * math.sin(cur["yaw"])
            if got is not None and self._np_body_hits(got, ex, ey, cur["yaw"]):
                continue
            if self._np_blocked(d):
                continue
            return p, d, dist
        return None

    def sp_out(self, plan, after):
        """직선으로 빠져나온 뒤 after() (출발: 회전 -> nav2)."""
        p, d, dist = plan
        # ⚠ 2026-11-17 self._np 는 건드리지 않는다. 다음 POI 의 '앞 자리 -> 직선' 계획이 이미 들어 있는데
        #   여기서 덮어써 지워져, 앞 자리 도착을 최종 도착으로 처리하고 0.6m 앞에서 리프트를 내렸다.
        self._np_last = None
        self._sp_out_dir = d
        self._sp_out_total = getattr(self, "_sp_out_total", 0.0) + dist

        def _then(after=after, d=d):
            # 직진으로 빠져나왔으면 다음 출발이 회전 없이 정면일 때 천천히 시작을 건너뛴다 (api_node _slow_start)
            self._sp_fwd_out = d > 0
            after()
        self._np_start(direction=d, along_target=d * dist, p=p, then=_then, mode="open")
        self.nav_state, self.nav_msg = "active", "직선으로 빠져나오는 중"
        self.get_logger().info("POI %s 출발: %s %.1fm 빠져나온 뒤 목적지로 회전"
                               % (p.get("name", "?"), "직진으로" if d > 0 else "직선 후진으로", dist))

    def sp_out_extend_plan(self):
        """빠져나왔는데 아직 돌 공간이 없으면 같은 방향으로 SP_OUT_STEP 더 (SP_OUT_MAX 까지)."""
        total = getattr(self, "_sp_out_total", 0.0)
        d = getattr(self, "_sp_out_dir", 0)
        if not d or total <= 0.0 or total >= self.SP_OUT_MAX - 1e-6:
            return None
        cur = self.tf_pose()
        if cur is None or self._np_blocked(d):
            return None
        step = min(self.SP_OUT_STEP, self.SP_OUT_MAX - total)
        with self.lock:
            png, meta = self.map_png, dict(self.map_meta)
        got = self._map_grid(png, meta) if png is not None and np is not None else None
        ex = cur["x"] + d * step * math.cos(cur["yaw"])
        ey = cur["y"] + d * step * math.sin(cur["yaw"])
        if got is not None and not self._np_body_hits(got, cur["x"], cur["y"], cur["yaw"]) \
                and self._np_body_hits(got, ex, ey, cur["yaw"]):
            return None
        p = {"name": "출발", "x": cur["x"], "y": cur["y"], "yaw": cur["yaw"]}
        return p, d, step

    def np_back_out(self, p, after):
        """축을 따라 곧게 후진해 앞 자리까지 나온 뒤 after() 를 부른다."""
        pre = self._np_pre_pose(p) or (None, None, None, self.NP_PRE_M)
        self._np = {"state": "out", "poi": dict(p), "seq": getattr(self, "nav_seq", None)}
        self._np_start(direction=-1, along_target=-pre[3], p=p, then=after)
        self.nav_state, self.nav_msg = "active", "좁은 곳에서 곧게 후진해 나오는 중"
        self.get_logger().info("좁은 POI %s: 돌 자리가 없어 축을 따라 %.1fm 후진해 나온 뒤 출발"
                               % (p["name"], pre[3]))

    # ------------------------------------------------------------ 곧게 가기
    def _np_start(self, direction, along_target, p, then, mode="narrow", v0=None):
        mo = self.tf_pose("map", "odom")
        if mo is None:
            self._np_fail("위치를 몰라 곧게 가기를 못 합니다")
            return
        # POI 축을 odom 기준으로 바꿔 둔다 (SLAM 보정이 계단으로 들어와도 직선이 흔들리지 않게)
        c, s = math.cos(-mo["yaw"]), math.sin(-mo["yaw"])
        dx, dy = p["x"] - mo["x"], p["y"] - mo["y"]
        self._np_axis = (dx * c - dy * s, dx * s + dy * c, _wrap(p["yaw"] - mo["yaw"]))
        self._np_axis_map = (p["x"], p["y"], p["yaw"])
        self._np_reanchor = bool(self.SP_REANCHOR and mode == "open" and then == "arrive")
        self._np_dir = direction
        self._np_target = along_target
        self._np_mode = mode
        self._np_tight = False
        self._np_v0 = v0                     # 넘겨받을 때 지금 속도 (None = 서 있다)
        self._np_v_prev = None
        self._np_w_prev = None
        self._np_then = then
        self._np_t = 0.0
        self._np_block_t = 0.0
        self._np_along0 = None
        self._np_flip_t = None
        self._np_log_t = -9.0                # 진입마다 중간 기록 (예전엔 지난 진입 시각이 남아 첫 회만 찍혔다)
        self._np_axis_rate = 0.0
        self._np_axis0 = self._np_axis
        # 서 있다 앞으로 출발할 때만: 직전 제자리 회전 반대 방향으로 미리 돌린다 (SP_KICK_W 주석)
        self._np_kick = (-getattr(self, "_spin_sign", 0)
                         if (v0 is None and direction > 0 and self.SP_KICK_W > 0) else 0)
        if self._np_kick:
            self.get_logger().info("  직전 회전 %s — 출발 %.1f초 동안 %+.3f rad/s 미리 돌림"
                                   % ("왼쪽" if self._np_kick < 0 else "오른쪽",
                                      self.SP_KICK_T, self._np_kick * self.SP_KICK_W))       # 진입 중 축이 지도 보정으로 옆으로 옮겨진 양을 끝에 남긴다
        old = getattr(self, "_np_timer", None)
        if old is not None:
            self._np_stop_timer()
        self._np_timer = self.create_timer(1.0 / self.NP_HZ, self._np_tick)

    def _np_stop_timer(self):
        t = getattr(self, "_np_timer", None)
        self._np_timer = None
        if t is not None:
            try:
                t.cancel()
                self.destroy_timer(t)
            except Exception:                                # noqa: BLE001
                pass

    def np_cancel(self):
        if getattr(self, "_np_timer", None) is not None:
            self._np_stop_timer()
            self._pub_zero()
            self.get_logger().info("좁은 POI 곧게 가기 멈춤")
        self._np = None

    def _np_fail(self, why):
        self._np_stop_timer()
        self._pub_zero()
        self._np = None
        self.nav_state, self.nav_msg = "failed", why
        self.get_logger().warn("좁은 POI: " + why)

    def _np_side_gap(self):
        """차체 옆(앞뒤 길이 안) 가장 가까운 점까지 여유 [m]. 점 없으면 None."""
        pts = self._obstacle_pts()
        if pts is None or len(pts) == 0:
            return None
        L, W = self.JOB_HALF_L + 0.10, self.JOB_HALF_W
        k = (np.abs(pts[:, 0]) <= L) & (np.abs(pts[:, 1]) > W)
        if not k.any():
            return None
        return float(np.abs(pts[k, 1]).min() - W)

    def _np_blocked(self, direction):
        pts = self._obstacle_pts()
        if pts is None or len(pts) == 0:
            return False
        L, W = self.JOB_HALF_L, self.JOB_HALF_W + 0.02
        band = np.abs(pts[:, 1]) <= W
        if direction > 0:
            band &= (pts[:, 0] > L) & (pts[:, 0] < L + self.NP_FRONT_M)
        else:
            band &= (pts[:, 0] < -L) & (pts[:, 0] > -L - self.NP_FRONT_M)
        return int(band.sum()) >= self.SPIN_PTS

    def _np_tick(self):
        dt = 1.0 / self.NP_HZ
        self._np_t += dt
        if self.estop or self.control_mode != "auto" or getattr(self, "_scan_dead", False):
            self._np_stop_timer()
            self._pub_zero()
            self._np = None
            self.nav_state, self.nav_msg = "canceled", "이동을 취소했습니다"
            return
        ro = self.tf_pose("odom")
        if ro is None:
            self._np_fail("위치를 몰라 멈췄습니다")
            return
        axis_prev = self._np_axis
        if getattr(self, "_np_reanchor", False):
            tfn = getattr(self, "_tf_nowait", None)
            mo = (tfn("map", "odom") if tfn is not None else None) or self.tf_pose("map", "odom")
            if mo is not None:
                px, py, pyaw = self._np_axis_map
                c0, s0 = math.cos(-mo["yaw"]), math.sin(-mo["yaw"])
                dx0, dy0 = px - mo["x"], py - mo["y"]
                self._np_axis = (dx0 * c0 - dy0 * s0, dx0 * s0 + dy0 * c0, _wrap(pyaw - mo["yaw"]))
        ax, ay, ayaw = self._np_axis
        c, s = math.cos(ayaw), math.sin(ayaw)
        dx, dy = ro["x"] - ax, ro["y"] - ay
        along, lat = dx * c + dy * s, -dx * s + dy * c
        if axis_prev is not self._np_axis:
            # 이번 주기에 축이 로봇 자리에서 옆으로 움직인 양 -> 속도 (거르기)
            pc, ps = math.cos(axis_prev[2]), math.sin(axis_prev[2])
            lat_prev = -(ro["x"] - axis_prev[0]) * ps + (ro["y"] - axis_prev[1]) * pc
            r = (lat_prev - lat) / dt
            k = dt / (self.SP_AXIS_TAU + dt)
            self._np_axis_rate = getattr(self, "_np_axis_rate", 0.0) + k * (r - getattr(self, "_np_axis_rate", 0.0))
        yerr = _wrap(ro["yaw"] - ayaw)
        d = self._np_dir
        remain = (self._np_target - along) * d
        v_now = abs(getattr(getattr(self, "odom", None), "twist", None).twist.linear.x) \
            if getattr(self, "odom", None) is not None else 0.0
        lag = self.SP_LAG_S if getattr(self, "_np_mode", "narrow") == "open" else self.NP_LAG_S
        early = self.SP_STOP_EARLY if (getattr(self, "_np_mode", "narrow") == "open"
                                       and self._np_then == "arrive") else 0.0
        if remain <= max(0.003, v_now * lag) + early:
            self._np_stop_timer()
            self._pub_zero()
            then = self._np_then
            a0 = getattr(self, "_np_axis0", None) or self._np_axis
            shift = -(ax - a0[0]) * s + (ay - a0[1]) * c          # 진입 동안 축이 옆으로 옮겨진 양
            self.get_logger().info("좁은 POI 곧게 %s 끝: 축 좌우 %+.0fmm 방향 %+.1f도 (남은 %+.0fmm, 진입 중 축 옆 이동 %+.0fmm, %.1f초)"
                                   % ("들어가기" if d > 0 else "나오기", lat * 1000, math.degrees(yerr),
                                      remain * 1000, shift * 1000, self._np_t))
            if then == "arrive":
                p = self._np["poi"] if self._np else None
                entry = (self._np or {}).get("entry") or p
                no_turn = bool((self._np or {}).get("no_turn"))
                self._np = None
                if p is not None:
                    self._np_last = dict(p, _mode=getattr(self, "_np_mode", "narrow"))
                    self.nav_target = (p["x"], p["y"], p["yaw"])
                if no_turn:
                    self.nav_target = (p["x"], p["y"], entry["yaw"])
                    self._finish_arrival("회전할 공간이 없어 들어온 방향으로 서 있습니다")
                elif p is not None and abs(_wrap(entry["yaw"] - p["yaw"])) > math.pi / 2:
                    # 도착 방향과 반대로 들어왔다 - 평소 도착 회전으로 (돌 공간이 없으면 보정만 멈춤)
                    self.nav_state, self.nav_msg = "active", "좁은 곳 도착 - 방향 맞추는 중"
                    self._np_turning = True
                    if not self._final_turn():
                        self._finish_arrival("좁은 곳 곧게 진입 (방향 보정 못 함)")
                    else:
                        # ⚠ 2026-11-16 곧게 들어간 뒤에는 기어가기·자리 맞추기를 하지 않고 방향만 맞춘다.
                        #   지도(SLAM) 기준 위치가 이 구역(지도에 없는 벽)에서 6cm 흔들려, '좌우 -62mm' 로
                        #   보고 목표점 쪽으로 71도 돌고 다시 들어가며 뚝뚝 끊겼다(바퀴·IMU 축 기준은 0mm).
                        self._pos_pass = self.POS_PASS_MAX
                        self._turn_phase = "settle"
                elif p is not None and getattr(self, "_np_mode", "narrow") == "open":
                    # 직선으로 들어왔다 - 남은 방향(보통 1~2도)만 제자리에서 맞춘다
                    self.nav_state, self.nav_msg = "active", "도착 - 방향 맞추는 중"
                    if not self._final_turn():
                        self._finish_arrival("직선 도착")
                    else:
                        self._pos_pass = self.POS_PASS_MAX
                        self._turn_phase = "settle"
                else:
                    self._finish_arrival("좁은 곳 곧게 진입")
            else:
                if (self._np or {}).get("state") == "out":
                    self._np = None
                then()
            return
        if self._np_t > self.NP_MAX_S:
            self._np_fail("곧게 가기 시간이 초과되었습니다")
            return
        if self._np_blocked(d):
            self._np_block_t += dt
            self._np_t -= dt
            self._pub_zero()
            self.nav_msg = "%s 막힘 — 치울 때까지 기다림" % ("앞이" if d > 0 else "뒤가")
            if self._np_block_t > self.NP_WAIT_S:
                self._np_fail("%s %.0f초 넘게 막혀 멈췄습니다" % ("앞이" if d > 0 else "뒤가", self.NP_WAIT_S))
            return
        self._np_block_t = 0.0
        open_ = getattr(self, "_np_mode", "narrow") == "open"
        vmax = self.SP_V if open_ else self.NP_V
        if open_ and abs(yerr) > math.radians(self.SP_TURN_DEG):
            vmax = min(vmax, self.SP_TURN_V)        # 넘겨받은 뒤 크게 틀어져 있으면 도는 동안 조금 천천히
        if open_ and not getattr(self, "_np_tight", False):
            gap = self._np_side_gap()
            if gap is not None and gap < self.SP_SIDE_SLOW:
                self._np_tight = True        # 한 번 좁으면 이번 직선은 끝까지 느리게 (속도가 오락가락하지 않게)
                self.get_logger().info("  직선 구간 옆 여유 %.0fmm — %.2f m/s 로" % (gap * 1000, self.SP_V_TIGHT))
        if open_ and getattr(self, "_np_tight", False):
            vmax = self.SP_V_TIGHT
        slow_m = (self.SP_OUT_SLOW_M if (open_ and (d < 0 or self.SP_OUT_FWD_ONLY)
                                         and getattr(self, "_np_then", None) != "arrive")
                  else self.NP_SLOW_M)
        v = vmax if remain > slow_m else max(self.NP_V_END, vmax * remain / slow_m)
        v0 = getattr(self, "_np_v0", None)
        v = min(v, (0.03 if v0 is None else max(0.03, v0)) + self._np_t * 0.2)   # 출발은 부드럽게
        if v0 is not None and remain > slow_m:
            # 달리며 넘겨받았다 — 급제동 없이 SP_DEC_A 로 줄인다 (끝 감속 구간은 그대로)
            vp = getattr(self, "_np_v_prev", None)
            v = max(v, min(v0, v0 if vp is None else vp) - self.SP_DEC_A * dt)
        self._np_v_prev = v
        if getattr(self, "_np_along0", None) is None:
            self._np_along0 = along
        gone = abs(along - self._np_along0)
        flip = v0 is None and gone < (self.SP_FLIP_M if open_ else self.NP_FLIP_M)   # 달리며 넘겨받았으면 없음
        if flip:
            v = min(v, self.SP_FLIP_V if open_ else self.NP_FLIP_V)
            self._np_flip_t = None
        elif v0 is None and self.SP_RAMP_A > 0:
            if getattr(self, "_np_flip_t", None) is None:
                self._np_flip_t = self._np_t          # 붙잡기가 막 끝났다
            v = min(v, (self.SP_FLIP_V if open_ else self.NP_FLIP_V)
                    + (self._np_t - self._np_flip_t) * self.SP_RAMP_A)
        # 축을 따라간다: 좌우 lat 를 NP_LOOK 앞에서 지우는 방향 (후진이면 진행 방향이 반대).
        #   캐스터가 풀리는 첫 구간은 좌우를 고치지 않고 방향만 붙잡는다.
        # 일반 POI 는 끝점을 바라보는 직선 (남은 거리 앞에서 좌우를 지운다 -> 1~2도). 좁은 곳은 0.3m 앞.
        look_k = self.SP_LOOK_K_HO if getattr(self, "_np_v0", None) is not None else self.SP_LOOK_K
        look = max(self.SP_LOOK_MIN, look_k * remain) if open_ else self.NP_LOOK
        if open_ and getattr(self, "_np_then", None) != "arrive":
            look = max(look, self.SP_OUT_LOOK_MIN)
        head = 0.0 if flip else math.atan2(-lat * d, look)
        if open_ and not flip and getattr(self, "_np_reanchor", False):
            # 축이 옆으로 움직이는 만큼 미리 따라간다: 옆 속도 d*v*sin(방향) = 축 속도
            ff = math.asin(max(-1.0, min(1.0, getattr(self, "_np_axis_rate", 0.0) * d / max(v, 0.03))))
            head += max(-self.SP_AXIS_FF, min(self.SP_AXIS_FF, ff))
        k = getattr(self, "_np_kick", 0)
        kick = k * self.SP_KICK_W if self._np_t < self.SP_KICK_T else 0.0
        if k and self.SP_KICK2_M[0] <= gone < self.SP_KICK2_M[1]:
            kick -= k * self.SP_KICK2_W
        if v0 is not None:
            o = getattr(self, "odom", None)
            w_act = o.twist.twist.angular.z if o is not None else 0.0
            wm = self.SP_W_MAX_HO
            w = max(-wm, min(wm, self.NP_K_YAW * (head - (yerr + w_act * self.SP_YAW_PRED_S)) + kick))
        else:
            w = max(-self.NP_W_MAX, min(self.NP_W_MAX, self.NP_K_YAW * (head - yerr) + kick))
        if v0 is not None:
            # 달리며 넘겨받았다 — 회전 지령이 한 번에 튀지 않게
            wp = getattr(self, "_np_w_prev", None)
            if wp is None:
                o = getattr(self, "odom", None)
                wp = o.twist.twist.angular.z if o is not None else 0.0
            w = max(wp - self.SP_W_ACC * dt, min(wp + self.SP_W_ACC * dt, w))
        self._np_w_prev = w
        t = Twist()
        t.linear.x = float(d * v)
        t.angular.z = float(w)            # 방향 제어는 전진·후진 같은 부호
        self.pub_twist_free.publish(t)
        if self._np_t - getattr(self, "_np_log_t", -9.0) >= 0.5:
            self._np_log_t = self._np_t
            o = getattr(self, "odom", None)
            w_act = o.twist.twist.angular.z if o is not None else float("nan")
            self.get_logger().info("  곧게 %s %4.1fs 남은 %4.0fmm 축 좌우 %+4.0fmm 방향 %+5.1f도 | v %.2f w %+.3f 실제 %+.3f | 축 옆속도 %+.1fmm/s"
                                   % ("들어가기" if d > 0 else "나오기", self._np_t, remain * 1000,
                                      lat * 1000, math.degrees(yerr), v, w, w_act,
                                      getattr(self, "_np_axis_rate", 0.0) * 1000))
