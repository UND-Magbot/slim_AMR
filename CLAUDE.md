# CLAUDE.md — slim_AMR (UND_AMR)

이 저장소에서 작업하는 Claude 를 위한 안내다. 사람용 상세 문서는 `docs/인수인계_2026-11-23.md` 이고, 이 파일은 그 요약과 작업 규칙이다.
**작업을 시작하기 전에 `docs/인수인계_2026-11-23.md` 를 끝까지 읽어라.** 최근 날짜별 상세는 `docs/일일업무보고_*.md` 에 있다(최신 것부터).

## 0. 반드시 지킬 것

- **사용자에게 보이는 모든 글은 한국어로 쓴다.** 도구 호출 사이 짧은 진행 알림도 포함한다. 코드·명령·파일명만 예외다. 
- 원인 판단을 이랬다저랬다 하지 않는다. 결론은 같은 조건 A/B 를 반복한 숫자로만 말한다. 효과가 없으면 없다고 짧게 말한다.
- 흔들림·S자를 **캐스터 탓으로 돌리지 않는다.** 구동·제어 쪽에서 해결책을 찾는다. 다른 회사 로봇(AutoXing)과 비교하지 않는다.
- 측정 도구부터 의심한다. 예전에 rclpy spin_once 줄 서기로 /odom 이 1~2초 늦게 찍혀 틀린 결론을 낸 적이 있다.
- 일일업무보고·업무일지는 **요청할 때만** 쓴다. 형식은 최신 `docs/일일업무보고_*.md` 를 따른다("Noah 일일 업무 보고 <날짜>").
- 로봇을 움직이는 명령은 확인한 뒤에 보낸다.
  - 무한 왕복(`/route/start`)은 사용자가 직접 돌리는 것을 선호한다.
  - 리프트 API(`/lift/up` 등)는 직접 호출하지 말고 사용자에게 요청한다.
  - 자동 모드에서 재시작·왕복이 거절되면 사용자에게 묻는다.
- **하드웨어 원인 조사는 지금 하지 않는다**(사용자 결정, 2026-11-23).

## 1. 사용자와 합의한 주행 원칙 (되돌리지 말 것)

- **휘청거림은 있으면 안 된다.** 속도보다 안정이 먼저다.
- POI 도착 뒤 왔다갔다(기어가기·재진입)로 자리를 맞추지 않는다. 진입 중 제어로만 맞춘다.
- 모든 POI 는 직선 도착·직선 빠져나오기다. **빠져나오기는 항상 앞으로, 짧게(0.3m).** 앞 자리에서 돌고 후진 직선으로 들어가는 안은 거절됐다.
- 지도 재작성은 보류 중이다. 렉에는 마커를 붙일 수 없다. 충전소에는 렉을 싣고 가지 않는다.
- POI1 출발(빠져나오기+회전) 9.1초 설정은 확정이다.
- 앞으로 주행은 노드바이노드(경유지 W 체인, 회피 대신 정지)로 바꾼다. **시작 전에 계획을 먼저 보여 준다.** 참고 저장소 UND-Magbot/RCS_LG 는 방식만 참고하고 코드는 복사하지 않는다.

## 2. 구조

```
ros2/und_amr_bringup/        ROS2 패키지 (실체). 로봇 PC 의 ~/ros2_ws/src/und_amr_bringup 은 여기로 가는 링크
  und_amr_bringup/api_node.py      핵심(10k줄): HTTP 8090·웹, 출발/도착 회전, 렉 작업, SLAM 띄우기·감시, 위치 게이트, 정지 감시
  und_amr_bringup/narrow_poi.py    직선 진입·빠져나오기(SP_*), 좁은 POI(NP_*), 넘겨받기, 접근 감속 — api_node 믹스인
  und_amr_bringup/charger_dock.py  충전소 후진 도킹(CD_*) — api_node 믹스인
  und_amr_bringup/base_node.py     STM32 시리얼(CRC), /odom, 모터 이상 잠금, 기동 자동 enable
  und_amr_bringup/web_ui.py        웹 화면 문자열 (api_node 재기동해야 바뀜)
  config/nav2.yaml                 Theta* + RPP. 값마다 이유 주석
  test/                            회귀 시험 398개
ros2/patches/                bluesea-ros2(라이다 드라이버) 수정 패치. 드라이버 자체는 ~/ros2_ws/src 에, git 밖
Core/ Drivers/ Makefile      STM32H743 펌웨어 (l2db4830.c = 드라이브·콘솔, can2.c = BMS, lift.c)
tools/                       systemd 진입 스크립트, fw_cmd.py, repeat_probe.py, spin_center.py, run_tests.sh …
tools/analysis/              일회성 측정·분석 스크립트 (휘청 측정 wobrec/wobana3, udpdrop 등)
deploy/                      systemd 유닛, sysctl(UDP 8MB) 정본
docs/                        인수인계, 일일업무보고, 설명서
```

git 밖: `~/ros2_ws/maps/`(지도, 지금 map_1113_1444), `logs/`(bringup.log 등 로그·녹화·`*.bak_*` 백업), `img/`.

## 3. 자주 쓰는 명령

```bash
~/projects/UND_AMR/tools/run_tests.sh                 # 고친 뒤 항상. 로봇·ROS 필요 없음 (~42초)

# api_node 만 재시작 (SLAM 은 살아 있어 위치 유지). 준비까지 14~109초
kill -INT $(pgrep -f "^/usr/bin/python3 .*lib/und_amr_bringup/api_node")
curl -s 127.0.0.1:8090/nav/status        # available true 확인
curl -s 127.0.0.1:8090/pois              # 충전소1 보이면 준비 끝

# nav2 만 재시작 (systemd 가 15~25초 뒤 다시 띄움). SIGINT 는 무시됨
kill -TERM $(pgrep -f "^/usr/bin/python3 /opt/ros/humble/bin/ros2 launch und_amr_bringup nav2.launch.py")

python3 tools/fw_cmd.py q                              # 펌웨어 콘솔 (CRC 자동)
grep -E "이동 종료|실패|모터 이상|TF 낡음" logs/bringup.log | tail -40
```

- symlink 설치라 기존 파일 수정은 노드 재시작만으로 반영된다. 새 파일·setup.py 변경 때만 `cd ~/ros2_ws && colcon build --symlink-install --packages-select und_amr_bringup`.
- **`pkill -f` 금지.** 자기 셸 명령줄까지 잡아 셸이 죽는다. `pgrep -f "^..."` 로 앞을 고정해 pid 를 찾는다.
- URDF 를 실행 중에 바꿀 때는 robot_state_publisher SetParameters 서비스를 쓴다(`ros2 param set` 은 xacro 가 깨진다).
- 수동 위치 고정(`/localization/manual`)은 SLAM 을 pause 시킨다. 풀 때는 `/localization/release`.
- POI 1 에서 위치를 잃었을 때: `POST /localization/manual {"x":4.1744,"y":-1.9278,"yaw":-0.35080}`.
- ROS_DOMAIN_ID=42. 스캔 구독은 `qos_profile_sensor_data`, 출력은 `python3 -u`.

## 4. 코드를 고칠 때

- 값 옆 주석에 실측 근거가 있다. 되돌리기 전에 읽는다. 시험이 깨지면 "값이 나빠졌다"는 뜻이다. 시험을 고치기 전에 실물을 다시 잰다.
- 주기적으로 불리는 함수(`_np_tick`, `np_handoff` 등)는 시험에서 두 번 이상 불러 본다. 예전에 넘겨받기가 0.05초마다 재실행된 버그가 있었다.
- 고친 뒤에는 run_tests.sh 를 돌린다. 이어서 같은 경로로 전후를 비교한다(1↔2 3회 + 1↔충전소1 3회).
- 휘청을 잴 때는 카메라 자이로를 0.3~3Hz 로 거른다. 정속 흔들림의 89% 가 넥 거치대의 8~25Hz 진동이다.
- 제자리 회전을 여러 번 시험한 뒤에는 위치 점수(`/nav/status` loc_mon)를 확인한다. ±160° 12회 뒤 106mm 틀어진 적이 있다.
- TF·odom 이 api_node 에서만 늦으면 UDP 버림부터 본다: `/proc/net/snmp` 의 RcvbufErrors, `/proc/net/udp` 의 drops.

## 5. 해 봤지만 안 된 것 (반복하지 말 것)

Smac Lattice/Hybrid 플래너, 경유점으로 축 위 올리기, 제자리 회전 지령 모양 바꾸기, 드라이브 속도/위치 고리 이득, 도착 축 앞먹임(SP_AXIS_FF), 출발 서서히 구간 단축, 경로 스무더, 지도 이어 그리기, 전역 footprint 원형, 자이로 받아치기, RPP min_lookahead 0.8.
이유와 수치는 인수인계 9장에 있다.

## 6. 지금 남은 일 (2026-11-23 기준)

1. 전원 재시작 뒤 점검: 모든 노드 UDP 버림 0, api_node 기동 시간, 드라이브 자동 enable
2. POI 1 도착 +20mm 이상값 재발 여부
3. 노드바이노드 설계안 (계획 먼저)
4. 축1(오른쪽) 모터 엔코더 알람 HW 점검 뒤 렉 운영 흐름 시험
5. 지도 재작성 (사용자 결정 시)
