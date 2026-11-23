#!/usr/bin/env bash
# systemd 가 직접 실행하는 nav2 진입점.
#
# systemd 의 After= 는 '시작 명령을 먼저 낸다' 는 뜻일 뿐, 앞 서비스가
# 쓸 준비가 됐다는 보장은 아니다. nav2 는 뜨는 순간 map->base_footprint
# 를 잡아야 하는데, 그게 아직 없으면 costmap 이 옛 시각을 물고 굳어
# 목표를 거절한다(2026-09-11 에 여러 번 겪음).
# 그래서 위치추정이 실제로 붙을 때까지 기다린 뒤에 띄운다.
set +u
source /opt/ros/humble/setup.bash
source /home/und_amr1/ros2_ws/install/setup.bash

echo "[nav2] 위치추정이 붙기를 기다리는 중..."
for i in $(seq 120); do
  if curl -s -m 2 http://127.0.0.1:8090/chassis/pose 2>/dev/null \
     | python3 -c 'import sys,json;sys.exit(0 if json.load(sys.stdin).get("map") else 1)' 2>/dev/null; then
    echo "[nav2] 위치추정 확인 (${i}회째). nav2 를 띄운다"
    ros2 launch und_amr_bringup nav2.launch.py &
    LPID=$!
    # ⚠ 시작 감시 (2026-10-02). 노드는 설정을 마쳤는데 change_state 응답이
    #   lifecycle_manager 에 전달되지 않아('failed to send response ... timeout')
    #   nav2 가 반쯤 뜬 채 영영 멈춘 일이 하루에 두 번 있었다. 전원을 켤 때 걸리면
    #   자율주행이 안 올라온다. 120초 안에 is_active 가 참이 아니면 내리고 끝낸다 —
    #   systemd(Restart=always, 10초)가 다시 띄운다.
    for j in $(seq 40); do
      sleep 3
      if ! kill -0 "$LPID" 2>/dev/null; then
        wait "$LPID"; exit $?
      fi
      if timeout 5 ros2 service call /lifecycle_manager_navigation/is_active \
           std_srvs/srv/Trigger 2>/dev/null | grep -q "success=True"; then
        echo "[nav2] 모두 활성 확인 ($((j * 3))초)"
        wait "$LPID"; exit $?
      fi
    done
    echo "[nav2] 120초 안에 다 올라오지 않았다 — 내리고 다시 띄운다"
    kill -TERM "$LPID" 2>/dev/null      # 백그라운드 작업은 INT 를 무시한다
    for k in $(seq 15); do kill -0 "$LPID" 2>/dev/null || break; sleep 1; done
    kill -KILL "$LPID" 2>/dev/null
    exit 1
  fi
  sleep 2
done
echo "[nav2] 240초 동안 위치추정이 안 붙었다. 띄우지 않고 물러난다"
exit 1
