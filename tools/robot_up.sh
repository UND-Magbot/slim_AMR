#!/usr/bin/env bash
# 로봇 전체 기동.
#
#   ./tools/robot_up.sh            켜기 (이미 떠 있으면 껐다 켠다)
#   ./tools/robot_up.sh restart    같음
#   ./tools/robot_up.sh down       끄기
#
# systemd 서비스(und-amr)가 설치돼 있으면 그쪽으로 넘긴다.
# 서비스가 도는데 이 스크립트가 따로 띄우면 스택이 두 벌 돌아 토픽이
# 겹치고 시리얼 포트를 두고 다툰다. 반드시 한 경로로만 띄워야 한다.
#
# /tmp 에 두면 재부팅·정리에 사라져 프로젝트 안에 둔다.
set +u
LOG=/home/und_amr1/projects/UND_AMR/logs/bringup.log
UNIT=und-amr.service

use_systemd() {
  systemctl list-unit-files "$UNIT" >/dev/null 2>&1 && \
  systemctl cat "$UNIT" >/dev/null 2>&1
}

down_manual() {
  for p in $(pgrep -f 'ros2 launch und_amr_bringup bringup'); do kill "$p" 2>/dev/null; done
  sleep 8
  P=$(ps -eo pid,cmd \
      | grep -E '/lib/und_amr_bringup/|robot_state_publisher|slam_toolbox|bluesea2_node|realsense2_camera|ekf_node' \
      | grep -v grep | awk '{print $1}')
  [ -n "$P" ] && kill -9 $P 2>/dev/null
  sleep 4
}

if use_systemd; then
  case "${1:-up}" in
    down)
      sudo systemctl stop "$UNIT"
      echo "정지 요청 $(date +%H:%M:%S) — systemd ($UNIT)"
      ;;
    *)
      sudo systemctl restart "$UNIT"
      echo "기동 요청 $(date +%H:%M:%S) — systemd ($UNIT), 로그: $LOG"
      ;;
  esac
  exit 0
fi

# --- 서비스가 없을 때의 예전 경로 -------------------------------------
down_manual
if [ "${1:-up}" = "down" ]; then
  echo "정지 $(date +%H:%M:%S)"
  exit 0
fi
setsid nohup bash -c '
  source /opt/ros/humble/setup.bash
  source /home/und_amr1/ros2_ws/install/setup.bash
  exec ros2 launch und_amr_bringup bringup.launch.py camera:=true ekf:=true base:=true
' > "$LOG" 2>&1 < /dev/null &
echo "기동 요청 $(date +%H:%M:%S) — 로그: $LOG"
