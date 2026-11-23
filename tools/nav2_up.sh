#!/usr/bin/env bash
# nav2 자율주행 기동.
#
#   ./tools/nav2_up.sh          켜기 (이미 떠 있으면 껐다 켠다)
#   ./tools/nav2_up.sh down     끄기
#
# systemd 서비스(und-amr-nav2)가 설치돼 있으면 그쪽으로 넘긴다.
# 그 서비스는 und-amr 에 PartOf 로 묶여 있어, bringup 을 껐다 켜면
# nav2 도 같이 따라간다. 예전에는 사람이 순서를 기억해야 했고 실제로
# 여러 번 잊어 목표가 거절됐다(2026-09-11).
#
# 패턴을 셸 명령줄에 직접 쓰면 pgrep 이 자기 셸을 잡으므로 파일로 둔다.
set +u
LOG=/home/und_amr1/projects/UND_AMR/logs/nav2.log
UNIT=und-amr-nav2.service

use_systemd() {
  systemctl cat "$UNIT" >/dev/null 2>&1
}

down_manual() {
  PAT='ros2 launch und_amr_bringup nav2'
  for p in $(pgrep -f "$PAT"); do kill "$p" 2>/dev/null; done
  sleep 3
  P=$(ps -eo pid,cmd \
      | grep -E 'controller_server|planner_server|behavior_server|bt_navigator|velocity_smoother|collision_monitor|lifecycle_manager_navigation' \
      | grep -v grep | awk '{print $1}')
  [ -n "$P" ] && kill -9 $P 2>/dev/null
  sleep 2
}

if use_systemd; then
  case "${1:-up}" in
    down)
      sudo systemctl stop "$UNIT"
      echo "nav2 정지 $(date +%H:%M:%S) — systemd ($UNIT)"
      ;;
    *)
      sudo systemctl restart "$UNIT"
      echo "nav2 기동 요청 $(date +%H:%M:%S) — systemd ($UNIT), 로그: $LOG"
      ;;
  esac
  exit 0
fi

# --- 서비스가 없을 때의 예전 경로 -------------------------------------
down_manual
if [ "${1:-up}" = "down" ]; then
  echo "nav2 정지 $(date +%H:%M:%S)"
  exit 0
fi
source /opt/ros/humble/setup.bash
source /home/und_amr1/ros2_ws/install/setup.bash
setsid nohup ros2 launch und_amr_bringup nav2.launch.py > "$LOG" 2>&1 < /dev/null &
echo "nav2 기동 요청 $(date +%H:%M:%S) — 로그: $LOG"
