#!/usr/bin/env bash
# 순수 계산 함수 회귀 시험. 로봇도 ROS 도 필요 없다.
#
#   ./tools/run_tests.sh          전부
#   ./tools/run_tests.sh -k 방향   이름에 '방향' 이 든 것만
#
# 여기 있는 숫자는 대부분 실물에서 재서 확인한 값이다.
# 시험이 깨지면 "값이 나빠졌다" 는 뜻이지 "시험이 낡았다" 가 아니다 —
# 고치기 전에 실물에서 다시 재 볼 것.
set -e
cd "$(dirname "$0")/../../../ros2_ws/src/und_amr_bringup" 2>/dev/null \
  || cd /home/und_amr1/ros2_ws/src/und_amr_bringup
exec python3 -m pytest test/ -q --no-header "$@"
