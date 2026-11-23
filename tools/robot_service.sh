#!/usr/bin/env bash
# systemd 가 직접 실행하는 진입점.
#
# robot_up.sh 와 다른 점은 '배경으로 보내지 않는다' 는 것뿐이다.
# setsid nohup 으로 떼어 놓으면 systemd 가 프로세스를 놓쳐 재시작도
# 종료도 제대로 못 한다. exec 로 이 셸을 그대로 대체해 systemd 가
# ros2 launch 를 직접 쥐게 한다.
#
# 손으로 띄울 때는 robot_up.sh 를 쓴다. 이 파일은 systemd 전용이다.
set +u          # ROS setup.bash 가 미설정 변수를 참조한다
source /opt/ros/humble/setup.bash
source /home/und_amr1/ros2_ws/install/setup.bash
exec ros2 launch und_amr_bringup bringup.launch.py \
     camera:=true ekf:=true base:=true
