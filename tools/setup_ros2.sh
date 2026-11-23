#!/usr/bin/env bash
# UND_AMR — ROS2 Humble 개발환경 구축 (Ubuntu 22.04 jammy)
#
#   bash tools/setup_ros2.sh
#
# 여러 번 실행해도 안전하다(이미 된 단계는 건너뜀).
set -e

WS="$HOME/ros2_ws"
ROS_DISTRO=humble

say() { echo; echo "===== $* ====="; }

say "0. 환경 확인"
. /etc/os-release
echo "  $PRETTY_NAME / $(uname -m)"
if [ "$VERSION_CODENAME" != "jammy" ]; then
  echo "  경고: 이 스크립트는 Ubuntu 22.04(jammy) 기준입니다."
fi

say "1. 로케일"
if ! locale | grep -q "UTF-8"; then
  sudo apt update
  sudo apt install -y locales
  sudo locale-gen en_US en_US.UTF-8
  sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
else
  echo "  UTF-8 이미 설정됨"
fi

say "2. ROS2 apt 저장소 등록"
if [ ! -f /etc/apt/sources.list.d/ros2.list ]; then
  sudo apt install -y software-properties-common curl gnupg lsb-release
  sudo add-apt-repository -y universe
  sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $VERSION_CODENAME main" \
    | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
  sudo apt update
else
  echo "  이미 등록됨"
fi

say "3. ROS2 $ROS_DISTRO 본체 설치 (수 GB, 시간 걸립니다)"
sudo apt install -y \
  ros-$ROS_DISTRO-desktop \
  ros-dev-tools \
  python3-colcon-common-extensions \
  python3-rosdep

say "4. AMR 용 추가 패키지"
#  slam-toolbox      : 2D SLAM (맵 작성)
#  nav2-bringup      : 자율주행 스택 (나중 단계)
#  realsense2-camera : D435i ROS 드라이버 (librealsense 포함)
#  robot-localization: 휠 오도메트리 + D435i IMU 융합 (헤딩 드리프트 보정)
#  teleop-*          : 키보드 수동 주행
sudo apt install -y \
  ros-$ROS_DISTRO-slam-toolbox \
  ros-$ROS_DISTRO-nav2-bringup \
  ros-$ROS_DISTRO-navigation2 \
  ros-$ROS_DISTRO-realsense2-camera \
  ros-$ROS_DISTRO-robot-localization \
  ros-$ROS_DISTRO-teleop-twist-keyboard \
  ros-$ROS_DISTRO-tf2-tools \
  ros-$ROS_DISTRO-xacro \
  ros-$ROS_DISTRO-joint-state-publisher \
  ros-$ROS_DISTRO-rqt-graph \
  ros-$ROS_DISTRO-rqt-tf-tree

say "5. rosdep 초기화"
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
  sudo rosdep init
fi
rosdep update || true

say "6. 워크스페이스 생성 : $WS"
mkdir -p "$WS/src"

say "7. 라이다 드라이버 (bluesea-ros2) 내려받기"
if [ ! -d "$WS/src/bluesea-ros2" ]; then
  git clone https://github.com/BlueSeaLidar/bluesea-ros2.git "$WS/src/bluesea-ros2"
else
  echo "  이미 존재 - 건너뜀"
fi

say "8. 워크스페이스 빌드"
set +e
. /opt/ros/$ROS_DISTRO/setup.bash
cd "$WS"
colcon build --symlink-install
BUILD_RC=$?
set -e

say "9. bashrc 설정"
add_line() {
  grep -qxF "$1" "$HOME/.bashrc" || echo "$1" >> "$HOME/.bashrc"
}
add_line "source /opt/ros/$ROS_DISTRO/setup.bash"
add_line "[ -f $WS/install/setup.bash ] && source $WS/install/setup.bash"
add_line "export ROS_DOMAIN_ID=42"
echo "  ~/.bashrc 에 source 구문 추가 완료 (ROS_DOMAIN_ID=42)"

say "완료"
echo "새 터미널을 열거나 아래를 실행하세요:"
echo "    source ~/.bashrc"
echo
echo "확인:"
echo "    ros2 --version"
echo "    ros2 topic list"
if [ $BUILD_RC -ne 0 ]; then
  echo
  echo "주의: 8단계 colcon build 가 실패했습니다. 출력을 확인하세요."
fi
