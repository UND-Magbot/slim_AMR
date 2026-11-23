#!/usr/bin/env bash
# STM32H743 개발환경 셋업 (Ubuntu/Debian 계열)
# 사용법:  bash setup_linux_stm32.sh
set -e

echo "===== 0. 시스템 정보 ====="
. /etc/os-release && echo "$PRETTY_NAME / $(uname -srm)"

echo "===== 1. 패키지 설치 ====="
sudo apt update
sudo apt install -y \
  gcc-arm-none-eabi binutils-arm-none-eabi libnewlib-arm-none-eabi \
  gdb-multiarch make cmake git \
  openocd stlink-tools \
  minicom picocom usbutils

echo "===== 2. 버전 확인 ====="
arm-none-eabi-gcc --version | head -1
make --version | head -1
openocd --version 2>&1 | head -1
st-info --version 2>/dev/null || true

echo "===== 3. ST-LINK udev 규칙 (sudo 없이 플래싱) ====="
if [ ! -f /etc/udev/rules.d/49-stlinkv2.rules ]; then
  sudo tee /etc/udev/rules.d/49-stlinkv2.rules >/dev/null <<'RULES'
# ST-LINK/V2
SUBSYSTEM=="usb", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="3748", MODE="0666", GROUP="plugdev"
# ST-LINK/V2-1
SUBSYSTEM=="usb", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="374b", MODE="0666", GROUP="plugdev"
# ST-LINK/V3
SUBSYSTEM=="usb", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="374f", MODE="0666", GROUP="plugdev"
SUBSYSTEM=="usb", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="3753", MODE="0666", GROUP="plugdev"
SUBSYSTEM=="usb", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="3754", MODE="0666", GROUP="plugdev"
RULES
  sudo udevadm control --reload-rules && sudo udevadm trigger
  echo "udev 규칙 설치 완료"
else
  echo "udev 규칙 이미 존재 - 건너뜀"
fi

echo "===== 4. 시리얼 포트 권한 (USB CDC / RS485 확인용) ====="
sudo usermod -aG dialout,plugdev "$USER"
echo "  ※ 그룹 반영은 재로그인 후 적용됩니다."

echo ""
echo "===== 완료 ====="
echo "다음 단계:"
echo "  1) 보드를 USB로 연결 후:  lsusb | grep -i -E 'stm|st-link|segger'"
echo "  2) 소스 압축 해제 위치로 이동 후:  make -j\$(nproc)"
echo "  3) 플래싱:  make flash"
