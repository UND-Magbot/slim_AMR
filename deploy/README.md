# 배포 설정 정본

새 PC 에 설치할 때:

```bash
ln -s ~/projects/UND_AMR/ros2/und_amr_bringup ~/ros2_ws/src/und_amr_bringup   # ROS 패키지 링크
sudo cp systemd/und-amr.service systemd/und-amr-nav2.service /etc/systemd/system/
sudo cp sysctl/60-ros-udp.conf /etc/sysctl.d/ && sudo sysctl --system
sudo systemctl daemon-reload && sudo systemctl enable und-amr.service und-amr-nav2.service
```

- `60-ros-udp.conf` (UDP 수신 버퍼 8MB)는 빼면 안 된다. 기본값(208KB)이면 api_node 가 카메라 영상 때문에 UDP 를 버리고 TF 가 멎어 출발 중 멈춘다(2026-11-23).
- 라이다 2대는 같은 IP(192.168.158.98)라 랜 포트(eno1, enp2s0)로 구분한다. nmcli 연결 `lidar2`(eno1)는 `ipv4.route-metric 200`.
- 사용자 계정은 `dialout`(시리얼), `audio`(AUX 스피커) 그룹이 필요하다.
