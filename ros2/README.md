# ros2/ — ROS2 패키지

로봇 PC 에서는 `~/ros2_ws/src/und_amr_bringup` 이 이 폴더의 `und_amr_bringup` 을 가리키는 심볼릭 링크다(colcon 은 링크를 따라간다).
새 PC 에서는 `ln -s ~/projects/UND_AMR/ros2/und_amr_bringup ~/ros2_ws/src/und_amr_bringup`.

| 패키지 | 내용 |
|---|---|
| `und_amr_bringup` | 우리 패키지. api_node(HTTP 8090·웹·주행 흐름), base_node(STM32 시리얼), 충전소 도킹, 직선 POI, 라이다 합치기 등 |
| `bluesea-ros2` | LDS-50C-E-12 라이다 드라이버(원본 클론, `~/ros2_ws/src` 에 둠, git 밖). 수정은 `patches/bluesea-ros2_bind_iface.patch`: `LIDAR_BIND_IFACE`(SO_BINDTODEVICE) 13줄. 아래 복원 참고 |

**처음이면 `../docs/인수인계_2026-11-23.md` 부터 읽을 것.**

```bash
cd ~/ros2_ws && colcon build --symlink-install     # 반드시 ~/ros2_ws 에서
~/projects/UND_AMR/tools/run_tests.sh              # 회귀 시험 (로봇·ROS 없이)
```

symlink 설치라 이미 있는 파일 수정은 해당 노드 재시작만으로 반영된다. 새 파일·setup.py 변경 때만 다시 빌드.
지도(`~/ros2_ws/maps/`)와 설정(`~/ros2_ws/und_amr_settings.json`, `battery_history.json`)은 저장소 밖이다.

## bluesea-ros2 복원

저장소에는 원본 대신 기준 커밋과 패치만 둔다.

```bash
cd ~/ros2_ws/src
git clone https://github.com/BlueSeaLidar/bluesea-ros2.git
cd bluesea-ros2 && git checkout 206ade9 && git apply ~/projects/UND_AMR/ros2/patches/bluesea-ros2_bind_iface.patch
```
