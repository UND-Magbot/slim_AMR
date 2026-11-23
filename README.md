# slim_AMR (UND_AMR)

2륜 차동구동 AMR — ROS2 패키지, STM32H743 펌웨어, 시험·분석 도구, 문서.

**처음이면 [docs/인수인계_2026-11-23.md](docs/인수인계_2026-11-23.md) 부터 읽을 것.** Claude Code 로 작업한다면 [CLAUDE.md](CLAUDE.md) 가 자동으로 읽힌다.

| 폴더 | 내용 |
|---|---|
| `ros2/und_amr_bringup/` | ROS2 패키지 (api_node, base_node, 충전소 도킹, 직선 POI …). 로봇 PC 에서는 `~/ros2_ws/src/und_amr_bringup` 이 여기를 가리키는 링크 |
| `ros2/patches/` | 라이다 드라이버(bluesea-ros2) 수정 패치 |
| `Core/`, `Drivers/`, `Middlewares/`, `USB_DEVICE/`, `Makefile`, `UND_ARM.ioc` | STM32 펌웨어 (CubeMX + Makefile). `make`, 굽기 `make dfu` |
| `tools/` | 기동 스크립트(systemd 가 호출), 펌웨어 콘솔(`fw_cmd.py`), 측정 도구, 회귀 시험(`run_tests.sh`) |
| `tools/analysis/` | 일회성 측정·분석 스크립트 모음 |
| `deploy/` | systemd 유닛, sysctl 설정 정본 |
| `docs/` | 인수인계(md·pptx), 설명서 |

지도(`~/ros2_ws/maps/`)와 로그(`logs/`)는 git 밖이다.
