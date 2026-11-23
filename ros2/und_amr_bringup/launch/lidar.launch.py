"""라이다 단독 실행.

bluesea2 드라이버는 노드만 띄우면 데이터가 안 나온다.
라이다가 'LSTARH' 를 받아야 스트리밍을 시작하므로,
노드 기동 뒤 bluesea2_client 로 start 를 호출해준다.

2026-10-23 라이다 2대 (앞 왼쪽 · 뒤 오른쪽 대각선). 둘 다 공장 IP 192.168.158.98
이라 IP 로는 못 가른다. 각자 다른 랜 포트(enp2s0 / eno1)에 꽂혀 있으므로 드라이버를
랜 포트별로 하나씩 띄운다 (bluesea SDK 에 넣은 LIDAR_BIND_IFACE, SO_BINDTODEVICE).
eno1 은 nmcli 연결 'lidar2' (192.168.158.16/24) 가 있어야 데이터가 들어온다.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('und_amr_bringup')
    params = os.path.join(share, 'config', 'lidar.yaml')
    params2 = os.path.join(share, 'config', 'lidar2.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=params),

        # 이 드라이버는 스트리밍 시작 직후 'drop packet' 을 몇 건 뱉는데
        # 동작에는 지장이 없다. WARN 이상만 남겨 로그를 깨끗하게 유지한다.
        # respawn: 드라이버가 죽으면 자동으로 다시 띄운다. 실제로 노드가
        # 죽은 채 방치돼 지도가 안 그려지는 일이 있었다.
        Node(package='bluesea2', executable='bluesea2_node',
             name='bluesea_node', output='screen',
             respawn=True, respawn_delay=3.0,
             additional_env={'LIDAR_BIND_IFACE': 'enp2s0'},
             arguments=['--ros-args', '--log-level', 'bluesea_node:=warn'],
             parameters=[LaunchConfiguration('params_file')]),
        # 라이다 2 (eno1, 뒤 오른쪽) -> /scan_rear. 라이다 1 (enp2s0, 앞 왼쪽) 은 /scan_front.
        Node(package='bluesea2', executable='bluesea2_node',
             name='bluesea_node2', output='screen',
             respawn=True, respawn_delay=3.0,
             additional_env={'LIDAR_BIND_IFACE': 'eno1'},
             arguments=['--ros-args', '--log-level', 'bluesea_node2:=warn'],
             parameters=[params2]),

        # 두 라이다를 360도 스캔 하나로 합친다: /scan_front + /scan_rear -> /scan (frame laser)
        Node(package='und_amr_bringup', executable='scan_merger',
             name='scan_merger', output='screen',
             respawn=True, respawn_delay=2.0),

        # 드라이버가 소켓을 열 시간을 준 뒤 스트리밍 시작
        TimerAction(period=3.0, actions=[
            ExecuteProcess(
                cmd=['ros2', 'run', 'bluesea2', 'bluesea2_client', 'scan_rear', 'start'],
                output='screen'),
            ExecuteProcess(
                cmd=['ros2', 'run', 'bluesea2', 'bluesea2_client', 'scan_front', 'start'],
                output='screen'),
        ]),
    ])
