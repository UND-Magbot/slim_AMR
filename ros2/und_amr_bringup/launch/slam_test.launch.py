"""정지 상태 SLAM 파이프라인 검증.

로봇이 움직이지 않으므로 맵은 커지지 않는다. 확인하려는 것은
    라이다 → /scan → TF → slam_toolbox → /map
이 계통이 끊김 없이 도는지, 프레임 관계가 맞는지다.

STM32 오도메트리 없이 돌리려고 odom→base_footprint 를 고정 변환으로
채운다. 실제 주행 단계에서는 이 static_tf 를 빼고 base_node 가 발행한다.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('und_amr_bringup')

    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(share, 'launch', 'description.launch.py'))),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(share, 'launch', 'lidar.launch.py'))),

        # 오도메트리 자리를 메우는 임시 고정 변환
        Node(package='tf2_ros', executable='static_transform_publisher',
             name='fake_odom',
             arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_footprint']),

        Node(package='slam_toolbox', executable='async_slam_toolbox_node',
             name='slam_toolbox', output='screen',
             parameters=[os.path.join(share, 'config', 'slam.yaml')]),
    ])
