"""웹 관리자 페이지 한 번에 띄우기.

    ros2 launch und_amr_bringup admin.launch.py
    → 브라우저에서 http://localhost:8090

구성
    라이다 + 카메라 + 구동부 + TF + EKF + SLAM + 웹 관리자

옵션
    camera:=false   카메라 없이 (USB 미연결 시)
    base:=false     구동부 없이 (STM32 미연결 시)
    slam:=true      SLAM 을 launch 가 함께 띄움 (기본 false;
                    기본값에서는 웹의 [새 매핑 시작] 이 SLAM 을 기동한다)
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('und_amr_bringup')
    slam = LaunchConfiguration('slam')

    return LaunchDescription([
        DeclareLaunchArgument('camera', default_value='true'),
        DeclareLaunchArgument('base', default_value='true'),
        # SLAM 은 api_node 가 직접 띄우고 내린다(백지 재시작을 위해).
        # 여기서 같이 띄우면 '새 매핑 시작' 이 그 노드를 못 내린다.
        DeclareLaunchArgument('slam', default_value='false'),
        DeclareLaunchArgument('api_port', default_value='8090'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(share, 'launch', 'bringup.launch.py')),
            launch_arguments={
                'camera': LaunchConfiguration('camera'),
                'base': LaunchConfiguration('base'),
                'api': 'true',
                'api_port': LaunchConfiguration('api_port'),
            }.items()),

        Node(package='slam_toolbox', executable='async_slam_toolbox_node',
             name='slam_toolbox', output='screen',
             condition=IfCondition(slam),
             parameters=[os.path.join(share, 'config', 'slam.yaml')]),

        # 구동부를 끄면 odom TF 가 없어 SLAM 이 못 돈다. 자리를 채워준다.
        Node(package='tf2_ros', executable='static_transform_publisher',
             name='fake_odom',
             condition=UnlessCondition(LaunchConfiguration('base')),
             arguments=['0','0','0','0','0','0','odom','base_footprint']),
    ])
