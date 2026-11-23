"""주행 SLAM — bringup 위에 slam_toolbox 를 올린다.

    ros2 launch und_amr_bringup slam.launch.py

정지 검증용 slam_test.launch.py 와 달리 가짜 odom TF 를 쓰지 않는다.
실제 오도메트리(EKF 융합)를 받아 움직이면서 맵을 넓힌다.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('und_amr_bringup')
    return LaunchDescription([
        DeclareLaunchArgument('camera', default_value='true'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(share, 'launch', 'bringup.launch.py')),
            launch_arguments={'camera': LaunchConfiguration('camera')}.items()),

        Node(package='slam_toolbox', executable='async_slam_toolbox_node',
             name='slam_toolbox', output='screen',
             parameters=[os.path.join(share, 'config', 'slam.yaml')]),
    ])
