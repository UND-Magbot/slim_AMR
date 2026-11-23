"""URDF 를 읽어 TF(base_link→laser, →camera_link ...) 를 발행한다."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    share = get_package_share_directory('und_amr_bringup')
    urdf = os.path.join(share, 'urdf', 'amr.urdf.xacro')
    robot_desc = xacro.process_file(urdf).toxml()

    return LaunchDescription([
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             output='screen',
             parameters=[{'robot_description': robot_desc,
                          'use_sim_time': False}]),
    ])
