"""nav2 자율주행. bringup.launch.py 가 먼저 떠 있어야 한다.

이 런치는 nav2 의 '주행' 부분만 띄운다. 지도와 위치추정은 우리 쪽이 이미
하고 있으므로 map_server 와 amcl 은 띄우지 않는다.

    지도      api_node 가 /map_nav 로 발행 (가상벽 포함)
    위치추정   slam_toolbox localization 모드가 map->odom 발행
    오도메트리 EKF 가 odom->base_footprint 발행

    ros2 launch und_amr_bringup nav2.launch.py

확인:
    ros2 topic echo /map_nav --once --field info      지도가 오는지
    ros2 run tf2_ros tf2_echo map base_footprint      좌표계가 이어지는지
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PKG = "und_amr_bringup"

# lifecycle 관리 대상. 순서가 중요하다 - costmap 이 뜬 뒤에 그것을 쓰는
# controller/planner 가 뜨고, 마지막에 bt_navigator 가 이들을 부른다.
NODES = [
    "controller_server",
    "planner_server",
    "smoother_server",
    "behavior_server",
    "bt_navigator",
    "velocity_smoother",
    "collision_monitor",
]


def generate_launch_description():
    params = os.path.join(get_package_share_directory(PKG), "config", "nav2.yaml")
    use_sim = LaunchConfiguration("use_sim_time")

    common = [{"use_sim_time": use_sim}, params]

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),

        Node(package="nav2_controller", executable="controller_server",
             name="controller_server", output="screen", parameters=common,
             respawn=True, respawn_delay=3.0,
             # controller_server 가 내는 속도는 velocity_smoother 를 거쳐
             # /cmd_vel 로 나간다. 급가속으로 바퀴가 미끄러지는 것을 막는다.
             remappings=[("cmd_vel", "cmd_vel_nav")]),

        Node(package="nav2_planner", executable="planner_server",
             name="planner_server", output="screen", parameters=common,
             respawn=True, respawn_delay=3.0),

        # 경로 다듬기 (2026-10-26). Theta* 경로는 직선을 이어 붙여 꺾이는 곳이 각지다 —
        # 사용자: "너무 좁게 돌아". 그린 직후 곡선으로 편다 (config/nav_to_pose.xml SmoothPath).
        Node(package="nav2_smoother", executable="smoother_server",
             name="smoother_server", output="screen", parameters=common,
             respawn=True, respawn_delay=3.0),

        Node(package="nav2_behaviors", executable="behavior_server",
             name="behavior_server", output="screen", parameters=common,
             respawn=True, respawn_delay=3.0),

        # 복구에서 제자리 회전·후진을 뺀 트리 (config/nav_to_pose.xml 설명 참고)
        Node(package="nav2_bt_navigator", executable="bt_navigator",
             name="bt_navigator", output="screen",
             parameters=common + [{"default_nav_to_pose_bt_xml": os.path.join(
                 get_package_share_directory(PKG), "config", "nav_to_pose.xml")}],
             respawn=True, respawn_delay=3.0),

        Node(package="nav2_velocity_smoother", executable="velocity_smoother",
             name="velocity_smoother", output="screen", parameters=common,
             respawn=True, respawn_delay=3.0,
             # 여기서 바로 /cmd_vel 로 내보내지 않는다. collision_monitor 를
             # 거쳐야 안전 계층이 실제로 지령을 막을 수 있다.
             remappings=[("cmd_vel", "cmd_vel_nav")]),

        Node(package="nav2_collision_monitor", executable="collision_monitor",
             name="collision_monitor", output="screen", parameters=common,
             respawn=True, respawn_delay=3.0),

        Node(package="nav2_lifecycle_manager", executable="lifecycle_manager",
             name="lifecycle_manager_navigation", output="screen",
             parameters=[{"use_sim_time": use_sim},
                         {"autostart": True},
                         {"node_names": NODES}]),
    ])
