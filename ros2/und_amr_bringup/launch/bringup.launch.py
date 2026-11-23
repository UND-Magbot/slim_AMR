"""UND_AMR 전체 브링업 — 구동부 + 라이다 + 카메라 + TF + EKF

    ros2 launch und_amr_bringup bringup.launch.py
    ros2 launch und_amr_bringup bringup.launch.py camera:=false
    ros2 launch und_amr_bringup bringup.launch.py ekf:=false

EKF 를 쓰면 odom→base_footprint TF 를 ekf_node 가 발행하므로
base_node 의 publish_odom_tf 를 false 로 넘긴다. 둘 다 발행하면
TF 트리에 부모가 둘인 상태가 되어 경고와 함께 좌표가 튄다.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('und_amr_bringup')
    use_camera = LaunchConfiguration('camera')
    use_ekf = LaunchConfiguration('ekf')
    use_base = LaunchConfiguration('base')

    return LaunchDescription([
        DeclareLaunchArgument('camera', default_value='true'),
        DeclareLaunchArgument('ekf', default_value='true'),
        DeclareLaunchArgument('base', default_value='true'),
        DeclareLaunchArgument('api', default_value='true'),
        DeclareLaunchArgument('api_port', default_value='8090'),

        # URDF → TF
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(share, 'launch', 'description.launch.py'))),

        # 라이다
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(share, 'launch', 'lidar.launch.py'))),

        # 카메라
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(share, 'launch', 'camera.launch.py')),
            condition=IfCondition(use_camera)),

        # 구동부. EKF 가 켜져 있으면 TF 발행을 EKF 에 넘긴다
        # respawn: USB 가 순간 끊기면 시리얼 쓰기에서 죽는다(Errno 5).
        # 그대로 두면 주행 중 지령이 끊긴 채 방치되므로 자동으로 되살린다.
        Node(package='und_amr_bringup', executable='base_node',
             name='base_node', output='screen',
             respawn=True, respawn_delay=3.0,
             condition=IfCondition(use_base),
             parameters=[
                 os.path.join(share, 'config', 'amr_params.yaml'),
                 {'publish_odom_tf': PythonExpression(
                     ["'false' if '", use_ekf, "' == 'true' else 'true'"])},
             ]),

        # 웹 관리자 페이지 + REST API (http://<로봇 IP>:8090)
        # respawn: 이 노드가 죽으면 화면만 끊기는 게 아니라 지도와 SLAM 까지
        # 같이 사라진다(SLAM 을 자식 프로세스로 들고 있다). 되살아나면
        # 마지막 메인 맵을 복구하므로 자동 재시작을 건다.
        Node(package='und_amr_bringup', executable='api_node',
             name='api_node', output='screen',
             respawn=True, respawn_delay=3.0,
             condition=IfCondition(LaunchConfiguration('api')),
             parameters=[{'port': ParameterValue(
                 LaunchConfiguration('api_port'), value_type=int)}]),

        # RealSense IMU 어댑터. EKF 는 이 노드를 거친 /imu/base_link 를 쓴다.
        # 원본 IMU 는 쿼터니언이 무효하고 프레임이 광학좌표계라 EKF 가
        # 각속도를 반영하지 못한다 (2026-08-26 확인). camera 를 끄면 같이 꺼진다.
        Node(package='und_amr_bringup', executable='imu_adapter',
             name='imu_adapter', output='screen',
             respawn=True, respawn_delay=3.0,
             condition=IfCondition(use_camera)),

        # 카메라 깊이 -> 장애물 스캔(/camera/obstacle_scan). 라이다 높이 밖의 물건을
        # nav2 costmap 에 넣는다(2026-10-27). camera 를 끄면 같이 꺼진다.
        # 2026-11-11: 넥 위 카메라 2대. 스캔은 카메라 바로 밑 수평 프레임
        # (camera_obstacle_frame)에서 낸다. 둘 다 collision_monitor 로 들어가 바로 감속 · 정지
        # (nav2.yaml). 15Hz (every 2).
        # 아래 D435F (65.6도 숙임): 로봇 앞 바닥 ~ 0.6m 높이. 바로 밑 좌우까지 보여 180도.
        Node(package='und_amr_bringup', executable='camera_obstacles',
             name='camera_obstacles', output='screen',
             respawn=True, respawn_delay=3.0,
             parameters=[{
                 'depth_topic': '/camera_down/camera_down/depth/image_rect_raw',
                 'info_topic': '/camera_down/camera_down/depth/camera_info',
                 'scan_frame': 'camera_obstacle_frame',
                 'fov_deg': 180.0,
                 'every': 2,
             }],
             condition=IfCondition(use_camera)),
        # 정면 D435i (1.0m, 수평): 아래 카메라 위로 지나가는 높이(손 · 책상 · 선반).
        # ⚠ 사용자: "대각선 아래카메라로는 멈추던데 정면카메라는 손가까이가도 안멈추고"
        #   아래 카메라 위쪽 끝은 0.5m 앞에서 높이 0.62m, 1m 앞 0.29m 까지라 그 위는 못 본다.
        Node(package='und_amr_bringup', executable='camera_obstacles',
             name='camera_obstacles_front', output='screen',
             respawn=True, respawn_delay=3.0,
             parameters=[{
                 'depth_topic': '/camera/camera/depth/image_rect_raw',
                 'info_topic': '/camera/camera/depth/camera_info',
                 'out_topic': '/camera/obstacle_scan_front',
                 'scan_frame': 'camera_obstacle_frame',
                 'every': 2,
             }],
             condition=IfCondition(use_camera)),

        # 대차(렉) 검출. 라이다로 다리 네 개를 찾고 카메라로 확인한다.
        # 라이다 처리라 부하가 작아 항상 띄워 두고, 화면에서 켜고 끈다.
        Node(package='und_amr_bringup', executable='cart_detector',
             name='cart_detector', output='screen',
             respawn=True, respawn_delay=3.0,
             parameters=[os.path.join(share, 'config', 'carts.yaml')]),

        Node(package='robot_localization', executable='ekf_node',
             name='ekf_filter_node', output='screen',
             condition=IfCondition(use_ekf),
             respawn=True, respawn_delay=3.0,
             parameters=[os.path.join(share, 'config', 'ekf.yaml')]),
    ])
