"""RealSense 2대 (2026-11-11): 정면 D435i(IMU 를 켜서 EKF 융합에 쓴다) + 아래 D435F.

unite_imu_method:=2 (linear_interpolation) 로 자이로/가속도를 하나의
sensor_msgs/Imu 토픽(/camera/camera/imu)으로 합친다. 이게 없으면
gyro/accel 이 따로 나와 robot_localization 이 못 쓴다.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    rs_share = get_package_share_directory('realsense2_camera')
    rs_launch = os.path.join(rs_share, 'launch', 'rs_launch.py')
    return LaunchDescription([
        # 정면 D435i: IMU(EKF) · 렉 인식 확인 · 웹 영상. 토픽 /camera/camera/...
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rs_launch),
            launch_arguments={
                # ⚠ 2026-11-11 카메라가 2대가 되자 serial 없이 띄운 노드가 IMU 없는
                #   D435F 를 먼저 잡아 EKF 가 자이로 없이 돌았다. 시리얼로 고정한다.
                'serial_no': "'243322071697'",
                'enable_gyro': 'true',
                'enable_accel': 'true',
                'unite_imu_method': '2',
                'enable_sync': 'true',
                'align_depth.enable': 'true',
                'rgb_camera.color_profile': '640,480,30',
                'depth_module.depth_profile': '640,480,30',
                # URDF 의 camera_link 와 이름을 맞춰 TF 가 이어지게 한다
                'camera_name': 'camera',
                'camera_namespace': 'camera',
                'publish_tf': 'true',
            }.items()),
        # 아래 D435F (2026-11-11, 넥 위 65.6도 숙임): 가까운 장애물 확인(camera_obstacles)
        # + 화면 영상(컬러, 2026-11-13). 토픽 /camera_down/camera_down/...
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rs_launch),
            launch_arguments={
                'serial_no': "'406122070374'",
                # 2026-11-13 컬러 켬 (화면에 두 카메라를 같이 보려고). USB 대역을 아끼려 15Hz.
                'enable_color': 'true',
                'rgb_camera.color_profile': '640,480,15',
                'enable_gyro': 'false',
                'enable_accel': 'false',
                'depth_module.depth_profile': '640,480,30',
                'camera_name': 'camera_down',
                'camera_namespace': 'camera_down',
                'publish_tf': 'true',
            }.items()),
    ])
