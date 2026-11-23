#!/usr/bin/env python3
"""RealSense IMU 어댑터 — robot_localization 이 쓸 수 있는 형태로 바꿔 재발행.

realsense2_camera 가 unite_imu_method 로 내보내는 /camera/camera/imu 는
robot_localization(EKF) 이 그대로 쓰지 못한다. 두 가지가 걸린다.

  1) orientation 이 (0,0,0,0) 인 무효 쿼터니언이다.
     공분산을 -1 로 두어 "자세 없음"을 규약대로 표시하고는 있지만,
     쿼터니언 자체가 노름 0 이라 필터가 메시지를 통째로 버린다.

  2) frame_id 가 camera_imu_optical_frame 이라 EKF 가 TF 로 base_link 까지
     변환해야 하는데, 이 경로에서 각속도가 반영되지 않았다.

그래서 여기서 미리
  - 쿼터니언을 단위값(0,0,0,1)으로 채우고 (공분산 -1 은 유지하므로 EKF 는
    자세를 쓰지 않는다. 각속도만 쓴다)
  - 축을 base_link 기준으로 돌리고 frame_id 를 base_link 로 바꾼다
서 내보낸다.

  base <- optical 회전행렬 = [[0,0,1],[-1,0,0],[0,-1,0]]
      optical x -> base -y,   optical y -> base -z,   optical z -> base x
  (URDF 의 camera_imu_optical_frame 이 base_link 대비 rpy(-90,0,-90) 인 데서 나옴)

실측 2026-08-26 검증: 제자리 약 100도 회전에서
    라이다 정답 +99.65도 / 이 노드를 거친 EKF +101.48도 (오차 1.8도)
    원본 IMU 를 그대로 준 EKF 는 +0.00도, yaw 공분산 1.3e7 로 발산
"""
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu


class ImuAdapter(Node):

    def __init__(self):
        super().__init__("imu_adapter")
        self.declare_parameter("in_topic", "/camera/camera/imu")
        self.declare_parameter("out_topic", "/imu/base_link")
        self.declare_parameter("target_frame", "base_link")
        self.declare_parameter("rotate_to_base", True)
        # 정지 중에 자이로 치우침(바이어스)을 추정해 빼줄지.
        # 이걸 안 하면 EKF 가 치우친 각속도를 계속 적분해서, 로봇이 멈춰
        # 있어도 방향이 흘러간다. 실측 -0.11 deg/s → 1분에 6.6도 어긋났다.
        self.declare_parameter("estimate_bias", True)

        self.tgt = self.get_parameter("target_frame").value
        self.rotate = bool(self.get_parameter("rotate_to_base").value)
        self.est_bias = bool(self.get_parameter("estimate_bias").value)
        self.bias = [0.0, 0.0, 0.0]
        self.bias_n = 0
        self.still_since = None
        self.moving = True
        src = self.get_parameter("in_topic").value
        dst = self.get_parameter("out_topic").value

        self.pub = self.create_publisher(Imu, dst, qos_profile_sensor_data)
        self.create_subscription(Imu, src, self.on_imu, qos_profile_sensor_data)
        self.create_subscription(Odometry, "/odom", self.on_odom, 10)
        self.n = 0
        self.bad_q = 0
        self.create_timer(10.0, self.report)
        self.get_logger().info(
            "IMU 어댑터: %s -> %s (축변환 %s, frame_id %s)"
            % (src, dst, "적용" if self.rotate else "안함", self.tgt))

    def on_odom(self, m):
        """바퀴 오도메트리로 '지금 서 있는가' 를 판단한다.

        움직이는 동안 모은 값으로 바이어스를 잡으면 실제 회전이 섞인다.
        멈춘 직후에도 흔들림이 남아 있어 1초 지난 뒤부터 모은다."""
        v = abs(m.twist.twist.linear.x)
        w = abs(m.twist.twist.angular.z)
        if v < 0.02 and w < 0.02:
            if self.still_since is None:
                self.still_since = time.time()
            self.moving = (time.time() - self.still_since) < 1.0
        else:
            self.still_since = None
            self.moving = True

    def on_imu(self, m):
        q = m.orientation
        if (q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w) < 1e-9:
            # 무효 쿼터니언을 단위값으로. 공분산 -1 은 건드리지 않으므로
            # EKF 는 여전히 자세를 쓰지 않는다.
            m.orientation.x = 0.0
            m.orientation.y = 0.0
            m.orientation.z = 0.0
            m.orientation.w = 1.0
            self.bad_q += 1

        if self.est_bias:
            g = (m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z)
            if not self.moving:
                # 정지 중에만, 아주 천천히 따라간다. 빠르게 따라가면 회전
                # 직후의 잔여값까지 바이어스로 먹어 버린다.
                a = 0.002 if self.bias_n > 400 else 0.02
                for k in range(3):
                    self.bias[k] += a * (g[k] - self.bias[k])
                self.bias_n += 1
            if self.bias_n > 200:
                m.angular_velocity.x = g[0] - self.bias[0]
                m.angular_velocity.y = g[1] - self.bias[1]
                m.angular_velocity.z = g[2] - self.bias[2]

        if self.rotate:
            gx, gy, gz = (m.angular_velocity.x,
                          m.angular_velocity.y,
                          m.angular_velocity.z)
            ax, ay, az = (m.linear_acceleration.x,
                          m.linear_acceleration.y,
                          m.linear_acceleration.z)
            m.angular_velocity.x = gz
            m.angular_velocity.y = -gx
            m.angular_velocity.z = -gy      # 로봇 요축
            m.linear_acceleration.x = az
            m.linear_acceleration.y = -ax
            m.linear_acceleration.z = -ay
            m.header.frame_id = self.tgt

        self.pub.publish(m)
        self.n += 1

    def report(self):
        b = ", 자이로 바이어스 %.4f/%.4f/%.4f rad/s (표본 %d)" % (
            self.bias[0], self.bias[1], self.bias[2], self.bias_n
        ) if self.est_bias else ""
        self.get_logger().info(
            "재발행 %d 건 (무효 쿼터니언 %d 건)%s" % (self.n, self.bad_q, b))


def main():
    rclpy.init()
    node = ImuAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
