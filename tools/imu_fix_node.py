#!/usr/bin/env python3
"""RealSense IMU 재발행 — 무효 쿼터니언만 고쳐서 내보낸다.

realsense2_camera 가 unite_imu_method 로 내보내는 IMU 메시지는
자세(orientation)가 (0,0,0,0) 인 무효 쿼터니언이고 공분산은 -1 이다.
규약상 "자세 없음"이 맞지만, robot_localization 은 이 메시지를
각속도까지 포함해 통째로 버린다.

여기서는 쿼터니언만 단위값(0,0,0,1)으로 바꾸고 공분산 -1 은 그대로 둔다.
-1 이 유지되므로 EKF 가 자세를 쓰지는 않고, 각속도만 정상적으로 들어간다.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu

class Fix(Node):
    def __init__(s):
        super().__init__("imu_fix")
        s.declare_parameter("in_topic", "/camera/camera/imu")
        s.declare_parameter("out_topic", "/imu/data_fixed")
        # base_link 기준으로 축까지 바꿔서 내보낼지. True 면 EKF 가 TF 를
        # 쓸 필요가 없어져 좌표변환 문제를 통째로 배제할 수 있다.
        s.declare_parameter("to_base_link", False)
        i = s.get_parameter("in_topic").value
        o = s.get_parameter("out_topic").value
        s.tobase = bool(s.get_parameter("to_base_link").value)
        s.pub = s.create_publisher(Imu, o, qos_profile_sensor_data)
        s.create_subscription(Imu, i, s.cb, qos_profile_sensor_data)
        s.n = 0
        s.get_logger().info("IMU 재발행 %s -> %s (쿼터니언 정규화)" % (i, o))
    def cb(s, m):
        q = m.orientation
        if (q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w) < 1e-9:
            m.orientation.x = 0.0; m.orientation.y = 0.0
            m.orientation.z = 0.0; m.orientation.w = 1.0
        if s.tobase:
            # base<-optical 회전:  [[0,0,1],[-1,0,0],[0,-1,0]]
            #   optical x -> base -y,  optical y -> base -z,  optical z -> base x
            gx, gy, gz = (m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z)
            ax_, ay_, az_ = (m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z)
            m.angular_velocity.x = gz
            m.angular_velocity.y = -gx
            m.angular_velocity.z = -gy
            m.linear_acceleration.x = az_
            m.linear_acceleration.y = -ax_
            m.linear_acceleration.z = -ay_
            m.header.frame_id = "base_link"
        s.pub.publish(m)
        s.n += 1
        if s.n % 2000 == 0:
            s.get_logger().info("재발행 %d 건" % s.n)

def main():
    rclpy.init(); n = Fix()
    try: rclpy.spin(n)
    except KeyboardInterrupt: pass
    finally: rclpy.shutdown()

if __name__ == "__main__":
    main()
