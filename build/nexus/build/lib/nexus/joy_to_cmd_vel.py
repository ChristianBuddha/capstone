#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist


class JoyToCmdVel(Node):
    def __init__(self):
        super().__init__('joy_to_cmd_vel')

        # 파라미터 선언 - 메카넘휠용
        self.declare_parameter('forward_scale', 0.3)  # 전/후진 스케일
        self.declare_parameter('strafe_scale', 0.3)  # 좌/우 스트래프 스케일
        self.declare_parameter('rotate_scale', 0.3)  # 회전 스케일
        self.declare_parameter('forward_axis', 1)  # 전/후진 축 (기본값: 1 = 좌측 Y축)
        self.declare_parameter('strafe_axis', 0)  # 스트래프 축 (기본값: 0 = 좌측 X축)
        self.declare_parameter('rotate_axis', 2)  # 회전 축 (기본값: 2 = 우측 X축)

        # 파라미터 값 가져오기
        self.forward_scale = self.get_parameter('forward_scale').value
        self.strafe_scale = self.get_parameter('strafe_scale').value
        self.rotate_scale = self.get_parameter('rotate_scale').value
        self.forward_axis = self.get_parameter('forward_axis').value
        self.strafe_axis = self.get_parameter('strafe_axis').value
        self.rotate_axis = self.get_parameter('rotate_axis').value

        # 구독자 및 발행자 생성
        self.joy_sub = self.create_subscription(Joy, 'joy', self.joy_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        self.get_logger().info(
            f"Mecanum Wheel Joy to Cmd_vel converter started\n"
            f"  Forward axis: {self.forward_axis}, scale: {self.forward_scale}\n"
            f"  Strafe axis: {self.strafe_axis}, scale: {self.strafe_scale}\n"
            f"  Rotate axis: {self.rotate_axis}, scale: {self.rotate_scale}"
        )

    def joy_callback(self, msg: Joy):
        """조이스틱 입력을 cmd_vel로 변환하여 발행"""
        cmd_vel = Twist()

        # 축 인덱스 확인 - 메카넘휠
        if self.forward_axis < len(msg.axes):
            cmd_vel.linear.x = msg.axes[self.forward_axis] * self.forward_scale
        
        if self.strafe_axis < len(msg.axes):
            cmd_vel.linear.y = msg.axes[self.strafe_axis] * self.strafe_scale
        
        if self.rotate_axis < len(msg.axes):
            cmd_vel.angular.z = msg.axes[self.rotate_axis] * self.rotate_scale

        # cmd_vel 발행
        self.cmd_vel_pub.publish(cmd_vel)


def main():
    rclpy.init()
    node = JoyToCmdVel()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
