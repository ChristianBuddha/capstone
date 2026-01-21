import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import math


def quat_to_yaw(quat) -> float:
    """Convert quaternion to yaw angle (in degrees)"""
    return math.degrees(math.atan2(2.0 * (quat.w * quat.z + quat.x * quat.y),
                                    1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z)))


class OdomMonitor(Node):
    def __init__(self):
        super().__init__('odom_monitor')
        
        self.sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10
        )
        
        self.get_logger().info('=== Odometry Monitor Started ===')

    def odom_callback(self, msg: Odometry):
        """Display odometry data in a readable format"""
        
        # Extract position
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        yaw = quat_to_yaw(msg.pose.pose.orientation)
        
        # Extract velocity
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        wz = msg.twist.twist.angular.z
        
        # Calculate speed magnitude
        speed = math.sqrt(vx**2 + vy**2)
        
        # Clear screen and print
        print("\033[2J\033[H")  # Clear terminal
        print("╔" + "="*70 + "╗")
        print("║" + " "*20 + "ODOMETRY MONITOR" + " "*35 + "║")
        print("╠" + "="*70 + "╣")
        
        # Position
        print(f"║ 위치 (Position):                                                      ║")
        print(f"║   X: {x:8.4f} m  |  Y: {y:8.4f} m  |  Yaw: {yaw:7.2f}°              ║")
        
        # Velocity
        print("║ 속도 (Velocity):                                                      ║")
        print(f"║   Vx: {vx:8.4f} m/s  |  Vy: {vy:8.4f} m/s  |  Speed: {speed:6.4f} m/s       ║")
        print(f"║   Wz (회전): {wz:8.4f} rad/s  ({math.degrees(wz):7.2f}°/s)          ║")
        
        # Movement direction
        direction = ""
        if speed > 0.001:
            if abs(vx) > abs(vy):
                direction = "→ 전진" if vx > 0 else "← 후진"
            else:
                direction = "↑ 좌진" if vy > 0 else "↓ 우진"
        else:
            direction = "정지"
        
        print(f"║ 이동 방향 (Direction): {direction:<40}              ║")
        
        print("╚" + "="*70 + "╝")


def main():
    rclpy.init()
    node = OdomMonitor()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
