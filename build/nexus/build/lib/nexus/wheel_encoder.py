import math
import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32MultiArray
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped
from tf2_ros import TransformBroadcaster


def yaw_to_quat(yaw: float) -> Quaternion:
    """Convert yaw angle to quaternion"""
    return Quaternion(
        x=0.0,
        y=0.0,
        z=math.sin(yaw * 0.5),
        w=math.cos(yaw * 0.5),
    )


def quat_to_yaw(quat: Quaternion) -> float:
    """Convert quaternion to yaw angle"""
    return math.atan2(2.0 * (quat.w * quat.z + quat.x * quat.y),
                      1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z))


def wrap_pi(a: float) -> float:
    """Wrap angle to [-pi, pi]"""
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


class WheelEncoderNode(Node):
    def __init__(self):
        super().__init__('wheel_encoder')

        # ===== Parameters =====
        self.declare_parameter('robot_L', 0.20)  # wheelbase length (m)
        self.declare_parameter('robot_W', 0.15)  # wheelbase width (m)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('dead_v_mps', 0.002)
        self.declare_parameter('dead_w_rps', 0.01)
        self.declare_parameter('use_imu_yaw', True)  # Use IMU yaw for heading

        # ===== State =====
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.wz = 0.0

        self.last_wheel_time = None
        self.imu_yaw = 0.0
        self.imu_wz = 0.0

        # ===== ROS I/O =====
        self.wheel_sub = self.create_subscription(
            Int32MultiArray, '/wheel_raw', self.wheel_callback, 10
        )
        self.imu_sub = self.create_subscription(
            Imu, '/imu', self.imu_callback, 10
        )
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.get_logger().info('wheel_encoder node started')

    def wheel_callback(self, msg: Int32MultiArray):
        """Process wheel encoder data"""
        if len(msg.data) < 5:
            return

        # Extract wheel speeds (mm/s)
        # data[0] = dt_ms (timestamp delta)
        # data[1] = FL wheel speed
        # data[2] = RL wheel speed
        # data[3] = RR wheel speed
        # data[4] = FR wheel speed
        
        dt_ms = int(msg.data[0])
        v_fl = float(msg.data[1])
        v_fr = float(msg.data[2])
        v_rl = float(msg.data[3])
        v_rr = float(msg.data[4])

        # Calculate dt
        current_time = self.get_clock().now()
        if self.last_wheel_time is None:
            self.last_wheel_time = current_time
            return

        dt = (current_time - self.last_wheel_time).nanoseconds / 1e9
        self.last_wheel_time = current_time

        if dt <= 0 or dt > 1.0:
            return

        # ===== Mecanum wheel kinematics =====
        # RAW1 = FL, RAW2 = RL, RAW3 = RR, RAW4 = FR
        # Mecanum forward kinematics
        vx_mm = (v_fl + v_fr + v_rr + v_rl) * 0.25
        vy_mm = (-v_fl + v_fr - v_rr + v_rl) * 0.25
        wz_calc = (-v_fl + v_fr + v_rr - v_rl) * 0.25 / ((self.get_parameter('robot_L').value + self.get_parameter('robot_W').value) * 1000.0)

        # Convert mm/s to m/s
        vx = vx_mm * 0.001
        vy = vy_mm * 0.001

        # Apply deadband
        dead_v = self.get_parameter('dead_v_mps').value
        dead_w = self.get_parameter('dead_w_rps').value

        if abs(vx) < dead_v and abs(vy) < dead_v and abs(wz_calc) < dead_w:
            vx = vy = wz_calc = 0.0

        # ===== Use IMU for yaw if enabled =====
        use_imu = self.get_parameter('use_imu_yaw').value
        if use_imu:
            wz = self.imu_wz  # Use IMU angular velocity
            self.yaw = self.imu_yaw  # Use IMU yaw
        else:
            wz = wz_calc  # Use wheel-based angular velocity

        # ===== Odometry integration =====
        dx = (vx * math.cos(self.yaw) - vy * math.sin(self.yaw)) * dt
        dy = (vx * math.sin(self.yaw) + vy * math.cos(self.yaw)) * dt

        if not use_imu:
            self.yaw = wrap_pi(self.yaw + wz * dt)

        self.x += dx
        self.y += dy
        self.vx = vx
        self.vy = vy
        self.wz = wz

        self.publish_odom()

    def imu_callback(self, msg: Imu):
        """Process IMU data for heading"""
        # Extract yaw from quaternion
        self.imu_yaw = quat_to_yaw(msg.orientation)
        
        # Extract angular velocity (yaw rate)
        self.imu_wz = msg.angular_velocity.z

    def publish_odom(self):
        """Publish odometry message and TF"""
        now = self.get_clock().now().to_msg()
        odom_frame = self.get_parameter('odom_frame').value
        base_frame = self.get_parameter('base_frame').value

        # ===== Odometry Message =====
        msg = Odometry()
        msg.header.stamp = now
        msg.header.frame_id = odom_frame
        msg.child_frame_id = base_frame

        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation = yaw_to_quat(self.yaw)

        msg.twist.twist.linear.x = self.vx
        msg.twist.twist.linear.y = self.vy
        msg.twist.twist.angular.z = self.wz

        self.odom_pub.publish(msg)

        # ===== Transform Broadcast =====
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = odom_frame
        t.child_frame_id = base_frame
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation = yaw_to_quat(self.yaw)

        self.tf_broadcaster.sendTransform(t)


def main():
    rclpy.init()
    node = WheelEncoderNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
