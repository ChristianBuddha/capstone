import math
import serial

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped
from tf2_ros import TransformBroadcaster


def yaw_to_quat(yaw: float) -> Quaternion:
    return Quaternion(
        x=0.0,
        y=0.0,
        z=math.sin(yaw * 0.5),
        w=math.cos(yaw * 0.5),
    )


def wrap_pi(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


class OdomSerialNode(Node):
    def __init__(self):
        super().__init__('odom_serial_node')

        # ===== parameters =====
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 115200)

        self.declare_parameter('robot_L', 0.20)  # m
        self.declare_parameter('robot_W', 0.15)  # m

        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')

        self.declare_parameter('dead_v_mps', 0.002)
        self.declare_parameter('dead_w_rps', 0.01)

        port = self.get_parameter('port').value
        baud = int(self.get_parameter('baud').value)

        self.ser = serial.Serial(port, baudrate=baud, timeout=0.0)
        self.get_logger().info(f'Opened serial {port} @ {baud}')

        # ===== ROS I/O =====
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # ===== state =====
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.vx = 0.0
        self.vy = 0.0
        self.wz = 0.0

        self._buf = b""

        # 200 Hz polling (WHEEL ~20 Hz)
        self.timer = self.create_timer(0.005, self.read_serial)

    # ============================================================
    # SERIAL RX
    # ============================================================
    def read_serial(self):
        n = self.ser.in_waiting
        if n <= 0:
            return

        try:
            self._buf += self.ser.read(n)
        except Exception:
            return

        while b'\n' in self._buf:
            line, self._buf = self._buf.split(b'\n', 1)
            line = line.strip().replace(b'\r', b'')
            if not line:
                continue

            s = line.decode('ascii', errors='ignore')

            if not s.startswith('WHEEL'):
                continue

            parts = s.split()
            if len(parts) != 6:
                continue

            try:
                dt_ms = int(parts[1])
                v1 = float(parts[2])
                v2 = float(parts[3])
                v3 = float(parts[4])
                v4 = float(parts[5])
            except ValueError:
                continue

            self.update_from_wheels(dt_ms, v1, v2, v3, v4)

    # ============================================================
    # ODOMETRY UPDATE (RAW → physical mapping)
    # ============================================================
    def update_from_wheels(self, dt_ms, v1, v2, v3, v4):
        if dt_ms <= 0 or dt_ms > 500:
            return

        dt = dt_ms * 0.001

        L = float(self.get_parameter('robot_L').value)
        W = float(self.get_parameter('robot_W').value)

        # ========================================================
        # RAW → physical wheel mapping (검증 완료)
        #
        # RAW1 = FL
        # RAW2 = RL
        # RAW3 = RR
        # RAW4 = FR
        # ========================================================
        v_fl = v1
        v_rl = v2
        v_rr = v3
        v_fr = v4

        # mecanum forward kinematics (x: forward, y: left)
        vx_mm = (v_fl + v_fr + v_rr + v_rl) * 0.25
        vy_mm = (-v_fl + v_fr - v_rr + v_rl) * 0.25
        wz = (-v_fl + v_fr + v_rr - v_rl) * 0.25 / ((L + W) * 1000.0)

        vx = vx_mm * 0.001
        vy = vy_mm * 0.001

        dead_v = float(self.get_parameter('dead_v_mps').value)
        dead_w = float(self.get_parameter('dead_w_rps').value)

        if abs(vx) < dead_v and abs(vy) < dead_v and abs(wz) < dead_w:
            vx = vy = wz = 0.0

        dx = (vx * math.cos(self.yaw) - vy * math.sin(self.yaw)) * dt
        dy = (vx * math.sin(self.yaw) + vy * math.cos(self.yaw)) * dt

        self.x += dx
        self.y += dy
        self.yaw = wrap_pi(self.yaw + wz * dt)

        self.vx = vx
        self.vy = vy
        self.wz = wz

        self.publish_odom()

    # ============================================================
    # PUBLISH ODOM + TF
    # ============================================================
    def publish_odom(self):
        now = self.get_clock().now().to_msg()
        odom_frame = self.get_parameter('odom_frame').value
        base_frame = self.get_parameter('base_frame').value

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
    node = OdomSerialNode()
    try:
        rclpy.spin(node)
    finally:
        try:
            node.ser.close()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
