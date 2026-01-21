import math
import serial

import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32MultiArray
from sensor_msgs.msg import JointState


class WheelSerialBridge(Node):
    def __init__(self):
        super().__init__('wheel_serial_bridge')

        # ===== parameters =====
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('base_frame', 'base_link')

        port = self.get_parameter('port').value
        baud = int(self.get_parameter('baud').value)

        self.ser = serial.Serial(port, baudrate=baud, timeout=0.0)
        self.get_logger().info(f'Opened serial {port} @ {baud}')

        # ===== publishers =====
        self.raw_pub = self.create_publisher(Int32MultiArray, '/wheel_raw', 50)
        self.js_pub = self.create_publisher(JointState, '/wheel_states', 50)

        self._buf = b""

        # 200 Hz polling (serial WHEEL ~20 Hz 정도면 충분)
        self.timer = self.create_timer(0.005, self.read_serial)

    def read_serial(self):
        n = self.ser.in_waiting
        if n <= 0:
            return

        try:
            self._buf += self.ser.read(n)
        except Exception as e:
            self.get_logger().warn(f"serial read error: {e}")
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
            # 기대 형식: WHEEL dt v1 v2 v3 v4  (총 6개)
            if len(parts) != 6:
                # 네 로그에서 가끔 "WHEEL 50 -1" 같이 깨지는 줄이 섞여서 그냥 버리는 게 맞음
                continue

            try:
                dt_ms = int(parts[1])
                v1 = int(float(parts[2]))
                v2 = int(float(parts[3]))
                v3 = int(float(parts[4]))
                v4 = int(float(parts[5]))
            except ValueError:
                continue

            self.publish_wheels(dt_ms, v1, v2, v3, v4)

    def publish_wheels(self, dt_ms: int, v1: int, v2: int, v3: int, v4: int):
        # 1) raw (그대로 저장/후처리용)
        raw = Int32MultiArray()
        raw.data = [dt_ms, v1, v2, v3, v4]
        self.raw_pub.publish(raw)

        # 2) JointState (velocity = m/s 로)
        now = self.get_clock().now().to_msg()
        base_frame = self.get_parameter('base_frame').value

        js = JointState()
        js.header.stamp = now
        js.header.frame_id = base_frame

        # ===== 너가 이전에 적어둔 매핑 주석 기준(필요하면 여기만 바꿔) =====
        # RAW1 = FL
        # RAW2 = RL
        # RAW3 = RR
        # RAW4 = FR
        js.name = ["wheel_fl", "wheel_rl", "wheel_rr", "wheel_fr"]

        # mm/s -> m/s
        js.velocity = [v1 * 0.001, v2 * 0.001, v3 * 0.001, v4 * 0.001]

        # position은 “틱 누적”을 가지고 있을 때 채우는 게 정석이라, 지금은 비워둠
        js.position = []
        js.effort = []

        self.js_pub.publish(js)


def main():
    rclpy.init()
    node = WheelSerialBridge()
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
