import time
import serial

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class OmniSerialBridge(Node):
    def __init__(self):
        super().__init__('omni_serial_bridge')

        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 256000)
        self.declare_parameter('send_hz', 20.0)
        self.declare_parameter('cmd_timeout', 0.25)  # seconds

        port = self.get_parameter('port').value
        baud = int(self.get_parameter('baud').value)

        self.ser = serial.Serial(port, baudrate=baud, timeout=0.0)
        self.get_logger().info(f"Opened serial: {port} @ {baud}")

        self.last_msg_time = time.time()
        self.last_vx = 0.0
        self.last_vy = 0.0
        self.last_wz = 0.0

        self.sub = self.create_subscription(Twist, '/cmd_vel', self.cb, 10)

        hz = float(self.get_parameter('send_hz').value)
        self.timer = self.create_timer(1.0 / hz, self.on_timer)

    def cb(self, msg: Twist):
        self.last_msg_time = time.time()
        self.last_vx = float(msg.linear.x)   # forward (m/s)
        self.last_vy = float(msg.linear.y)   # left (m/s)
        self.last_wz = float(msg.angular.z)  # yaw (rad/s)

    def on_timer(self):
        now = time.time()
        timeout = float(self.get_parameter('cmd_timeout').value)

        vx = self.last_vx
        vy = self.last_vy
        wz = self.last_wz

        if (now - self.last_msg_time) > timeout:
            vx = vy = wz = 0.0

        vx_mmps = int(round(vx * 1000.0))
        vy_mmps = int(round(vy * 1000.0))
        wz_mrad = int(round(wz * 1000.0))

        line = f"TWIST {vx_mmps} {vy_mmps} {wz_mrad}\n"
        try:
            self.ser.write(line.encode('ascii'))
        except Exception as e:
            self.get_logger().error(f"serial write failed: {e}")

def main():
    rclpy.init()
    node = OmniSerialBridge()
    try:
        rclpy.spin(node)
    finally:
        try:
            node.ser.close()
        except:
            pass
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
