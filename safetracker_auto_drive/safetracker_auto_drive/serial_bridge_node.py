# serial_bridge_node.py (피드백 수신 기능 추가)
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
import serial
import time
import threading

class SerialBridgeNode(Node):
    def __init__(self):
        super().__init__('serial_bridge_node')

        try:
            self.serial_port = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
            time.sleep(2)
            self.get_logger().info('Successfully connected to OpenCR board on /dev/ttyACM0')
        except Exception as e:
            self.get_logger().error(f"Failed to connect to OpenCR board: {e}")
            rclpy.shutdown()
            return

        # --- [피드백 수신 스레드 추가] ---
        self.reader_thread = threading.Thread(target=self.read_serial_data)
        self.reader_thread.daemon = True
        self.reader_thread.start()
        # --------------------------------

        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.bumper_angle_sub = self.create_subscription(
            Float32, '/bumper_angle', self.bumper_angle_callback, 10)

    def read_serial_data(self):
        """Continuously reads from the serial port and logs the data."""
        while rclpy.ok():
            try:
                if self.serial_port.in_waiting > 0:
                    line = self.serial_port.readline().decode('utf-8').strip()
                    if line:
                        self.get_logger().info(f'Received from OpenCR: "{line}"')
            except Exception as e:
                self.get_logger().error(f"Error reading from serial: {e}")
                break
            time.sleep(0.01)

    def cmd_vel_callback(self, msg):
        command = f"v {msg.linear.x} {msg.angular.z}\n"
        self.serial_port.write(command.encode())
        self.get_logger().info(f'Sent to OpenCR: "{command.strip()}"')

    def bumper_angle_callback(self, msg):
        command = f"b {msg.data}\n"
        self.serial_port.write(command.encode())
        self.get_logger().info(f'Sent to OpenCR: "{command.strip()}"')

def main(args=None):
    rclpy.init(args=args)
    node = SerialBridgeNode()
    rclpy.spin(node)
    node.destroy_node()

if __name__ == '__main__':
    main()

