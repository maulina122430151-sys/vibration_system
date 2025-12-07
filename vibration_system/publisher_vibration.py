import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import serial

class VibrationPublisher(Node):
    def __init__(self):
        super().__init__('vibration_publisher')

        # SERIAL (publisher satu-satunya pemegang COM)
        self.ser = serial.Serial('COM10', 115200, timeout=1)

        # PUBLISHER VIBRATION
        self.pub = self.create_publisher(Bool, 'vibration', 10)

        # SUBSCRIBER BUZZER COMMAND
        self.sub_cmd = self.create_subscription(
            Bool,
            'buzzer_cmd',
            self.buzzer_command_callback,
            10
        )

        # TIMER UNTUK BACA SERIAL
        self.timer = self.create_timer(0.1, self.read_serial)

    # Baca data dari ESP8266
    def read_serial(self):
        if self.ser.in_waiting:
            line = self.ser.readline().decode(errors='ignore').strip()

            if line.startswith("VIB:"):
                getar = line.split(":")[1]
                msg = Bool()
                msg.data = (getar == "1")
                self.pub.publish(msg)
                self.get_logger().info(f"[PUBLISH] Vibration: {msg.data}")

    # Kirim perintah buzzer ke ESP8266
    def buzzer_command_callback(self, msg):
        if msg.data:
            self.ser.write(b"BUZZ:ON\n")
            self.get_logger().info("[SEND] BUZZ:ON")
        else:
            self.ser.write(b"BUZZ:OFF\n")
            self.get_logger().info("[SEND] BUZZ:OFF")


def main(args=None):
    rclpy.init(args=args)
    node = VibrationPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
