import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

class BuzzerSubscriber(Node):
    def __init__(self):
        super().__init__('buzzer_subscriber')

        # SUBSCRIBE VIBRATION
        self.sub_vib = self.create_subscription(
            Bool,
            'vibration',
            self.vibration_callback,
            10
        )

        # PUBLISH COMMAND KE PUBLISHER
        self.pub_cmd = self.create_publisher(Bool, 'buzzer_cmd', 10)

        self.get_logger().info("Subscriber buzzer aktif (tanpa serial).")

    def vibration_callback(self, msg):
        if msg.data:
            self.get_logger().info("[EVENT] Vibration detected → Turn ON buzzer")
            cmd = Bool()
            cmd.data = True
            self.pub_cmd.publish(cmd)
        else:
            self.get_logger().info("[EVENT] No vibration → Turn OFF buzzer")
            cmd = Bool()
            cmd.data = False
            self.pub_cmd.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = BuzzerSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
