import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class ScanInfoNode(Node):
    def __init__(self):
        super().__init__('scan_info_node')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',  # Passe das Topic ggf. an
            self.scan_callback,
            10
        )

    def scan_callback(self, msg):
        self.get_logger().info(
            f"angle_min: {msg.angle_min:.3f}, angle_max: {msg.angle_max:.3f}, angle_increment: {msg.angle_increment:.6f}"
        )

def main(args=None):
    rclpy.init(args=args)
    node = ScanInfoNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()