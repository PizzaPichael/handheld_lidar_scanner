import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math

counter = 0

class ScanInfoNode(Node):
    
    def __init__(self):
        super().__init__('scan_info_node')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',  # Passe das Topic ggf. an
            self.scan_callback,
            10
        )
    
    def _restrict_laser_scan_by_angle(self, msg, angle=None ):
        if angle:
            angle_min = msg.angle_min
            angle_max = msg.angle_max
            angle_inc = msg.angle_increment
            zero_angle_forward_along_x = 0

            input_rad_angle = math.radians(angle)

            restricted_angle_min = zero_angle_forward_along_x - input_rad_angle/2
            restricted_angle_max = zero_angle_forward_along_x + input_rad_angle/2

            num_ranges = int(round((angle_max - angle_min) / angle_inc)) + 1

            angles_array = []
            current_angle = angle_min
            for _ in range(num_ranges):
                angles_array.append(current_angle)
                current_angle += angle_inc

            self.get_logger().info(
                f"angles_array: {angles_array}, angles_array_len: {len(angles_array)}"
            )

    def scan_callback(self, msg):
        global counter

        if counter <1:
            self.get_logger().info(
                f"angle_min: {msg.angle_min:.3f}, angle_max: {msg.angle_max:.3f}, angle_increment: {msg.angle_increment:.6f}"
            )
            self.get_logger().info(
                f"ranges amount: {len(msg.ranges)}"
            )
            self._restrict_laser_scan_by_angle(msg, 45)

            counter += 1

def main(args=None):
    rclpy.init(args=args)
    node = ScanInfoNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()