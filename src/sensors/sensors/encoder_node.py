"""
Encoder node.
Reads hall sensor data from wheel encoders.
Publishes wheel speed and odometry.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32


class EncoderNode(Node):
    """Reads encoder data from hall sensors."""

    def __init__(self):
        super().__init__('encoder_node')
        self.get_logger().info('Encoder Node initialized')

        # TODO: Initialize hall sensor GPIO
        # TODO: Create publishers for speed and odometry
        # TODO: Implement edge detection loop

    def process_encoder_pulse(self):
        """Process encoder pulses and calculate speed."""
        # TODO: Detect pulse edges
        # TODO: Calculate wheel speed
        # TODO: Integrate for odometry
        pass


def main(args=None):
    rclpy.init(args=args)
    node = EncoderNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
