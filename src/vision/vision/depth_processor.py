"""
Depth sensor processor.
Estimates distance and direction to target.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point


class DepthProcessor(Node):
    """Processes depth data for distance estimation."""

    def __init__(self):
        super().__init__('depth_processor')
        self.get_logger().info('Depth Processor initialized')

        # TODO: Subscribe to depth frames
        # TODO: Extract depth at target location
        # TODO: Publish distance and bearing


def main(args=None):
    rclpy.init(args=args)
    processor = DepthProcessor()
    rclpy.spin(processor)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
