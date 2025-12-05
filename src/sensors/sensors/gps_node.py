"""
GPS sensor node.
Publishes location and heading data.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import TwistStamped


class GPSNode(Node):
    """Reads GPS sensor data."""

    def __init__(self):
        super().__init__('gps_node')
        self.get_logger().info('GPS Node initialized')

        # TODO: Initialize GPS hardware interface
        # TODO: Create publisher for GPS data
        # TODO: Implement reading loop

    def read_gps(self):
        """Read GPS data and publish."""
        # TODO: Read from GPS sensor
        # TODO: Publish NavSatFix message
        pass


def main(args=None):
    rclpy.init(args=args)
    node = GPSNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
