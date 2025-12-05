"""
IMU (Inertial Measurement Unit) sensor node.
Publishes acceleration, gyroscope, and magnetometer data.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


class IMUNode(Node):
    """Reads IMU sensor data."""

    def __init__(self):
        super().__init__('imu_node')
        self.get_logger().info('IMU Node initialized')

        # TODO: Initialize IMU hardware interface
        # TODO: Create publisher for IMU data
        # TODO: Implement reading loop

    def read_imu(self):
        """Read IMU data and publish."""
        # TODO: Read from IMU sensor
        # TODO: Publish Imu message
        pass


def main(args=None):
    rclpy.init(args=args)
    node = IMUNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
