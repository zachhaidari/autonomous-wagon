"""
Main wagon coordinator node.
Orchestrates communication between all modules.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class WagonCoordinator(Node):
    """Main coordinator node for the autonomous wagon."""

    def __init__(self):
        super().__init__('wagon_coordinator')
        self.get_logger().info('Wagon Coordinator initialized')

        # TODO: Subscribe to topics from all modules
        # TODO: Implement system health checks
        # TODO: Handle emergency stop commands

    def run(self):
        """Main coordinator loop."""
        self.get_logger().info('Wagon system is running')


def main(args=None):
    rclpy.init(args=args)
    coordinator = WagonCoordinator()
    rclpy.spin(coordinator)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
