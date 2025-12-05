"""
Steering controller node.
Controls steering actuator based on target direction.
Uses magnetic encoder feedback for position.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


class SteeringController(Node):
    """Controls steering with magnetic encoder feedback."""

    def __init__(self):
        super().__init__('steering_controller')
        self.get_logger().info('Steering Controller initialized')

        # TODO: Subscribe to target heading
        # TODO: Read magnetic encoder position
        # TODO: Implement PID control for steering
        # TODO: Publish stepper motor commands

    def control_steering(self, target_heading, current_heading):
        """Implement PID control for steering angle."""
        # TODO: PID control implementation
        pass


def main(args=None):
    rclpy.init(args=args)
    controller = SteeringController()
    rclpy.spin(controller)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
