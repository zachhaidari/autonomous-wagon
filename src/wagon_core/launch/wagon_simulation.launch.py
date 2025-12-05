# ROS 2 launch file for simulation mode

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """Generate launch description for wagon system in simulation."""
    
    wagon_core_dir = get_package_share_directory('wagon_core')
    config_dir = os.path.join(wagon_core_dir, 'config')
    params = os.path.join(config_dir, 'wagon_params.yaml')
    
    return LaunchDescription([
        # Launch gazebo or other simulator here
        # TODO: Add simulator nodes
        
        # Same control and safety nodes as hardware
        Node(
            package='control',
            executable='motor_controller',
            name='motor_controller',
            parameters=[params],
            output='screen'
        ),
        Node(
            package='control',
            executable='steering_controller',
            name='steering_controller',
            parameters=[params],
            output='screen'
        ),
        Node(
            package='safety',
            executable='safety_manager',
            name='safety_manager',
            parameters=[params],
            output='screen'
        ),
    ])
