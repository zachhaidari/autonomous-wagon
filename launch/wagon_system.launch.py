# ROS 2 launch file for the complete wagon system
# Usage: ros2 launch wagon_core wagon_system.launch.py

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """Generate launch description for complete wagon system."""
    
    # Get package directory
    wagon_core_dir = get_package_share_directory('wagon_core')
    config_dir = os.path.join(wagon_core_dir, '..', '..', 'config')
    
    # Load parameters
    params = os.path.join(config_dir, 'wagon_params.yaml')
    
    return LaunchDescription([
        # Vision nodes
        Node(
            package='vision',
            executable='target_tracker',
            name='target_tracker',
            parameters=[params],
            output='screen'
        ),
        Node(
            package='vision',
            executable='depth_processor',
            name='depth_processor',
            parameters=[params],
            output='screen'
        ),
        
        # Sensor nodes
        Node(
            package='sensors',
            executable='imu_node',
            name='imu_node',
            parameters=[params],
            output='screen'
        ),
        Node(
            package='sensors',
            executable='gps_node',
            name='gps_node',
            parameters=[params],
            output='screen'
        ),
        Node(
            package='sensors',
            executable='encoder_node',
            name='encoder_node',
            parameters=[params],
            output='screen'
        ),
        
        # Control nodes
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
        
        # Safety nodes
        Node(
            package='safety',
            executable='obstacle_detector',
            name='obstacle_detector',
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
        
        # Core coordinator
        Node(
            package='wagon_core',
            executable='wagon_coordinator',
            name='wagon_coordinator',
            parameters=[params],
            output='screen'
        ),
    ])
