from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the package share directory
    pkg_share = get_package_share_directory('ros_comm')
    config_file = os.path.join(pkg_share, 'config', 'robot_params.yaml')

    return LaunchDescription([
        # Computer Vision Node
        Node(
            package='ros_comm',
            node_executable='cv_model',
            node_name='cv_model',
            output='screen',
            parameters=[config_file]
        ),
        
        # Perception Node
        Node(
            package='ros_comm',
            node_executable='perception',
            node_name='perception',
            output='screen',
            parameters=[config_file]
        ),
        
        # Robot Controller
        Node(
            package='ros_comm',
            node_executable='robot_controller',
            node_name='robot_controller',
            output='screen',
            parameters=[config_file]
        ),
        
        # Motor Controller (ESP Forwarder)
        Node(
            package='ros_comm',
            node_executable='esp_forwarder',
            node_name='motor_controller',
            output='screen',
            parameters=[config_file]
        ),

        # User Control (on Jetson)
        Node(
            package='ros_comm',
            node_executable='user_control',
            node_name='user_control',
            output='screen',
            parameters=[config_file]
        )
    ])