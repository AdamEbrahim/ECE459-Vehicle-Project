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
            executable='dummy_cv_model.py',
            name='cv_model',
            output='screen',
            parameters=[config_file]
        ),
        
        # Perception Node
        Node(
            package='ros_comm',
            executable='perception_model.py',
            name='perception',
            output='screen',
            parameters=[config_file]
        ),
        
        # Robot Controller
        Node(
            package='ros_comm',
            executable='robot_controller.py',
            name='robot_controller',
            output='screen',
            parameters=[config_file]
        ),
        
        # Motor Controller (ESP Forwarder)
        Node(
            package='ros_comm',
            executable='esp_forwarder2.py',
            name='motor_controller',
            output='screen',
            parameters=[config_file]
        ),

        # User Control (on Jetson)
        Node(
            package='ros_comm',
            executable='user_control.py',
            name='user_control',
            output='screen',
            parameters=[config_file]
        )
    ])