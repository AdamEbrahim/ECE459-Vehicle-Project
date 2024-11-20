from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
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
            executable='src/computer_vision/dummy_cv_model.py',
            name='cv_model',
            output='screen',
            parameters=[config_file]
        ),
        
        # Perception Node
        Node(
            package='ros_comm',
            executable='src/perception/perception_model.py',
            name='perception',
            output='screen',
            parameters=[config_file]
        ),
        
        # Robot Controller
        Node(
            package='ros_comm',
            executable='src/control/robot_controller.py',
            name='robot_controller',
            output='screen',
            parameters=[config_file]
        ),
        
        # Motor Controller (ESP Forwarder)
        Node(
            package='ros_comm',
            executable='src/esp/esp_forwarder2.py',
            name='motor_controller',
            output='screen',
            parameters=[config_file]
        )
    ])