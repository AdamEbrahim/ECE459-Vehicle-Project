from setuptools import setup
import os
from glob import glob

package_name = 'ros_comm'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, 
              f'{package_name}.computer_vision',
              f'{package_name}.perception',
              f'{package_name}.control',
              f'{package_name}.esp'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # Include config files
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=[
        'setuptools',
        'opencv-python',
        'numpy'
    ],
    zip_safe=True,
    maintainer='ece459',
    maintainer_email='ece459@todo.todo',
    description='Robotic Vision Control System',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cv_model = ros_comm.computer_vision.dummy_cv_model:main',
            'perception = ros_comm.perception.perception_model:main',
            'robot_controller = ros_comm.control.robot_controller:main',
            'esp_forwarder = ros_comm.esp.esp_forwarder2:main',
            'user_control = ros_comm.control.user_control:main',
            'yolo_detection = ros_comm.computer_vision.yolo_detection:main',
            'basic_cv_detection = ros_comm.computer_vision.basic_cv_detection:main',
        ],
    }
)
