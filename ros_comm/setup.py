from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'ros_comm'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(),  # This will find ros_comm package
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
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
        ],
    }
)
