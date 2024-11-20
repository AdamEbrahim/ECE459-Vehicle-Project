import os

# Create directory structure
dirs = [
    # Source code directories
    'ros_comm/src/perception',
    'ros_comm/src/control',
    'ros_comm/src/computer_vision',
    'ros_comm/src/esp',
    # Additional ROS 2 package directories
    'ros_comm/config',
    'ros_comm/test',
    'ros_comm/msg',
    'ros_comm/srv'
]

for dir_path in dirs:
    os.makedirs(dir_path, exist_ok=True)
    print(f'Created directory: {dir_path}')
