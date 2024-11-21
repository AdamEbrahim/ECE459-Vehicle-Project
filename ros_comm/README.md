# ROS2 Vehicle Control System

This repository contains the ROS2 communication and control system for the ECE459 Vehicle Project. The system consists of multiple nodes that work together to control the vehicle based on computer vision input and user commands.

## System Architecture

### Node Overview and Topics

1. **Computer Vision Node (`cv_model`)**
   - Publishes to: `perception_commands`
   - Function: Processes camera input and detects objects/signs
   - Currently uses a dummy implementation `dummy_cv_model` that randomly generates detections at a given rate (frequency)

2. **Robot Controller Node (`robot_controller`)**
   - Subscribes to: 
     - `user_commands`: Commands from user control
     - `perception_commands`: Commands from CV system
   - Publishes to: `motor_commands`
   - Function: Main control logic that combines user input and perception data to generate motor commands
     - takes in user commands and perception commands and generates motor commands based on priorities determined by object type

3. **Motor Controller Node (`motor_controller`)**
   - Subscribes to: `motor_commands`
   - Function: Forwards commands to the ESP32 via serial connection
   - `robot_controller` sends command to this topic and `esp_forwarder2` receives them and forwards them to the ESP

4. **User Control Node (`user_control`)**
   - Publishes to: `user_commands`
   - Function: Handles user input for vehicle control
     - Runs on Jetson and listens for commands over WiFi socket
     - Receives commands from `laptop_control.py` running on remote laptop
     - Publishes received commands to the `user_commands` topic

### System Flow Diagram
```
        [Camera] --> [cv_model] --> perception_commands ----> [robot_controller]
                                                          ^         |
[Laptop]                                                  |         |
    |                                                     |         v
    v                                                     |   motor_commands
laptop_control.py                                         |         |
    |                                                     |         v
    v                                                     |  [esp_forwarder2]
(WiFi)                                                    |         |
    |                                                     |         v
    v                                                     |      [ESP32]
[user_control] ---------> user_commands ------------------>         |
                                                                    v
                                                             [Motor Control]
```

### Configuration

The system's behavior can be configured through the `config/robot_params.yaml` file. For example,key parameters include:

1. **Robot Controller Parameters**
   ```yaml
   robot_controller:
     ros__parameters:
       default_speed: 50  # Default movement speed (0-100)
       slow_speed: 25   # Speed when careful movement needed
       stop_distance: 1.0  # Meters before stopping
       turn_speed: 35   # Speed during turns
       state_update_rate: 1.0  # Control loop frequency (Hz)
   ```

2. **Computer Vision Parameters**
   ```yaml
   computer_vision:
     ros__parameters:
       camera_width: 1280
       camera_height: 720
       frame_rate: 0.33  # Detection frequency (Hz)
       model_confidence: 0.5
       detection_classes: ["person", "stop sign", "yield sign", ...]
   ```

To modify these parameters:
1. Edit `config/robot_params.yaml`
2. Rebuild the package: `colcon build --packages-select ros_comm`
3. Source the workspace: `source install/local_setup.bash`
4. Restart the nodes for changes to take effect

### Command Format

- **Motor Commands**: Format `<DIRECTION>_<SPEED>`
  - Directions: FORWARD, BACKWARD, LEFT, RIGHT, STOP
  - Speed: 0-100 (percentage)
  - Example: "FORWARD_50" for 50% forward speed

## Running the System

### Setup on Jetson

1. **Initial Setup**
   ```bash
   cd ~/ros2_ws
   rm -rf build/ install/ log/
   colcon build --symlink-install --packages-select ros_comm
   source /opt/ros/dashing/setup.bash
   source ~/ros2_ws/install/local_setup.bash
   ```

2. **Launch Full System**
   ```bash
   ros2 launch ros_comm robot_system.launch.py
   ```

3. **Running Individual Nodes**
   ```bash
   # Run robot controller only
   ros2 run ros_comm robot_controller

   # Run CV model only
   ros2 run ros_comm cv_model

   # Run motor controller only
   ros2 run ros_comm esp_forwarder
   ```

### Remote Control Setup (Laptop)

1. **Network Setup**
   - Ensure laptop and Jetson are on the same network
   - Set ROS_DOMAIN_ID to match Jetson's (default: 0)
   ```bash
   export ROS_DOMAIN_ID=0
   ```

2. **Run Laptop Control**
   - Need to run in order to send commands to Jetson over WiFi
   ```bash
   python laptop_control.py
   ```

   **Key Commands:**
   - `t`: Enable Command Sending
   - `w`: Move Forward
   - `s`: Move Backward
   - `a`: Turn Left
   - `d`: Turn Right
   - `x`: Stop
   - `q`: Quit/Exit Control

### Useful ROS2 Commands

1. **List all topics**
   ```bash
   ros2 topic list
   ```

2. **Monitor specific topics**
   ```bash
   # Monitor motor commands
   ros2 topic echo /motor_commands

   # Monitor user commands
   ros2 topic echo /user_commands

   # Monitor perception commands
   ros2 topic echo /perception_commands
   ```
   - These will allow you to see the data flowing through the system
      - Objects Detected by CV model: `detected_objects`
      - Commands decided from detections: `perception_commands`
      - Motor Commands sent to ESP: `motor_commands`
      - User Commands received from Laptop: `user_commands`


3. **View node graph**
   ```bash
   ros2 node list
   ros2 node info /robot_controller
   ```

## Troubleshooting

1. **No Motor Response**
   - Check if ESP32 is connected: `ls /dev/ttyUSB*`
   - Verify motor_controller node is running
   - Monitor motor_commands topic

2. **CV System Issues**
   - Check camera connection
   - Monitor perception_commands topic
   - Check CV model logs: `ros2 run ros_comm cv_model --ros-args --log-level debug`

3. **Remote Control Issues**
   - Verify network connectivity
   - Check ROS_DOMAIN_ID on both machines
   - Monitor user_commands topic
   - Ensure `user_control` node is running on Jetson
   - Verify WiFi connection between laptop and Jetson
