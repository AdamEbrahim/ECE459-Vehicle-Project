# ROS Communication Package

This package implements the communication and control system for an autonomous vehicle project using ROS2 (Robot Operating System 2). The system integrates computer vision, user control, and hardware communication to enable autonomous navigation and object detection.

## Architecture Overview

The system is built on a publish-subscribe architecture with the following key components:

```
                     [Computer Vision]
                           ↓
[User Control] → [ESP Forwarder] → [Hardware Control]
                     ↑
              [Status Logging]
```

- **Computer Vision Node**: Processes camera feed for object detection (traffic signs, obstacles)
- **User Control Node**: Handles manual control inputs from a laptop
- **ESP Forwarder Node**: Central node that processes commands and communicates with hardware
- **Status Consolidator**: Monitors and logs system state and events

## Nodes Description

### 1. Computer Vision Control (`cv_control.py`)
- **Purpose**: Processes real-time video feed for object detection
- **Features**:
  - Uses Jetson Inference for object detection
  - Detects traffic signs, obstacles, and road conditions
  - Publishes detected objects to 'detected_objects' topic
- **Key Objects**:
  - Stop signs
  - Traffic lights (Red/Green)
  - Speed limit signs
  - Other road obstacles

### 2. User Control (`user_control.py`)
- **Purpose**: Handles manual control inputs from a laptop interface
- **Features**:
  - TCP/IP server for remote control
  - Processes directional commands (Forward, Backward, Left, Right)
  - Publishes commands to 'user_control' topic
- **Controls**:
  - Basic movement (W,A,S,D keys)
  - Speed control
  - Emergency stop

### 3. ESP Forwarder (`esp_forwarder.py`)
- **Purpose**: Bridge between ROS2 and hardware control
- **Features**:
  - Subscribes to both 'user_control' and 'detected_objects' topics
  - Manages vehicle speed and movement based on inputs
  - Communicates with ESP32 via I2C
- **Functionality**:
  - Movement control
  - Speed regulation
  - Safety checks (stop signs, traffic lights)

### 4. Status Consolidator (`logs_consolidator.py`)
- **Purpose**: System monitoring and status logging
- **Features**:
  - Tracks current system state
  - Logs commands and detections
  - Publishes status updates to 'status' topic
- **Monitoring**:
  - Current speed
  - Active commands
  - Detected objects
  - System events

### 5. Laptop Control (`laptop_control.py`)
- **Purpose**: Client-side interface for remote control
- **Features**:
  - TCP/IP client for remote connection
  - Keyboard input handling
  - Command transmission to user control node

## Communication Flow

1. **Input Sources**:
   - Computer vision processes camera feed
   - User inputs from laptop interface

2. **Command Processing**:
   - ESP Forwarder receives commands from both sources
   - Prioritizes safety-critical detections (stop signs, red lights)
   - Manages speed limits and movement restrictions

3. **Hardware Control**:
   - Commands are converted to I2C signals
   - Sent to ESP32 for motor control
   - Real-time feedback and status updates

4. **Status Monitoring**:
   - Continuous logging of system state
   - Real-time status updates
   - Event tracking and reporting

## Topics

- `detected_objects`: Computer vision detections
- `user_control`: Manual control commands
- `status`: System status and events

## Dependencies

- ROS2
- Jetson Inference
- Python 3.x
- SMBus (I2C communication)
- Socket (TCP/IP communication)
