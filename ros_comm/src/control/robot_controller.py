#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from enum import Enum, auto
import re

class RobotState(Enum):
    SPEED_LIMITED = auto()  # Following a speed limit
    STOPPING = auto()       # Coming to a stop
    SLOWING = auto()        # Temporary slow down
    NORMAL = auto()         # Normal operation

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        
        # Declare parameters
        self.declare_parameter('default_speed', 50)
        self.declare_parameter('slow_speed', 25)
        self.declare_parameter('stop_distance', 1.0)
        self.declare_parameter('turn_speed', 35)
        self.declare_parameter('state_update_rate', 10.0)
        
        # Get parameter values
        self.default_speed = self.get_parameter('default_speed').value
        self.slow_speed = self.get_parameter('slow_speed').value
        self.stop_distance = self.get_parameter('stop_distance').value
        self.turn_speed = self.get_parameter('turn_speed').value
        update_rate = self.get_parameter('state_update_rate').value
        
        # Initialize state
        self.current_state = RobotState.NORMAL
        self.last_command = None
        self.current_speed_limit = self.default_speed
        
        # Publishers and subscribers
        self.command_publisher = self.create_publisher(
            String, 
            'motor_commands', 
            10
        )
        
        self.user_command_sub = self.create_subscription(
            String,
            'user_commands',
            self.handle_user_command,
            10
        )
        
        self.perception_sub = self.create_subscription(
            String,
            'perception_commands',
            self.handle_perception_command,
            10
        )
        
        # Create timer for state updates
        self.create_timer(1.0/update_rate, self.update_state)
        
        self.get_logger().info('Robot Controller initialized')
        self.get_logger().info(f'Default speed: {self.default_speed}')
    
    def update_state(self):
        """Update robot state and apply appropriate speed modifications"""
        if self.last_command is None:
            return
            
        # Apply speed modifications based on state
        if self.current_state == RobotState.STOPPING:
            self.command_publisher.publish(String(data='STOP'))
        elif self.current_state == RobotState.SLOWING:
            cmd = f"{self.last_command}_{self.slow_speed}"
            self.command_publisher.publish(String(data=cmd))
        elif self.current_state == RobotState.SPEED_LIMITED:
            cmd = f"{self.last_command}_{self.current_speed_limit}"
            self.command_publisher.publish(String(data=cmd))
        else:  # NORMAL
            cmd = f"{self.last_command}_{self.default_speed}"
            self.command_publisher.publish(String(data=cmd))
    
    def handle_user_command(self, msg):
        """Handle commands from user control"""
        command = msg.data
        
        if command == 'STOP':
            self.current_state = RobotState.STOPPING
            self.last_command = None
        else:
            self.last_command = command
            # Keep current state unchanged
    
    def handle_perception_command(self, msg):
        """
        Handle commands from perception system
        Commands can be:
        - STOP_DETECTED: Emergency stop
        - SLOW_DOWN: Temporary slow down
        - SPEED_X: Set speed limit to X (e.g., SPEED_30)
        - CLEAR: Resume normal operation
        """
        command = msg.data
        
        if command == 'STOP_DETECTED':
            self.current_state = RobotState.STOPPING
        elif command == 'SLOW_DOWN':
            self.current_state = RobotState.SLOWING
        elif command == 'CLEAR':
            self.current_state = RobotState.NORMAL
            self.current_speed_limit = self.default_speed
        else:
            # Check for speed limit commands (SPEED_X)
            speed_match = re.match(r'SPEED_(\d+)', command)
            if speed_match:
                speed = int(speed_match.group(1))
                self.current_speed_limit = min(speed, self.default_speed)
                self.current_state = RobotState.SPEED_LIMITED
                self.get_logger().info(f'Speed limit set to {self.current_speed_limit}')

def main(args=None):
    rclpy.init(args=args)
    controller = RobotController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
