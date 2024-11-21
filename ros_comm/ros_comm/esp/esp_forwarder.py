#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from smbus import SMBus
from std_msgs.msg import String

class MotorController(Node):
    # Command mapping for the ESP
    CMD_STOP = 0
    CMD_FORWARD = 1
    CMD_LEFT = 2
    CMD_RIGHT = 3
    CMD_BACKWARD = 4
    
    # Speed modifiers
    SPEED_SLOW = 0x10    # Add to base command for slow speed
    SPEED_NORMAL = 0x20  # Add to base command for normal speed
    SPEED_CUSTOM = 0x30  # Add to base command for custom speed
    
    def __init__(self):
        super().__init__('motor_controller')
        
        # Initialize I2C
        self.bus = SMBus(1)
        self.addr = 0x48
        
        # Subscribe to motor commands
        self.subscription = self.create_subscription(
            String,
            'motor_commands',
            self.command_callback,
            10
        )
        
        self.get_logger().info('Motor Controller initialized')
    
    def command_callback(self, msg):
        """Handle incoming motor commands with speed modifiers"""
        command = msg.data
        
        # Parse command and speed
        base_cmd = None
        speed_modifier = self.SPEED_NORMAL
        
        if command == 'STOP':
            self._send_command(self.CMD_STOP)
            return
            
        # Parse commands with speed modifiers
        if '_' in command:
            base_command, speed_str = command.split('_')
            if speed_str == 'SLOW':
                speed_modifier = self.SPEED_SLOW
            else:
                try:
                    # Convert speed (0-100) to byte value (0-255)
                    speed = int(speed_str)
                    speed = max(0, min(100, speed))  # Clamp between 0-100
                    speed_modifier = self.SPEED_CUSTOM | (speed & 0x0F)
                except ValueError:
                    self.get_logger().warn(f'Invalid speed value: {speed_str}')
                    return
        else:
            base_command = command
        
        # Map base command to motor command
        if base_command == 'FORWARD':
            base_cmd = self.CMD_FORWARD
        elif base_command == 'LEFT':
            base_cmd = self.CMD_LEFT
        elif base_command == 'RIGHT':
            base_cmd = self.CMD_RIGHT
        elif base_command == 'BACKWARD':
            base_cmd = self.CMD_BACKWARD
        
        if base_cmd is not None:
            # Combine base command with speed modifier
            final_cmd = base_cmd | speed_modifier
            self._send_command(final_cmd)
    
    def _send_command(self, command):
        """Send command byte to ESP via I2C"""
        try:
            self.bus.write_byte(self.addr, command)
            self.get_logger().debug(f'Sent command: {hex(command)}')
        except Exception as e:
            self.get_logger().error(f'Failed to send command: {e}')

def main(args=None):
    rclpy.init(args=args)
    motor_controller = MotorController()
    
    try:
        rclpy.spin(motor_controller)
    except KeyboardInterrupt:
        pass
    finally:
        motor_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
