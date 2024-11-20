#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import smbus2
from std_msgs.msg import String

class ESPForwarder(Node):
    """
    Enhanced ESP Forwarder that handles speed-controlled commands
    
    Protocol Specification:
    Command Format (1 byte):
    - Bits 7-4: Speed modifier
      0001: Slow speed    (0x10)
      0010: Normal speed  (0x20)
      0011: Custom speed  (0x30 | speed_value)
    - Bits 3-0: Base command
      0000: STOP     (0x00)
      0001: FORWARD  (0x01)
      0010: LEFT     (0x02)
      0011: RIGHT    (0x03)
      0100: BACKWARD (0x04)
    """
    
    # Protocol Constants - these are fixed parts of the communication protocol
    # and should not be in the config file
    CMD_STOP = 0x00
    CMD_FORWARD = 0x01
    CMD_LEFT = 0x02
    CMD_RIGHT = 0x03
    CMD_BACKWARD = 0x04
    
    # Speed modifiers (bits 7-4)
    SPEED_SLOW = 0x10    # Fixed slow speed
    SPEED_NORMAL = 0x20  # Fixed normal speed
    SPEED_CUSTOM = 0x30  # Custom speed in lower 4 bits
    
    def __init__(self):
        super().__init__('esp_forwarder')
        
        # Load configurable parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('i2c_address', 0x48),
                ('i2c_bus', 1),
                ('command_rate', 50.0),
                ('speed_levels.slow', 25),
                ('speed_levels.normal', 50),
                ('speed_levels.fast', 75)
            ]
        )
        
        # Get parameter values
        self.i2c_address = self.get_parameter('i2c_address').value
        self.i2c_bus = self.get_parameter('i2c_bus').value
        self.command_rate = self.get_parameter('command_rate').value
        
        # Initialize I2C
        try:
            self.bus = smbus2.SMBus(self.i2c_bus)
            self.get_logger().info(f'I2C initialized on bus {self.i2c_bus}')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize I2C: {str(e)}')
            raise
        
        # Create command subscription
        self.command_sub = self.create_subscription(
            String,
            'motor_commands',
            self.handle_command,
            10
        )
        
        self.get_logger().info('ESP Forwarder initialized')
        
        # Subscribe to motor commands
        self.subscription = self.create_subscription(
            String,
            'motor_commands',
            self.command_callback,
            10
        )
        
        self.slow_speed = self.get_parameter('speed_levels.slow').value
        self.normal_speed = self.get_parameter('speed_levels.normal').value
        self.fast_speed = self.get_parameter('speed_levels.fast').value
        
    def command_callback(self, msg):
        """
        Handle incoming motor commands
        Format: COMMAND or COMMAND_SPEED
        Examples: 
        - "FORWARD"
        - "FORWARD_SLOW"
        - "FORWARD_50" (50% speed)
        """
        command = msg.data
        
        # Parse command and speed
        base_cmd = None
        speed_modifier = self.SPEED_NORMAL  # Default to normal speed
        
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
                    # Convert speed percentage (0-100) to 4-bit value (0-15)
                    speed = int(speed_str)
                    speed = max(0, min(100, speed))  # Clamp between 0-100
                    speed_4bit = (speed * 15) // 100  # Convert to 4-bit value
                    speed_modifier = self.SPEED_CUSTOM | speed_4bit
                except ValueError:
                    self.get_logger().warn(f'Invalid speed value: {speed_str}')
                    return
        else:
            base_command = command
        
        # Map base command
        if base_command == 'FORWARD':
            base_cmd = self.CMD_FORWARD
        elif base_command == 'LEFT':
            base_cmd = self.CMD_LEFT
        elif base_command == 'RIGHT':
            base_cmd = self.CMD_RIGHT
        elif base_command == 'BACKWARD':
            base_cmd = self.CMD_BACKWARD
        else:
            self.get_logger().warn(f'Unknown command: {base_command}')
            return
        
        # Combine base command with speed modifier
        final_cmd = base_cmd | speed_modifier
        self._send_command(final_cmd)
    
    def _send_command(self, command):
        """Send command byte to ESP via I2C"""
        try:
            self.bus.write_byte(self.i2c_address, command)
            self.get_logger().debug(f'Sent command: {hex(command)}')
        except Exception as e:
            self.get_logger().error(f'Failed to send command: {e}')

def main(args=None):
    rclpy.init(args=args)
    forwarder = ESPForwarder()
    
    try:
        rclpy.spin(forwarder)
    except KeyboardInterrupt:
        pass
    finally:
        forwarder.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
