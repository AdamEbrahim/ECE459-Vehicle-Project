#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json

class PerceptionModel(Node):
    def __init__(self):
        super().__init__('perception')
        
        # Declare parameters
        self.declare_parameter('detection_threshold', 0.7)
        self.declare_parameter('min_detection_size', 50)
        self.declare_parameter('max_detection_distance', 3.0)
        self.declare_parameter('processing_rate', 0.5)
        
        # Get parameter values
        self.detection_threshold = self.get_parameter('detection_threshold').value
        self.min_detection_size = self.get_parameter('min_detection_size').value
        self.max_distance = self.get_parameter('max_detection_distance').value
        processing_rate = self.get_parameter('processing_rate').value
        
        # Publishers and subscribers
        self.command_publisher = self.create_publisher(
            String,
            'perception_commands',
            10
        )
        
        self.detection_sub = self.create_subscription(
            String,
            'detected_objects',
            self.process_detection,
            10
        )
        
        self.get_logger().info('Perception Model initialized')
        self.get_logger().info(f'Detection threshold: {self.detection_threshold}')
    
    def process_detection(self, msg):
        """Process detections and issue appropriate commands"""
        try:
            detections = json.loads(msg.data)
            self.get_logger().info(f'Processing detections: {msg.data}')
            
            highest_priority_command = None
            highest_priority = -1
            
            for detection in detections:
                obj_class = detection['class']
                confidence = detection['confidence']
                
                self.get_logger().info(f'Checking detection: {obj_class} (confidence: {confidence})')
                
                if confidence < self.detection_threshold:
                    self.get_logger().info(f'Skipping {obj_class} - confidence too low')
                    continue
                
                # Priority-based command selection
                command = None
                priority = 0
                
                if obj_class == "stop sign":
                    command = "STOP"
                    priority = 100
                elif obj_class == "person":
                    command = "STOP"
                    priority = 110  # Highest priority
                elif obj_class == "yield sign":
                    command = "SLOW"
                    priority = 80
                elif obj_class == "traffic light":
                    color = detection.get('color', 'unknown')
                    self.get_logger().info(f'Traffic light color: {color}')
                    if color == 'red':
                        command = "STOP"
                        priority = 90
                    elif color == 'yellow':
                        command = "SLOW"
                        priority = 70
                elif "speed limit" in obj_class:
                    speed = detection.get('speed_value', 0)
                    self.get_logger().info(f'Speed limit value: {speed}')
                    command = f"SPEED_{speed}"
                    priority = 50
                elif obj_class == "school zone":
                    command = "SPEED_30"  # Force slow speed in school zones
                    priority = 60
                
                self.get_logger().info(f'Generated command for {obj_class}: {command} (priority: {priority})')
                
                if command and priority > highest_priority:
                    highest_priority_command = command
                    highest_priority = priority
                    self.get_logger().info(f'New highest priority command: {command}')
            
            if highest_priority_command:
                cmd_msg = String()
                cmd_msg.data = highest_priority_command
                self.command_publisher.publish(cmd_msg)
                self.get_logger().info(f'Published final command: {highest_priority_command}')
            else:
                self.get_logger().info('No command issued')
        
        except json.JSONDecodeError as e:
            self.get_logger().error(f'Failed to parse detection JSON: {e}')
        except Exception as e:
            self.get_logger().error(f'Error processing detection: {e}')

def main(args=None):
    rclpy.init(args=args)
    perception = PerceptionModel()
    try:
        rclpy.spin(perception)
    except KeyboardInterrupt:
        pass
    finally:
        perception.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
