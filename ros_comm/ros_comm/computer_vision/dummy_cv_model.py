#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import random
import json

class DummyCVModel(Node):
    def __init__(self):
        super().__init__('computer_vision')
        
        # Declare parameters individually to ensure correct typing
        self.declare_parameter('model_confidence', 0.5)
        self.declare_parameter('detection_classes', ['person', 'stop sign', 'yield sign', 'traffic light', 
                                                   'speed limit 30', 'speed limit 50', 'speed limit 70', 'school zone'])
        self.declare_parameter('frame_rate', 1.0/3.0)  # One detection every 3 seconds
        
        # Get parameters
        self.model_confidence = self.get_parameter('model_confidence').value
        self.detection_classes = self.get_parameter('detection_classes').value
        frame_rate = self.get_parameter('frame_rate').value
        
        # Create detection publisher
        self.detection_pub = self.create_publisher(
            String,
            'detected_objects',
            10
        )
        
        # Create timer for simulated detections
        self.timer = self.create_timer(1.0 / frame_rate, self.simulate_detection)
        
        self.get_logger().info('Dummy CV Model initialized')
        self.get_logger().info(f'Detection classes: {self.detection_classes}')
    
    def simulate_detection(self):
        """
        Simulate object detection by selecting one random object
        """
        # Select one random object
        obj_class = random.choice(self.detection_classes)
        
        # Create the detection with high confidence
        detection = {
            'class': obj_class,
            'confidence': round(random.uniform(0.8, 1.0), 3),
            'bbox': self._generate_bbox()
        }
        
        # Add specific details for certain classes
        if 'speed limit' in obj_class:
            speed = int(obj_class.split()[-1])  # Extract speed value from class name
            detection['speed_value'] = speed
        elif obj_class == 'traffic light':
            detection['color'] = random.choice(['red', 'yellow', 'green'])
        
        # Create and publish message
        msg = String()
        msg.data = json.dumps([detection])  # Wrap in list as we're sending one detection
        self.detection_pub.publish(msg)
        self.get_logger().info(f'Published detection: {msg.data}')
    
    def _generate_bbox(self):
        """Generate a random bounding box"""
        x = random.uniform(0, 1)
        y = random.uniform(0, 1)
        w = random.uniform(0.1, 0.3)
        h = random.uniform(0.1, 0.3)
        return [x, y, w, h]

def main(args=None):
    rclpy.init(args=args)
    cv_model = DummyCVModel()
    rclpy.spin(cv_model)
    cv_model.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
