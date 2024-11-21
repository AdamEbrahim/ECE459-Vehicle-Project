#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import random
import json

class DummyCVModel(Node):
    """Dummy Computer Vision model that simulates object detection
    Used for testing the perception and control system"""
    
    def __init__(self):
        super().__init__('computer_vision')
        
        # Declare parameters individually to ensure correct typing
        self.declare_parameter('model_confidence', 0.5)
        self.declare_parameter('detection_classes', ['person', 'stop sign', 'yield sign', 'traffic light', 
                                                   'speed limit 30', 'speed limit 50', 'speed limit 70', 'school zone'])
        self.declare_parameter('frame_rate', 0.5)
        
        # Create a dictionary for detection probabilities
        default_probs = {
            'person': 0.3,
            'stop sign': 0.4,
            'yield sign': 0.4,
            'traffic light': 0.4,
            'speed limit 30': 0.3,
            'speed limit 50': 0.3,
            'speed limit 70': 0.3,
            'school zone': 0.3
        }
        # Convert dict to string for parameter storage
        self.declare_parameter('detection_probabilities', json.dumps(default_probs))
        
        # Get parameters
        self.model_confidence = self.get_parameter('model_confidence').value
        self.detection_classes = self.get_parameter('detection_classes').value
        self.detection_probs = json.loads(self.get_parameter('detection_probabilities').value)
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
        Simulate object detection with configurable probabilities
        Returns a JSON string with detection info
        """
        detections = []
        
        # For each class, randomly decide if it's detected
        for obj_class in self.detection_classes:
            if random.random() < self.detection_probs[obj_class]:
                detection = {
                    'class': obj_class,
                    'confidence': round(random.uniform(self.model_confidence, 1.0), 3),
                    'bbox': self._generate_bbox()
                }

                # Add specific details for certain classes
                if 'speed limit' in obj_class:
                    speed = int(obj_class.split()[-1])  # Extract speed value from class name
                    detection['speed_value'] = speed
                elif obj_class == 'traffic light':
                    detection['color'] = random.choice(['red', 'yellow', 'green'])
                    
                detections.append(detection)

        
        # Create message
        if detections:
            msg = String()
            msg.data = json.dumps(detections)
            self.detection_pub.publish(msg)
            self.get_logger().debug(f'Published detections: {msg.data}')
    
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
