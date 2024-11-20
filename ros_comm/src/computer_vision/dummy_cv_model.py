#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import random
import json

class DummyCVModel(Node):
    """
    Dummy Computer Vision model that simulates object detection
    Used for testing the perception and control system
    """
    
    def __init__(self):
        super().__init__('computer_vision')
        
        # Declare parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('model_confidence', 0.5),
                ('detection_classes', [
                    'person',
                    'stop sign',
                    'yield sign',
                    'traffic light',
                    'speed limit 30',
                    'speed limit 50',
                    'speed limit 70',
                    'school zone'
                ]),
                ('detection_probabilities', {
                    'person': 0.3,
                    'stop sign': 0.4,
                    'yield sign': 0.4,
                    'traffic light': 0.4,
                    'speed limit 30': 0.3,
                    'speed limit 50': 0.3,
                    'speed limit 70': 0.3,
                    'school zone': 0.3
                }),
                ('frame_rate', 30.0)
            ]
        )
        
        # Get parameters
        self.model_confidence = self.get_parameter('model_confidence').value
        self.detection_classes = self.get_parameter('detection_classes').value
        self.detection_probs = self.get_parameter('detection_probabilities').value
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
        
        # Simulate detection for each class
        for obj_class in self.detection_classes:
            if random.random() < self.detection_probs.get(obj_class, 0.3):
                detection = {
                    'class': obj_class,
                    'confidence': round(random.uniform(self.model_confidence, 1.0), 2),
                    'bbox': self._generate_bbox(),
                }
                
                # Add specific details for certain classes
                if 'speed limit' in obj_class:
                    speed = int(obj_class.split()[-1])
                    detection['speed_value'] = speed
                elif obj_class == 'traffic light':
                    detection['color'] = random.choice(['red', 'yellow', 'green'])
                
                detections.append(detection)
        
        if detections:
            # Publish detections as JSON string
            msg = String()
            msg.data = json.dumps(detections)
            self.detection_pub.publish(msg)
            self.get_logger().debug(f'Published detections: {msg.data}')
    
    def _generate_bbox(self):
        """Generate a random bounding box"""
        x = random.randint(0, 1280)
        y = random.randint(0, 720)
        w = random.randint(50, 200)
        h = random.randint(50, 200)
        return [x, y, w, h]

def main(args=None):
    rclpy.init(args=args)
    cv_model = DummyCVModel()
    
    try:
        rclpy.spin(cv_model)
    except KeyboardInterrupt:
        pass
    finally:
        cv_model.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
