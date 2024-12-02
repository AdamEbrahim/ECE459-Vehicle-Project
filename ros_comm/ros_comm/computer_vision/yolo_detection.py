#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import cv2
import numpy as np
from ultralytics import YOLO
import os

class YOLODetectionNode(Node):
    def __init__(self):
        super().__init__('yolo_detection')
        
        # Create publisher
        self.detection_pub = self.create_publisher(
            String,
            'detected_objects',
            10
        )
        
        # Parameters
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('frame_rate', 30.0)  # Target FPS
        
        # Get parameters
        self.conf_threshold = self.get_parameter('confidence_threshold').value
        frame_rate = self.get_parameter('frame_rate').value
        
        # Get the path to the model file
        current_dir = os.path.dirname(os.path.abspath(__file__))
        model_path = os.path.join(current_dir, 'models', 'best.pt')
        
        if not os.path.exists(model_path):
            self.get_logger().error(f'Model file not found at {model_path}')
            raise FileNotFoundError(f'Model file not found at {model_path}')
            
        self.get_logger().info(f'Loading model from {model_path}')
        
        # Load YOLO model
        self.model = YOLO(model_path)
        
        # Initialize camera using OpenCV
        self.cap = cv2.VideoCapture(0)  # Use default camera
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        
        if not self.cap.isOpened():
            self.get_logger().error('Failed to open camera')
            raise RuntimeError('Failed to open camera')
        
        # Create detection timer
        self.timer = self.create_timer(1.0 / frame_rate, self.detect_objects)
        
        # Window name for display
        self.window_name = 'YOLOv8 Detections'
        cv2.namedWindow(self.window_name, cv2.WINDOW_AUTOSIZE)
        
        self.get_logger().info('YOLO Detection Node initialized')
    
    def detect_objects(self):
        """Process frame and detect objects"""
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn('Failed to grab frame')
            return
            
        # Run inference
        results = self.model(frame, conf=self.conf_threshold, verbose=False)[0]
        
        # Process detections
        detections = []
        annotated_frame = frame.copy()
        
        for r in results.boxes.data.tolist():
            x1, y1, x2, y2, conf, cls = r
            
            # Get class name
            class_name = self.model.names[int(cls)]
            
            # Create detection dict
            detection = {
                'class': class_name,
                'confidence': round(float(conf), 3),
                'bbox': {
                    'x1': round(float(x1), 2),
                    'y1': round(float(y1), 2),
                    'x2': round(float(x2), 2),
                    'y2': round(float(y2), 2)
                }
            }
            detections.append(detection)
            
            # Draw on frame
            cv2.rectangle(annotated_frame, 
                        (int(x1), int(y1)), 
                        (int(x2), int(y2)), 
                        (0, 255, 0), 2)
            cv2.putText(annotated_frame, 
                       f'{class_name} {conf:.2f}', 
                       (int(x1), int(y1 - 10)), 
                       cv2.FONT_HERSHEY_SIMPLEX, 
                       0.5, 
                       (0, 255, 0), 
                       2)
        
        # Publish detections
        if detections:
            msg = String()
            msg.data = json.dumps(detections)
            self.detection_pub.publish(msg)
            self.get_logger().debug(f'Published {len(detections)} detections')
        
        # Display frame
        cv2.imshow(self.window_name, annotated_frame)
        cv2.waitKey(1)
    
    def destroy_node(self):
        """Cleanup when node is shut down"""
        self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = YOLODetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
