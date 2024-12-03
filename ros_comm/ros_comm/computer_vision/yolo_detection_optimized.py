#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import cv2
import numpy as np
import jetson.inference
import jetson.utils
import os

class YOLODetectionNode(Node):
    def __init__(self):
        super().__init__('yolo_detection_node')
        
        # Initialize publishers
        self.detection_publisher = self.create_publisher(String, 'detection_data', 10)
        
        # Initialize model
        try:
            # Define class names
            self.class_names = ['stop', 'yield', 'signalAhead', 'pedestrianCrossing', 'speedLimit25', 'speedLimit35']
            
            # Load model - simplified initialization
            current_dir = os.path.dirname(os.path.abspath(__file__))
            model_path = os.path.join(current_dir, 'models', 'best.engine')
            
            if not os.path.exists(model_path):
                self.get_logger().error(f"Model not found at {model_path}")
                raise FileNotFoundError(f"Model not found at {model_path}")
                
            self.net = jetson.inference.detectNet(model_path, threshold=0.5)
            self.get_logger().info(f'Model loaded from: {model_path}')
            
        except Exception as e:
            self.get_logger().error(f"Failed to load model: {str(e)}")
            raise
        
        # Initialize camera
        try:
            self.camera = jetson.utils.gstCamera(1280, 720, "/dev/video0")
            self.get_logger().info("Successfully initialized USB webcam")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize camera: {str(e)}")
            raise
            
        # Create timer for detection
        self.create_timer(0.1, self.detection_callback)  # 10Hz detection rate
        self.get_logger().info('YOLOv8 Detection Node Initialized')

    def detection_callback(self):
        try:
            # Capture frame
            frame, width, height = self.camera.CaptureRGBA()
            
            # Log input frame info
            self.get_logger().debug(f'Input frame: {width}x{height}')
            
            # Run detection
            detections = self.net.Detect(frame)
            
            # Process detections
            detection_results = []
            for detection in detections:
                class_id = int(detection.ClassID)
                if class_id < len(self.class_names):
                    detection_results.append({
                        'class': self.class_names[class_id],
                        'confidence': float(detection.Confidence),
                        'bbox': {
                            'x': float(detection.Left),
                            'y': float(detection.Top),
                            'width': float(detection.Width),
                            'height': float(detection.Height)
                        }
                    })
                    self.get_logger().info(f'Detected {self.class_names[class_id]} with confidence {detection.Confidence}')
            
            # Publish results
            if detection_results:
                msg = String()
                msg.data = json.dumps(detection_results)
                self.detection_publisher.publish(msg)
                self.get_logger().debug(f'Published detections: {msg.data}')
            
        except Exception as e:
            self.get_logger().error(f'Detection error: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = YOLODetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
