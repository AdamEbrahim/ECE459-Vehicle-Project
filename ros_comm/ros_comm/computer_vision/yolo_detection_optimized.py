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
        
        # Initialize publishers and subscribers
        self.detection_publisher = self.create_publisher(String, 'detection_data', 10)
        self.create_timer(0.1, self.detection_callback)  # 10Hz detection rate
        
        # Load YOLO model using TensorRT
        current_dir = os.path.dirname(os.path.abspath(__file__))
        model_path = os.path.join(current_dir, 'models', 'best.onnx')
        labels_path = os.path.join(current_dir, 'models', 'labels.txt')
        
        # Configure network settings with more detailed parameters
        self.net = jetson.inference.detectNet(argv=[
            '--model=' + model_path,
            '--labels=' + labels_path,
            '--input-blob=images',
            '--output-blob=output0',
            '--input-width=640',
            '--input-height=640',
            '--threshold=0.5',
            '--network-type=custom'  # Specify custom network
        ])
        
        self.get_logger().info(f'Model loaded from: {model_path}')
        self.get_logger().info(f'Labels loaded from: {labels_path}')
        
        # Initialize camera using the working approach from basic_cv_detection
        try:
            self.camera = jetson.utils.gstCamera(1280, 720, "/dev/video0")
            self.get_logger().info("Successfully initialized USB webcam")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize camera: {str(e)}")
            raise RuntimeError("Camera initialization failed")
            
        # Initialize display
        self.display = jetson.utils.glDisplay()
        if not self.display.IsOpen():
            self.get_logger().error("Failed to open display window")
            raise RuntimeError("Display initialization failed")
        
        # Class mapping
        self.class_names = ['stop', 'yield', 'signalAhead', 'pedestrianCrossing', 'speedLimit25', 'speedLimit35']
        
        self.get_logger().info('YOLOv8 Detection Node Initialized')
    
    def detection_callback(self):
        try:
            # Capture frame in RGBA format
            frame, width, height = self.camera.CaptureRGBA()
            
            # Create output image for overlay
            output = jetson.utils.cudaAllocMapped(width=width, height=height, format='rgba8')
            
            # Detect objects
            detections = self.net.Detect(frame, width, height, overlay=output)
            
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
            
            # Display output
            self.display.RenderOnce(output if detections else frame, width, height)
            self.display.SetTitle(f"Object Detection | Network {self.net.GetNetworkFPS():.0f} FPS")
            
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
