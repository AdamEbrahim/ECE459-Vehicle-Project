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
        
        # Configure network settings
        self.net = jetson.inference.detectNet(argv=[
            '--model=' + model_path,
            '--input-blob=images',     # Input tensor name from ONNX
            '--output-cvg=output0',    # Output tensor name from ONNX
            '--input-width=640',
            '--input-height=640',
            '--threshold=0.5'          # Detection confidence threshold
        ])
        
        # Initialize camera
        self.camera = jetson.utils.videoSource("csi://0")
        self.display = jetson.utils.videoOutput("display://0")
        
        # Class mapping
        self.class_names = ['stop', 'yield', 'signalAhead', 'pedestrianCrossing', 'speedLimit25', 'speedLimit35']
        
        self.get_logger().info('YOLOv8 Detection Node Initialized')
    
    def detection_callback(self):
        try:
            # Capture frame
            frame = self.camera.Capture()
            
            # Detect objects
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
            
            # Publish results
            if detection_results:
                msg = String()
                msg.data = json.dumps(detection_results)
                self.detection_publisher.publish(msg)
                self.get_logger().debug(f'Published detections: {msg.data}')
            
            # Display output (optional)
            self.display.Render(frame)
            self.display.SetStatus("Object Detection | Network {:.0f} FPS".format(self.net.GetNetworkFPS()))
            
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
