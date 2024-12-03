#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import jetson.inference
import jetson.utils
import threading
import time
import json
import os

class YOLODetectionNode(Node):
    def __init__(self):
        super().__init__('yolo_detection')
        
        # Create publisher
        self.publisher_ = self.create_publisher(String, 'detected_objects', 10)
        
        # Parameters
        self.declare_parameter('confidence_threshold', 0.6)
        self.declare_parameter('input_width', 640)
        self.declare_parameter('input_height', 480)
        
        # Get parameters
        self.detection_threshold = self.get_parameter('confidence_threshold').value
        self.input_width = self.get_parameter('input_width').value
        self.input_height = self.get_parameter('input_height').value

        # Get the path to the model files
        current_dir = os.path.dirname(os.path.abspath(__file__))
        model_dir = os.path.join(current_dir, 'models')
        
        try:
            # Initialize the detection network
            model_path = os.path.join(model_dir, 'best.onnx')  # Use best.onnx specifically
            labels_path = os.path.join(model_dir, 'labels.txt')

            if not os.path.exists(model_path):
                raise FileNotFoundError(f'Model file not found: {model_path}')
            if not os.path.exists(labels_path):
                raise FileNotFoundError(f'Labels file not found: {labels_path}')

            self.get_logger().info(f'Loading ONNX model from: {model_path}')
            self.net = jetson.inference.detectNet(
                model=model_path,
                labels=labels_path,
                input_blob="input",
                output_cvg="output_0",
                output_bbox="output_1",
                threshold=self.detection_threshold
            )
            
            # Initialize camera with higher resolution for better detection
            self.camera = jetson.utils.gstCamera(1280, 720, "/dev/video0")
            self.display = jetson.utils.glDisplay()
            
            self.get_logger().info('Camera and display initialized')
            
            # Start detection thread
            self.detection_thread = threading.Thread(target=self.run_detection_loop)
            self.detection_thread.daemon = True
            self.detection_thread.start()
            
            self.get_logger().info('YOLO Detection Node initialized successfully')
            
        except Exception as e:
            self.get_logger().error(f'Failed to initialize: {str(e)}')
            raise RuntimeError(f'Failed to initialize: {str(e)}')

    def run_detection_loop(self):
        """Main detection loop"""
        try:
            while self.display.IsOpen():
                # Capture frame
                img, width, height = self.camera.CaptureRGBA()
                
                # Run detection
                detections = self.net.Detect(img, width, height)
                
                # Process detections
                if detections:
                    detection_msg = {
                        'timestamp': time.time(),
                        'detections': []
                    }
                    
                    for detection in detections:
                        det_info = {
                            'class': self.net.GetClassDesc(detection.ClassID),
                            'confidence': float(detection.Confidence),
                            'bbox': {
                                'left': int(detection.Left),
                                'top': int(detection.Top),
                                'right': int(detection.Right),
                                'bottom': int(detection.Bottom)
                            }
                        }
                        detection_msg['detections'].append(det_info)
                    
                    # Publish detections
                    msg = String()
                    msg.data = json.dumps(detection_msg)
                    self.publisher_.publish(msg)
                
                # Display the image
                self.display.RenderOnce(img, width, height)
                self.display.SetTitle(f"Object Detection | Network {self.net.GetNetworkFPS():.0f} FPS")
                
        except Exception as e:
            self.get_logger().error(f'Detection loop error: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = YOLODetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
