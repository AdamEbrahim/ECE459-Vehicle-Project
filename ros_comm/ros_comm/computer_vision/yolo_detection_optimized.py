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
import signal

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
            model_path = os.path.join(model_dir, 'best.onnx')
            labels_path = os.path.join(model_dir, 'labels.txt')

            if not os.path.exists(model_path):
                raise FileNotFoundError(f'Model file not found: {model_path}')
            if not os.path.exists(labels_path):
                raise FileNotFoundError(f'Labels file not found: {labels_path}')

            self.get_logger().info(f'Loading ONNX model from: {model_path}')
            self.get_logger().info(f'Model file size: {os.path.getsize(model_path)} bytes')
            
            try:
                # First attempt: Try loading with default input/output names
                self.get_logger().info('Attempting to load model with default names...')
                self.net = jetson.inference.detectNet(
                    model=model_path,
                    labels=labels_path,
                    threshold=self.detection_threshold
                )
            except Exception as e1:
                self.get_logger().warn(f'First loading attempt failed: {str(e1)}')
                try:
                    # Second attempt: Try with explicit YOLO naming
                    self.get_logger().info('Attempting to load model with YOLO naming...')
                    self.net = jetson.inference.detectNet(
                        model=model_path,
                        labels=labels_path,
                        input_blob="images",
                        output_cvg="output0",
                        output_bbox="output1",
                        threshold=self.detection_threshold
                    )
                except Exception as e2:
                    self.get_logger().error(f'Second loading attempt failed: {str(e2)}')
                    self.get_logger().error('Both loading attempts failed. Please verify model format and blob names.')
                    raise RuntimeError("Failed to load detection network")
            
            # Initialize camera and display
            self.camera = jetson.utils.gstCamera(1280, 720, "/dev/video0")
            self.display = jetson.utils.glDisplay()
            if not self.display.IsOpen():
                self.get_logger().error("Failed to open display window")
                return
                
            self.get_logger().info('Camera and display initialized')
            
            # Control flag for the detection loop
            self.running = True
            
            # Start detection thread
            self.detection_thread = threading.Thread(target=self.run_detection_loop)
            self.detection_thread.daemon = True
            self.detection_thread.start()
            
        except Exception as e:
            self.get_logger().error(f'Failed to initialize: {str(e)}')
            raise RuntimeError(f'Failed to initialize: {str(e)}')

    def run_detection_loop(self):
        """Main detection loop"""
        try:
            while self.running and self.display.IsOpen():
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
                        self.get_logger().info(f'Detected: {det_info["class"]} ({det_info["confidence"]:.2f})')
                    
                    # Publish detections
                    msg = String()
                    msg.data = json.dumps(detection_msg)
                    self.publisher_.publish(msg)
                
                # Display the image
                self.display.RenderOnce(img, width, height)
                self.display.SetTitle(f"YOLO Detection | Network {self.net.GetNetworkFPS():.0f} FPS")
                
        except Exception as e:
            self.get_logger().error(f'Detection loop error: {str(e)}')
            self.running = False

    def cleanup(self):
        """Cleanup resources"""
        self.get_logger().info('Cleaning up...')
        self.running = False
        if hasattr(self, 'detection_thread'):
            self.detection_thread.join(timeout=1.0)
        if hasattr(self, 'display'):
            self.display.Close()

def main(args=None):
    rclpy.init(args=args)
    
    # Create and initialize node
    node = YOLODetectionNode()
    
    # Handle shutdown gracefully
    def signal_handler(sig, frame):
        node.get_logger().info('Shutdown signal received...')
        node.cleanup()
        rclpy.shutdown()
        
    signal.signal(signal.SIGINT, signal_handler)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cleanup()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
