#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import jetson.inference
import jetson.utils
import threading
import json
import time
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
        try:
            # Initialize the detection network
            # Try multiple possible locations for the model files
            possible_model_dirs = [
                os.path.join(current_dir, 'models'),  # Local development
                os.path.join(os.path.dirname(os.path.dirname(current_dir)), 'models'),  # One level up
                os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(current_dir))), 'models'),  # Two levels up
            ]
            
            model_path = None
            labels_path = None
            
            # Search for model files
            for dir_path in possible_model_dirs:
                temp_model = os.path.join(dir_path, 'best.onnx')
                temp_labels = os.path.join(dir_path, 'labels.txt')
                if os.path.exists(temp_model) and os.path.exists(temp_labels):
                    model_path = temp_model
                    labels_path = temp_labels
                    self.get_logger().info(f'Found model files in: {dir_path}')
                    break
            
            if not model_path or not labels_path:
                raise FileNotFoundError(f'Model files not found in any of the searched directories: {possible_model_dirs}')

            self.get_logger().info(f'Loading ONNX model from: {model_path}')
            self.get_logger().info(f'Using labels from: {labels_path}')
            self.get_logger().info(f'Model file size: {os.path.getsize(model_path)} bytes')
            
            try:
                # Initialize the detection network with correct parameter names
                self.net = jetson.inference.detectNet(
                    model=model_path,
                    labels=labels_path,
                    input_name='images',
                    output_names=['output0'],
                    threshold=self.detection_threshold
                )
                self.get_logger().info('Successfully loaded model with YOLO v8 naming')
            except Exception as e:
                self.get_logger().error(f'Model loading failed: {str(e)}')
                raise RuntimeError('Failed to load detection network')

            if not self.net:
                raise RuntimeError('Network initialization failed')
            
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

    def process_detections(self, detections):
        """Process detections and create JSON message."""
        detection_msg = {
            'timestamp': time.time(),
            'detections': []
        }
        
        for detection in detections:
            det_info = {
                'class': detection.ClassLabel,
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
        
        return json.dumps(detection_msg)

    def run_detection_loop(self):
        """Main detection loop"""
        try:
            while rclpy.ok():
                # Capture image
                img, width, height = self.camera.CaptureRGBA(zeroCopy=1)
                
                # Detect objects
                detections = self.net.Detect(img, width, height)
                
                # Process detections
                if detections:
                    msg = String()
                    msg.data = self.process_detections(detections)
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
