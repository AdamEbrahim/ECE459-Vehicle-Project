#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import jetson.inference
import jetson.utils
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
        model_path = os.path.join(current_dir, 'models', 'best.engine')  # TensorRT engine file
        
        if not os.path.exists(model_path):
            self.get_logger().error(f'Model file not found at {model_path}')
            raise FileNotFoundError(f'Model file not found at {model_path}')
            
        self.get_logger().info(f'Loading model from {model_path}')
        
        # Initialize camera and display
        self.camera = jetson.utils.gstCamera(640, 480, "/dev/video0")
        self.display = jetson.utils.glDisplay()
        
        # Load detection network
        self.net = jetson.inference.detectNet(
            model=model_path,
            labels=os.path.join(current_dir, 'models', 'labels.txt'),
            input_blob="input_0",
            output_cvg="scores",
            output_bbox="boxes",
            threshold=self.conf_threshold
        )
        
        # Create detection timer
        self.timer = self.create_timer(1.0 / frame_rate, self.detect_objects)
        
        self.get_logger().info('YOLO Detection Node initialized')
    
    def detect_objects(self):
        """Process frame and detect objects"""
        # Capture frame
        img, width, height = self.camera.CaptureRGBA(zeroCopy=1)
        
        # Run inference
        detections = self.net.Detect(img, width, height)
        
        # Process detections
        detection_list = []
        for detection in detections:
            # Get detection info
            class_name = self.net.GetClassDesc(detection.ClassID)
            confidence = detection.Confidence
            
            # Create detection dict
            detection_info = {
                'class': class_name,
                'confidence': round(float(confidence), 3),
                'bbox': {
                    'x1': round(float(detection.Left), 2),
                    'y1': round(float(detection.Top), 2),
                    'x2': round(float(detection.Right), 2),
                    'y2': round(float(detection.Bottom), 2)
                }
            }
            detection_list.append(detection_info)
        
        # Publish detections
        if detection_list:
            msg = String()
            msg.data = json.dumps(detection_list)
            self.detection_pub.publish(msg)
            self.get_logger().debug(f'Published {len(detection_list)} detections')
        
        # Display frame
        self.display.RenderOnce(img, width, height)
        self.display.SetTitle(f"Object Detection | Network {self.net.GetNetworkFPS():.0f} FPS")
    
    def destroy_node(self):
        """Cleanup when node is shut down"""
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
