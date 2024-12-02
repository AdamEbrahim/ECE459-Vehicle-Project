#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import cv2
import numpy as np
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
        self.declare_parameter('input_width', 640)
        self.declare_parameter('input_height', 640)
        
        # Get parameters
        self.conf_threshold = self.get_parameter('confidence_threshold').value
        frame_rate = self.get_parameter('frame_rate').value
        self.input_width = self.get_parameter('input_width').value
        self.input_height = self.get_parameter('input_height').value
        
        # Get the path to the model files
        current_dir = os.path.dirname(os.path.abspath(__file__))
        model_dir = os.path.join(current_dir, 'models')
        model_path = os.path.join(model_dir, 'best.onnx')
        
        if not os.path.exists(model_path):
            self.get_logger().error(f'Model file not found at {model_path}')
            raise FileNotFoundError(f'Model file not found at {model_path}')
            
        # Load class names
        self.class_names = []
        with open(os.path.join(model_dir, 'labels.txt'), 'r') as f:
            self.class_names = [line.strip() for line in f.readlines()]
            
        self.get_logger().info(f'Loaded {len(self.class_names)} classes')
        
        # Load YOLO model using OpenCV DNN
        self.net = cv2.dnn.readNetFromONNX(model_path)
        
        # Use CUDA backend if available
        self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_CUDA)
        self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CUDA)
        
        # Initialize camera
        self.cap = cv2.VideoCapture(0)
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
        
        # Prepare image for inference
        blob = cv2.dnn.blobFromImage(frame, 1/255.0, (self.input_width, self.input_height), 
                                   swapRB=True, crop=False)
        self.net.setInput(blob)
        
        # Run inference
        outputs = self.net.forward()
        
        # Process detections
        detection_list = []
        frame_height, frame_width = frame.shape[:2]
        
        # YOLOv8 output format: (num_boxes, 84) where 84 = 4(bbox) + 80(class scores)
        outputs = outputs[0].transpose((1, 0))
        
        # Process each detection
        for detection in outputs:
            confidence = float(detection[4])
            if confidence < self.conf_threshold:
                continue
            
            # Get class scores
            class_scores = detection[4:]
            class_id = np.argmax(class_scores)
            
            if class_id < len(self.class_names):  # Make sure class_id is valid
                # Get bounding box coordinates
                x = float(detection[0])
                y = float(detection[1])
                w = float(detection[2])
                h = float(detection[3])
                
                # Convert to corner coordinates
                x1 = int((x - w/2) * frame_width)
                y1 = int((y - h/2) * frame_height)
                x2 = int((x + w/2) * frame_width)
                y2 = int((y + h/2) * frame_height)
                
                # Create detection dict
                detection_info = {
                    'class': self.class_names[class_id],
                    'confidence': round(confidence, 3),
                    'bbox': {
                        'x1': x1,
                        'y1': y1,
                        'x2': x2,
                        'y2': y2
                    }
                }
                detection_list.append(detection_info)
                
                # Draw on frame
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(frame, 
                           f'{self.class_names[class_id]} {confidence:.2f}',
                           (x1, y1 - 10),
                           cv2.FONT_HERSHEY_SIMPLEX,
                           0.5,
                           (0, 255, 0),
                           2)
        
        # Publish detections
        if detection_list:
            msg = String()
            msg.data = json.dumps(detection_list)
            self.detection_pub.publish(msg)
            self.get_logger().debug(f'Published {len(detection_list)} detections')
        
        # Display frame
        cv2.imshow(self.window_name, frame)
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
