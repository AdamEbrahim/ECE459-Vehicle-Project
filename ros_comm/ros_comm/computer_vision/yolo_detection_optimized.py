#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import cv2
import numpy as np
import jetson.inference
from jetson.utils import gstCamera, glDisplay, cudaAllocMapped, cudaResize, cudaMemcpy
import os

class YOLODetectionNode(Node):
    def __init__(self):
        super().__init__('yolo_detection_node')
        
        # Initialize publishers and subscribers
        self.detection_publisher = self.create_publisher(String, 'detection_data', 10)
        
        # Initialize model
        try:
            # Define class names
            self.class_names = ['stop', 'yield', 'signalAhead', 'pedestrianCrossing', 'speedLimit25', 'speedLimit35']
            
            # Load model
            model_path = os.path.expanduser('~/best_openvino_model_6n')
            self.net = jetson.inference.detectNet(model=model_path, 
                                                labels=','.join(self.class_names),
                                                input_blob='images',
                                                output_cvg='output_1',
                                                output_bbox='output_2',
                                                threshold=0.5)
            
            self.get_logger().info("Successfully loaded model")
        except Exception as e:
            self.get_logger().error(f"Failed to load model: {str(e)}")
            raise
        
        # Initialize camera using the working approach from basic_cv_detection
        try:
            self.camera = gstCamera(1280, 720, "/dev/video0")
            self.get_logger().info("Successfully initialized USB webcam")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize camera: {str(e)}")
            raise
            
        # Initialize display
        self.display = glDisplay()
        if not self.display.IsOpen():
            self.get_logger().error("Failed to open display window")
            raise RuntimeError("Display initialization failed")
            
        # Create timer for detection
        self.create_timer(0.1, self.detection_callback)  # 10Hz detection rate
        self.get_logger().info('YOLOv8 Detection Node Initialized')
    
    def detection_callback(self):
        try:
            # Capture frame in RGBA format
            frame, width, height = self.camera.CaptureRGBA()
            
            # Log input frame info
            self.get_logger().debug(f'Input frame: {width}x{height} format={frame.format}')
            
            # Create resized input for the model (640x640)
            input_frame = jetson.utils.cudaAllocMapped(width=640, height=640, format="rgba32f")
            
            # Resize the input frame
            jetson.utils.cudaResize(frame, input_frame)
            
            # Run detection on resized input
            detections = self.net.Detect(input_frame)  # Let detectNet handle dimensions internally
            
            # Process detections and scale coordinates back to original frame size
            detection_results = []
            for detection in detections:
                class_id = int(detection.ClassID)
                if class_id < len(self.class_names):
                    # Scale detection coordinates back to original size
                    scale_x = width / 640
                    scale_y = height / 640
                    left = detection.Left * scale_x
                    top = detection.Top * scale_y
                    det_width = detection.Width * scale_x
                    det_height = detection.Height * scale_y
                    
                    detection_results.append({
                        'class': self.class_names[class_id],
                        'confidence': float(detection.Confidence),
                        'bbox': {
                            'x': float(left),
                            'y': float(top),
                            'width': float(det_width),
                            'height': float(det_height)
                        }
                    })
                    self.get_logger().info(f'Detected {self.class_names[class_id]} with confidence {detection.Confidence}')
            
            # Publish results
            if detection_results:
                msg = String()
                msg.data = json.dumps(detection_results)
                self.detection_publisher.publish(msg)
                self.get_logger().debug(f'Published detections: {msg.data}')
            
            # Display the frame with detections (original frame with overlays)
            self.display.RenderOnce(frame, width, height)
            self.display.SetTitle(f"Traffic Sign Detection | Network {self.net.GetNetworkFPS():.0f} FPS")
            
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
