#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import cv2
import numpy as np
import tensorrt as trt
import pycuda.driver as cuda
import pycuda.autoinit
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
            
            # Load TensorRT engine
            current_dir = os.path.dirname(os.path.abspath(__file__))
            model_path = os.path.join(current_dir, 'models', 'best.engine')
            
            if not os.path.exists(model_path):
                self.get_logger().error(f"Model not found at {model_path}")
                raise FileNotFoundError(f"Model not found at {model_path}")
            
            # Initialize TensorRT
            logger = trt.Logger(trt.Logger.INFO)
            with open(model_path, 'rb') as f:
                engine_data = f.read()
            
            runtime = trt.Runtime(logger)
            self.engine = runtime.deserialize_cuda_engine(engine_data)
            self.context = self.engine.create_execution_context()
            
            # Allocate memory
            self.inputs = []
            self.outputs = []
            self.bindings = []
            
            for binding in self.engine:
                size = trt.volume(self.engine.get_binding_shape(binding))
                dtype = trt.nptype(self.engine.get_binding_dtype(binding))
                # Allocate host and device memory
                host_mem = cuda.pagelocked_empty(size, dtype)
                device_mem = cuda.mem_alloc(host_mem.nbytes)
                # Append the device buffer to bindings.
                self.bindings.append(int(device_mem))
                # Append to the appropriate list.
                if self.engine.binding_is_input(binding):
                    self.inputs.append({'host': host_mem, 'device': device_mem})
                else:
                    self.outputs.append({'host': host_mem, 'device': device_mem})
            
            self.get_logger().info(f'Model loaded successfully from: {model_path}')
            
            # Initialize camera
            self.cap = cv2.VideoCapture(0)
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
            
            # Create timer for detection
            self.create_timer(1.0/30.0, self.detect_callback)  # 30 FPS
            
        except Exception as e:
            self.get_logger().error(f"Failed to initialize: {str(e)}")
            raise
    
    def preprocess_image(self, frame):
        # Resize to model input size
        input_size = (640, 640)
        resized = cv2.resize(frame, input_size)
        # Convert to RGB and normalize
        rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
        normalized = rgb.astype(np.float32) / 255.0
        # Change to CHW format
        chw = normalized.transpose((2, 0, 1))
        # Add batch dimension
        return np.expand_dims(chw, axis=0)
    
    def detect_callback(self):
        try:
            ret, frame = self.cap.read()
            if not ret:
                self.get_logger().warn("Failed to grab frame")
                return
                
            # Preprocess image
            preprocessed = self.preprocess_image(frame)
            
            # Copy preprocessed data to input buffer
            np.copyto(self.inputs[0]['host'], preprocessed.ravel())
            
            # Transfer input data to device
            for inp in self.inputs:
                cuda.memcpy_htod(inp['device'], inp['host'])
                
            # Run inference
            self.context.execute_v2(self.bindings)
            
            # Transfer predictions back
            for out in self.outputs:
                cuda.memcpy_dtoh(out['host'], out['device'])
            
            # Process detections (output shape: 1x10x8400)
            predictions = self.outputs[0]['host'].reshape(1, 10, 8400)
            
            # Get detections
            detections = []
            for i in range(predictions.shape[2]):  # iterate over 8400 predictions
                class_scores = predictions[0, 4:, i]  # class scores start at index 4
                class_id = np.argmax(class_scores)
                confidence = class_scores[class_id]
                
                if confidence > 0.5:  # confidence threshold
                    x, y, w, h = predictions[0, :4, i]
                    detection = {
                        'class': self.class_names[class_id],
                        'confidence': float(confidence),
                        'bbox': [float(x), float(y), float(w), float(h)]
                    }
                    detections.append(detection)
            
            # Publish detections
            msg = String()
            msg.data = json.dumps({'detections': detections})
            self.detection_publisher.publish(msg)
            
        except Exception as e:
            self.get_logger().error(f"Detection error: {str(e)}")
    
    def __del__(self):
        if hasattr(self, 'cap'):
            self.cap.release()

def main(args=None):
    rclpy.init(args=args)
    node = YOLODetectionNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
