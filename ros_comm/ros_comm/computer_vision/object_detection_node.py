import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import jetson.inference
import jetson.utils
import json

class TrafficSignDetectionNode(Node):
    def __init__(self):
        super().__init__('traffic_sign_detection_node')
        self.publisher_ = self.create_publisher(String, 'detected_objects', 10)
        self.detection_threshold = 0.5  # Minimum threshold for detection confidence
        self.net = jetson.inference.detectNet("models/ssd-mobilenet-v2.engine", threshold=self.detection_threshold)
        self.camera = jetson.utils.gstCamera(1280, 720, "/dev/video0")

        self.get_logger().info("Traffic Sign Detection Node Initialized")
    
    def detect_objects(self):
        frame, width, height = self.camera.CaptureRGBA()
        detections = self.net.Detect(frame, width, height)

        results = []
        for detection in detections:
            class_name = self.net.GetClassDesc(detection.ClassID)
            results.append({
                "class": class_name,
                "confidence": detection.Confidence,
                "bbox": [detection.Left, detection.Top, detection.Right, detection.Bottom]
            })
        
        msg = String()
        msg.data = json.dumps(results)
        self.publisher_.publish(msg)
        self.get_logger().info(f"Published Detections: {msg.data}")
    
    def start_detection(self):
        while True:
            self.detect_objects()

def main(args=None):
    rclpy.init(args=args)
    node = TrafficSignDetectionNode()
    try:
        node.start_detection()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
