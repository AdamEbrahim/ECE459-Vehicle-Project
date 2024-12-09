import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import jetson.inference
import jetson.utils
import threading
import time

class ObjectDetectionPublisher(Node):
    def __init__(self):
        super().__init__('object_detection_publisher')
        self.publisher_ = self.create_publisher(String, 'detected_objects', 10)
        self.timer_period = 1.0  # seconds
        self.detection_threshold = 0.5  # Minimum threshold for detection confidence
        self.net = jetson.inference.detectNet("ssd-mobilenet-v2", threshold=self.detection_threshold)
        self.camera = jetson.utils.gstCamera(1280, 720, "/dev/video0")
 #       self.display = jetson.utils.glDisplay()

        self.detection_thread = threading.Thread(target=self.run_detection_loop)
        self.detection_thread.daemon = True
        self.detection_thread.start()

        self.valid_objects = ['stop sign']

    def run_detection_loop(self):
        while True:
            img, width, height = self.camera.CaptureRGBA()
            detections = self.net.Detect(img, width, height)
            
            for detection in detections:
                class_name = self.net.GetClassDesc(detection.ClassID)
                if class_name in self.valid_objects:                    
                        self.get_logger().info("Valid Object Detected!")
                        self.publish_detection(str(class_name))
                else:
                        self.get_logger().info("No Object!")
                        self.publish_detection("Cleared!")

            #self.display.RenderOnce(img, width, height)
           # self.display.SetTitle(f"Object Detection | Network {self.net.GetNetworkFPS():.0f} FPS")

    def publish_detection(self, detection_message):
        """Publish a message about the detection."""
        msg = String()
        msg.data = detection_message
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)

    object_detection_publisher = ObjectDetectionPublisher()

    rclpy.spin(object_detection_publisher)

    object_detection_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
