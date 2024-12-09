import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import jetson.inference
import jetson.utils
import threading

class ObjectDetectionPublisher(Node):
    def __init__(self):
        super().__init__('object_detection_publisher')
        self.publisher_ = self.create_publisher(String, 'detected_objects', 10)
        self.timer_period = 1.0  # seconds
        self.detection_threshold = 0.5  # Minimum threshold for detection confidence
        self.min_bbox_area = 5000  # Minimum bounding box area to publish
        self.net = jetson.inference.detectNet("ssd-mobilenet-v2", threshold=self.detection_threshold)
        self.camera = jetson.utils.gstCamera(1280, 720, "/dev/video0")
 #       self.display = jetson.utils.glDisplay()

        self.detection_thread = threading.Thread(target=self.run_detection_loop)
        self.detection_thread.daemon = True
        self.detection_thread.start()

        # self.valid_objects = ['stop sign', 'dog', 'sheep', 'cat', 'elephant', 'person', 'bird', 'giraffe']
        # Define the mapping of detected objects to classifications
        self.object_to_classification = {
            'stop sign': 'stopSign',
            'dog': 'trafficLightGreen',
            'sheep': 'speedLimit50',
            'cat': 'trafficLightRed',
            'elephant': 'speedLimit20',
            'person': 'pedestrianCrossing',
            'bird': 'speedLimit100',
            'giraffe': 'speedLimit70'
        }

    def run_detection_loop(self):
        while True:
            img, width, height = self.camera.CaptureRGBA()
            detections = self.net.Detect(img, width, height)
            
            for detection in detections:
                class_name = self.net.GetClassDesc(detection.ClassID)

                # Calculate bounding box area
                bbox_width = detection.Right - detection.Left
                bbox_height = detection.Bottom - detection.Top
                bbox_area = bbox_width * bbox_height
                
                if class_name in self.object_to_classification:
                        if bbox_area > self.min_bbox_area: # Only publish if the bounding box area is large enough          
                            classification = self.object_to_classification[class_name]
                            self.get_logger().info(f"Detected {class_name}, classified as {classification}")
                            self.publish_detection(classification)
                        else:
                            self.get_logger().info(f"Object {class_name} ignored (Area: {bbox_area} too small)")

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
