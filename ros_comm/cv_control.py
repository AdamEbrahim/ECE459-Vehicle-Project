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
        self.stop_sign_threshold = 0.75  # Min threshold for stop detection confidence
        self.cat_threshold = 0.6
        self.min_bbox_area = 50000  # Minimum bounding box area to publish
        
        self.net = jetson.inference.detectNet("ssd-mobilenet-v2", threshold=self.detection_threshold)
        self.camera = jetson.utils.gstCamera(1280, 720, "/dev/video0")
        # self.camera = jetson.utils.gstCamera(640, 480, "/dev/video0")

        self.gst_pipeline = "v4l1src device=/dev/video0 ! video/x-raw,width=1280,height=720,framerate=10/1 ! videoconvert !appsink"
        # self.display = jetson.utils.glDisplay()

        self.detection_thread = threading.Thread(target=self.run_detection_loop)
        self.detection_thread.daemon = True
        self.detection_thread.start()

        # Define the mapping of detected objects to classifications
        self.object_to_classification = {
            'stop sign': 'stopSign',
            'dog': 'trafficLightGreen',
            'clock': 'speedLimit100',
            'cat': 'trafficLightRed',
            'teddy bear': 'speedLimit20',
            'bottle': 'speedLimit50'
            # 'sheep': 'speedLimit70'
        }

        self.min_areas = {
            'stop sign': 43000,
            'dog': 80000,
            'bottle': 140000,
            'cat': 80000,
            'teddy bear': 36000,
            'clock': 34500
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
                        if bbox_area >= self.min_areas[class_name]:     
                            if ((class_name == 'stop_sign' or class_name == 'clock') and detection.Confidence < self.stop_sign_threshold) or (class_name == 'cat' and detection.Confidence < self.cat_threshold):
                                continue

                            classification = self.object_to_classification[class_name]
                            self.publish_detection(classification)
                            self.get_logger().info(f"Detected {class_name}, classified as {classification} (Area: {bbox_area})\n")
                            
                        else:
                            self.get_logger().info(f"Object {class_name} ignored (Area: {bbox_area} too small)\n")

                else:
                        self.publish_detection("Cleared!")
                        self.get_logger().info("No Object!\n")


                time.sleep(0.2)

            # self.display.RenderOnce(img, width, height)
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
