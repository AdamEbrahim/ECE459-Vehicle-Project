import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class StatusConsolidator(Node):

    def __init__(self):
        super().__init__('status_consolidator')

        self.last_command = None
        self.last_object = None
        self.current_speed = 0
        self.red_light_active = False

        self.publisher_ = self.create_publisher(String, 'status', 10)

        self.subscription1 = self.create_subscription(
            String,
            'user_control',
            self.user_control_callback,
            10)

        self.subscription2 = self.create_subscription(
            String,
            'detected_objects',
            self.detected_objects_callback,
            10)

    def user_control_callback(self, msg):
        """Handles user control commands."""
        command = msg.data

        # Publish only if the command has changed
        if command != self.last_command:
            self.last_command = command
            self.publish_status(f"Command: {command}, Speed: {self.current_speed}")

    def detected_objects_callback(self, msg):
        """Handles detected objects."""
        detected_object = msg.data

        # Publish only if the detected object has changed
        if detected_object != self.last_object:
            self.last_object = detected_object
            self.publish_status(f"Detection: {detected_object}")

        # Handle specific object logic
        if detected_object == 'trafficLightRed':
            if not self.red_light_active:
                self.red_light_active = True
                self.publish_status("Red light detected: Stopping all movement until green light is detected")
        elif detected_object == 'trafficLightGreen':
            if self.red_light_active:
                self.red_light_active = False
                self.publish_status("Green light detected: Movement allowed to resume")
        elif detected_object.startswith('speedLimit'):
            # Speed limit logic
            new_speed = int(detected_object.replace('speedLimit', ''))
            if new_speed != self.current_speed:
                self.current_speed = new_speed
                self.publish_status(f"Speed updated to {self.current_speed}")
        elif detected_object == 'pedestrianCrossing':
            # Pedestrian crossing logic
            reduced_speed = max(self.current_speed // 2, 10)
            if reduced_speed != self.current_speed:
                self.current_speed = reduced_speed
                self.publish_status(f"Speed reduced for pedestrian crossing: {self.current_speed}")

    def publish_status(self, message):
        """Publishes a status message to the 'status' topic."""
        msg = String()
        msg.data = message
        self.publisher_.publish(msg)
        self.get_logger().info(f"Status Published: {msg.data}")


def main(args=None):
    rclpy.init(args=args)
    status_consolidator = StatusConsolidator()
    rclpy.spin(status_consolidator)
    status_consolidator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
