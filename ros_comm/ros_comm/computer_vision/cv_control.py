import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import threading
import time

class DummyPerceptionNode(Node):
    def __init__(self):
        super().__init__('dummy_perception_node')
        # Publisher for detected signs
        self.sign_publisher = self.create_publisher(String, 'detected_signs', 10)
        
        # Dictionary to store valid traffic signs and their actions
        self.valid_signs = {
            'stop': 'STOP',
            'speed_limit_20': 'SPEED_20',
            'speed_limit_30': 'SPEED_30',
            'speed_limit_40': 'SPEED_40',
            'speed_limit_50': 'SPEED_50',
            'school_zone': 'SCHOOL_ZONE',
            'yield': 'YIELD'
        }
        
        # Create a timer for simulated detections (checking every 0.5 seconds)
        self.timer = self.create_timer(0.5, self.simulate_detection)
        
        # Store the last detected sign to avoid spam
        self.last_detection = None
        
        self.get_logger().info('Dummy Perception Node initialized')
        
    def simulate_detection(self):
        """
        This method would normally process camera input.
        For testing, it will just wait for manual input.
        The actual implementation will be replaced with real CV later.
        """
        pass  # Empty because we'll manually trigger detections
        
    def process_sign_detection(self, sign_type):
        """
        Process a detected sign and publish appropriate commands
        """
        if sign_type in self.valid_signs:
            msg = String()
            msg.data = self.valid_signs[sign_type]
            self.sign_publisher.publish(msg)
            self.get_logger().info(f'Detected sign: {sign_type} -> Command: {msg.data}')
            return True
        else:
            self.get_logger().warn(f'Invalid sign type: {sign_type}')
            return False
            
    def manual_trigger(self, sign_type):
        """
        Manually trigger a sign detection (for testing)
        """
        return self.process_sign_detection(sign_type)

def main(args=None):
    rclpy.init(args=args)
    node = DummyPerceptionNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
