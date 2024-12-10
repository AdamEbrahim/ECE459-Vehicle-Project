import rclpy
from rclpy.node import Node
from smbus import SMBus
from std_msgs.msg import String

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.bus = SMBus(1)
        self.addr = 0x48
        self.speed = 0
        self.send_speed = 0
        self.red_light = 0
        self.stop_sign = 0
        
        self.subscription1 = self.create_subscription(
            String,
            'user_control', 
            self.listener_callback_1,
            10)
        
        self.subscription2 = self.create_subscription(
            String,
            'detected_objects', 
            self.listener_callback_2,
            10)

        self.subscription1  
        self.subscription2  

    def listener_callback_1(self, msg):
        """Handles user control commands from the laptop."""
        data = msg.data
        split_data = data.split(' ')
        self.new_speed = self.speed

        if split_data[0] == 'Red': #laptop RED command
            self.get_logger().info('Red Light Inputed')
            self.red_light = 1

        if split_data[0] == 'Speed': #laptop SPEED command
            self.speed = int(split_data[2])
            self.new_speed = self.speed
            self.get_logger().info('Speed Inputed {}'.format(self.speed))

        elif split_data[0] == 'Pedestrian': #laptop PEDESTRIAN command
            self.get_logger().info('Pedestrian Inputted, halving speed')
            self.new_speed = self.speed // 2
        
        # Movement controls
        if self.red_light or data == 'Nothing': # no movement if no command or red light flag is set
            self.get_logger().info('All movements stopped due to detected conditions')
            self.bus.write_byte(self.addr, 0)  # Stop motors
        elif data == 'Forward':
            self.get_logger().info('Motors Moving Forward')
            self.bus.write_byte(self.addr, 1)
        elif data == 'Left':
            self.get_logger().info('Turning Left')
            self.bus.write_byte(self.addr, 2)
        elif data == 'Right':
            self.get_logger().info('Turning Right')
            self.bus.write_byte(self.addr, 3)
        elif data == 'Backward':
            self.get_logger().info('Motors Moving Backward')
            self.bus.write_byte(self.addr, 4)
           
        # Write speed and stop sign state
        self.bus.write_byte(self.addr, self.new_speed)
        self.bus.write_byte(self.addr, self.stop_sign) # send stop sign state to ESP

    def listener_callback_2(self, msg):
        """Handles detected objects from the CV model."""
        data = msg.data
        if data == 'stopSign':
            self.get_logger().info('Stop Sign Detected')
            self.stop_sign = 1 # set stop sign flag
        elif data == 'trafficLightRed':
            self.get_logger().info('Red Light Detected')
            self.red_light = 1 # set red light flag
        elif data == 'trafficLightGreen':
            self.get_logger().info('Green Light Detected')
            self.red_light = 0  # reset red light flag , Allow movement
        elif data == 'speedLimit20':
            self.get_logger().info('Speed Limit 20 Detected')
            self.speed = 20
        elif data == 'speedLimit50':
            self.get_logger().info('Speed Limit 50 Detected')
            self.speed = 50
        elif data == 'speedLimit70':
            self.get_logger().info('Speed Limit 70 Detected')
            self.speed = 70
        elif data == 'speedLimit100':
            self.get_logger().info('Speed Limit 100 Detected')
            self.speed = 100
        elif data == 'pedestrianCrossing':
            self.get_logger().info('Pedestrian Crossing Detected, halving speed')
            self.speed = max(self.speed // 2, 10)  # Reduce speed but not below 10
        else:
            self.get_logger().info('No relevant object detected, normal operation')
            self.stop_sign = 0 # is not a stop sign, reset flag

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()