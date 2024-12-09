# import rclpy
# from rclpy.node import Node
# from smbus import SMBus
# import time

# from std_msgs.msg import String


# class MinimalSubscriber(Node):

#     def __init__(self):
#         super().__init__('minimal_subscriber')
#         self.bus = SMBus(1)
#         self.addr = 0x48
#         self.subscription = self.create_subscription(
#             String,
#             'topic',
#             self.listener_callback,
#             10)
#         self.subscription  # prevent unused variable warning

#     def listener_callback(self, msg):
#         data = msg.data
#         if data == 'FORWARD':
#             self.get_logger().info('Motors Moving Forward')
#             self.bus.write_byte(self.addr, 1)
#         elif data == 'LEFT':
#             self.get_logger().info('Turning Left')
#             self.bus.write_byte(self.addr, 2)
#         elif data == 'RIGHT':
#             self.get_logger().info('Turning Right')
#             self.bus.write_byte(self.addr, 3)
#         elif data == 'BACKWARD':
#             self.get_logger().info('Motors Moving Backward')
#             self.bus.write_byte(self.addr, 4)

# def main(args=None):
#     rclpy.init(args=args)

#     minimal_subscriber = MinimalSubscriber()

#     rclpy.spin(minimal_subscriber)

#     # Destroy the node explicitly
#     # (optional - otherwise it will be done automatically
#     # when the garbage collector destroys the node object)
#     minimal_subscriber.destroy_node()
#     rclpy.shutdown()


# if __name__ == '__main__':
#     main()

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
        
        self.is_object_detected = False

        self.subscription1 = self.create_subscription(
            String,
            'topic', 
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
        data = msg.data
        split_data = data.split(' ')
        self.new_speed = self.speed
   
        if split_data[0] == 'Red':
            self.red_light = 1
        else:
            self.red_light = 0

        if split_data[0] == 'Speed':
            self.speed = int(split_data[2])
            self.new_speed = self.speed
        elif split_data[0] == 'Pedestrian':
            self.new_speed = self.speed // 2
      
        if self.red_light or data == 'Nothing':
            self.bus.write_byte(self.addr, 0)
    
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
           
        self.bus.write_byte(self.addr, self.new_speed)
        self.bus.write_byte(self.addr, self.stop_sign)

    def listener_callback_2(self, msg):
        data = msg.data
        if data == 'stop sign':
            self.get_logger().info('Stop Sign Detected')
            self.stop_sign = 1
        else:
            self.stop_sign = 0
            self.is_object_detected = False	
            self.get_logger().info('Object no longer detected, all movements allowed')

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
