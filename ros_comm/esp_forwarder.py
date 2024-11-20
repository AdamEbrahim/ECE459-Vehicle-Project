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

        if self.is_object_detected:
            if data == 'FORWARD':
                self.get_logger().info('Ignored FORWARD command: Object detected!')
            else:
                # Process other movement commands normally
                if data == 'LEFT':
                    self.get_logger().info('Turning Left')
                    self.bus.write_byte(self.addr, 2)
                elif data == 'RIGHT':
                    self.get_logger().info('Turning Right')
                    self.bus.write_byte(self.addr, 3)
                elif data == 'BACKWARD':
                    self.get_logger().info('Motors Moving Backward')
                    self.bus.write_byte(self.addr, 4)

        else:
            if data == 'FORWARD':
                self.get_logger().info('Motors Moving Forward')
                self.bus.write_byte(self.addr, 1)
            elif data == 'LEFT':
                self.get_logger().info('Turning Left')
                self.bus.write_byte(self.addr, 2)
            elif data == 'RIGHT':
                self.get_logger().info('Turning Right')
                self.bus.write_byte(self.addr, 3)
            elif data == 'BACKWARD':
                self.get_logger().info('Motors Moving Backward')
                self.bus.write_byte(self.addr, 4)

    def listener_callback_2(self, msg):
        data = msg.data
        if data == 'Person Detected':
            self.get_logger().info('Object detected, disabling FORWARD movement')
            self.is_object_detected = True
        else:
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
