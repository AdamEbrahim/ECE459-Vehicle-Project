#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import socket
import threading
import time

class UserControl(Node):
    def __init__(self):
        super().__init__('user_control')
        self.publisher_ = self.create_publisher(String, 'user_commands', 10)
        
        # Start socket server in a separate thread
        self.socket_thread = threading.Thread(target=self.start_server)
        self.socket_thread.daemon = True
        self.socket_thread.start()

        # Timer to periodically send a default message if there's no activity
        self.timer_period = 1.0  # seconds
        self.last_received_time = time.time()
        self.timer = self.create_timer(self.timer_period, self.check_inactivity)
        
        self.get_logger().info('User Control initialized')
        self.get_logger().info('Listening for commands on port 12000')

    def start_server(self):
        server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_socket.bind(('0.0.0.0', 12000))
        server_socket.listen(1)

        
        while True:
            conn, addr = server_socket.accept()
            self.get_logger().info(f'Connection established with {addr}')
            last_command = None

            while rclpy.ok():
                try:
                    data = conn.recv(1024)
                    if not data:
                        break

                    self.last_received_time = time.time()
                    command = data.decode().lower()

                    # Map commands to robot controller format
                    if command == 'w':
                        msg = 'FORWARD'
                    elif command == 'a':
                        msg = 'LEFT'
                    elif command == 's':
                        msg = 'BACKWARD'
                    elif command == 'd':
                        msg = 'RIGHT'
                    elif command == 'x':
                        msg = 'STOP'
                    else:
                        msg = last_command if last_command else 'STOP'

                    if msg:
                        last_command = msg
                        message = String()
                        message.data = msg
                        self.publisher_.publish(message)
                        self.get_logger().debug(f'Published: {msg}')

                except Exception as e:
                    self.get_logger().error(f'Error handling connection: {e}')
                    break

            conn.close()
            self.get_logger().info('Connection closed')

    def check_inactivity(self):
        """Stop the robot if no commands received for a while"""
        if time.time() - self.last_received_time > self.timer_period:
            message = String()
            message.data = 'STOP'
            self.publisher_.publish(message)

def main(args=None):
    rclpy.init(args=args)
    user_control = UserControl()
    
    try:
        rclpy.spin(user_control)
    except KeyboardInterrupt:
        pass
    finally:
        user_control.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

