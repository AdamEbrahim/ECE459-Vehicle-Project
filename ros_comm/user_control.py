import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import socket
import threading
import time

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'user_control', 10)
        self.i = 0
        self.socket_thread = threading.Thread(target=self.start_server)
        self.socket_thread.daemon = True
        self.socket_thread.start()

        # Timer to periodically send a default message if there's no activity
        self.timer_period = 1.0  # seconds
        self.last_received_time = time.time()
        self.default_message = 'Idle'
        self.timer = self.create_timer(self.timer_period, self.check_inactivity)

    def start_server(self):
        server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_socket.bind(('0.0.0.0', 12000))
        server_socket.listen(1)
        self.get_logger().info('Server is listening on port 12000')

        conn, addr = server_socket.accept()
        self.get_logger().info(f'Connection established with {addr}')
        old_response = 'w'

        while rclpy.ok():
            data = conn.recv(1024)
            if not data:
                continue 

            self.last_received_time = time.time()

            decode_data = data.decode()
            if decode_data == 'w':
                response = 'Forward'
            elif decode_data == 'a':
                response = 'Left'
            elif decode_data == 's':
                response = 'Backward'
            elif decode_data == 'd':
                response = 'Right'
            elif decode_data == 'f':
                response = 'Speed Limit 30'
            elif decode_data == 'g':
                response = 'Speed Limit 50'
            elif decode_data == 'h':
                response = 'Speed Limit 70'
            elif decode_data == 'j':
                response = 'Stop Sign'
            elif decode_data == 'k':
                response = 'Red Light'
            elif decode_data == 'l':
                response = 'Green Light'
            else:
                response = 'Nothing'

            old_response = decode_data
            self.get_logger().info(f'Received: {decode_data}')
            self.publish_response(response)

            conn.sendall(response.encode())

        conn.close()
        server_socket.close()

    def publish_response(self, response):
        msg = String()
        msg.data = response
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')

    def check_inactivity(self):
        """Check if enough time has passed since the last received data."""
        if time.time() - self.last_received_time > self.timer_period:
            self.publish_response(self.default_message)
    

def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
