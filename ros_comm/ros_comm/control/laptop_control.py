#!/usr/bin/env python3
import socket
from pynput import keyboard
import time

current_command = "FORWARD"
sending_enabled = False  # Flag to indicate whether sending is enabled
last_print_time = 0  # To track when the last print occurred

def on_press(key):
    global current_command, sending_enabled
    
    try:
        if key.char == 't':
            sending_enabled = True  # Start sending commands when 's' is pressed
        if key.char == 'w':
            current_command = "FORWARD"
        elif key.char == 'a':
            current_command = "LEFT"
        elif key.char == 's':
            current_command = "BACKWARD"
        elif key.char == 'd':
            current_command = "RIGHT"
        elif key.char == 'q':
            print("Exiting...")
            sending_enabled = False  # Stop sending commands
            return False  # Exit the listener and the program
    except AttributeError:
        pass  # Handle special keys (like shift, ctrl, etc.)

def start_client():
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    
    server_ip = '10.194.159.230'  
    server_port = 12000

    print(f"Connecting to {server_ip}:{server_port}...")
    client_socket.connect((server_ip, server_port))
    print("Connected to the server.")
    
    listener = keyboard.Listener(on_press=on_press)
    listener.start()

    global last_print_time  # To access and update the last_print_time

    try:
        while True:
            if sending_enabled:
                client_socket.sendall(current_command.encode())  # Send the current command

                # Check if 2 seconds have passed since the last print
                current_time = time.time()
                if current_time - last_print_time >= 2:
                    print(f"Sending command: {current_command}")
                    last_print_time = current_time  # Update the last print time

                response = client_socket.recv(1024)  # Receive the server's response
                print(f"Response from server: {response.decode()}")

            time.sleep(0.1)  # Small delay to reduce CPU usage
            
    finally:
        client_socket.close()  # Close the socket
        listener.stop()  # Stop the listener

if __name__ == "__main__":
    start_client()
