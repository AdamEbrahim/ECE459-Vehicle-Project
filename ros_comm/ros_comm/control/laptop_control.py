#!/usr/bin/env python3
import socket
from pynput import keyboard
import time

current_command = "x"  # Start with stop command
sending_enabled = False  # Flag to indicate whether sending is enabled
last_print_time = 0  # To track when the last print occurred

def on_press(key):
    global current_command, sending_enabled
    
    try:
        if key.char == 't':
            sending_enabled = True  # Start sending commands when 't' is pressed
            print("Command sending enabled!")
        if key.char == 'w':
            current_command = 'w'
        elif key.char == 'a':
            current_command = 'a'
        elif key.char == 's':
            current_command = 's'
        elif key.char == 'd':
            current_command = 'd'
        elif key.char == 'x':
            current_command = 'x'
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
    print("\nControls:")
    print("t - Enable command sending")
    print("w - Forward")
    print("s - Backward")
    print("a - Left")
    print("d - Right")
    print("x - Stop")
    print("q - Quit")
    
    listener = keyboard.Listener(on_press=on_press)
    listener.start()

    global last_print_time

    try:
        while True:
            if sending_enabled:
                client_socket.sendall(current_command.encode())  # Send the current command

                # Check if 2 seconds have passed since the last print
                current_time = time.time()
                if current_time - last_print_time >= 2:
                    print(f"Sending command: {current_command}")
                    last_print_time = current_time  # Update the last print time

            time.sleep(0.1)  # Small delay to reduce CPU usage
            
    finally:
        client_socket.close()  # Close the socket
        listener.stop()  # Stop the listener

if __name__ == "__main__":
    start_client()
