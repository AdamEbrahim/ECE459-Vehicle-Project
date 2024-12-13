import socket
from pynput import keyboard
import time

pressed_keys = set()  # Store currently pressed keys
sending_enabled = False  # Flag to indicate whether sending is enabled

def on_press(key):
    global sending_enabled

    try:
        if key.char == 't':
            sending_enabled = True  # Start sending commands when 't' is pressed
            print("Command sending enabled!")
        elif key.char == 'q':
            print("Exiting...")
            sending_enabled = False  # Stop sending commands
            return False  # Exit the listener and the program
        else:
            pressed_keys.add(key.char)  # Add pressed key to the set
    except AttributeError:
        pass  # Handle special keys (like shift, ctrl, etc.)

def on_release(key):
    try:
        pressed_keys.discard(key.char)  # Remove released key from the set
    except AttributeError:
        pass  # Handle special keys (like shift, ctrl, etc.)

def start_client():
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    
    server_ip = '10.194.159.230'  
    server_port = 12001 

    print(f"Connecting to {server_ip}:{server_port}...")
    client_socket.connect((server_ip, server_port))
    print("Connected to the server.")
    print("\nControls:")
    print("t - Enable command sending")
    print("w - Forward")
    print("s - Backward")
    print("a - Left")
    print("d - Right")
    print("f - Speed Limit 30")
    print("g - Speed Limit 50")
    print("h - Speed Limit 100")
    print("j - Stop Sign")
    print("k - Red Light")
    print("l - Green Light")
    print("y - Pedestrian")
    print("x - Stop")
    print("q - Quit")
    
    listener = keyboard.Listener(on_press=on_press, on_release=on_release)
    listener.start()

    try:
        while True:
            if sending_enabled and pressed_keys:
                # Combine pressed keys into a single string
                current_command = ''.join(sorted(pressed_keys))
                client_socket.sendall(current_command.encode())  # Send the combined command
                print(f"Sending command: {current_command}")
            elif sending_enabled and not pressed_keys:
                # Send a stop command ("x") when no keys are pressed
                client_socket.sendall(b'x')
                print("No keys pressed. Sending stop command: x")
                time.sleep(0.1)  # Add a slight delay to avoid flooding the server
            time.sleep(0.1)  # Small delay to reduce CPU usage
            
    finally:
        client_socket.close()  # Close the socket
        listener.stop()  # Stop the listener

if __name__ == "__main__":
    start_client()
