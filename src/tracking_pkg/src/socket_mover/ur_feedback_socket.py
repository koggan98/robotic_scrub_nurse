import socket
import sys
import time
import threading

from package import Package

class URFeedback:
    def __init__(self, robot_ip, robot_feedback_port):
        self.robot_ip = robot_ip
        self.robot_feedback_port = robot_feedback_port
        self.client_socket = None
        self.program_running = None
        self.rate = 0.01 

    def connect(self):
        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.client_socket.settimeout(4)
        try:
            self.client_socket.connect((self.robot_ip, self.robot_feedback_port))
            print(f"Established connection to {self.robot_ip}:{self.robot_feedback_port}")
        except socket.timeout as e:
            print(f"Timeout error: {e}")
            sys.exit(1)
        except socket.error as e:
            print(f"Could not connect to {self.robot_ip}:{self.robot_feedback_port} Error: {e}")
            sys.exit(1)

    def disconnect(self):
        if self.client_socket:
            self.client_socket.close()
            print(f"Closed connection to {self.robot_ip}:{self.robot_feedback_port}")

    def update_program_status(self):
        while True:
            try:
                new_message = self.client_socket.recv(4096)
                new_package = Package(new_message)
                
                robot_mode_data = new_package.get_subpackage("Robot Mode Data")
                if robot_mode_data is not None:
                    self.program_running = robot_mode_data.subpackage_variables.isProgramRunning
                else:
                    self.program_running = None
                
            except socket.timeout:
                print("Socket timeout, no data received.")
                self.program_running = None
            except socket.error as e:
                print(f"Socket error: {e}")
                self.program_running = None
                break
            except Exception as e:
                print(f"Unexpected error: {e}")
                self.program_running = None
                break
            
            time.sleep(self.rate)

    def is_robot_running(self):
        return self.program_running

if __name__ == '__main__':
    ROBOT_IP = '192.168.1.10'
    ROBOT_FEEDBACK_PORT = 30001
    client = URFeedback(ROBOT_IP, ROBOT_FEEDBACK_PORT)
    client.connect()

    # Start the thread to update the program status
    status_thread = threading.Thread(target=client.update_program_status)
    status_thread.daemon = True
    status_thread.start()
    
    try:
        while True:
            is_running = client.is_robot_running()
            if is_running is not None:
                print(f"Program Running: {is_running}")
            else:
                print("Could not determine program running status.")
            time.sleep(0.10)
    except KeyboardInterrupt:
        print("Interrupted by user.")
    finally:
        client.disconnect()
