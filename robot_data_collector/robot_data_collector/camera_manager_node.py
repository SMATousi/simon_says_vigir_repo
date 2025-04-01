#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import subprocess
import threading
import signal
import os
import sys

class CameraManagerNode(Node):
    def __init__(self):
        super().__init__('camera_manager_node')
        
        # Dictionary to store camera processes
        self.camera_processes = {}
        self._shutdown = False
        self._lock = threading.Lock()  # Add lock for thread safety
        
        # Create subscriptions
        self.start_sub = self.create_subscription(
            String,
            '/camera_serial_number_start',
            self.start_callback,
            10
        )
        
        self.end_sub = self.create_subscription(
            String,
            '/camera_serial_number_end',
            self.end_callback,
            10
        )
        
        self.get_logger().info('Camera Manager Node initialized')

    def start_callback(self, msg):
        if self._shutdown:
            return
            
        serial_number = msg.data
        with self._lock:
            if serial_number in self.camera_processes:
                self.get_logger().warn(f'Camera with serial {serial_number} is already running')
                return

        def run_camera():
            try:
                # Start process in a new process group
                cmd = ['ros2', 'launch', 'realsense2_camera', 'rs_launch.py', f'serial_no:=_{serial_number}']
                process = subprocess.Popen(
                    cmd,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    preexec_fn=os.setsid
                )
                with self._lock:
                    if not self._shutdown:
                        self.camera_processes[serial_number] = process
                        self.get_logger().info(f'Started camera with serial {serial_number}')
                
                # Wait for process to complete
                process.wait()
                
                with self._lock:
                    if not self._shutdown and serial_number in self.camera_processes:
                        del self.camera_processes[serial_number]
                        self.get_logger().info(f'Camera process with serial {serial_number} ended naturally')
                    
            except Exception as e:
                self.get_logger().error(f'Error running camera process: {str(e)}')
                with self._lock:
                    if serial_number in self.camera_processes:
                        del self.camera_processes[serial_number]

        # Start camera process in a new thread
        thread = threading.Thread(target=run_camera)
        thread.daemon = True
        thread.start()

    def end_callback(self, msg):
        if self._shutdown:
            return
            
        serial_number = msg.data
        process = None
        
        with self._lock:
            if serial_number in self.camera_processes:
                process = self.camera_processes[serial_number]
                del self.camera_processes[serial_number]
            
        if process:
            try:
                # Send SIGTERM to the process group
                os.killpg(os.getpgid(process.pid), signal.SIGTERM)
                process.wait(timeout=5)  # Wait up to 5 seconds for graceful shutdown
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(os.getpgid(process.pid), signal.SIGKILL)  # Force kill if timeout
                except:
                    pass
            except Exception as e:
                self.get_logger().error(f'Error stopping camera process: {str(e)}')
            
            self.get_logger().info(f'Stopped camera with serial {serial_number}')
        else:
            self.get_logger().warn(f'No running camera found with serial {serial_number}')

    def cleanup(self):
        self._shutdown = True
        processes_to_cleanup = []
        
        # Get all processes under lock
        with self._lock:
            processes_to_cleanup = [(serial, process) for serial, process in self.camera_processes.items()]
            self.camera_processes.clear()
            
        # Clean up processes outside the lock
        for serial, process in processes_to_cleanup:
            try:
                os.killpg(os.getpgid(process.pid), signal.SIGTERM)
                process.wait(timeout=2)
            except:
                try:
                    os.killpg(os.getpgid(process.pid), signal.SIGKILL)
                except:
                    pass
            self.get_logger().info(f'Cleaned up camera with serial {serial}')

def main():
    rclpy.init()
    node = CameraManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cleanup()
        node.destroy_node()
        try:
            rclpy.shutdown()
        except:
            pass  # Ignore shutdown errors
        
if __name__ == '__main__':
    main()
