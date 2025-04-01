#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from kinova_msgs.action import ArmPose
from std_msgs.msg import Header, String
from geometry_msgs.msg import Point, Quaternion
from kinova_msgs.action import SetFingersPosition
from rclpy.executors import MultiThreadedExecutor
import threading
import numpy as np

import time
import json
import os
import sys

list_of_serial_numbers = ['936322070193', 
                            '937622070994', 
                            '935322072821', 
                            '935422072205', 
                            '014122071597', 
                            '934222072312', 
                            '021222072467', 
                            '939722070164']

class CartesianTrajectoryWithCamera(Node):
    def __init__(self, trajectory_file, base_dir):
        super().__init__('trajectory_camera_follower')

        # Store base directory for saving images
        self.base_dir = base_dir
        
        # Load cleaned trajectory from JSON
        with open(trajectory_file, 'r') as f:
            self.cleaned_trajectory = json.load(f)

        # Action clients for arm control
        self.client = ActionClient(self, ArmPose, '/m1n6s200_driver/tool_pose')
        self.finger_client = ActionClient(self, SetFingersPosition, '/m1n6s200_driver/finger_positions')

        # Camera subscriber
        self.cv_bridge = CvBridge()
        self._image_lock = threading.Lock()
        
        # Store both RGB and depth images
        self.latest_rgb = None
        self.latest_depth = None
        
        # RGB image subscriber
        self.rgb_sub = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.rgb_callback,
            10
        )
        
        # Depth image subscriber
        self.depth_sub = self.create_subscription(
            Image,
            '/camera/camera/depth/image_rect_raw',
            self.depth_callback,
            10
        )

        # Camera serial number publishers
        self.start_pub = self.create_publisher(String, '/camera_serial_number_start', 10)
        self.end_pub = self.create_publisher(String, '/camera_serial_number_end', 10)

        self.get_logger().info('Waiting for action servers...')
        self.client.wait_for_server()
        self.finger_client.wait_for_server()

    def rgb_callback(self, msg):
        """Callback function to store the latest RGB image"""
        try:
            with self._image_lock:
                self.latest_rgb = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
                self.get_logger().debug('Received new RGB image')
        except Exception as e:
            self.get_logger().error(f'Error converting RGB image: {str(e)}')

    def depth_callback(self, msg):
        """Callback function to store the latest depth image"""
        try:
            with self._image_lock:
                self.latest_depth = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
                self.get_logger().debug('Received new depth image')
        except Exception as e:
            self.get_logger().error(f'Error converting depth image: {str(e)}')

    def save_images(self, frame_name, serial_number):
        """Save both RGB and depth images to the appropriate directory"""
        rgb_image = None
        depth_image = None
        
        with self._image_lock:
            if self.latest_rgb is None or self.latest_depth is None:
                self.get_logger().warn('No RGB or depth image available to save')
                return False
            rgb_image = self.latest_rgb.copy()
            depth_image = self.latest_depth.copy()

        # Create frame directory if it doesn't exist
        frame_dir = os.path.join(self.base_dir, str(frame_name))
        os.makedirs(frame_dir, exist_ok=True)

        success = True
        try:
            # Save RGB image
            rgb_path = os.path.join(frame_dir, f'camera_{serial_number}_rgb.jpg')
            cv2.imwrite(rgb_path, rgb_image)
            self.get_logger().info(f'Saved RGB image to {rgb_path}')

            # Save depth image
            # Convert depth to uint16 if not already
            if depth_image.dtype != np.uint16:
                depth_image = depth_image.astype(np.uint16)
            depth_path = os.path.join(frame_dir, f'camera_{serial_number}_depth.png')
            cv2.imwrite(depth_path, depth_image)
            self.get_logger().info(f'Saved depth image to {depth_path}')
            
        except Exception as e:
            self.get_logger().error(f'Error saving images: {str(e)}')
            success = False
            
        return success

    def send_goal(self, position, orientation, finger, frame_name):
        """Send a single ArmPose goal and save the camera images"""
        # Create the action goal
        goal = ArmPose.Goal()

        finger_goal = SetFingersPosition.Goal()
        finger_goal.fingers.finger1 = finger["finger1"]
        finger_goal.fingers.finger2 = finger["finger2"]
        finger_goal.fingers.finger3 = finger["finger3"]

        # Stamp the pose header with the current time
        now = self.get_clock().now().to_msg()
        goal.pose.header = Header(
            frame_id='m1n6s200_link_base',
            stamp=now
        )

        # Fill in position
        goal.pose.pose.position = Point(
            x=position["x"],
            y=position["y"],
            z=position["z"]
        )

        # Fill in orientation
        goal.pose.pose.orientation = Quaternion(
            x=orientation["x"],
            y=orientation["y"],
            z=orientation["z"],
            w=orientation["w"]
        )

        # Send the goal
        self.get_logger().info(f'Sending goal: pos=({position["x"]:.3f}, {position["y"]:.3f}, {position["z"]:.3f})')
        self.get_logger().info(f'Sending finger goal {finger})')
        self.client.send_goal_async(goal)
        time.sleep(1.0)
        self.finger_client.send_goal_async(finger_goal)

        # For each camera serial number, start it, capture images, then end it
        for serial in list_of_serial_numbers:
            # Start camera
            start_msg = String()
            start_msg.data = serial
            self.start_pub.publish(start_msg)
            self.get_logger().info(f'Started camera {serial}')

            time.sleep(2.0)  # Wait for camera to initialize

            # Save both RGB and depth images
            self.save_images(frame_name, serial)
            
            # End camera
            end_msg = String()
            end_msg.data = serial
            self.end_pub.publish(end_msg)
            self.get_logger().info(f'Ended camera {serial}')

            time.sleep(2.0)  # Wait before next camera

        # Wait briefly to ensure movement is complete
        time.sleep(1.0)

    def follow_trajectory(self):
        """Iterate through each frame in the cleaned trajectory"""
        self.get_logger().info(f'Following trajectory with {len(self.cleaned_trajectory)} waypoints...')

        for i, frame in enumerate(self.cleaned_trajectory):
            pos = frame["position"]
            orientation = frame["orientation"]
            finger = frame["fingers"]
            frame_name = frame["frame"]
            self.get_logger().info(f'Processing frame {frame_name} ({i+1}/{len(self.cleaned_trajectory)})')
            self.send_goal(pos, orientation, finger, frame_name)

def main(args=None):
    rclpy.init(args=args)
    
    if len(sys.argv) != 3:
        print("Usage: follow_traj_with_camera.py <trajectory_file> <base_directory>")
        sys.exit(1)
        
    trajectory_file = sys.argv[1]
    base_dir = sys.argv[2]
    
    if not os.path.exists(trajectory_file):
        print(f"Error: Trajectory file '{trajectory_file}' not found")
        sys.exit(1)
        
    # Create base directory if it doesn't exist
    os.makedirs(base_dir, exist_ok=True)

    # Create and setup the node
    node = CartesianTrajectoryWithCamera(trajectory_file, base_dir)
    
    # Create a MultiThreadedExecutor
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    # Spin in a separate thread
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        # Give some time for subscriptions to be established
        time.sleep(2.0)
        node.follow_trajectory()
    except KeyboardInterrupt:
        pass
    finally:
        # Cleanup
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
