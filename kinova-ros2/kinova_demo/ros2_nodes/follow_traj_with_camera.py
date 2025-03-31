#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

from builtin_interfaces.msg import Time
from kinova_msgs.action import ArmPose
from std_msgs.msg import Header
from geometry_msgs.msg import Point, Quaternion
from kinova_msgs.action import SetFingersPosition

import time
import json
import os

class CartesianTrajectoryWithCamera(Node):
    def __init__(self, trajectory_file):
        super().__init__('trajectory_camera_follower')

        # Load cleaned trajectory from JSON
        with open(trajectory_file, 'r') as f:
            self.cleaned_trajectory = json.load(f)

        # Action clients for arm control
        self.client = ActionClient(self, ArmPose, '/m1n6s200_driver/tool_pose')
        self.finger_client = ActionClient(self, SetFingersPosition, '/m1n6s200_driver/finger_positions')

        # Camera subscriber
        self.cv_bridge = CvBridge()
        self.latest_image = None
        self.image_sub = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.image_callback,
            10
        )

        self.get_logger().info('Waiting for action servers...')
        self.client.wait_for_server()
        self.finger_client.wait_for_server()

    def image_callback(self, msg):
        """Callback function to store the latest image"""
        try:
            self.latest_image = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f'Error converting image: {str(e)}')

    def display_current_image(self):
        """Display the current camera image if available"""
        if self.latest_image is not None:
            cv2.imshow('Camera Feed', self.latest_image)
            cv2.waitKey(1000)  # Display for 1 second
        else:
            self.get_logger().warn('No image available to display')

    def send_goal(self, position, orientation, finger):
        """Send a single ArmPose goal and display the camera image"""
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

        # Display the camera image at this waypoint
        self.display_current_image()

        # Wait briefly to ensure movement is complete
        time.sleep(1.0)

    def follow_trajectory(self):
        """Iterate through each frame in the cleaned trajectory"""
        self.get_logger().info(f'Following trajectory with {len(self.cleaned_trajectory)} waypoints...')

        for i, frame in enumerate(self.cleaned_trajectory):
            pos = frame["position"]
            orientation = frame["orientation"]
            fingers = frame["fingers"]

            self.get_logger().info(f'Waypoint {i+1}/{len(self.cleaned_trajectory)}: Moving to frame={frame["frame"]}')
            self.send_goal(pos, orientation, fingers)
            time.sleep(1.0)

def main(args=None):
    rclpy.init(args=args)

    # Path to your cleaned JSON file:
    root_path = "/root/dev_ws/data"
    file_name = "cleared2.json"
    trajectory_file = os.path.join(root_path, file_name)

    node = CartesianTrajectoryWithCamera(trajectory_file)
    
    # Create a MultiThreadedExecutor
    from rclpy.executors import MultiThreadedExecutor
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    # Spin in a separate thread
    import threading
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        # Give some time for subscriptions to be established
        time.sleep(2.0)
        node.follow_trajectory()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
