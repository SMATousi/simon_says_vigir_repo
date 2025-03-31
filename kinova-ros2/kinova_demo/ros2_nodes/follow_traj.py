#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from builtin_interfaces.msg import Time
from kinova_msgs.action import ArmPose
from std_msgs.msg import Header
from geometry_msgs.msg import Point, Quaternion
from kinova_msgs.action import SetFingersPosition

import time
import json
import os

class CartesianTrajectoryFollower(Node):
    def __init__(self, trajectory_file):
        super().__init__('trajectory_follower')

        # Load cleaned trajectory from JSON
        with open(trajectory_file, 'r') as f:
            self.cleaned_trajectory = json.load(f)

        self.client = ActionClient(self, ArmPose, '/m1n6s200_driver/tool_pose')
        self.finger_client = ActionClient(self, SetFingersPosition, '/m1n6s200_driver/finger_positions')

        self.get_logger().info('Waiting for action server...')
        self.client.wait_for_server()
        self.finger_client.wait_for_server()

    def send_goal(self, position, orientation, finger):
        """
        Send a single ArmPose goal to move the arm to the given absolute position & orientation.
        """
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

        # Wait for the result before proceeding
        self.get_logger().info('Waiting for result...')
        # self.client.wait_for_result()

        # result = self.client.get_result()
        # if not result:
        #     self.get_logger().error('Goal failed or was aborted')
        # else:
        #     self.get_logger().info(f'Goal reached: {result.pose.pose.position}, {result.pose.pose.orientation}')

    def follow_trajectory(self):
        """
        Iterate through each frame in the cleaned trajectory and move to its absolute position/orientation.
        """
        self.get_logger().info(f'Following trajectory with {len(self.cleaned_trajectory)} waypoints...')

        for i, frame in enumerate(self.cleaned_trajectory):
            pos = frame["position"]
            # If your cleaned JSON also stored orientation as "orientation", use that directly:
            # (e.g., "orientation": {"x": 0.7, "y": ... "z": ..., "w": ...})
            # If not, adjust to match how you stored it in your script.
            # Below assumes "orientation" key is in the final JSON:
            #    "position": {...}, "delta_position": {...}, "delta_orientation_rpy": {...}, "delta_fingers": {...},
            #    and also "orientation": {...} if you included absolute orientation. 
            # If your script only stored "delta_orientation_rpy", you’ll need to keep track of absolute orientation separately.

            # For demonstration, let's assume your cleaned JSON includes "orientation" (the absolute quaternion).
            # If it doesn't, replace frame["orientation"] with however you stored it.
            orientation = frame["orientation"]
            fingers = frame["fingers"]

            self.get_logger().info(f'Waypoint {i+1}/{len(self.cleaned_trajectory)}: Moving to frame={frame["frame"]}')
            self.send_goal(pos, orientation, fingers)
            time.sleep(1.0)  # short pause between moves (optional)

def main(args=None):
    rclpy.init(args=args)

    # Path to your cleaned JSON file:
    root_path = "/root/dev_ws/data"
    file_name = "cleaned_glass.json"
    trajectory_file = os.path.join(root_path, file_name)

    node = CartesianTrajectoryFollower(trajectory_file)
    node.follow_trajectory()

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
