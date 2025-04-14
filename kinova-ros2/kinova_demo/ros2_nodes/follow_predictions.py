#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import Float32MultiArray
from builtin_interfaces.msg import Time
from kinova_msgs.action import ArmPose
from std_msgs.msg import Header
from geometry_msgs.msg import Point, Quaternion, PoseStamped
from kinova_msgs.action import SetFingersPosition
import numpy as np
from scipy.spatial.transform import Rotation as R
import time
from rclpy.qos import qos_profile_sensor_data

class PredictionFollower(Node):
    def __init__(self):
        super().__init__('prediction_follower')
        self.client = ActionClient(self, ArmPose, '/m1n6s200_driver/tool_pose')
        self.finger_client = ActionClient(self, SetFingersPosition, '/m1n6s200_driver/finger_positions')

        self.get_logger().info('Waiting for action servers...')
        self.client.wait_for_server()
        self.finger_client.wait_for_server()

        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/predicted_actions',
            self.prediction_callback,
            qos_profile_sensor_data
        )
        self.get_logger().info('Subscribed to /predicted_actions')

        self.pose_initialized = False
        self.pose_subscription = self.create_subscription(
            PoseStamped,
            '/m1n6s200_driver/out/tool_pose',
            self.pose_callback,
            qos_profile_sensor_data
        )
        self.get_logger().info('Subscribed to /m1n6s200_driver/out/tool_pose for initial pose.')

    def pose_callback(self, msg):
        pos = msg.pose.position
        quat = msg.pose.orientation
        self.current_position = np.array([pos.x, pos.y, pos.z])
        self.current_orientation = R.from_quat([quat.x, quat.y, quat.z, quat.w])
        if not self.pose_initialized:
            self.get_logger().info(f'Initial robot pose set to position: {self.current_position}, orientation: {[quat.x, quat.y, quat.z, quat.w]}')
            self.pose_initialized = True

    def prediction_callback(self, msg):
        if not self.pose_initialized:
            self.get_logger().warn('Prediction received before initial pose was set. Ignoring.')
            return
        self.get_logger().info(f'Callback triggered! Received message on /predicted_actions.')
        data = msg.data
        self.get_logger().info(f'Message data: {data}, length: {len(data)}')
        if len(data) < 7:
            self.get_logger().error('Received prediction with insufficient data length.')
            return
        # Delta position
        delta_pos = np.array(data[0:3])
        # Delta orientation in RPY (radians)
        delta_rpy = np.array(data[3:6])
        # Delta quaternion
        delta_quat = R.from_euler('xyz', delta_rpy).as_quat()  # [x, y, z, w]
        # Update absolute orientation
        self.current_orientation = self.current_orientation * R.from_quat(delta_quat)
        # Update absolute position
        self.current_position += delta_pos
        # Fingers
        finger_val = data[6]
        finger_cmd = float(finger_val * 3000)
        # Prepare pose and orientation dicts
        pos_dict = {
            'x': float(self.current_position[0]),
            'y': float(self.current_position[1]),
            'z': float(self.current_position[2])
        }
        quat = self.current_orientation.as_quat()  # [x, y, z, w]
        orientation_dict = {
            'x': float(quat[0]),
            'y': float(quat[1]),
            'z': float(quat[2]),
            'w': float(quat[3])
        }
        fingers_dict = {
            'finger1': finger_cmd,
            'finger2': finger_cmd,
            'finger3': finger_cmd
        }
        self.get_logger().info(f'Received prediction: Δpos={delta_pos}, Δrpy={delta_rpy}, fingers={finger_val}')
        self.send_goal(pos_dict, orientation_dict, fingers_dict)

    def send_goal(self, position, orientation, finger):
        goal = ArmPose.Goal()
        finger_goal = SetFingersPosition.Goal()
        finger_goal.fingers.finger1 = finger["finger1"]
        finger_goal.fingers.finger2 = finger["finger2"]
        finger_goal.fingers.finger3 = finger["finger3"]

        now = self.get_clock().now().to_msg()
        goal.pose.header = Header(
            frame_id='m1n6s200_link_base',
            stamp=now
        )
        goal.pose.pose.position = Point(
            x=position["x"],
            y=position["y"],
            z=position["z"]
        )
        goal.pose.pose.orientation = Quaternion(
            x=orientation["x"],
            y=orientation["y"],
            z=orientation["z"],
            w=orientation["w"]
        )

        self.get_logger().info(f'Sending goal: pos=({position["x"]:.3f}, {position["y"]:.3f}, {position["z"]:.3f})')
        self.get_logger().info(f'Sending finger goal {finger}')
        self.client.send_goal_async(goal)
        time.sleep(1.0)
        self.finger_client.send_goal_async(finger_goal)
        self.get_logger().info('Goals sent.')


def main(args=None):
    rclpy.init(args=args)
    node = PredictionFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
