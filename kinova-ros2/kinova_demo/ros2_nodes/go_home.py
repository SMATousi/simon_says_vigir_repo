#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from builtin_interfaces.msg import Time
from kinova_msgs.action import ArmPose
from std_msgs.msg import Header
from geometry_msgs.msg import Point, Quaternion
import time

class CartesianPoseClient(Node):
    def __init__(self):
        super().__init__('go_home')

        self.position = [-0.14314740896224976, -0.3382762372493744, 0.08447998762130737]
        self.orientation = [0.999634325504303, -4.836401785723865e-05, -0.01209217868745327,  0.024186542257666588]

        self.client = ActionClient(self, ArmPose, '/m1n6s200_driver/tool_pose')

        self.get_logger().info('Waiting for action server...')
        self.client.wait_for_server()

    def send_goal(self, current_time):
        time_msg = current_time.to_msg()
        t = Time()
        t.sec = time_msg.sec
        t.nanosec = time_msg.nanosec

        goal = ArmPose.Goal()
        goal.pose.header = Header(frame_id='m1n6s200_link_base')
        goal.pose.header.stamp = t
        goal.pose.pose.position = Point(x=self.position[0], y=self.position[1], z=self.position[2])
        goal.pose.pose.orientation = Quaternion(x=self.orientation[0], y=self.orientation[1], z=self.orientation[2], w=self.orientation[3])

        self.get_logger().info('Sending goal...')
        self.client.send_goal(goal)

        self.get_logger().info('Waiting for result...')
        self.client.wait_for_result()

        result = self.client.get_result()
        if not result:
            self.get_logger().info('Goal failed')
        else:
            self.get_logger().info(f'Result: {result.pose.pose}')

def main(args=None):
    rclpy.init(args=args)

    node = CartesianPoseClient()
    current_time = node.get_clock().now()
    # client_node.send_goal(current_time)
    node.send_goal(current_time)

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
