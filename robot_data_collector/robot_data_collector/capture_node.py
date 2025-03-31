import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from kinova_msgs.msg import FingerPosition
import json
import time
import os
import threading

class RobotDataCollector(Node):
    def __init__(self):
        super().__init__('robot_data_collector')

        # Get capture name from user input
        self.capture_name = input("Enter capture name: ").strip()
        if not self.capture_name:
            self.capture_name = "default"

        # Create subscriptions
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/m1n6s200_driver/out/tool_pose',
            self.pose_callback,
            10
        )
        self.finger_sub = self.create_subscription(
            FingerPosition,
            '/m1n6s200_driver/out/finger_position',
            self.finger_callback,
            10
        )

        # Data storage
        self.data = []
        self.current_pose = None
        self.current_finger = None

        # Timer to capture data at 5 Hz
        self.timer = self.create_timer(0.2, self.capture_data)

        print("Capturing data... Type 'exit' and press Enter to stop.")

    def pose_callback(self, msg):
        self.current_pose = {
            "position": {
                "x": msg.pose.position.x,
                "y": msg.pose.position.y,
                "z": msg.pose.position.z
            },
            "orientation": {
                "x": msg.pose.orientation.x,
                "y": msg.pose.orientation.y,
                "z": msg.pose.orientation.z,
                "w": msg.pose.orientation.w
            }
        }
        # print(f"[DEBUG] Received Pose Data: {self.current_pose}")

    def finger_callback(self, msg):
        self.current_finger = {
            "finger1": msg.finger1,
            "finger2": msg.finger2,
            "finger3": msg.finger3
        }
        # print(f"[DEBUG] Received Finger Data: {self.current_finger}")

    def capture_data(self):
        if self.current_pose and self.current_finger:
            self.data.append({
                "pose": self.current_pose,
                "finger_position": self.current_finger,
                "timestamp": time.time()
            })
            # print(f"[DEBUG] Captured Data Point: {self.data[-1]}")
        else:
            print("[DEBUG] No data to capture yet.")

    def save_data(self):
        save_dir = os.path.expanduser("./data")
        os.makedirs(save_dir, exist_ok=True)
        filename = os.path.join(save_dir, f"{self.capture_name}.json")

        with open(filename, "w") as f:
            json.dump({"data": self.data}, f, indent=4)
        print(f"Data saved to {filename}")


def main():
    rclpy.init()
    node = RobotDataCollector()

    # Spin the node in a separate thread so callbacks will be processed
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        while True:
            user_input = input().strip().lower()
            if user_input == "exit":
                node.save_data()
                print("Exiting...")
                break
    except KeyboardInterrupt:
        print("Interrupted. Saving data before exiting...")
        node.save_data()

    node.destroy_node()
    rclpy.shutdown()
    spin_thread.join()

if __name__ == '__main__':
    main()
