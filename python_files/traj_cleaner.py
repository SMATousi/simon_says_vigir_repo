import json
import argparse
import numpy as np
from scipy.spatial.transform import Rotation as R

def quaternion_to_rpy_difference(q1, q2):
    """Compute roll, pitch, yaw (Euler angles) difference between two quaternions."""
    r1 = R.from_quat([q1["x"], q1["y"], q1["z"], q1["w"]])
    r2 = R.from_quat([q2["x"], q2["y"], q2["z"], q2["w"]])
    
    # Quaternion difference
    delta_rotation = r2 * r1.inv()
    
    # Convert to roll, pitch, yaw (RPY) in degrees
    roll, pitch, yaw = delta_rotation.as_euler('xyz', degrees=True)
    return {"roll": roll, "pitch": pitch, "yaw": yaw}

def process_trajectory(input_file, output_file):
    """Load, process, and save the cleaned trajectory."""
    # Load trajectory data
    with open(input_file, "r") as f:
        data = json.load(f)

    trajectory = data["data"]  # Assuming trajectory is stored under "pick_1"

    cleaned_frames = []
    previous_frame = None

    for frame in trajectory:
        position = frame["pose"]["position"]
        orientation = frame["pose"]["orientation"]
        fingers = frame["finger_position"]

        if previous_frame is not None:
            prev_position = previous_frame["pose"]["position"]
            prev_orientation = previous_frame["pose"]["orientation"]
            prev_fingers = previous_frame["finger_position"]

            # Compute deltas
            delta_position = {
                "x": position["x"] - prev_position["x"],
                "y": position["y"] - prev_position["y"],
                "z": position["z"] - prev_position["z"]
            }
            delta_orientation_rpy = quaternion_to_rpy_difference(prev_orientation, orientation)
            delta_fingers = {
                "finger1": fingers["finger1"] - prev_fingers["finger1"],
                "finger2": fingers["finger2"] - prev_fingers["finger2"],
                "finger3": fingers["finger3"] - prev_fingers["finger3"]
            }

            # Check if there's movement (position or finger change)
            if any(abs(v) > 1e-6 for v in delta_position.values()) or any(abs(v) > 0 for v in delta_fingers.values()):
                cleaned_frames.append({
                    "frame": frame["timestamp"],
                    "position": position,
                    "orientation": orientation,
                    "fingers": fingers,
                    "delta_position": delta_position,
                    "delta_orientation_rpy": delta_orientation_rpy,  # Roll, pitch, yaw
                    "delta_fingers": delta_fingers
                })

        previous_frame = frame

    # Save cleaned trajectory
    with open(output_file, "w") as f:
        json.dump(cleaned_frames, f, indent=4)

    print(f"Processed trajectory saved to: {output_file}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Clean trajectory and compute deltas.")
    parser.add_argument("input_file", type=str, help="Path to input JSON file")
    parser.add_argument("output_file", type=str, help="Path to output JSON file")
    
    args = parser.parse_args()
    
    process_trajectory(args.input_file, args.output_file)