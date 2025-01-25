import rosbag
from sensor_msgs.msg import Imu
import os
import numpy as np
from scipy.spatial.transform import Rotation as R


def compute_angular_velocity(prev_pose, curr_pose, dt):
    """
    Compute angular velocity using finite differences based on quaternion rotations.

    Args:
        prev_pose (PoseWithCovarianceStamped): The previous pose message.
        curr_pose (PoseWithCovarianceStamped): The current pose message.
        dt (float): Time difference between the two poses in seconds.

    Returns:
        tuple: Angular velocity [vx, vy, vz]
    """
    # Extract orientations as quaternions
    prev_orientation = [
        prev_pose.pose.pose.orientation.x,
        prev_pose.pose.pose.orientation.y,
        prev_pose.pose.pose.orientation.z,
        prev_pose.pose.pose.orientation.w,
    ]

    curr_orientation = [
        curr_pose.pose.pose.orientation.x,
        curr_pose.pose.pose.orientation.y,
        curr_pose.pose.pose.orientation.z,
        curr_pose.pose.pose.orientation.w,
    ]

    # Convert quaternions to rotation matrices
    R1 = R.from_quat(prev_orientation).as_matrix()
    R2 = R.from_quat(curr_orientation).as_matrix()

    # Compute the relative rotation matrix
    R_rel = R2 @ R1.T  # R_rel = R2 * R1^-1

    # Compute the skew-symmetric matrix from the relative rotation
    omega_skew = (R_rel - R_rel.T) / (2 * dt)

    # Extract the angular velocity vector from the skew-symmetric matrix
    angular_velocity = np.array([omega_skew[2, 1], omega_skew[0, 2], omega_skew[1, 0]])

    return angular_velocity


def pose_to_imu(prev_pose, curr_pose, dt):
    """
    Convert two PoseWithCovarianceStamped messages to an Imu message,
    computing angular acceleration using finite differences.

    Args:
        prev_pose (PoseWithCovarianceStamped): The previous pose message.
        curr_pose (PoseWithCovarianceStamped): The current pose message.
        dt (float): Time difference between the two poses in seconds.

    Returns:
        Imu: An IMU message with angular acceleration populated.
    """
    imu_msg = Imu()

    # Copy header from the current pose message
    imu_msg.header = curr_pose.header

    # Compute angular velocity
    angular_velocity = compute_angular_velocity(prev_pose, curr_pose, dt)

    # Populate angular velocity in IMU message
    imu_msg.angular_velocity.x = angular_velocity[0]
    imu_msg.angular_velocity.y = angular_velocity[1]
    imu_msg.angular_velocity.z = angular_velocity[2]

    return imu_msg


def process_rosbag(input_bag_path, output_bag_path):
    """
    Process a ROS bag file to convert PoseWithCovarianceStamped messages
    to Imu messages and save them to a new bag file.

    Args:
        input_bag_path (str): Path to the input ROS bag.
        output_bag_path (str): Path to save the output ROS bag.
    """
    if not os.path.exists(input_bag_path):
        print(f"Input bag file does not exist: {input_bag_path}")
        return

    prev_msg = None

    with rosbag.Bag(output_bag_path, "w") as outbag:
        with rosbag.Bag(input_bag_path, "r") as inbag:
            for topic, msg, t in inbag.read_messages():
                if topic == "/gt_box/inertial_explorer/tc/gt_poses_novatel":
                    if prev_msg is not None:
                        # Compute time difference in seconds
                        dt = (msg.header.stamp - prev_msg.header.stamp).to_sec()
                        if dt > 0:
                            imu_msg = pose_to_imu(prev_msg, msg, dt)
                            outbag.write("/gt_box/inertial_explorer/tc/gt_fake_imu", imu_msg, t)
                    prev_msg = msg


if __name__ == "__main__":
    input_bag = "/media/jonfrey/Untitled/box_paper_dataset_v2/2024-11-03-13-51-43_eigergletscher_hike_down/2024-11-03-13-51-43_cpt7_ie_tc.bag"
    output_bag = "/media/jonfrey/Untitled/box_paper_dataset_v2/2024-11-03-13-51-43_eigergletscher_hike_down/2024-11-03-13-51-43_cpt7_ie_tc_to_imu.bag"

    process_rosbag(input_bag, output_bag)
    print(f"Processed bag saved to {output_bag}")
