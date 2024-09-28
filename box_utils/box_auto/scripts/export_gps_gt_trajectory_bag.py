import numpy as np
import pandas as pd
import rosbag
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, Point, Quaternion, TransformStamped, Vector3
import argparse
from tf2_msgs.msg import TFMessage
from scipy.spatial.transform import Rotation
from pathlib import Path


def add_arguments():
    parser = argparse.ArgumentParser(description="Export GPS optimized trajectory to a bag file")
    parser.set_defaults(main=main)
    parser.add_argument("--gps_file", "-g", help="Path to the GPS optimized trajectory")
    parser.add_argument("--output", "-o", help="Output bag path", default="./gps_gt_output.bag")
    parser.add_argument("--directory", "-d", help="Directory")
    return parser


def main(args):
    if args.directory is not None:
        date = [(str(s.name)).split("_")[0] for s in Path(args.directory).glob("*_nuc_livox*.bag")][0]
        args.output = str(Path(args.directory) / f"{date}_cpt7_gps_optimized_trajectory.bag")
        args.gps_file = str(Path(args.directory) / "ie/ie.txt")

    print(args.output, args.gps_file)

    gps_file_path = args.gps_file
    gps_file = pd.read_csv(gps_file_path)
    gps_file.columns = gps_file.columns.str.strip()
    position_columns = ["X-ECEF", "Y-ECEF", "Z-ECEF"]
    xyz_stdeviation_columns = ["SDX-ECEF", "SDY-ECEF", "SDZ-ECEF"]
    orientation_columns = ["ECTX", "ECTY", "ECTZ"]
    orientation_stdeviation_columns = ["ECTXSD", "ECTYSD", "ECTZSD"]
    times = gps_file.iloc[:, 0]
    positions = np.array(gps_file[position_columns].to_numpy())
    orientations = np.array(gps_file[orientation_columns].to_numpy())
    xyz_stdeviations = np.array(gps_file[xyz_stdeviation_columns].to_numpy())
    orientation_stdeviations = np.array(gps_file[orientation_stdeviation_columns].to_numpy())
    orientation = orientations[0]
    position = positions[0]
    T_world_origin = euler_and_translation_to_SE3(orientation, position)
    T_origin_world = np.linalg.inv(T_world_origin)
    T_cpt7_robot = np.eye(4)
    T_cpt7_robot[:3, :3] = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]])
    T_robot_cpt7 = np.linalg.inv(T_cpt7_robot)
    T_headingcorrection = np.eye(4)
    correction_angle = np.deg2rad(20)
    T_headingcorrection[:3, :3] = np.array(
        [
            [np.cos(correction_angle), -np.sin(correction_angle), 0],
            [np.sin(correction_angle), np.cos(correction_angle), 0],
            [0, 0, 1],
        ]
    )

    start_time = None
    with rosbag.Bag(args.output, "w", compression="lz4") as bag:
        for i, (time, position, orientation, sigma, sigma_orientation) in enumerate(
            zip(times, positions, orientations, xyz_stdeviations, orientation_stdeviations)
        ):
            if start_time is None:
                start_time = time
            T_world_robot = euler_and_translation_to_SE3(orientation, position)
            R_origin_world = T_origin_world[:3, :3]
            xyz_sigma_origin = np.diag(R_origin_world @ np.diag(sigma) @ R_origin_world.T)
            orientation_sigma_origin = np.diag(R_origin_world @ np.diag(sigma_orientation) @ R_origin_world.T)
            T_origin_robot = T_origin_world @ T_world_robot
            T_origin_cpt7 = T_origin_robot @ T_robot_cpt7
            T_origin_cpt7 = T_origin_cpt7 @ T_headingcorrection

            angle, axis = R_to_angle_axis(T_origin_cpt7[:3, :3])
            local_point = T_origin_cpt7[:3, -1]
            timestamp = rospy.Time.from_sec(time)
            output_msg = PoseWithCovarianceStamped()
            output_msg.header.seq = i
            output_msg.header.stamp = timestamp
            output_msg.header.frame_id = "world"
            output_msg.pose.pose.position = Point(x=local_point[0], y=local_point[1], z=local_point[2])
            quaternion = R_to_quat_xyzw(T_origin_cpt7[:3, :3])
            output_msg.pose.pose.orientation = Quaternion(
                x=quaternion[0], y=quaternion[1], z=quaternion[2], w=quaternion[3]
            )
            covariance = np.diag(np.concatenate([np.deg2rad(orientation_sigma_origin), xyz_sigma_origin]) ** 2)
            output_msg.pose.covariance = covariance.flatten().tolist()
            bag.write(topic="/gt_box/gt_poses_novatel", msg=output_msg, t=timestamp)
            odometry_msg = Odometry()
            odometry_msg.header = output_msg.header
            odometry_msg.pose.pose = output_msg.pose.pose
            bag.write(topic="/gt_box/odometry", msg=odometry_msg, t=timestamp)
            tf_message = TFMessage()
            tf_message.transforms = []
            box_transform = TransformStamped()
            box_transform.header = odometry_msg.header
            box_transform.child_frame_id = "box_base"
            box_transform.transform.translation = Vector3(*local_point)
            box_transform.transform.rotation = output_msg.pose.pose.orientation
            tf_message.transforms.append(box_transform)
            bag.write(topic="/tf", msg=tf_message, t=timestamp)


def R_to_angle_axis(orientation):
    angle_axis = Rotation.from_matrix(orientation).as_rotvec()
    angle = np.linalg.norm(angle_axis) + 1e-6
    axis = angle_axis / angle
    return angle, axis


def R_to_quat_xyzw(orientation):
    return Rotation.from_matrix(orientation).as_quat()


def euler_and_translation_to_SE3(orientation, position):
    return np.vstack(
        (
            np.hstack((Rotation.from_euler("xyz", orientation, degrees=True).as_matrix(), position.reshape(3, 1))),
            np.array([[0, 0, 0, 1.0]]),
        )
    )


if __name__ == "__main__":
    parser = add_arguments()
    args = parser.parse_args()
    main(args)
