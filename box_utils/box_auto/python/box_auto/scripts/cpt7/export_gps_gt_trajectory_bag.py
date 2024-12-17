import numpy as np
import pandas as pd
import rosbag
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, Point, Quaternion, TransformStamped, Vector3, PoseStamped
from tf2_msgs.msg import TFMessage
from scipy.spatial.transform import Rotation
from pathlib import Path
from nav_msgs.msg import Path as PathRos
from box_auto.utils import MISSION_DATA, GPS_utils


def main():
    """
    Processes a GPS path and converts it into various ROS messages.

    NOTE: For consistency across different post-processing methods, it is assumed that the enu_origin always starts at (0, 0, 0) based on the first pose.
    However, this assumption may be incorrect when considering a global ENU frame, as different post-processing methods might initialize at distinct ENU_COORDINATES
    but still treat them as (0, 0, 0) for the enu_origin.

    As a result, conflicting TFs (Transforms) might be published between the ROS bags.
    """

    date = [(str(s.name)).split("_")[0] for s in Path(MISSION_DATA).glob("*_nuc_livox*.bag")][0]
    gps_files = [str(s) for s in (Path(MISSION_DATA) / "ie").rglob("*_GrandTour-LocalFrame-minimal.txt")]
    post_proc_modes = [s.split("/")[-1].split("_")[1] for s in gps_files]
    bag_paths = [
        str(Path(s).parent.parent / (date + f"_cpt7_ie_{mode}.bag")) for s, mode in zip(gps_files, post_proc_modes)
    ]

    for gps_file_path, bag_path, post_proc_mode in zip(gps_files, bag_paths, post_proc_modes):
        gps_file = pd.read_csv(gps_file_path)
        gps_file.columns = gps_file.columns.str.strip()
        position_columns = ["X-ECEF", "Y-ECEF", "Z-ECEF"]
        positions_std_columns = ["SDX-ECEF", "SDY-ECEF", "SDZ-ECEF"]
        orientation_columns = ["Heading", "Roll", "Pitch"]
        orientations_rpy_std_columns = ["HdngSD", "RollSD", "PitchSD"]
        times = gps_file.iloc[:, 0]
        positions = np.array(gps_file[position_columns].to_numpy())
        orientations_rpy = np.array(gps_file[orientation_columns].to_numpy())
        positions_std = np.array(gps_file[positions_std_columns].to_numpy())
        orientations_rpy_std = np.array(gps_file[orientations_rpy_std_columns].to_numpy())

        utils = GPS_utils()
        lat_long_h = utils.ecef2geo(positions[0, 0], positions[0, 1], positions[0, 2])
        utils.setENUorigin(lat_long_h[0], lat_long_h[1], lat_long_h[2])
        start_time = None

        frame_id = "enu_origin"
        gps_path_msg = PathRos()
        gps_path_msg.header.frame_id = frame_id

        with rosbag.Bag(bag_path, "w", compression="lz4") as bag:
            for i, (time, position, orientation_rpy, position_std, orientation_rpy_std) in enumerate(
                zip(times, positions, orientations_rpy, positions_std, orientations_rpy_std)
            ):
                if start_time is None:
                    start_time = time

                R_enu__ned = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]])

                # Position
                position_ned = utils.ecef2enu(position[0], position[1], position[2])
                position_enu = R_enu__ned @ position_ned
                # Absolutely not sure if this is correct
                position_enu_std = R_enu__ned @ utils.R @ position_std

                # Covariance is diagonal - 6x6 matrix ( dx, dy, dz, droll, dpitch, dyaw)
                # TO ROS convention: fixed axis https://www.ros.org/reps/rep-0103.html
                quaternion_xyzw = Rotation.from_matrix(
                    R_enu__ned @ Rotation.from_euler("ZXY", orientation_rpy, degrees=True).as_matrix()
                ).as_quat()
                rot_std = Rotation.from_matrix(
                    R_enu__ned @ Rotation.from_euler("ZXY", orientation_rpy_std, degrees=True).as_matrix()
                ).as_euler("xyz", degrees=False)

                # Missing **2
                covariance = np.diag(np.concatenate([rot_std[:], np.array(position_enu_std)[0, :]], axis=0))

                timestamp = rospy.Time.from_sec(time)
                output_msg = PoseWithCovarianceStamped()
                output_msg.header.seq = i
                output_msg.header.stamp = timestamp
                output_msg.header.frame_id = frame_id
                output_msg.pose.pose.position = Point(x=position_enu[1], y=position_enu[0], z=-position_enu[2])

                output_msg.pose.pose.orientation = Quaternion(
                    x=quaternion_xyzw[0], y=quaternion_xyzw[1], z=quaternion_xyzw[2], w=quaternion_xyzw[3]
                )
                output_msg.pose.covariance = covariance.astype(np.float32).flatten().tolist()
                bag.write(
                    topic=f"/gt_box/inertial_explorer/{post_proc_mode}/gt_poses_novatel", msg=output_msg, t=timestamp
                )

                odometry_msg = Odometry()
                odometry_msg.header = output_msg.header
                odometry_msg.pose.pose = output_msg.pose.pose
                odometry_msg.child_frame_id = "box_base"
                bag.write(topic=f"/gt_box/inertial_explorer/{post_proc_mode}/odometry", msg=odometry_msg, t=timestamp)

                path_pose = PoseStamped()
                path_pose.header = output_msg.header
                path_pose.pose = output_msg.pose.pose
                gps_path_msg.poses.append(path_pose)
                if i % 100 == 0:
                    bag.write(
                        topic=f"/gt_box/inertial_explorer/{post_proc_mode}/gps_path", msg=gps_path_msg, t=timestamp
                    )

                tf_message = TFMessage()
                tf_message.transforms = []
                box_transform = TransformStamped()
                box_transform.header = odometry_msg.header
                box_transform.header.frame_id = "box_base"
                box_transform.child_frame_id = frame_id

                position_enu = np.array(position_enu)[:, 0]

                SE3 = np.eye(4)
                SE3[:3, :3] = Rotation.from_quat(quaternion_xyzw).as_matrix()
                SE3[:3, 3] = [position_enu[1], position_enu[0], -position_enu[2]]
                SE3 = np.linalg.inv(SE3)
                box_transform.transform.translation = Vector3(x=SE3[0, 3], y=SE3[1, 3], z=SE3[2, 3])

                q = Rotation.from_matrix(SE3[:3, :3]).as_quat()
                box_transform.transform.rotation = Quaternion(q[0], q[1], q[2], q[3])

                tf_message.transforms.append(box_transform)
                bag.write(topic="/tf", msg=tf_message, t=timestamp)


if __name__ == "__main__":
    main()
