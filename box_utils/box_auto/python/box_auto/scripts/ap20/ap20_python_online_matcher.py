import rosbag
import numpy as np
from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import Odometry
from collections import deque
import rospy
import os
import tf2_msgs.msg
import geometry_msgs.msg

from box_auto.utils import get_bag, upload_bag


class LastMatch:
    """Class for keeping track of an item in inventory."""

    def __init__(self, imu_debug_msg, timestamp_debug_msg):
        self.imu_arrivial_ros_time = imu_debug_msg.header.stamp.to_sec()
        self.imu_counter = imu_debug_msg.header.seq
        self.imu_time = imu_debug_msg.imu.header.stamp.to_sec()
        self.imu_seq = imu_debug_msg.imu.header.seq

        self.timestamp_time = timestamp_debug_msg.timestamp.data.to_sec()

        self.imu_debug_msg = imu_debug_msg
        self.timestamp_debug_msg = timestamp_debug_msg

    def get_line_delay(self):
        return self.imu_arrivial_ros_time - self.timestamp_time

    def line_delay_valid(self):
        delay = self.get_line_delay()  # timestamp arrives before imu but not more than 3ms
        return delay > 0 and delay < 0.005

    def get_stats(self, other):
        res = {}
        res["N_imu_messages"] = self.imu_seq - other.imu_seq
        res["N_imu_count"] = self.imu_counter - other.imu_counter
        res["N_imu_time"] = round(float(self.imu_time - other.imu_time) / 0.005)
        res["AP20_time_accuracy_check"] = (
            abs(float(other.imu_time + res["N_imu_time"] * 0.005) - self.imu_time) < 0.0005
        )
        res["N_imu_arrivial_ros_time"] = round(float(self.imu_arrivial_ros_time - other.imu_arrivial_ros_time) / 0.005)
        res["N_timestamp_time"] = round(float(self.timestamp_time - other.timestamp_time) / 0.005)
        res["HardwareTS_time_accuracy_check"] = (
            abs((other.timestamp_time + res["N_timestamp_time"] * 0.005) - self.timestamp_time) < 0.001
        )
        res["HardwareTS_shutdown_timestamp"] = (self.timestamp_time - other.timestamp_time) < 0.001
        res["ROS_arrivial_time_check"] = self.line_delay_valid()
        res["ROS_arrivial_line_delay"] = self.get_line_delay()

        return res

    def publish(self, bag, counter):
        self.imu_debug_msg.imu.header.stamp = rospy.Time.from_sec(self.timestamp_time)
        self.imu_debug_msg.imu.header.seq = counter
        self.imu_debug_msg.imu.header.frame_id = "ap20_imu"
        bag.write("/gt_box/ap20/imu", self.imu_debug_msg.imu, self.imu_debug_msg.imu.header.stamp)
        counter += 1


def read_bag_file(bag_path):
    last_match = None
    imu_data = deque()
    tps_data = deque()
    imu_debug_data = deque()
    position_debug_data = deque()
    timestamp_debug_data = deque()
    counter = 0
    line_delay = []
    matching_results = {
        "prefect_match": 0,
        "arrivial_time_mismatch": 0,
        "failed": 0,
        "reset": 0,
        "skipped_imu": 0,
        "initial_skip": 0,
    }

    all_matches = []
    counter_ts = 0
    output_bag_path = bag_path.replace("jetson_ap20_aux", "jetson_ap20_synced")

    if os.path.exists(output_bag_path):
        os.remove(output_bag_path)

    with rosbag.Bag(output_bag_path, "w") as bag_out:
        with rosbag.Bag(bag_path, "r") as bag:

            for topic, msg, t in bag.read_messages(
                topics=["/gt_box/ap20/imu_debug", "/gt_box/ap20/position_debug", "/gt_box/ap20/timestamp_debug"]
            ):
                if topic == "/gt_box/ap20/imu_debug":
                    imu_debug_data.append(msg)
                elif topic == "/gt_box/ap20/position_debug":
                    position_debug_data.append(msg)
                elif topic == "/gt_box/ap20/timestamp_debug":
                    timestamp_debug_data.append(msg)

                if len(position_debug_data) > 0:
                    position = position_debug_data.popleft()
                    t = position.position.header.stamp.to_sec()
                    ref = -1
                    for i in range(len(all_matches)):

                        if all_matches[i].imu_time > t:
                            ref = i
                            delta = all_matches[i].imu_time - t
                            break

                    if ref > 0 and ref != -1:
                        p1 = all_matches[ref - 1]
                        p2 = all_matches[ref]
                        if not (delta > 0.005 + 1e-5):
                            rate = (t - p1.imu_time) / (p2.imu_time - p1.imu_time)
                            new_ts = p1.timestamp_time + ((p2.timestamp_time - p1.timestamp_time) * rate)

                            new_pose_stamped_msgs = PoseStamped()
                            new_pose_stamped_msgs.header.stamp = rospy.Time.from_sec(new_ts)
                            new_pose_stamped_msgs.header.seq = counter_ts
                            new_pose_stamped_msgs.header.frame_id = "leica_total_station"
                            new_pose_stamped_msgs.pose.position = position.position.point
                            new_pose_stamped_msgs.pose.orientation.x = 0
                            new_pose_stamped_msgs.pose.orientation.y = 0
                            new_pose_stamped_msgs.pose.orientation.z = 0
                            new_pose_stamped_msgs.pose.orientation.w = 1

                            bag_out.write(
                                "/gt_box/ap20/prism_position_posestamped",
                                new_pose_stamped_msgs,
                                new_pose_stamped_msgs.header.stamp,
                            )

                            new_odometry_msg = Odometry()
                            new_odometry_msg.header.stamp = rospy.Time.from_sec(new_ts)
                            new_odometry_msg.header.seq = counter_ts
                            new_odometry_msg.header.frame_id = "leica_total_station"
                            new_odometry_msg.child_frame_id = "prism"
                            new_odometry_msg.pose.pose.position = position.position.point
                            new_odometry_msg.pose.pose.orientation.x = 0
                            new_odometry_msg.pose.pose.orientation.y = 0
                            new_odometry_msg.pose.pose.orientation.z = 0
                            new_odometry_msg.pose.pose.orientation.w = 1
                            new_odometry_msg.twist.twist.linear.x = 0
                            new_odometry_msg.twist.twist.linear.y = 0
                            new_odometry_msg.twist.twist.linear.z = 0
                            new_odometry_msg.twist.twist.angular.x = 0
                            new_odometry_msg.twist.twist.angular.y = 0
                            new_odometry_msg.twist.twist.angular.z = 0

                            bag_out.write(
                                "/gt_box/ap20/prism_position_odometry",
                                new_odometry_msg,
                                new_odometry_msg.header.stamp,
                            )

                            # Create and write the /tf message
                            tf_msg = tf2_msgs.msg.TFMessage()
                            transform = geometry_msgs.msg.TransformStamped()
                            transform.header.stamp = rospy.Time.from_sec(new_ts)
                            transform.header.frame_id = "leica_total_station"
                            transform.child_frame_id = "prism"
                            transform.transform.translation.x = position.position.point.x
                            transform.transform.translation.y = position.position.point.y
                            transform.transform.translation.z = position.position.point.z
                            transform.transform.rotation.x = 0
                            transform.transform.rotation.y = 0
                            transform.transform.rotation.z = 0
                            transform.transform.rotation.w = 1
                            tf_msg.transforms.append(transform)

                            bag_out.write("/tf", tf_msg, transform.header.stamp)

                            new_msg = PointStamped()
                            new_msg.header.stamp = rospy.Time.from_sec(new_ts)
                            new_msg.header.seq = counter_ts
                            new_msg.header.frame_id = "leica_total_station"
                            new_msg.point.x = position.position.point.x
                            new_msg.point.y = position.position.point.y
                            new_msg.point.z = position.position.point.z
                            counter_ts += 1
                            bag_out.write("/gt_box/ap20/prism_position", new_msg, new_msg.header.stamp)

                            all_matches = all_matches[i - 1 :]
                        else:
                            print("delta to high - removing the timestamp")
                    else:
                        if ref == -1:
                            position_debug_data.appendleft(position)

                if len(imu_debug_data) > 0 and len(timestamp_debug_data) > 0:
                    new_imu_msg = imu_debug_data.popleft()
                    new_timestamp_msg = timestamp_debug_data.popleft()

                    # Create inital match based on guessing
                    if last_match is None:
                        candidate = LastMatch(new_imu_msg, new_timestamp_msg)
                        valid = candidate.line_delay_valid()
                        if valid:
                            last_match = candidate
                            last_match.publish(bag_out, counter)
                            all_matches.append(last_match)
                        else:
                            if candidate.get_line_delay() < 0.0:
                                # Lets skip the IMU message
                                timestamp_debug_data.appendleft(new_timestamp_msg)
                                print(
                                    str(new_imu_msg.header.seq)
                                    + " --- "
                                    + f"{new_imu_msg.header.seq} - ERROR - Skipping Inital IMU message poor matching"
                                )
                                matching_results["initial_skip"] += 1
                    else:
                        candidate = LastMatch(new_imu_msg, new_timestamp_msg)
                        res = candidate.get_stats(last_match)

                        # if  not res["AP20_time_accuracy_check"]:
                        #     print(f"{new_imu_msg.header.seq} ERROR - AP20 Time Accuracy bad")
                        #     ap20_reset in time , stopped streaming

                        if not res["HardwareTS_time_accuracy_check"]:
                            print(
                                str(new_imu_msg.header.seq)
                                + " --- "
                                + f"{new_imu_msg.header.seq} ERROR - Hardware Accuracy bad"
                            )

                        if res["N_imu_messages"] == res["N_imu_count"] != res["N_imu_time"]:
                            print(
                                str(new_imu_msg.header.seq)
                                + " --- "
                                + f"{new_imu_msg.header.seq} WARNING - We most likely lost some IMU messages however timestamps all look good"
                            )
                            # ap20_resumed sync, ap20_reset

                        # This case can always happen.
                        # if not res["ROS_arrivial_time_check"]:
                        #     print(f"{new_imu_msg.header.seq} INFO - Timestamp arrived after imu or with more then 5ms delay")

                        if res["N_imu_time"] == res["N_imu_arrivial_ros_time"] == res["N_timestamp_time"]:
                            # perfect match
                            matching_results["prefect_match"] += 1
                            last_match = candidate
                            last_match.publish(bag_out, counter)
                            all_matches.append(last_match)
                            line_delay.append(res["ROS_arrivial_line_delay"])
                            if line_delay[-1] > 0.01:
                                print(str(new_imu_msg.header.seq) + " --- " + "Really high line delay:", line_delay[-1])

                        elif res["N_imu_time"] == res["N_timestamp_time"]:
                            # good match
                            matching_results["prefect_match"] += 1
                            last_match = candidate
                            last_match.publish(bag_out, counter)
                            all_matches.append(last_match)
                            line_delay.append(res["ROS_arrivial_line_delay"])
                            if line_delay[-1] > 0.01:
                                print(str(new_imu_msg.header.seq) + " --- " + "Really high line delay:", line_delay[-1])

                        elif res["HardwareTS_shutdown_timestamp"] or (
                            res["N_imu_time"] < 0 and res["N_timestamp_time"] == 0
                        ):
                            # ap20_reset
                            print(
                                str(new_imu_msg.header.seq)
                                + " --- "
                                + "WARNING - Throw away timestamp - detected double timestamp shut down behaviour by AP20 when reseting or debouncing of button"
                            )
                            matching_results["reset"] += 1
                            imu_debug_data.appendleft(new_imu_msg)

                        elif res["N_imu_time"] < 0 and res["N_timestamp_time"] > 10:
                            # ap20_resumed_sync
                            print(
                                str(new_imu_msg.header.seq)
                                + " --- "
                                + "WARNING - We assume it is now working again and we are synced == 1.005 ???"
                            )
                            last_match = candidate
                            last_match.publish(bag_out, counter)
                            all_matches.append(last_match)
                            line_delay.append(res["ROS_arrivial_line_delay"])

                        elif (
                            res["N_imu_messages"] == res["N_imu_count"] == res["N_imu_time"]
                            and res["N_timestamp_time"] > res["N_imu_time"]
                        ):
                            print(
                                str(new_imu_msg.header.seq) + " --- " + "Skipping IMU message",
                                res["N_imu_messages"],
                                "- we have to skip: ",
                                res["N_timestamp_time"],
                            )
                            matching_results["skipped_imu"] += 1
                            timestamp_debug_data.appendleft(new_timestamp_msg)

                        else:
                            # failed
                            matching_results["failed"] += 1
                            print(
                                str(new_imu_msg.header.seq) + " --- " + "ERROR - Case not understood",
                                res["N_imu_messages"],
                                " ",
                                res["N_imu_count"],
                                " ",
                                res["N_imu_time"],
                                " ",
                                res["N_imu_arrivial_ros_time"],
                                " ",
                                res["N_timestamp_time"],
                            )
    upload_bag(output_bag_path)

    print(matching_results)
    print(
        "LINE DELAY min: ",
        np.array(line_delay).min(),
        " max: ",
        np.array(line_delay).max(),
        " median: ",
        np.median(np.array(line_delay)),
    )

    print(np.where(np.array(line_delay) == np.array(line_delay).max()))

    return {
        "imu": imu_data,
        "tps": tps_data,
        "imu_debug": imu_debug_data,
        "position_debug": position_debug_data,
        "timestamp_debug": timestamp_debug_data,
    }


if __name__ == "__main__":
    ap20_bag = get_bag("*_jetson_ap20_aux.bag")

    # Usage example
    data = read_bag_file(ap20_bag)
