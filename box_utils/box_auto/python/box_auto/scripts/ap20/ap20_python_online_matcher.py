import rosbag
import numpy as np
from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import Odometry
from collections import deque
import rospy
import argparse
from pathlib import Path
from box_auto.utils.atomic_file import atomic_output
import tf2_msgs.msg
import geometry_msgs.msg
from box_auto.utils import get_bag, upload_bag


# =========================
# Hard-coded parameters
# =========================

# Bag selection / naming
INPUT_BAG_GLOB = "*_jetson_ap20_aux.bag"
INPUT_BAG_NAME_TOKEN = "jetson_ap20_aux"
OUTPUT_BAG_NAME_TOKEN = "jetson_ap20_synced"

# Topics (input)
TOPIC_IMU_DEBUG = "/gt_box/ap20/imu_debug"
TOPIC_POSITION_DEBUG = "/gt_box/ap20/position_debug"
TOPIC_TIMESTAMP_DEBUG = "/gt_box/ap20/timestamp_debug"

# Topics (output)
TOPIC_IMU_OUT = "/gt_box/ap20/imu"
TOPIC_PRISM_POSITION_POSESTAMPED_OUT = "/gt_box/ap20/prism_position_posestamped"
TOPIC_PRISM_POSITION_ODOMETRY_OUT = "/gt_box/ap20/prism_position_odometry"
TOPIC_PRISM_POSITION_POINT_OUT = "/gt_box/ap20/prism_position"
TOPIC_TF_OUT = "/tf"

# Frame IDs
FRAME_ID_IMU = "ap20_imu"
FRAME_ID_LEICA = "leica_total_station"
CHILD_FRAME_ID_PRISM = "prism"

# Timebase / thresholds (seconds)
IMU_TICK_SEC = 0.005  # assumed IMU/timestamp tick (200 Hz)
BAG_START_OFFSET_SEC = 2.0  # skip initial bag portion

LINE_DELAY_VALID_MIN_SEC = 0.0
LINE_DELAY_VALID_MAX_SEC = 0.005  # arrival delay validity window

AP20_TIME_ACCURACY_TOL_SEC = 0.0005
HARDWARE_TS_TIME_ACCURACY_TOL_SEC = 0.001
HARDWARE_TS_SHUTDOWN_TOL_SEC = 0.001

INTERP_DELTA_EPS_SEC = 1e-5  # used with IMU_TICK_SEC in delta check
HIGH_LINE_DELAY_WARN_SEC = 0.01

RESUMED_SYNC_TIMESTAMP_TICKS_THRESHOLD = 10
POSITION_MATCH_RESET_THRESHOLD_SEC = 30

# Preserve FIFO association by default. A wider --window-size is available for
# recordings whose timestamp association has been independently validated;
# monotonic output alone does not establish more accurate synchronization.
TIMESTAMP_MATCH_WINDOW_SIZE = 1

# Fixed pose/twist outputs
IDENTITY_QUATERNION = (0.0, 0.0, 0.0, 1.0)
ZERO_3 = (0.0, 0.0, 0.0)


def d(stamp):
    return rospy.Duration(stamp.secs, stamp.nsecs)


def d_from_sec(s):
    return rospy.Duration.from_sec(s)


class LastMatch:
    """Class for keeping track of an item in inventory."""

    def __init__(self, imu_debug_msg, timestamp_debug_msg):
        self.imu_arrivial_ros_time = imu_debug_msg.header.stamp
        self.imu_counter = imu_debug_msg.header.seq
        self.imu_time = imu_debug_msg.imu.header.stamp
        self.imu_seq = imu_debug_msg.imu.header.seq
        self.timestamp_time = timestamp_debug_msg.timestamp.data

        self.imu_debug_msg = imu_debug_msg
        self.timestamp_debug_msg = timestamp_debug_msg

    def get_line_delay(self):
        if self.imu_arrivial_ros_time > self.timestamp_time:
            return (self.imu_arrivial_ros_time - self.timestamp_time).to_sec()
        else:
            return -(self.timestamp_time - self.imu_arrivial_ros_time).to_sec()

    def line_delay_valid(self):
        delay = self.get_line_delay()  # timestamp arrives before imu but not more than 3ms
        return delay > LINE_DELAY_VALID_MIN_SEC and delay < LINE_DELAY_VALID_MAX_SEC

    def get_stats(self, other):
        res = {}
        res["N_imu_messages"] = self.imu_seq - other.imu_seq
        res["N_imu_count"] = self.imu_counter - other.imu_counter
        res["N_imu_time"] = round(float((self.imu_time - other.imu_time).to_sec()) / IMU_TICK_SEC)

        res["AP20_time_accuracy_check"] = (
            abs(((other.imu_time.to_sec() + res["N_imu_time"] * IMU_TICK_SEC) - self.imu_time.to_sec()))
            < AP20_TIME_ACCURACY_TOL_SEC
        )
        res["N_imu_arrivial_ros_time"] = round(
            (self.imu_arrivial_ros_time.to_sec() - other.imu_arrivial_ros_time.to_sec()) / IMU_TICK_SEC
        )

        res["N_timestamp_time"] = round((self.timestamp_time.to_sec() - other.timestamp_time.to_sec()) / IMU_TICK_SEC)
        res["HardwareTS_time_accuracy_check"] = (
            abs(
                (
                    other.timestamp_time.to_sec()
                    + (res["N_timestamp_time"] * IMU_TICK_SEC)
                    - self.timestamp_time.to_sec()
                )
            )
            < HARDWARE_TS_TIME_ACCURACY_TOL_SEC
        )
        res["HardwareTS_shutdown_timestamp"] = (
            self.timestamp_time.to_sec() - other.timestamp_time.to_sec()
        ) < HARDWARE_TS_SHUTDOWN_TOL_SEC
        res["ROS_arrivial_time_check"] = self.line_delay_valid()
        res["ROS_arrivial_line_delay"] = self.get_line_delay()

        return res

    def publish(self, bag, counter):
        self.imu_debug_msg.imu.header.stamp = self.timestamp_time
        self.imu_debug_msg.imu.header.seq = counter
        self.imu_debug_msg.imu.header.frame_id = FRAME_ID_IMU
        bag.write(TOPIC_IMU_OUT, self.imu_debug_msg.imu, self.imu_debug_msg.imu.header.stamp)
        counter += 1
        return counter


def pop_deque_index(dq, idx):
    dq.rotate(-idx)
    item = dq.popleft()
    dq.rotate(idx)
    return item


def push_deque_index(dq, item, idx):
    dq.rotate(-idx)
    dq.appendleft(item)
    dq.rotate(idx)


def select_timestamp_for_imu(imu_msg, ts_deque, last_match, window_size=TIMESTAMP_MATCH_WINDOW_SIZE):
    n = min(len(ts_deque), window_size)
    if n <= 0:
        return None, None

    best_idx = 0
    best_score = None

    # Evaluate only the first window_size timestamps (time-local association).
    for i in range(n):
        ts_msg = ts_deque[i]
        candidate = LastMatch(imu_msg, ts_msg)

        if last_match is None:
            # Prefer a valid arrival delay; otherwise choose the closest-to-zero delay.
            delay = candidate.get_line_delay()
            if candidate.line_delay_valid():
                score = (0, delay)
            else:
                score = (1, abs(delay))
        else:
            res = candidate.get_stats(last_match)

            # Primary: timestamp tick consistency with imu tick.
            primary = abs(res["N_timestamp_time"] - res["N_imu_time"])

            # Secondary: minimize line delay (and prefer valid delay).
            delay = candidate.get_line_delay()
            penalty = 0 if candidate.line_delay_valid() else 1
            score = (primary, penalty, abs(delay))

        if best_score is None or score < best_score:
            best_score = score
            best_idx = i

    chosen = pop_deque_index(ts_deque, best_idx)
    return chosen, best_idx


def read_bag_file(
    bag_path,
    output_bag_path=None,
    *,
    upload=True,
    window_size=TIMESTAMP_MATCH_WINDOW_SIZE,
    start_offset=BAG_START_OFFSET_SEC,
):
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
    counter_positions_in = 0
    if not np.isfinite(start_offset) or start_offset < 0:
        raise ValueError("Startup offset must be finite and nonnegative")
    input_counts = {}
    read_counts = {TOPIC_IMU_DEBUG: 0, TOPIC_POSITION_DEBUG: 0, TOPIC_TIMESTAMP_DEBUG: 0}
    if window_size < 1:
        raise ValueError("Timestamp matching window must be positive")
    bag_path = str(bag_path)
    output_bag_path = str(output_bag_path or bag_path.replace(INPUT_BAG_NAME_TOKEN, OUTPUT_BAG_NAME_TOKEN))
    if Path(bag_path).resolve() == Path(output_bag_path).resolve():
        raise ValueError("Provide a distinct output path for this input filename")

    with atomic_output(output_bag_path) as temporary, rosbag.Bag(str(temporary), "w", compression="lz4") as bag_out:
        with rosbag.Bag(bag_path, "r") as bag:
            if not bag.get_message_count():
                raise ValueError("AP20 input bag is empty")
            bag_start_time = next(bag.read_messages()).timestamp
            # Preserve the existing startup guard, but make its data loss explicit
            # in the return report and allow callers to choose an offset of zero.
            target_time = bag_start_time + rospy.Duration.from_sec(start_offset)
            input_counts = {topic: bag.get_message_count(topic_filters=[topic]) for topic in read_counts}

            for topic, msg, t in bag.read_messages(
                topics=[TOPIC_IMU_DEBUG, TOPIC_POSITION_DEBUG, TOPIC_TIMESTAMP_DEBUG],
                start_time=target_time,
            ):
                read_counts[topic] += 1
                if topic == TOPIC_IMU_DEBUG:
                    imu_debug_data.append(msg)
                elif topic == TOPIC_POSITION_DEBUG:
                    position_debug_data.append(msg)
                    counter_positions_in += 1
                elif topic == TOPIC_TIMESTAMP_DEBUG:
                    timestamp_debug_data.append(msg)

                if len(position_debug_data) > 0:
                    position = position_debug_data.popleft()
                    t = position.position.header.stamp  # .to_sec()
                    ref = -1
                    for i in range(len(all_matches)):
                        if all_matches[i].imu_time > t:
                            ref = i
                            delta = (all_matches[i].imu_time - t).to_sec()
                            break

                    if ref > 0 and ref != -1:
                        p1 = all_matches[ref - 1]
                        p2 = all_matches[ref]
                        if not (delta > IMU_TICK_SEC + INTERP_DELTA_EPS_SEC):
                            rate = (t - p1.imu_time) / (p2.imu_time - p1.imu_time)
                            new_ts = p1.timestamp_time + ((p2.timestamp_time - p1.timestamp_time) * rate)

                            new_pose_stamped_msgs = PoseStamped()
                            new_pose_stamped_msgs.header.stamp = new_ts
                            new_pose_stamped_msgs.header.seq = counter_ts
                            new_pose_stamped_msgs.header.frame_id = FRAME_ID_LEICA
                            new_pose_stamped_msgs.pose.position = position.position.point
                            new_pose_stamped_msgs.pose.orientation.x = IDENTITY_QUATERNION[0]
                            new_pose_stamped_msgs.pose.orientation.y = IDENTITY_QUATERNION[1]
                            new_pose_stamped_msgs.pose.orientation.z = IDENTITY_QUATERNION[2]
                            new_pose_stamped_msgs.pose.orientation.w = IDENTITY_QUATERNION[3]

                            bag_out.write(
                                TOPIC_PRISM_POSITION_POSESTAMPED_OUT,
                                new_pose_stamped_msgs,
                                new_pose_stamped_msgs.header.stamp,
                            )

                            new_odometry_msg = Odometry()
                            new_odometry_msg.header.stamp = new_ts
                            new_odometry_msg.header.seq = counter_ts
                            new_odometry_msg.header.frame_id = FRAME_ID_LEICA
                            new_odometry_msg.child_frame_id = CHILD_FRAME_ID_PRISM
                            new_odometry_msg.pose.pose.position = position.position.point
                            new_odometry_msg.pose.pose.orientation.x = IDENTITY_QUATERNION[0]
                            new_odometry_msg.pose.pose.orientation.y = IDENTITY_QUATERNION[1]
                            new_odometry_msg.pose.pose.orientation.z = IDENTITY_QUATERNION[2]
                            new_odometry_msg.pose.pose.orientation.w = IDENTITY_QUATERNION[3]
                            new_odometry_msg.twist.twist.linear.x = ZERO_3[0]
                            new_odometry_msg.twist.twist.linear.y = ZERO_3[1]
                            new_odometry_msg.twist.twist.linear.z = ZERO_3[2]
                            new_odometry_msg.twist.twist.angular.x = ZERO_3[0]
                            new_odometry_msg.twist.twist.angular.y = ZERO_3[1]
                            new_odometry_msg.twist.twist.angular.z = ZERO_3[2]

                            bag_out.write(
                                TOPIC_PRISM_POSITION_ODOMETRY_OUT,
                                new_odometry_msg,
                                new_odometry_msg.header.stamp,
                            )

                            # Create and write the /tf message
                            tf_msg = tf2_msgs.msg.TFMessage()
                            transform = geometry_msgs.msg.TransformStamped()
                            transform.header.stamp = new_ts
                            transform.header.frame_id = FRAME_ID_LEICA
                            transform.child_frame_id = CHILD_FRAME_ID_PRISM
                            transform.transform.translation.x = position.position.point.x
                            transform.transform.translation.y = position.position.point.y
                            transform.transform.translation.z = position.position.point.z
                            transform.transform.rotation.x = IDENTITY_QUATERNION[0]
                            transform.transform.rotation.y = IDENTITY_QUATERNION[1]
                            transform.transform.rotation.z = IDENTITY_QUATERNION[2]
                            transform.transform.rotation.w = IDENTITY_QUATERNION[3]
                            tf_msg.transforms.append(transform)

                            bag_out.write(TOPIC_TF_OUT, tf_msg, transform.header.stamp)

                            new_msg = PointStamped()
                            new_msg.header.stamp = new_ts
                            new_msg.header.seq = counter_ts
                            new_msg.header.frame_id = FRAME_ID_LEICA
                            new_msg.point.x = position.position.point.x
                            new_msg.point.y = position.position.point.y
                            new_msg.point.z = position.position.point.z
                            counter_ts += 1
                            bag_out.write(TOPIC_PRISM_POSITION_POINT_OUT, new_msg, new_msg.header.stamp)

                            all_matches = all_matches[i - 1 :]
                        else:
                            print("delta to high - removing the timestamp")
                    else:
                        if ref == -1:
                            position_debug_data.appendleft(position)
                        else:
                            print("Skipped writing the message ?", ref)
                            print(
                                "Last IMU match ",
                                all_matches[0].imu_time.to_sec(),
                                "Match position data",
                                position.position.header.stamp.to_sec(),
                            )
                            if (
                                all_matches[0].imu_time.to_sec()
                                > position.position.header.stamp.to_sec() + POSITION_MATCH_RESET_THRESHOLD_SEC
                            ):
                                print("Wrong reset matching positions")

                if len(imu_debug_data) > 0 and len(timestamp_debug_data) > 0:
                    new_imu_msg = imu_debug_data.popleft()
                    new_timestamp_msg, ts_idx = select_timestamp_for_imu(
                        new_imu_msg, timestamp_debug_data, last_match, window_size=window_size
                    )

                    # Create inital match based on guessing
                    if last_match is None:
                        candidate = LastMatch(new_imu_msg, new_timestamp_msg)
                        valid = candidate.line_delay_valid()
                        if valid:
                            last_match = candidate
                            counter = last_match.publish(bag_out, counter)

                            all_matches = []

                            all_matches.append(last_match)
                        else:
                            if candidate.get_line_delay() < 0.0:
                                # Lets skip the IMU message
                                push_deque_index(timestamp_debug_data, new_timestamp_msg, ts_idx)
                                print(
                                    str(new_imu_msg.header.seq)
                                    + " --- "
                                    + f"{new_imu_msg.header.seq} - ERROR - Skipping Inital IMU message poor matching"
                                )
                                matching_results["initial_skip"] += 1
                    else:
                        candidate = LastMatch(new_imu_msg, new_timestamp_msg)
                        # A repeated/backward hardware stamp cannot timestamp a
                        # new IMU sample. Retain the IMU for the next timestamp.
                        if candidate.timestamp_time <= last_match.timestamp_time:
                            matching_results["reset"] += 1
                            imu_debug_data.appendleft(new_imu_msg)
                            continue
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
                            # Correct way would be to only discard the timestamp

                            if res["N_imu_time"] > 0:
                                imu_debug_data.appendleft(new_imu_msg)
                                continue

                            print("resync")

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
                            counter = last_match.publish(bag_out, counter)

                            # if all_matches[-1].imu_time.to_sec() > last_match.imu_time.to_sec():
                            #     all_matches = []

                            all_matches.append(last_match)
                            line_delay.append(res["ROS_arrivial_line_delay"])
                            if line_delay[-1] > HIGH_LINE_DELAY_WARN_SEC:
                                print(str(new_imu_msg.header.seq) + " --- " + "Really high line delay:", line_delay[-1])

                        elif res["N_imu_time"] == res["N_timestamp_time"]:
                            # good match
                            matching_results["prefect_match"] += 1
                            last_match = candidate
                            counter = last_match.publish(bag_out, counter)

                            # if all_matches[-1].imu_time.to_sec() > last_match.imu_time.to_sec():
                            #     all_matches = []
                            all_matches.append(last_match)
                            line_delay.append(res["ROS_arrivial_line_delay"])
                            if line_delay[-1] > HIGH_LINE_DELAY_WARN_SEC:
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

                        elif res["N_imu_time"] < 0 and res["N_timestamp_time"] > RESUMED_SYNC_TIMESTAMP_TICKS_THRESHOLD:
                            # ap20_resumed_sync
                            print(
                                str(new_imu_msg.header.seq)
                                + " --- "
                                + "WARNING - We assume it is now working again and we are synced == 1.005 ???"
                            )

                            last_match = candidate
                            counter = last_match.publish(bag_out, counter)
                            # if all_matches[-1].imu_time.to_sec() > last_match.imu_time.to_sec():
                            #     all_matches = []

                            if all_matches[-1].imu_time.to_sec() > last_match.imu_time.to_sec():
                                all_matches = []

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
                            push_deque_index(timestamp_debug_data, new_timestamp_msg, ts_idx)

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
        if not counter or (counter_positions_in and not counter_ts):
            raise ValueError("AP20 synchronization produced no usable IMU/position output")
    if upload:
        upload_bag(output_bag_path)

    print(matching_results)
    if line_delay:
        print("LINE DELAY min/max/median:", min(line_delay), max(line_delay), np.median(line_delay))
    print("counter_positions_in:", counter_positions_in, "counter_ts:", counter_ts)

    return {
        "input_counts": input_counts,
        "read_counts": read_counts,
        "startup_skipped": {topic: input_counts[topic] - read_counts[topic] for topic in read_counts},
        "unpublished_positions_after_start": counter_positions_in - counter_ts,
        "imu_count": counter,
        "position_count": counter_ts,
        "matching_results": matching_results,
        "imu": imu_data,
        "tps": tps_data,
        "imu_debug": imu_debug_data,
        "position_debug": position_debug_data,
        "timestamp_debug": timestamp_debug_data,
    }


def main(argv=None):
    parser = argparse.ArgumentParser(description="Synchronize AP20 using hardware timestamp and IMU tick consistency")
    parser.add_argument("--input", type=Path)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--window-size", type=int, default=TIMESTAMP_MATCH_WINDOW_SIZE)
    parser.add_argument("--start-offset", type=float, default=BAG_START_OFFSET_SEC)
    parser.add_argument("--no-upload", action="store_true")
    args = parser.parse_args(argv)
    read_bag_file(
        args.input or get_bag(INPUT_BAG_GLOB),
        args.output,
        upload=not args.no_upload,
        window_size=args.window_size,
        start_offset=args.start_offset,
    )


if __name__ == "__main__":
    main()
