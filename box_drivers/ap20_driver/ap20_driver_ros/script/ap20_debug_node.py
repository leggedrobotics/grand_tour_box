#!/usr/bin/env python3

import rospy
from std_msgs.msg import Time
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PointStamped
from std_srvs.srv import SetBool
from collections import deque
import threading
import subprocess
from ap20_driver_ros.msg import ImuDebug, PositionDebug, TimestampDebug
import numpy as np


class ContinuousTimestampReader:
    def __init__(self, timestamps_fn):
        self.timestamps_fn = timestamps_fn
        self.timestamps_file = None
        self.open_file()

    def open_file(self):
        if self.timestamps_file is None or self.timestamps_file.closed:
            try:
                self.timestamps_file = open(self.timestamps_fn, "r")
            except IOError as e:
                rospy.logerr(f"Failed to open timestamps file: {self.timestamps_fn}. Error: {e}")

    def read_timestamps(self):
        ret = []
        if self.timestamps_file is None or self.timestamps_file.closed:
            self.open_file()
            if self.timestamps_file is None:
                return ret

        self.timestamps_file.seek(0)
        for line in self.timestamps_file:
            try:
                secs, nsecs = map(int, line.strip().split("."))
                ret.append(rospy.Time(secs, nsecs))
            except ValueError:
                rospy.logwarn("Unexpected format in timestamps file")
                continue

        return ret


class AP20Node:
    def __init__(self):
        rospy.init_node("~ap20")

        self.verbose = rospy.get_param("~verbose", True)
        self.debug = rospy.get_param("~debug", False)
        self.timestamp_queue = deque()
        self.imu_message_queue = deque()
        self.position_message_queue = deque()

        self.imu_timestamp_mapping = deque()

        self.imu_message_counter = 0
        self.timestamp_counter = 0
        self.position_message_counter = 0
        self.internal_counter_imu_ap20 = -1
        self.shutdown = False
        self.imu_message_queue_lock = threading.Lock()
        self.offset_between_seq_numbers = 0
        self.last_published_tpos = -1

        self.ap20_imu_pub = rospy.Publisher("~imu", Imu, queue_size=200)
        self.ap20_position_pub = rospy.Publisher("~prism_position", PointStamped, queue_size=100)

        if self.debug:
            # New debug publishers
            self.imu_debug_pub = rospy.Publisher("~imu_debug", ImuDebug, queue_size=200)
            self.position_debug_pub = rospy.Publisher("~position_debug", PositionDebug, queue_size=100)
            self.timestamp_debug_pub = rospy.Publisher("~timestamp_debug", TimestampDebug, queue_size=100)

        self.imu_sub = rospy.Subscriber("/ap20/imu", Imu, self.imu_callback, queue_size=100)
        self.position_sub = rospy.Subscriber("/ap20/tps", PointStamped, self.position_callback, queue_size=20)

        self.timestamps_fn = "/sys/kernel/time_stamper/ts_buffer"
        self.timestamp_reader = ContinuousTimestampReader(self.timestamps_fn)

    def run_command_async(self, command):
        subprocess.Popen(command, shell=True)

    def change_imu_mode(self, enable_streaming):
        service_name = "/ap20/enable_streaming"
        rospy.wait_for_service(service_name, timeout=15.0)
        try:
            change_mode = rospy.ServiceProxy(service_name, SetBool)
            response = change_mode(enable_streaming)
            if response.success:
                rospy.loginfo(
                    f"IMU mode changed successfully. Streaming is now {'enabled' if enable_streaming else 'disabled'}."
                )
                return True
            else:
                rospy.logwarn(f"Failed to change IMU mode: {response.message}")
                return False
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            return False

    def imu_callback(self, msg):
        self.imu_message_queue.appendleft(msg)
        # print("IMU        : ", msg.header.seq, msg.header.stamp.to_sec())
        # Create and publish ImuDebug message
        if self.debug:
            imu_debug = ImuDebug()
            imu_debug.header.stamp = rospy.Time.now()
            imu_debug.header.seq = self.imu_message_counter
            imu_debug.imu = msg
            self.imu_debug_pub.publish(imu_debug)
            self.offset_between_seq_numbers = msg.header.seq - self.internal_counter_imu_ap20

        if msg.header.seq != self.internal_counter_imu_ap20 + 1 and self.internal_counter_imu_ap20 != -1:
            rospy.logerr(f"Missed message: {self.internal_counter_imu_ap20} -> {msg.header.seq}")
            self.shutdown = True

        self.internal_counter_imu_ap20 = msg.header.seq
        self.imu_message_counter += 1

    def position_callback(self, msg):
        # print("Position: ", msg.header.seq, msg.header.stamp.to_sec())
        self.position_message_queue.appendleft(msg)

        if self.debug:
            # Create and publish PositionDebug message
            position_debug = PositionDebug()
            position_debug.header.stamp = rospy.Time.now()
            position_debug.header.seq = self.position_message_counter
            position_debug.position = msg
            self.position_debug_pub.publish(position_debug)

        self.position_message_counter += 1

    def run(self):
        rate = rospy.Rate(500)
        steps_without_new_timestamps = 0

        while not rospy.is_shutdown():
            while True:
                self.change_imu_mode(False)
                _ = self.timestamp_reader.read_timestamps()
                rospy.loginfo("Wait 0.5s and verify that no new timestamps are streaming")

                self.imu_message_queue.clear()
                self.imu_message_counter = 0
                self.timestamp_counter = 0
                self.internal_counter_imu_ap20 = -1
                rospy.sleep(1)

                old_timestamps_ = self.timestamp_reader.read_timestamps()

                if len(old_timestamps_) == 0 and self.imu_message_counter == 0:
                    break
                rospy.logerr("Still timestamps are streaming.")

            rospy.loginfo("Driver successfully started")
            self.change_imu_mode(True)

            max_size_timestamps = 0
            max_size_imu = 0
            max_size_position = 0
            max_added = 0
            last_time = None
            diff_time_max = 0

            # This is neede because the tps are continued to be streamed
            self.position_message_queue.clear()
            self.position_message_counter = 0
            self.lookup_seq = []
            self.lookup_ts = []
            self.line_delay = []
            self.last_published_imu_seq = -1

            while True:
                if self.imu_message_counter % 1000 == 0:
                    print(
                        f"IMU messages: {self.imu_message_counter} - Timestamps: {self.timestamp_counter} - Position messages: {self.position_message_counter}"
                    )
                steps_without_new_timestamps += 1

                if steps_without_new_timestamps > 2000:
                    rospy.logerr("No new timestamps for 2000 steps. Restart.")
                    steps_without_new_timestamps = 0
                    break

                stamps = self.timestamp_reader.read_timestamps()
                max_added = max(max_added, len(stamps))

                for ts in stamps:
                    steps_without_new_timestamps = 0
                    time_msg = Time(ts)

                    if self.debug:
                        if last_time is not None:
                            # Create a new Time object with the difference
                            diff_time = (time_msg.data) - (last_time.data)
                            # Print the difference
                            diff_time_max = max(diff_time_max, diff_time.to_sec())
                        last_time = time_msg

                    self.timestamp_queue.appendleft(time_msg)

                    if self.debug:
                        # Create and publish TimestampDebug message
                        timestamp_debug = TimestampDebug()
                        timestamp_debug.header.stamp = rospy.Time.now()
                        timestamp_debug.header.seq = self.timestamp_counter
                        timestamp_debug.timestamp = time_msg
                        self.timestamp_debug_pub.publish(timestamp_debug)

                    self.timestamp_counter += 1

                self.publish_imu_messages()
                self.publish_position_messages()

                if self.debug:
                    max_size_timestamps = max(max_size_timestamps, len(self.timestamp_queue))
                    max_size_imu = max(max_size_imu, len(self.imu_message_queue))
                    max_size_position = max(max_size_position, len(self.position_message_queue))

                    log_msg = f"""
Current queue sizes --- Max added new timestamps {max_added} --- Max time difference {diff_time_max}  -- Offset seq numbers {self.offset_between_seq_numbers}:
    Timestamps:        {len(self.timestamp_queue)} - Counts {self.timestamp_counter}  - Max size {max_size_timestamps}
    IMU messages:      {len(self.imu_message_queue)} - Counts {self.imu_message_counter} - Max size {max_size_imu}
    Position messages: {len(self.position_message_queue)} - Counts {self.position_message_counter} - Max size {max_size_position}
    Memory List:       {len(self.lookup_seq)}
                    """
                    rospy.loginfo(log_msg)

                rate.sleep()

                if self.shutdown:
                    rospy.logerr("Shutting down due to line-delay error.")
                    break

    def publish_position_messages(self):
        if self.position_message_queue:
            position = self.position_message_queue.pop()

            ap20_original_ts = float(position.header.stamp.to_sec())

            if len(self.lookup_seq) == 0:
                return

            j = 0
            delta = 0
            for k, imu_original in enumerate(self.lookup_seq):
                delta = imu_original - ap20_original_ts
                if delta > 0:
                    j = k
                    break

            if delta > 0.01 + 1e-5:
                # rospy.logerr(f"Delta is too high: {delta}")
                return

            if j == 0:
                # rospy.logerr("TPS message is too old")
                return

            if j == len(self.lookup_ts):
                self.position_message_queue.appendright(position)
                # rospy.logerr("TPS message is too new")
                return

            rate = (ap20_original_ts - self.lookup_seq[j - 1]) / (self.lookup_seq[j] - self.lookup_seq[j - 1])
            new_ts = self.lookup_ts[j - 1] + ((self.lookup_ts[j] - self.lookup_ts[j - 1]) * rate)

            position.header.stamp = new_ts
            self.ap20_position_pub.publish(position)
            self.lookup_seq = self.lookup_seq[j - 1 :]
            self.lookup_ts = self.lookup_ts[j - 1 :]

    def publish_imu_messages(self):
        if self.timestamp_queue and self.imu_message_queue:
            time_msg = self.timestamp_queue.pop()
            imu = self.imu_message_queue.pop()

            self.lookup_seq.append(float(imu.header.stamp.to_sec()))
            self.lookup_ts.append(time_msg.data.to_sec())

            new_line_delay = self.lookup_seq[-1] - self.lookup_ts[-1]
            if len(self.line_delay) == 20:
                self.line_delay.pop(0)
                est = np.array(self.line_delay).mean()
                # IMU Messages come in at 100 Hz so we should be robous up to half a message delay 5ms
                if abs(est - new_line_delay) > 0.005:
                    self.shutdown = True
                    rospy.logerr("Line delay is too high. Time skipp detected.")

            self.line_delay.append(new_line_delay)

            if len(self.lookup_seq) > 100:
                self.lookup_seq = self.lookup_seq[-100:]
                self.lookup_ts = self.lookup_ts[-100:]

            self.last_published_imu_seq = imu.header.seq
            imu.header.stamp = time_msg.data
            imu.header.frame_id = "ap20_imu"
            self.ap20_imu_pub.publish(imu)


if __name__ == "__main__":
    node = AP20Node()
    node.run()
