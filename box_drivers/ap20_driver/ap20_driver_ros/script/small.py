#!/usr/bin/env python3

import rospy
from std_msgs.msg import Time
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PointStamped
from std_srvs.srv import SetBool
from collections import deque
import threading
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


def change_imu_mode(enable_streaming):
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


class AP20Node:
    def __init__(self):
        rospy.init_node("~ap20")

        self.publish_timestamps = rospy.get_param("~publish_timestamps", True)
        self.timestamp_queue = deque()
        self.imu_message_queue = deque()
        self.position_message_queue = deque()
        self.imu_timestamp_mapping = deque()

        self.imu_message_counter = 0
        self.timestamp_counter = 0
        self.internal_counter_imu_ap20 = -1
        self.shutdown = False
        self.imu_message_queue_lock = threading.Lock()
        self.offset_between_seq_numbers = 0
        self.last_published_tpos = -1

        self.timestamps_fn = "/sys/kernel/time_stamper/ts_buffer"
        self.timestamp_reader = ContinuousTimestampReader(self.timestamps_fn)

        self.ap20_imu_pub = rospy.Publisher("~imu", Imu, queue_size=200)
        self.ap20_position_pub = rospy.Publisher("~prism_position", PointStamped, queue_size=100)

        self.imu_sub = rospy.Subscriber("/ap20/imu", Imu, self.imu_callback, queue_size=100)
        self.position_sub = rospy.Subscriber("/ap20/tps", PointStamped, self.position_callback, queue_size=20)

    def imu_callback(self, msg):
        self.imu_message_queue.appendleft(msg)

        if msg.header.seq != self.internal_counter_imu_ap20 + 1 and self.internal_counter_imu_ap20 != -1:
            rospy.logerr(f"Missed message: {self.internal_counter_imu_ap20} -> {msg.header.seq}")
            self.shutdown = True

        self.internal_counter_imu_ap20 = msg.header.seq
        self.imu_message_counter += 1

    def position_callback(self, msg):
        # print("Position: ", msg.header.seq, msg.header.stamp.to_sec())
        self.position_message_queue.appendleft(msg)

    def run(self):
        rate = rospy.Rate(500)

        while not rospy.is_shutdown():
            while True:
                change_imu_mode(False)
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

            change_imu_mode(True)

            max_added = 0
            self.shutdown = False
            # This is neede because the tps are continued to be streamed
            self.position_message_queue.clear()
            self.lookup_seq = []
            self.lookup_ts = []
            self.line_delay = []
            self.last_published_imu_seq = -1

            while True:
                stamps = self.timestamp_reader.read_timestamps()
                max_added = max(max_added, len(stamps))
                for ts in stamps:
                    time_msg = Time(ts)
                    self.timestamp_queue.appendleft(time_msg)
                    self.timestamp_counter += 1

                self.publish_imu_messages()
                self.publish_position_messages()
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

            position.header.stamp = rospy.Time.from_sec(new_ts)
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
            if len(self.line_delay) == 5:
                self.line_delay.pop(0)
                est = np.median(np.array(self.line_delay))
                # IMU Messages come in at 100 Hz so we should be robous up to half a message delay 5ms
                if abs(est - new_line_delay) > 0.005:
                    print(new_line_delay)
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
