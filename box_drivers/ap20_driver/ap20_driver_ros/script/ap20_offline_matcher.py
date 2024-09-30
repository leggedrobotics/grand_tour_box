import rosbag
import numpy as np
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PointStamped
from ap20_driver_ros.msg import ImuDebug, PositionDebug, TimestampDebug
import rosbag
from collections import deque

def read_bag_file(bag_path):
    imu_data = deque()
    tps_data = deque()
    imu_debug_data = deque()
    position_debug_data = deque()
    timestamp_debug_data = deque()

    with rosbag.Bag(bag_path, 'r') as bag:
        for topic, msg, t in bag.read_messages(topics=['/ap20/imu', '/ap20/tps', '/gt_box/ap20/imu_debug',
                                                       '/gt_box/ap20/position_debug', '/gt_box/ap20/timestamp_debug']):
            if topic == '/ap20/imu':
                imu_data.append([
                    msg.header.stamp.to_sec(),
                    msg.header.seq,
                ])
            elif topic == '/ap20/tps':
                tps_data.append([
                    msg.header.stamp.to_sec(),
                    msg.header.seq,
                ])
            elif topic == '/gt_box/ap20/imu_debug':
                imu_debug_data.append([
                    msg.header.stamp.to_sec(),
                    msg.header.seq,
                    msg.imu.header.stamp.to_sec(),
                    msg.imu.header.seq,
                ])
            elif topic == '/gt_box/ap20/position_debug':
                position_debug_data.append([
                    msg.header.stamp.to_sec(),
                    msg.header.seq,
                    msg.position.header.stamp.to_sec(),
                    msg.position.header.seq,
                ])
            elif topic == '/gt_box/ap20/timestamp_debug':
                timestamp_debug_data.append([
                    msg.header.stamp.to_sec(),
                    msg.header.seq,
                    msg.timestamp.data.to_sec(),
                ])

    return {
        'imu': imu_data,
        'tps': tps_data,
        'imu_debug': imu_debug_data,
        'position_debug': position_debug_data,
        'timestamp_debug': timestamp_debug_data
    }

# Usage example
bag_path = '/Data/Projects/GrandTour/ap20_matching_data/2024-09-28-19-33-31_jetson_ap20_aux_0.bag'
data = read_bag_file(bag_path)

# Access the data
print("IMU data shape:", data['imu'])
print("TPS data shape:", len(data['tps']))
print("IMU debug data shape:", len(data['imu_debug']))
print("Position debug data shape:", len(data['position_debug']))
print("Timestamp debug data shape:", len(data['timestamp_debug']))

class LastMatch:
    """Class for keeping track of an item in inventory."""
    def __init__( self, imu_debug, timestamp_debug):
        self.imu_arrivial_ros_time = imu_debug[0]
        self.imu_counter = imu_debug[1]
        self.imu_time = imu_debug[2]
        self.imu_seq = imu_debug[3]

        self.timestamp_time = timestamp_debug[2]
        self.timestamp_counter = timestamp_debug[1]
    def get_line_delay(self):
        return self.imu_arrivial_ros_time - self.timestamp_time
    
matching_results = {
    "prefect_match": 0,
    "arrivial_time_mismatch": 0,
    "failed": 0,
}

# Manual alignment THIS IS IMPORTANT SHOULD BE FIXED VIA RECORDING PACKAGE NOW
data['imu_debug'].popleft()
latest_match = LastMatch(data['imu_debug'].popleft(), data['timestamp_debug'].popleft())

line_delay = []

while len(data['imu_debug']) > 0 and len(data['timestamp_debug']) > 0:
    imu = data['imu_debug'].popleft()
    
    N_imu_messages = imu[3] - latest_match.imu_seq 
    N_imu_count = imu[1] - latest_match.imu_counter
    N_imu_time = round( float(imu[2] - latest_match.imu_time) / 0.005 )

    if  abs( float(latest_match.imu_time + N_imu_time * 0.005 ) - imu[2]) > 0.0005:
        print("AP20 Time Accuracy fucked")

    N_imu_arrivial_ros_time = round(  float(imu[0] - latest_match.imu_arrivial_ros_time) / 0.005 )


    timestamp = data['timestamp_debug'].popleft()
    N_timestamp_time = round( float(timestamp[2] - latest_match.timestamp_time) / 0.005 )
    err = abs( (latest_match.timestamp_time + N_timestamp_time * 0.005 ) - timestamp[2])
    if  err > 0.001:
        print("Hardware Accuracy fucked: ", err)

    if N_imu_messages == N_imu_count !=  N_imu_time:
        print("We most likely lost some IMU messages however timestamps all look good")

    if N_imu_time != 1:
        print("(no error - maybe communication drop) Multiple Messages Missed", N_imu_messages , " ",N_imu_count , " ", N_imu_time , " ", N_imu_arrivial_ros_time, " ", N_timestamp_time)
    
    # perfect match
    if N_imu_time == N_imu_arrivial_ros_time == N_timestamp_time:
        matching_results["prefect_match"] += 1
        latest_match = LastMatch(imu, timestamp)
        line_delay.append( latest_match.get_line_delay() )

    elif N_imu_time == N_timestamp_time:
        matching_results["prefect_match"] += 1
        latest_match = LastMatch(imu, timestamp)
        line_delay.append( latest_match.get_line_delay() )
        continue
    elif N_imu_time < 0 and N_timestamp_time == 0:
        print("Throw away timestamp completly random shut down behaviour by AP20 when reseting")
        data['imu_debug'].appendleft(imu)

    elif N_imu_time < 0 and N_timestamp_time > 10:
        print(f"We assume it is now working again and we are synced {imu[2]}  == 1.005 ???" )
        latest_match = LastMatch(imu, timestamp)
        line_delay.append( latest_match.get_line_delay() )
    else:
        matching_results["failed"] += 1
        print("Mismatch in sequence numbers", N_imu_messages , " ",N_imu_count , " ", N_imu_time , " ", N_imu_arrivial_ros_time, " ", N_timestamp_time)

print("LINE DELAY", np.array( line_delay).max(), np.array( line_delay).min(), np.array( line_delay).mean())


