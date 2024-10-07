import rosbag
import numpy as np
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PointStamped
from ap20_driver_ros.msg import ImuDebug, PositionDebug, TimestampDebug
import rosbag
from collections import deque
import rospy
from pathlib import Path
import os
from copy import deepcopy

MISSION_DATA = os.environ.get("MISSION_DATA", "/mission_data")

def get_bag(directory, pattern):
    return [str(s) for s in Path(directory).rglob(pattern)][0]


def read_bag_file(bag_path):
    counter = 0
    counter_ts = 0
    data_imu = []
    data_position = []
    with rosbag.Bag(bag_path.replace("jetson_ap20_aux", "jetson_ap20_ros_arrivial_time"), 'w', compression='lz4') as bag_out:
        with rosbag.Bag(bag_path, 'r') as bag:
            for topic, msg, t in bag.read_messages(topics=['/gt_box/ap20/imu_debug', '/gt_box/ap20/position_debug']):
                if topic == '/gt_box/ap20/imu_debug':
                    data_imu.append(msg)
                    out = deepcopy(msg.imu)
                    out.header.stamp =  msg.header.stamp
                    out.header.seq = counter
                    out.header.frame_id = "ap20_imu"
                    bag_out.write('/gt_box/ap20/imu_ros_arrival_timestamp', out, out.header.stamp)
                    counter += 1

                if topic == '/gt_box/ap20/position_debug':
                    data_position.append(msg)

                if len(data_position) > 0:
                    position = data_position[0]
                    t = np.array(position.position.header.stamp.to_sec(), dtype=np.float64)
                    ref = -1
                    for i, imu in enumerate( data_imu) :
                        if imu.imu.header.stamp.to_sec() > t:
                            ref = i
                            delta = imu.imu.header.stamp.to_sec() - t
                            break
                    
                    if ref > 0 and ref != -1:
                        p1 = data_imu[ref-1]
                        p2 = data_imu[ref]
                        if not ( delta > 0.005 + 1e-5) :
                            suc = True

                            def ti(p):
                                return np.array( p.imu.header.stamp.to_sec(), dtype=np.float64)
                            def th(p):
                                return np.array( p.header.stamp.to_sec(), dtype=np.float64)

                            rate =  (t - ti(p1)) / ( ti(p2) - ti(p1))
                            new_ts = th( p1)  + ((th( p2) - th( p1)) * rate)
                            
                            print("{:<20} {:<15} {:<20} {:<20}".format("IMU t-1  --- AP20_TIME:", ti(p1), ", ARRIVAL_TIME:", th(p1)))
                            print("{:<20} {:<15} {:<20} {:<20}".format("IMU t    --- AP20_TIME:", ti(p2), ", ARRIVAL_TIME:", th(p2) ))
                            print("{:<20} {:<15} {:<20} {:<20} {:<20} {:<20} ".format("Position --- AP20_TIME:", t, ", ARRIVAL_TIME:", th( position ), "NEW INTERPOL TIME:", new_ts))

                            print("")
                            new_msg = PointStamped()
                            new_msg.header.stamp = rospy.Time.from_sec(new_ts)
                            new_msg.header.seq = counter_ts
                            new_msg.header.frame_id = "leica_total_station"
                            new_msg.point.x = position.position.point.x
                            new_msg.point.y = position.position.point.y
                            new_msg.point.z = position.position.point.z                            
                            counter_ts += 1
                            bag_out.write('/gt_box/ap20/prism_position_ros_arrival_timestamp', new_msg, new_msg.header.stamp)
                            data_imu = data_imu[i-1:]
                            data_position = data_position[1:]
                        else:
                            print("delta to high - removing the timestamp")
                            data_position = data_position[1:]
                    else:
                        if ref == -1:
                            print("Imu message not yet available?")
                        else:
                            print("Removing timestamp ?")
                            data_position = data_position[1:]
                            

ap20_bag = get_bag(MISSION_DATA, "*_jetson_ap20_aux.bag")

# Usage example
data = read_bag_file(ap20_bag)
