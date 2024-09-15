from pathlib import Path
import numpy as np
from rosbag import Bag
from pathlib import Path
from scipy.interpolate import interp1d
import numpy as np
import matplotlib.pyplot as plt

def get_bag(directory, pattern):
    return [str(s) for s in Path(directory).rglob(pattern)][0]


def extract_imu_data(bag_file, topic, start_time, end_time):
    imu_data = []  # Stores acceleration and rotational velocity data
    with Bag(bag_file, 'r') as bag:
        for topic_name, msg, t in bag.read_messages(topics=[topic]):
            ts = t.to_sec()
            if ts < start_time:
                continue

            if ts > end_time:
                break

            # Extract accelerometer (linear acceleration) and gyroscope (angular velocity)
            acc = [msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z]
            rot_vel = [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z]

            imu_data.append([ts] + acc + rot_vel)

    return np.array( imu_data )


def interpolate_data(new_timestamps, data):
    # Assuming transformations are represented as (timestamp, translation, rotation)
    # Interpolate translations
    new_data = interp1d(data[:,0], data[:,1:], axis=0, kind="linear", fill_value="extrapolate")(
        new_timestamps.copy()
    )
    return new_data


total_time_in_seconds = 20
offset_in_seconds = 20
imu_topics = {
    "/gt_box/zed2i/zed_node/imu/data" : "*_jetson_zed2i_proprioceptive.bag",
    "/gt_box/ap20/imu": "*_jetson_ap20.bag" ,
    "/gt_box/adis16475_node/imu": "*_jetson_adis.bag",
    "/gt_box/livox/imu": "*_nuc_livox.bag",
    "/gt_box/alphasense_driver_node/imu": "*_nuc_alphasense.bag",
    "/gt_box/cpt7/imu/data_raw": "*_nuc_cpt7.bag",
}

orientation = {
    "/gt_box/zed2i/zed_node/imu/data" : [3, 1],
    "/gt_box/ap20/imu":  [4, -1],
    "/gt_box/adis16475_node/imu": [5, -1],
    "/gt_box/livox/imu":  [5, 1],
    "/gt_box/alphasense_driver_node/imu": [5, 1],
    "/gt_box/cpt7/imu/data_raw": [5, 1],
}

# Example usage
directory = "/mission_data"  # Path where ROS bags are stored

imu_data = {}
start_time, end_time = None, None

for topic, bag_suffix in imu_topics.items():
    bag_file = get_bag(directory, bag_suffix)

    if start_time is None:
        with Bag(bag_file, 'r') as bag:
            start_time = bag.get_start_time() + offset_in_seconds
            end_time = bag.get_start_time() + offset_in_seconds + total_time_in_seconds

    print(f"Processing bag: {bag_file} for topic: {topic}")
    data = extract_imu_data(bag_file, topic, start_time, end_time)
    imu_data[topic] = data 


new_timestamps = imu_data["/gt_box/adis16475_node/imu"][:,0]

same_timestamp = {}
# Accessing data example
for topic_name, data in imu_data.items():
    print(f"Topic: {topic_name}", data.shape)
    same_timestamp[topic_name] = interpolate_data(new_timestamps, data)


import numpy as np
import matplotlib.pyplot as plt

# Start plotting
plt.figure(figsize=(10, 6))

# Loop through each key and array in the dictionary
for key, array in same_timestamp.items():
    axis = orientation[key][0]
    sign = orientation[key][1]
    time = new_timestamps[1:-1]  # Time in seconds (x-axis)
    y_values = array[1:-1, axis] * sign # Index 5 for y-axis
    plt.plot(time, y_values, label=key)

# Add labels and title
plt.xlabel('Time (seconds)')
plt.ylabel('Sensor Values (Index 5)')
plt.title('Sensor Data over Time')
plt.legend()

# Show the plot
plt.show()