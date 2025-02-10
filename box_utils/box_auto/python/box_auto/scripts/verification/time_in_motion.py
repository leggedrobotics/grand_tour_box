import rosbag
from box_auto.utils import get_bag
import rospy
import math

# Get the input bag file path
input_bag_path = get_bag("*_lpc_state_estimator.bag")

# Initialize variables
time_in_motion = rospy.Duration(0)  # Total time in motion
messages_in_motion = 0  # Number of messages in motion
previous_timestamp = None  # To store the previous message's timestamp
distance_walked = 0
dt = 1 / 20  # Convert time difference to seconds


# Open the bag file
with rosbag.Bag(input_bag_path, "r") as bag:
    # Iterate over messages in the /state_estimator/twist_throttle topic
    for topic, msg, t in bag.read_messages(topics=["/state_estimator/twist_throttle"]):

        # Check if the linear or angular velocities meet the conditions
        if (
            abs(msg.twist.twist.linear.x) > 0.1
            or abs(msg.twist.twist.linear.y) > 0.1
            or abs(msg.twist.twist.linear.z) > 0.1
        ) or (
            abs(msg.twist.twist.angular.x) > 0.01
            or abs(msg.twist.twist.angular.y) > 0.01
            or abs(msg.twist.twist.angular.z) > 0.01
        ):

            # If this is not the first message, compute the time difference
            if previous_timestamp is not None:
                # time_difference = t - previous_timestamp

                time_in_motion += rospy.Duration(dt)

            # Update the previous timestamp
            previous_timestamp = t

            # Increment the number of messages in motion
            messages_in_motion += 1

        linear_velocity = math.sqrt(
            msg.twist.twist.linear.x**2 + msg.twist.twist.linear.y**2 + msg.twist.twist.linear.z**2
        )
        distance_walked += linear_velocity * dt

# Convert the total time in motion to seconds
time_in_motion_sec = time_in_motion.to_sec()

# Output the results
print(f"Total time in motion: {time_in_motion_sec} seconds")
print(f"Number of messages in motion: {messages_in_motion}")
print(f"Total distance walked: {distance_walked} meters")
