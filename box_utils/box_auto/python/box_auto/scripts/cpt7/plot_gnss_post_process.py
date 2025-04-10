import rosbag
import numpy as np
import matplotlib.pyplot as plt
from tf.transformations import euler_from_quaternion
from box_auto.utils import get_bag, ARTIFACT_FOLDER
from pathlib import Path


if __name__ == "__main__":
    # Path to the rosbag file

    rosbag_path = get_bag("*cpt7_ie_tc.bag")
    out = Path(ARTIFACT_FOLDER) / "plot_gnss"
    out.mkdir(exist_ok=True, parents=True)
    print(f"Plotting GNSS Data to : {out}")

    # Initialize lists to store data
    timestamps = []
    positions = {"x": [], "y": [], "z": []}
    position_covariances = []
    orientations = {"roll": [], "pitch": [], "yaw": []}
    orientation_covariances = []

    desired_type = "nav_msgs/Odometry"

    with rosbag.Bag(rosbag_path, "r") as bag:
        topic_info = bag.get_type_and_topic_info().topics

        # Find all topics with the desired message type
        matching_topics = [topic for topic, info in topic_info.items() if info.msg_type == desired_type]

        print(f"Topics with type {desired_type}:")
        for topic in matching_topics:
            print(f"  {topic}")

    # Read the rosbag
    with rosbag.Bag(rosbag_path, "r") as bag:
        for topic, msg, t in bag.read_messages(topics=matching_topics):
            # Extract timestamp
            timestamps.append(t.to_sec())

            # Position
            pos = msg.pose.pose.position
            positions["x"].append(pos.x)
            positions["y"].append(pos.y)
            positions["z"].append(pos.z)

            # Covariance matrix
            cov = np.array(msg.pose.covariance).reshape(6, 6)
            position_covariances.append([cov[0, 0], cov[1, 1], cov[2, 2]])

            # Orientation
            quat = msg.pose.pose.orientation
            roll, pitch, yaw = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
            orientations["roll"].append(roll)
            orientations["pitch"].append(pitch)
            orientations["yaw"].append(yaw)

            # Orientation covariance
            orientation_covariances.append([cov[3, 3], cov[4, 4], cov[5, 5]])

    # PLOT CONVERTED DATA (Output of export_gps_gt_trajectory_bag.py)

    # Convert data to numpy arrays
    timestamps = np.array(timestamps)
    positions = {key: np.array(values) for key, values in positions.items()}
    position_covariances = np.array(position_covariances)
    orientations = {key: np.array(values) for key, values in orientations.items()}
    orientation_covariances = np.array(orientation_covariances)

    # Create the plots
    fig, axs = plt.subplots(4, 1, figsize=(10, 16))

    # Plot 1: x, y, z positions
    for key, color in zip(["x", "y", "z"], ["r", "g", "b"]):
        axs[0].plot(timestamps, positions[key], label=f"{key} position", color=color)
    axs[0].set_title("Position ENU (x, y, z)")
    axs[0].set_xlabel("Time (s)")
    axs[0].set_ylabel("Position (m)")
    axs[0].legend()

    # Plot 2: Covariance for x, y, z positions
    for i, (key, color) in enumerate(zip(["x", "y", "z"], ["r", "g", "b"])):
        axs[1].plot(timestamps, position_covariances[:, i], label=f"{key} covariance", color=color)
    axs[1].set_title("Position Covariance ENU (x, y, z)")
    axs[1].set_xlabel("Time (s)")
    axs[1].set_ylabel("Pos Covariance (m2)")
    axs[1].legend()

    # Plot 3: Roll, pitch, yaw orientations
    for key, color in zip(["roll", "pitch", "yaw"], ["r", "g", "b"]):
        axs[2].plot(timestamps, orientations[key], label=f"{key} angle", color=color)
    axs[2].set_title("Orientation ENU (Roll, Pitch, Yaw)")
    axs[2].set_xlabel("Time (s)")
    axs[2].set_ylabel("Angle (rad)")
    axs[2].legend()

    # Plot 4: Covariance for roll, pitch, yaw orientations
    for i, (key, color) in enumerate(zip(["roll", "pitch", "yaw"], ["r", "g", "b"])):
        axs[3].plot(timestamps, orientation_covariances[:, i], label=f"{key} covariance", color=color)
    axs[3].set_title("Orientation Covariance (Roll, Pitch, Yaw)")
    axs[3].set_xlabel("Time (s)")
    axs[3].set_ylabel("Covariance (rad2)")
    axs[3].legend()

    plt.tight_layout()
    plt.savefig(str(out / "converted_data_ie.png"))

    # PLOT RAW DATA

    # Path to the rosbag file
rosbag_path = get_bag(pattern="*_cpt7_ie_tc.bag", auto_download=False, rglob=False)

topic_name = "/gt_box/inertial_explorer/tc/raw"

# Initialize storage
timestamps = {"position": [], "position_std": [], "orientation": [], "orientation_std": []}
data = {
    "position": {"x": [], "y": [], "z": []},
    "position_std": {"x": [], "y": [], "z": []},
    "orientation": {"roll": [], "pitch": [], "yaw": []},
    "orientation_std": {"roll": [], "pitch": [], "yaw": []},
}

# Read rosbag
with rosbag.Bag(rosbag_path, "r") as bag:
    for _, msg, t in bag.read_messages(topics=[topic_name]):
        # Timestamps
        ts = t.to_sec()
        timestamps["position"].append(ts)
        timestamps["position_std"].append(ts)
        timestamps["orientation"].append(ts)
        timestamps["orientation_std"].append(ts)

        # Position (ECEF)
        data["position"]["x"].append(msg.position_ecef.x)
        data["position"]["y"].append(msg.position_ecef.y)
        data["position"]["z"].append(msg.position_ecef.z)

        # Position Std Dev
        data["position_std"]["x"].append(msg.position_ecef_std.x)
        data["position_std"]["y"].append(msg.position_ecef_std.y)
        data["position_std"]["z"].append(msg.position_ecef_std.z)

        # Orientation (HRP)
        data["orientation"]["roll"].append(msg.orientation_hrp.x)
        data["orientation"]["pitch"].append(msg.orientation_hrp.y)
        data["orientation"]["yaw"].append(msg.orientation_hrp.z)

        # Orientation Std Dev
        data["orientation_std"]["roll"].append(msg.orientation_hrp_std.x)
        data["orientation_std"]["pitch"].append(msg.orientation_hrp_std.y)
        data["orientation_std"]["yaw"].append(msg.orientation_hrp_std.z)

    # Convert lists to numpy arrays
    for key in timestamps:
        timestamps[key] = np.array(timestamps[key])
    for category in data:
        for key in data[category]:
            data[category][key] = np.array(data[category][key])

    # Create the plots
    fig, axs = plt.subplots(4, 1, figsize=(10, 16))

    for key in ["x", "y", "z"]:
        data["position"][key] -= data["position"][key][0]

    # Plot 1: Position (ECEF x, y, z)
    for key, color in zip(["x", "y", "z"], ["r", "g", "b"]):
        axs[0].plot(timestamps["position"], data["position"][key], label=f"{key} position", color=color)
    axs[0].set_title("Position (ECEF x, y, z)")
    axs[0].set_xlabel("Time (s)")
    axs[0].set_ylabel("Position (m)")
    axs[0].legend()

    # Plot 2: Position standard deviations (ECEF x, y, z)
    for key, color in zip(["x", "y", "z"], ["r", "g", "b"]):
        axs[1].plot(timestamps["position_std"], data["position_std"][key], label=f"{key} std deviation", color=color)
    axs[1].set_title("Position Standard Deviations (ECEF x, y, z)")
    axs[1].set_xlabel("Time (s)")
    axs[1].set_ylabel("Standard Deviation (m)")
    axs[1].legend()

    # Plot 3: Orientation (HRP roll, pitch, yaw)
    for key, color in zip(["yaw", "roll", "pitch"], ["b", "r", "g"]):
        axs[2].plot(timestamps["orientation"], data["orientation"][key], label=f"{key} angle", color=color)
    axs[2].set_title("Orientation (HRP)")
    axs[2].set_xlabel("Time (s)")
    axs[2].set_ylabel("Angle (deg)")
    axs[2].legend()

    # Plot 4: Orientation standard deviations (HRP roll, pitch, yaw)
    for key, color in zip(["yaw", "roll", "pitch"], ["b", "r", "g"]):
        axs[3].plot(
            timestamps["orientation_std"], data["orientation_std"][key], label=f"{key} std deviation", color=color
        )
    axs[3].set_title("Orientation Standard Deviations (HRP Roll, Pitch, Yaw)")
    axs[3].set_xlabel("Time (s)")
    axs[3].set_ylabel("Standard Deviation (deg)")
    axs[3].legend()

    plt.tight_layout()
    plt.savefig(str(out / "raw_data_ie.png"))
