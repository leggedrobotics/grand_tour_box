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

    # Topic to read
    topic_name = "/gt_box/inertial_explorer/tc/gt_poses_novatel"

    # Initialize lists to store data
    timestamps = []
    positions = {"x": [], "y": [], "z": []}
    position_covariances = []
    orientations = {"roll": [], "pitch": [], "yaw": []}
    orientation_covariances = []

    # Read the rosbag
    with rosbag.Bag(rosbag_path, "r") as bag:
        for topic, msg, t in bag.read_messages(topics=[topic_name]):
            # Extract timestamp
            timestamps.append(t.to_sec())

            # Extract position
            positions["x"].append(msg.pose.pose.position.x)
            positions["y"].append(msg.pose.pose.position.y)
            positions["z"].append(msg.pose.pose.position.z)

            cov = np.array(msg.pose.covariance).reshape(6, 6)

            # Extract position covariance (diagonal elements)
            position_covariances.append([cov[0, 0], cov[1, 1], cov[2, 2]])

            # Extract orientation
            quat = msg.pose.pose.orientation
            roll, pitch, yaw = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
            orientations["roll"].append(roll)
            orientations["pitch"].append(pitch)
            orientations["yaw"].append(yaw)

            # Extract orientation covariance (diagonal elements for roll, pitch, yaw)
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
    rosbag_path = "/media/jonfrey/BoxiS4-2TB/deployment_day_1/2024-10-01-11-29-55/2024-10-01-11-29-55_cpt7_ie_tc.bag"

    # Topics to read
    topics = {
        "position": "/gt_box/inertial_explorer/tc/raw/position_ecef",
        "position_std": "/gt_box/inertial_explorer/tc/raw/position_ecef_std",
        "orientation": "/gt_box/inertial_explorer/tc/raw/orientation_hrp",
        "orientation_std": "/gt_box/inertial_explorer/tc/raw/orientation_hrp_std",
    }

    # Initialize lists to store data
    timestamps = {"position": [], "position_std": [], "orientation": [], "orientation_std": []}
    data = {
        "position": {"x": [], "y": [], "z": []},
        "position_std": {"x": [], "y": [], "z": []},
        "orientation": {"roll": [], "pitch": [], "yaw": []},
        "orientation_std": {"roll": [], "pitch": [], "yaw": []},
    }

    # Read the rosbag
    with rosbag.Bag(rosbag_path, "r") as bag:
        for topic, msg, t in bag.read_messages(topics=topics.values()):
            if topic == topics["position"]:
                timestamps["position"].append(t.to_sec())
                data["position"]["x"].append(msg.x)
                data["position"]["y"].append(msg.y)
                data["position"]["z"].append(msg.z)
            elif topic == topics["position_std"]:
                timestamps["position_std"].append(t.to_sec())
                data["position_std"]["x"].append(msg.x)
                data["position_std"]["y"].append(msg.y)
                data["position_std"]["z"].append(msg.z)
            elif topic == topics["orientation"]:
                timestamps["orientation"].append(t.to_sec())
                data["orientation"]["roll"].append(msg.y)
                data["orientation"]["pitch"].append(msg.z)
                data["orientation"]["yaw"].append(msg.x)
            elif topic == topics["orientation_std"]:
                timestamps["orientation_std"].append(t.to_sec())
                data["orientation_std"]["roll"].append(msg.y)
                data["orientation_std"]["pitch"].append(msg.z)
                data["orientation_std"]["yaw"].append(msg.x)

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
