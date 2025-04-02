from pathlib import Path
from box_auto.utils import get_bag, ARTIFACT_FOLDER, RosbagMessageGenerator
import rosbag

# from cv_bridge import CvBridge
from sortedcontainers import SortedDict
from cv_bridge import CvBridge
import ros_numpy
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from matplotlib.colors import Normalize
import cv2
from box_auto.utils import read_sheet_data


# --- Global Matplotlib Settings ---
plt.rcParams.update(
    {
        "font.family": "sans-serif",
        "font.sans-serif": ["Roboto"],
        "font.size": 12,
        "axes.titlesize": 14,
        "axes.labelsize": 12,
        "axes.titlepad": 10,
        "figure.facecolor": "white",
        "figure.edgecolor": "white",
        "text.color": "#757575",  # Default text color for all text
        "axes.labelcolor": "#757575",  # Axes labels
        "axes.titlecolor": "#5e5e5e",  # Axes titles
        "xtick.color": "#757575",  # X tick labels
        "ytick.color": "#757575",  # Y tick labels
    }
)

# Increase default whitespace (if desired)
plt.rcParams["figure.subplot.left"] = 0.1
plt.rcParams["figure.subplot.right"] = 0.9
plt.rcParams["figure.subplot.top"] = 0.9
plt.rcParams["figure.subplot.bottom"] = 0.1

# --- Constants ---
ROS_BAG_PATHS = [
    get_bag("*_jetson_hdr_front.bag"),
    get_bag("*_jetson_hdr_right.bag"),
    get_bag("*_jetson_hdr_left.bag"),
    get_bag("*_nuc_livox.bag"),
    get_bag("*_hesai_post_processed.bag"),
    get_bag("*_lpc_state_estimator.bag"),
]  # List of bag files
ODOM_BAG_PATH = get_bag("*_lpc_state_estimator.bag")

MISSION_NAME = Path(ROS_BAG_PATHS[0]).name.replace("_jetson_hdr_front.bag", "")
OUTPUT_VIDEO_PATH = Path(ARTIFACT_FOLDER) / "youtube" / f"{MISSION_NAME}.mp4"
FRAME_SAVE_DIR = Path(ARTIFACT_FOLDER) / "youtube" / "frames"
FRAME_SAVE_DIR.mkdir(parents=True, exist_ok=True)


def find_first_movement_timestamp_and_bounds(rosbag_path, distance_threshold=1.0, video_duration_seconds=np.inf):
    started_moving_ts = -1
    min_x, max_x, min_y, max_y = float("inf"), -float("inf"), float("inf"), -float("inf")
    with rosbag.Bag(rosbag_path) as bag:
        # Get the first pose to use as reference
        for _topic, msg, t in bag.read_messages(topics=["/state_estimator/pose_in_odom"]):
            initial_x = msg.pose.pose.position.x
            initial_y = msg.pose.pose.position.y
            initial_z = msg.pose.pose.position.z
            break

        # Analyze all poses to find when robot moves beyond threshold
        for _topic, msg, t in bag.read_messages(topics=["/state_estimator/pose_in_odom"]):
            current_x = msg.pose.pose.position.x
            current_y = msg.pose.pose.position.y
            current_z = msg.pose.pose.position.z

            distance = np.sqrt(
                (current_x - initial_x) ** 2 + (current_y - initial_y) ** 2 + (current_z - initial_z) ** 2
            )

            if distance > distance_threshold and started_moving_ts == -1:
                print("Robot started moving at time: ", t.to_sec())
                started_moving_ts = t.to_sec()
                first_z = current_z

            if started_moving_ts != -1:
                if t.to_sec() - started_moving_ts > video_duration_seconds:
                    break
                if current_x < min_x:
                    min_x = current_x
                if current_x > max_x:
                    max_x = current_x
                if current_y < min_y:
                    min_y = current_y
                if current_y > max_y:
                    max_y = current_y

    return started_moving_ts, max_x, min_x, max_y, min_y, first_z


class VideoGenerator:
    def __init__(self, bag_paths):
        self.generator = RosbagMessageGenerator(bag_paths)
        # Create sorted message queues for each topic type
        self.hdr_front_queue = SortedDict()
        self.hdr_left_queue = SortedDict()
        self.hdr_right_queue = SortedDict()
        self.hesai_queue = SortedDict()
        self.livox_queue = SortedDict()
        self.pose_queue = SortedDict()
        self.ap20_queue = SortedDict()
        self.gnss_queue = SortedDict()
        self.bridge = CvBridge()

    def _add_to_queue(self, topic, msg, t):
        timestamp = t.to_sec()
        if "/gt_box/hdr_front/image_raw" in topic:
            self.hdr_front_queue[timestamp] = (topic, msg, t)
        elif "/gt_box/hdr_left/image_raw" in topic:
            self.hdr_left_queue[timestamp] = (topic, msg, t)
        elif "/gt_box/hdr_right/image_raw" in topic:
            self.hdr_right_queue[timestamp] = (topic, msg, t)
        elif "/gt_box/hesai/points" in topic:
            self.hesai_queue[timestamp] = (topic, msg, t)
        elif "/gt_box/livox/lidar" in topic:
            self.livox_queue[timestamp] = (topic, msg, t)
        elif "/state_estimator/pose_in_odom" in topic:
            self.pose_queue[timestamp] = (topic, msg, t)
        elif "/gt_box/ap20/position_debug" in topic:
            self.ap20_queue[timestamp] = (topic, msg, t)
        elif "/gt_box/inertial_explorer/tc/gt_poses_novatel" in topic:
            self.gnss_queue[timestamp] = (topic, msg, t)

    def _find_closest_message_and_convert(self, target_time, queue, max_diff=0.1):
        if not queue:
            return None

        closest_time = min(queue.keys(), key=lambda x: abs(x - target_time))
        if abs(closest_time - target_time) <= max_diff:
            for time in list(queue.keys()):
                if time < closest_time:
                    del queue[time]
            return self.convert(queue.pop(closest_time))
        return None

    def convert(self, full_msg):
        topic, msg, t = full_msg
        if "CompressedImage" in str(type(msg)):
            cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="rgb8")
            return cv_image
        elif "Image" in str(type(msg)):
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
            return cv_image
        elif "PointCloud2" in str(type(msg)):
            pc_data = ros_numpy.point_cloud2.pointcloud2_to_array(msg)
            point_cloud = np.vstack((pc_data["x"], pc_data["y"], pc_data["z"]))
            return point_cloud
        elif "PoseWithCovarianceStamped" in str(type(msg)):
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            z = msg.pose.pose.position.z
            return x, y, z
        elif "PointStamped" in str(type(msg)):
            x = msg.point.x
            y = msg.point.y
            z = msg.point.z
            return x, y, z
        else:
            raise ValueError("Conversion not implemented for msg type: ", str(type(msg)))
            return topic, msg, t

    def process(self):
        last_hdr_timestamp = -1
        secs_between_frames = 0.5
        i = 0
        FPS = 10
        H, W = 1080, 1920

        # Define the spreadsheet ID and sheet name
        SPREADSHEET_ID = "1mENfskg_jO_vJGFM5yonqPuf-wYUNmg26IPv3pOu3gg"
        # Read the data and print the list
        topic_data, MISSION_DATA = read_sheet_data(SPREADSHEET_ID)

        start_time = float(MISSION_DATA[MISSION_NAME]["mission_start_time"])
        stop_time = float(MISSION_DATA[MISSION_NAME]["mission_stop_time"])

        MAX_FRAMES = (stop_time - start_time) / secs_between_frames

        robot_start_moving_time_in_s, max_x, min_x, max_y, min_y, first_z = find_first_movement_timestamp_and_bounds(
            ODOM_BAG_PATH, distance_threshold=1.0, video_duration_seconds=MAX_FRAMES * secs_between_frames
        )

        robot_start_moving_time_in_s += 5  # Add some buffer time
        x_spread = max_x - min_x
        y_spread = max_y - min_y
        adjusted_min_x = np.floor(0 - 0.1 * x_spread)
        adjusted_max_x = np.ceil(x_spread * 1.1)
        adjusted_min_y = np.floor(0 - 0.1 * y_spread)
        adjusted_max_y = np.ceil(y_spread * 1.1)
        tick_spread_x = np.ceil(x_spread / 10)
        tick_spread_y = np.ceil(y_spread / 10)

        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        video_writer = cv2.VideoWriter(str(OUTPUT_VIDEO_PATH), fourcc, FPS, (W, H))
        cumulative_poses = []
        try:
            for topic, message, timestamp in self.generator:
                if timestamp.to_sec() < start_time:
                    print(timestamp.to_sec(), start_time)
                    continue

                self._add_to_queue(topic, message, timestamp)
                if topic == "/gt_box/hdr_front/image_raw/compressed" and len(self.hdr_front_queue) > 1:
                    k, v = self.hdr_front_queue.popitem(0)
                    target_time_in_s = v[2].to_sec()

                    if target_time_in_s - last_hdr_timestamp > secs_between_frames:
                        last_hdr_timestamp = target_time_in_s
                        print("Next frame found")
                    else:
                        continue

                    i += 1
                    if i == MAX_FRAMES:
                        print("Exit max frames")
                        break
                    print(f"{i}/{MAX_FRAMES}")

                    front_image = self.convert(v)
                    left_image = self._find_closest_message_and_convert(target_time_in_s, self.hdr_left_queue)
                    right_image = self._find_closest_message_and_convert(target_time_in_s, self.hdr_right_queue)
                    livox = self._find_closest_message_and_convert(target_time_in_s, self.livox_queue)
                    hesai = self._find_closest_message_and_convert(target_time_in_s, self.hesai_queue)
                    pose = self._find_closest_message_and_convert(target_time_in_s, self.pose_queue)
                    # ap20 = self._find_closest_message_and_convert(target_time_in_s, self.ap20_queue)
                    # gnss = self._find_closest_message_and_convert(target_time_in_s, self.gnss_queue)

                    if pose is not None:
                        x = pose[0] - min_x
                        y = pose[1] - min_y
                        z = pose[2] - first_z
                        cumulative_poses.append((x, y, z))

                    # --- Create Figure using GridSpec ---
                    fig = plt.figure(figsize=(W / 100, H / 100), dpi=100)
                    # We'll use 3 rows:
                    #   Row 0: Top images (3 columns)
                    #   Row 1: Bottom plots (3 columns) – same height for all
                    #   Row 2: A small row for the common horizontal colorbar
                    gs = gridspec.GridSpec(3, 3, height_ratios=[1, 0.8, 0.2])
                    gs.update(wspace=0.02, hspace=0.1)
                    plt.subplots_adjust(left=0.03, right=0.97, top=0.97, bottom=0.03)

                    # Top row: Image panels
                    top_left_ax = fig.add_subplot(gs[0, 0])
                    top_middle_ax = fig.add_subplot(gs[0, 1])
                    top_right_ax = fig.add_subplot(gs[0, 2])

                    for ax, title, img in zip(
                        [top_left_ax, top_middle_ax, top_right_ax],
                        ["Left Image", "Front Image", "Right Image"],
                        [left_image, front_image, right_image],
                    ):
                        ax.imshow(img)
                        ax.set_title(title, fontsize=18)
                        ax.axis("off")

                    # Bottom row (row 1): Other plots
                    S = 40
                    light_blue = "#dde9eb"  # Very light blue background
                    white = "white"  # White grid lines

                    bottom_gs = gridspec.GridSpecFromSubplotSpec(1, 3, subplot_spec=gs[1, :], wspace=0.25)
                    ax_traj = fig.add_subplot(bottom_gs[0])
                    ax_traj_pos = ax_traj.get_position()
                    ax_traj.set_position([ax_traj_pos.x0 + 0.02, ax_traj_pos.y0, ax_traj_pos.width, ax_traj_pos.height])
                    ax_livox = fig.add_subplot(bottom_gs[1])
                    ax_hesai = fig.add_subplot(bottom_gs[2])
                    ax_hesai_pos = ax_hesai.get_position()
                    ax_hesai.set_position(
                        [ax_hesai_pos.x0 - 0.02, ax_hesai_pos.y0, ax_hesai_pos.width, ax_hesai_pos.height]
                    )

                    # Bottom Left: Trajectory / Other Information
                    # ax_traj = fig.add_subplot(gs[1, 0])
                    ax_traj.set_xlim(adjusted_min_x, adjusted_max_x)
                    ax_traj.set_ylim(adjusted_min_y, adjusted_max_y)
                    ax_traj.set_xticks(np.arange(adjusted_min_x, adjusted_max_x, tick_spread_x))
                    ax_traj.set_yticks(np.arange(adjusted_min_y, adjusted_max_y, tick_spread_y))
                    ax_traj.grid(True, linestyle="-", color=white, alpha=1.0)
                    ax_traj.set_facecolor(light_blue)
                    ax_traj.set_title("Leg Odometry", fontsize=18)
                    ax_traj.set_xlabel("X (m)")
                    ax_traj.set_ylabel("Y (m)")
                    for spine in ax_traj.spines.values():
                        spine.set_visible(False)
                    if cumulative_poses:
                        # Plot the cumulative x,y coordinates with z a color for height
                        x, y, z = zip(*cumulative_poses)
                        ax_traj.scatter(x, y, c=z, cmap="plasma", s=20.0, vmin=-5, vmax=5, zorder=3)

                    # Bottom Middle: Livox Pointcloud
                    # ax_livox = fig.add_subplot(gs[1, 1])
                    if livox is not None:
                        x, y, z = livox
                        _ = ax_livox.scatter(x, y, c=z, cmap="plasma", s=0.3, vmin=-5, vmax=5, zorder=3)
                    else:
                        _ = ax_livox.scatter([], [])
                    ax_livox.set_xlim(-S, S)
                    ax_livox.set_ylim(-S, S)
                    ax_livox.set_xticks(np.arange(-S, S + 1, 10))
                    ax_livox.set_yticks(np.arange(-S, S + 1, 10))
                    ax_livox.grid(True, linestyle="-", color=white, alpha=1.0)
                    ax_livox.set_facecolor(light_blue)
                    ax_livox.set_title("Livox Pointcloud", fontsize=18)
                    ax_livox.set_xlabel("X (m)")
                    ax_livox.set_ylabel("Y (m)")
                    for spine in ax_livox.spines.values():
                        spine.set_visible(False)

                    # Bottom Right: Hesai Pointcloud
                    # ax_hesai = fig.add_subplot(gs[1, 2])
                    if hesai is not None:
                        x, y, z = hesai
                        _ = ax_hesai.scatter(x, y, c=z, cmap="plasma", s=0.3, vmin=-5, vmax=5)
                    else:
                        _ = ax_hesai.scatter([], [])
                    ax_hesai.set_xlim(-S, S)
                    ax_hesai.set_ylim(-S, S)
                    ax_hesai.set_xticks(np.arange(-S, S + 1, 10))
                    ax_hesai.set_yticks(np.arange(-S, S + 1, 10))
                    ax_hesai.grid(True, linestyle="-", color=white, alpha=1.0)
                    ax_hesai.set_facecolor(light_blue)
                    ax_hesai.set_title("Hesai Pointcloud", fontsize=18)
                    ax_hesai.set_xlabel("X (m)")
                    ax_hesai.set_ylabel("Y (m)")
                    for spine in ax_hesai.spines.values():
                        spine.set_visible(False)

                    # --- Create a common horizontal colorbar ---
                    # Use a dummy ScalarMappable with the same colormap and norm:
                    norm = Normalize(vmin=-5, vmax=5)
                    sm = plt.cm.ScalarMappable(cmap="plasma", norm=norm)
                    sm.set_array([])  # Dummy array for the colorbar

                    # Manually add an axes for the colorbar:
                    # [left, bottom, width, height] are in figure-relative coordinates.
                    # Adjust these values to control the horizontal width.
                    cax = fig.add_axes([0.2, 0.06, 0.6, 0.006])
                    cbar = fig.colorbar(sm, cax=cax, orientation="horizontal")
                    cbar.outline.set_visible(False)
                    cbar.set_label("Heights (m)", fontsize=10)

                    # Calculate elapsed time in seconds
                    elapsed_time = target_time_in_s - robot_start_moving_time_in_s

                    # Add a text annotation in the top right corner of the figure
                    fig.text(
                        0.96,
                        0.05,
                        f"{elapsed_time:.2f} s",  # Display time with two decimals
                        ha="right",
                        va="bottom",
                        fontsize=16,
                        color="#4a4a4a",
                        bbox=dict(boxstyle="round,pad=0.6", edgecolor="none", facecolor=light_blue, alpha=0.7, pad=5),
                    )
                    # --- Adjust Layout & Save Figure ---
                    plt.tight_layout(pad=3)
                    path = str(FRAME_SAVE_DIR / f"{i:06d}.png")
                    plt.savefig(path)
                    plt.close(fig)
                    frame = cv2.imread(path)
                    video_writer.write(frame)
        except Exception as e:
            print(e)
        finally:
            video_writer.release()


# Example usage:
if __name__ == "__main__":
    generator = VideoGenerator(ROS_BAG_PATHS)
    generator.process()
