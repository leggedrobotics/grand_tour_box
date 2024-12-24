from pathlib import Path
from box_auto.utils import get_bag, ARTIFACT_FOLDER, RosbagMessageGenerator
import rosbag

# from cv_bridge import CvBridge
from sortedcontainers import SortedDict
from cv_bridge import CvBridge
import ros_numpy
import numpy as np
import matplotlib.pyplot as plt
import cv2

# Constants
ROS_BAG_PATHS = [
    get_bag("*_jetson_hdr_front.bag"),
    get_bag("*_jetson_hdr_right.bag"),
    get_bag("*_jetson_hdr_left.bag"),
    get_bag("*_nuc_livox.bag"),
    get_bag("*_hesai_post_processed.bag"),
]  # List of bag files

ODOM_BAG_PATH = get_bag("*_lpc_state_estimator.bag")

MISSION_NAME = Path(ROS_BAG_PATHS[0]).parent.stem
OUTPUT_VIDEO_PATH = Path(ARTIFACT_FOLDER) / "youtube" / f"{MISSION_NAME}.mp4"
FRAME_SAVE_DIR = Path(ARTIFACT_FOLDER) / "youtube" / "frames"
FRAME_SAVE_DIR.mkdir(parents=True, exist_ok=True)


def find_first_movement_timestamp(rosbag_path, distance_threshold=1.0):
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

            # Calculate Euclidean distance from initial position
            distance = np.sqrt(
                (current_x - initial_x) ** 2 + (current_y - initial_y) ** 2 + (current_z - initial_z) ** 2
            )

            # Check if distance exceeds threshold
            if distance > distance_threshold:
                print("Robot started moving at time: ", t.to_sec())
                return t.to_sec()  # Convert to seconds

    return -1


class VideoGenerator:
    def __init__(self, bag_paths):
        self.generator = RosbagMessageGenerator(bag_paths)
        # Create sorted message queues for each topic type
        self.hdr_front_queue = SortedDict()
        self.hdr_left_queue = SortedDict()
        self.hdr_right_queue = SortedDict()
        self.hesai_queue = SortedDict()
        self.livox_queue = SortedDict()

        self.bridge = CvBridge()

    def _add_to_queue(self, topic, msg, t):
        """Helper method to add messages to appropriate queues"""
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

    def _find_closest_message_and_convert(self, target_time, queue, max_diff=0.1):
        """Find closest message in queue within max_diff seconds"""
        if not queue:
            return None

        closest_time = min(queue.keys(), key=lambda x: abs(x - target_time))
        if abs(closest_time - target_time) <= max_diff:
            for time in list(queue.keys()):
                if time < closest_time:
                    del queue[time]

            # Remove all elements smaller ten clostes time from queue
            return self.convert(queue.pop(closest_time))

        return None

    def convert(self, full_msg):
        topic, msg, t = full_msg
        if "CompressedImage" in str(type(msg)):
            # Convert CompressedImage to a CV2 image
            cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="rgb8")
            return cv_image

        elif "Image" in str(type(msg)):
            # Convert Image to a CV2 image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
            return cv_image

        elif "PointCloud2" in str(type(msg)):
            # Convert PointCloud2 to a numpy array
            pc_data = ros_numpy.point_cloud2.pointcloud2_to_array(msg)
            point_cloud = np.vstack((pc_data["x"], pc_data["y"], pc_data["z"]))
            return point_cloud

        else:
            raise ValueError("Conversion not implemented for msg type: ", str(type(msg)))
            # Return the message unchanged if no conversion is needed
            return topic, msg, t

    def process(self):
        last_hdr_timestamp = -1
        secs_between_frames = 0.5
        i = 0
        FPS = 10
        H, W = 1080, 1920
        MAX_FRAMES = 100

        # Start 5 seconds before the robot moved for 1m
        robot_start_moving_time_in_s = find_first_movement_timestamp(ODOM_BAG_PATH, distance_threshold=1.0) - 5

        # Initialize video writer
        fourcc = cv2.VideoWriter_fourcc(*"mp4v")  # or use 'XVID' for AVI format
        video_writer = cv2.VideoWriter(str(OUTPUT_VIDEO_PATH), fourcc, FPS, (W, H))
        try:
            for topic, message, timestamp in self.generator:
                if timestamp.to_sec() < robot_start_moving_time_in_s:
                    print(timestamp.to_sec(), robot_start_moving_time_in_s)
                    continue

                self._add_to_queue(topic, message, timestamp)
                if topic == "/gt_box/hdr_front/image_raw/compressed" and len(self.hdr_front_queue) > 1:

                    k, v = self.hdr_front_queue.popitem(0)
                    target_time_in_s = v[2].to_sec()

                    if target_time_in_s - last_hdr_timestamp > secs_between_frames:
                        last_hdr_timestamp = target_time_in_s
                        print("return to close")
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

                    # Adjust figure to include a third empty plot on the bottom right
                    fig = plt.figure(figsize=(W / 100, H / 100), dpi=100)

                    # Top half: Images side-by-side
                    top_left_ax = fig.add_subplot(2, 3, 1)
                    top_middle_ax = fig.add_subplot(2, 3, 2)
                    top_right_ax = fig.add_subplot(2, 3, 3)

                    for ax, title, img in zip(
                        [top_left_ax, top_middle_ax, top_right_ax],
                        ["Left Image", "Front Image", "Right Image"],
                        [left_image, front_image, right_image],
                    ):
                        ax.imshow(img)  # Replace with actual images
                        ax.set_title(title, fontsize=12)
                        ax.axis("off")

                    S = 40
                    # Bottom left: Empty plot with fixed grid and limits
                    empty_ax_left = fig.add_subplot(2, 3, 4)
                    empty_ax_left.set_xlim(-S, S)
                    empty_ax_left.set_ylim(-S, S)
                    empty_ax_left.set_xticks(np.arange(-S, 21, 5))
                    empty_ax_left.set_yticks(np.arange(-S, 21, 5))
                    empty_ax_left.grid(True, linestyle="--", alpha=0.7)
                    empty_ax_left.set_title("Trajectory / Other Information", fontsize=12)

                    if livox is not None:
                        scatter_ax = fig.add_subplot(2, 3, 5)
                        x, y, z = livox
                        scatter = scatter_ax.scatter(x, y, c=z, cmap="plasma", s=0.3, vmin=-5, vmax=5)
                        scatter_ax.set_xlim(-S, S)
                        scatter_ax.set_ylim(-S, S)
                        scatter_ax.set_xticks(np.arange(-S, S + 1, 10))
                        scatter_ax.set_yticks(np.arange(-S, S + 1, 10))
                        scatter_ax.grid(True, linestyle="--", alpha=0.7)
                        scatter_ax.set_title("Livox Pointcloud", fontsize=12)
                        cbar = plt.colorbar(scatter, ax=scatter_ax, orientation="vertical")
                        cbar.set_label("Heights", fontsize=10)

                    if hesai is not None:
                        # Bottom right: Another empty plot with fixed grid and limits
                        x, y, z = hesai
                        scatter_ax = fig.add_subplot(2, 3, 6)
                        scatter = scatter_ax.scatter(x, y, c=z, cmap="plasma", s=0.3, vmin=-5, vmax=5)
                        scatter_ax.set_xlim(-S, S)
                        scatter_ax.set_ylim(-S, S)
                        scatter_ax.set_xticks(np.arange(-S, S + 1, 10))
                        scatter_ax.set_yticks(np.arange(-S, S + 1, 10))
                        scatter_ax.grid(True, linestyle="--", alpha=0.7)
                        scatter_ax.set_title("Hesai Pointcloud", fontsize=12)
                        cbar = plt.colorbar(scatter, ax=scatter_ax, orientation="vertical")
                        cbar.set_label("Height", fontsize=10)

                    # Tight layout for better spacing
                    plt.tight_layout()

                    # Save the updated figure
                    path = str(FRAME_SAVE_DIR / f"{i:06d}.png")
                    plt.savefig(path)
                    plt.show()
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
