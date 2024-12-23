from pathlib import Path
from box_auto.utils import get_bag, ARTIFACT_FOLDER, RosbagMessageGenerator

# from cv_bridge import CvBridge
from sortedcontainers import SortedDict

# Constants
ROS_BAG_PATHS = [
    get_bag("*_jetson_hdr_front.bag"),
    get_bag("*_jetson_hdr_right.bag"),
    get_bag("*_jetson_hdr_left.bag"),
    get_bag("*_nuc_livox.bag"),
]  # List of bag files
OUTPUT_VIDEO_PATH = Path(ARTIFACT_FOLDER) / "rerun" / "output_video.mp4"
FRAME_SAVE_DIR = Path(ARTIFACT_FOLDER) / "rerun" / "frames"

FRAME_SAVE_DIR.mkdir(parents=True, exist_ok=True)


# class VideoGenerator:
#     def __init__(self, bag_paths):
#         self.generator = RosbagMessageGenerator(bag_paths)

#     def _add_to_queue(self, topic, msg, t, bag_path):
#         """Helper method to add messages to appropriate queues"""
#         timestamp = t.to_sec()
#         if "hdr_front" in topic:
#             self.hdr_front_queue[timestamp] = (topic, msg, t, bag_path)
#         elif "hdr_left" in topic:
#             self.hdr_left_queue[timestamp] = (topic, msg, t, bag_path)
#         elif "hdr_right" in topic:
#             self.hdr_right_queue[timestamp] = (topic, msg, t, bag_path)
#         elif "livox" in topic:  # Assuming this is the lidar topic
#             self.lidar_queue[timestamp] = (topic, msg, t, bag_path)

#     def _find_closest_message(self, target_time, queue, max_diff=0.1):
#         """Find closest message in queue within max_diff seconds"""
#         if not queue:
#             return None

#         closest_time = min(queue.keys(), key=lambda x: abs(x - target_time))
#         if abs(closest_time - target_time) <= max_diff:
#             return queue.pop(closest_time)
#         return None

#     def process(self):
#         for topic, message, timestamp in self.generator:
#             self._add_to_queue(topic, message, timestamp, bag_path)
#             if topic == "/hdr_front/image_raw/compressed":
#                 self._find_closest_message(timestamp, self.hdr_front_queue)


# Example usage:
if __name__ == "__main__":

    # Create sorted message queues for each topic type
    hdr_front_queue = SortedDict()
    hdr_left_queue = SortedDict()
    hdr_right_queue = SortedDict()
    lidar_queue = SortedDict()

    generator = RosbagMessageGenerator(ROS_BAG_PATHS)
    for topic, message, timestamp in generator:
        pass
        # print(f"Topic: {topic}, Timestamp: {timestamp}, Message:")
