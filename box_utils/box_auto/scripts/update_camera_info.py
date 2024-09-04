
import rosbag
import yaml
from sensor_msgs.msg import CameraInfo
from pathlib import Path
from tqdm import tqdm

class RosbagPostprocessor:
    def __init__(self, mission_folder, pattern, camera_info_topics, calibration_files, output_pattern):
        self.mission_folder = Path(mission_folder)
        self.pattern = pattern
        self.camera_info_topics = camera_info_topics
        self.calibration_files = calibration_files
        self.output_pattern = output_pattern
        self.camera_info_dict = {}
        self.load_camera_info_data()

    def get_rosbags(self):
        return [str(s) for s in self.mission_folder.glob("*"+self.pattern) if self.output_pattern not in str(s)]

    def get_calibration_file(self, relative_path):
        priority_folders = [
            self.mission_folder / "post_calibration",
            self.mission_folder / "runtime_calibration",
            Path("/home/catkin_ws/src/grand_tour_box/box_calibration/box_calibration/calibration")
        ]

        for folder in priority_folders:
            file_path = folder / relative_path
            if file_path.exists():
                return file_path

        raise FileNotFoundError(f"Calibration file not found: {relative_path}")

    def load_camera_info_data(self):
        for topic, cal_file in zip(self.camera_info_topics, self.calibration_files):
            cal_path = self.get_calibration_file(cal_file)
            with open(cal_path, 'r') as f:
                cal_data = yaml.safe_load(f)
                self.camera_info_dict[topic] = self.create_camera_info_msg(cal_data)

    def create_camera_info_msg(self, cal_data):
        msg = CameraInfo()
        msg.header.frame_id = cal_data['camera_name']
        msg.height = cal_data['image_height']
        msg.width = cal_data['image_width']
        msg.distortion_model = cal_data['distortion_model']
        msg.D = cal_data['distortion_coefficients']['data']
        msg.K = cal_data['camera_matrix']['data']
        msg.R = cal_data['rectification_matrix']['data']
        msg.P = cal_data['projection_matrix']['data']
        msg.binning_x = 0
        msg.binning_y = 0

        return msg

    def process_bag(self, input_bag_path, output_bag_path):
        with rosbag.Bag(input_bag_path, 'r') as inbag:
            total_messages = inbag.get_message_count()
    
        with rosbag.Bag(output_bag_path, 'w', compression='lz4') as outbag:
            with tqdm(total=total_messages, desc=f"Processing {Path(input_bag_path).name}", unit="msgs") as pbar:
                for topic, msg, t in rosbag.Bag(input_bag_path).read_messages():
                    if topic in self.camera_info_dict:
                        new_msg = self.camera_info_dict[topic]
                        new_msg.header = msg.header
                        outbag.write(topic, new_msg, t)
                    else:
                        outbag.write(topic, msg, t)
                    pbar.update(1)
    def run(self):
        bags = self.get_rosbags()
        for bag in bags:
            output_bag = bag.replace(self.pattern, self.output_pattern)
            self.process_bag(bag, output_bag)
            print(f"Processed {bag} -> {output_bag}")

if __name__ == "__main__":
    mission_folder = "/mission_data"
    output_pattern = "_nuc_alphasense_updated_intrinsics.bag"
    pattern = "_nuc_alphasense.bag"
    camera_info_topics = [f"/gt_box/alphasense_driver_node/cam{n}/color/camera_info" for n in [1,2,3,4,5]]
    calibration_files = [f"alphasense/cam{n}.yaml" for n in [1,2,3,4,5] ]

    processor = RosbagPostprocessor(mission_folder, pattern, camera_info_topics, calibration_files, output_pattern)
    processor.run()

    # Add futher configs for HDR camera