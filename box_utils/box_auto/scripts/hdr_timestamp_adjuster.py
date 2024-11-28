import rosbag
from tqdm import tqdm
from pathlib import Path
import os
import numpy as np

MISSION_DATA = os.environ.get("MISSION_DATA", "/mission_data")

# Constants for ISX021, from Tier4, see GrandTour Wiki
VMAX = 1400  # Current ISX021's setting
FPS = 30  # Current ISX021's setting
H_TIME_SEC = 1 / FPS / VMAX
# Delay between trigger signal and exposure start
TRIGGER_TO_EXPOSURE_START_DELAY_LINES = 40
TRIGGER_TO_EXPOSURE_START_DELAY_NSEC = TRIGGER_TO_EXPOSURE_START_DELAY_LINES * H_TIME_SEC * 1_000_000_000
print(f"TRIGGER_TO_EXPOSURE_START_DELAY_NSEC: {TRIGGER_TO_EXPOSURE_START_DELAY_NSEC}")
# Delay between exposure start and middle row exposure
ROWS_IN_FRAME = 1280
MIDDLE_ROW_DELAY_NSEC = (ROWS_IN_FRAME / 2) * H_TIME_SEC * 1_000_000_000
print(f"MIDDLE_ROW_DELAY_NSEC: {MIDDLE_ROW_DELAY_NSEC}")
# Exposure time of middle of row
EXPOSURE_TIME_MS = 11.0
MIDDLE_ROW_EXPOSURE_NSEC = (EXPOSURE_TIME_MS / 2) * 1_000_000
print(f"MIDDLE_ROW_EXPOSURE_NSEC: {MIDDLE_ROW_EXPOSURE_NSEC}")
# Offset between trigger signal and middle row exposure
OFFSET_NSEC = MIDDLE_ROW_DELAY_NSEC + TRIGGER_TO_EXPOSURE_START_DELAY_NSEC + MIDDLE_ROW_EXPOSURE_NSEC
print(f"OFFSET_NSEC: {OFFSET_NSEC}")
OFFSET_NSEC = int(OFFSET_NSEC)


class RosbagValidatorAndProcessor:
    def __init__(self, mission_folder, cameras, output_suffix):
        self.mission_folder = Path(mission_folder)
        self.cameras = cameras
        self.output_suffix = output_suffix

    def get_rosbags(self, camera):
        return [str(s) for s in self.mission_folder.glob(f"*_{camera}.bag") if self.output_suffix not in str(s)]

    def validate_and_process_bag(self, input_bag_path, output_bag_path, camera):
        kernel_timestamps = []
        v4l2_timestamps = []
        image_stamps = []

        with rosbag.Bag(input_bag_path, "r") as inbag:
            total_messages = inbag.get_message_count()
            print("number of messages: ", total_messages)
            if total_messages == 0:
                print(f"Skipping empty bag: {Path(input_bag_path).name}")
                return

            # Read all relevant data first
            with tqdm(total=total_messages, desc=f"[1/2] Validating {Path(input_bag_path).name}", unit="msgs") as pbar:
                for topic, msg, t in inbag.read_messages():
                    if topic == f"/gt_box/{camera}/kernel_timestamp":
                        kernel_timestamps.append((msg.header.seq, msg.time_ref.secs, msg.time_ref.nsecs))
                    elif topic == f"/gt_box/{camera}/v4l2_timestamp":
                        v4l2_timestamps.append((msg.header.seq, msg.time_ref.secs, msg.time_ref.nsecs))
                    elif topic == f"/gt_box/{camera}/image_raw/compressed":
                        image_stamps.append((msg.header.seq, msg.header.stamp.secs, msg.header.stamp.nsecs))
                    pbar.update(1)

        # Validation
        errors = []
        deltas = []
        kernel_deltas = []

        # Ensure kernel_timestamp and v4l2_timestamp alternate correctly
        kernel_iter = iter(kernel_timestamps)
        v4l2_iter = iter(v4l2_timestamps)
        img_iter = iter(image_stamps)

        kernel_entry = next(kernel_iter, None)
        v4l2_entry = next(v4l2_iter, None)
        img_entry = next(img_iter, None)

        # Discard first v4l2_timestamp if it appears before the first kernel_timestamp
        if kernel_entry and v4l2_entry:
            kernel_ts = kernel_entry[1] * 1_000_000_000 + kernel_entry[2]
            v4l2_ts = v4l2_entry[1] * 1_000_000_000 + v4l2_entry[2]
            if v4l2_ts < kernel_ts:
                print(
                    f"[WARNING] First v4l2_timestamp secs={v4l2_entry[1]}, nsecs={v4l2_entry[2]} appears before first kernel_timestamp secs={kernel_entry[1]}, nsecs={kernel_entry[2]}."
                )
                v4l2_entry = next(v4l2_iter, None)
                img_entry = next(img_iter, None)

        paired_timestamps = []
        prev_kernel_ts = None

        while kernel_entry and v4l2_entry:
            k_seq, k_secs, k_nsecs = kernel_entry
            v_seq, v_secs, v_nsecs = v4l2_entry
            i_seq, i_secs, i_nsecs = img_entry

            kernel_ts = kernel_entry[1] * 1_000_000_000 + kernel_entry[2]
            v4l2_ts = v4l2_entry[1] * 1_000_000_000 + v4l2_entry[2]
            img_ts = img_entry[1] * 1_000_000_000 + img_entry[2]

            if kernel_ts >= v4l2_ts:
                errors.append(
                    f"Kernel_timestamp secs={k_secs}, nsecs={k_nsecs} appears after v4l2_timestamp secs={v_secs}, nsecs={v_nsecs}"
                )
                break

            if img_ts != v4l2_ts:
                errors.append(
                    f"image_raw/compressed secs={i_secs}, nsecs={i_nsecs} does not match v4l2_timestamp secs={v_secs}, nsecs={v_nsecs}. Is use_kernel_buffer_ts set to true in recorder.launch.py?"
                )
                break

            if not k_seq == v_seq == i_seq:
                print(
                    f"[WARNING] Kernel_timestamp seq={k_seq}, v4l2_timestamp seq={v_seq} and image_raw seq={i_seq} do not match."
                )

            delta_ns = (v_secs - k_secs) * 1_000_000_000 + (v_nsecs - k_nsecs)
            # if delta_ns < OFFSET_NSEC:
            #     errors.append(
            #         f"Time difference between trigger time and register read is less than time-delay constant {OFFSET_NSEC} ns. This should not be possible."
            #     )
            deltas.append(delta_ns)

            paired_timestamps.append((k_secs, k_nsecs, v_secs, v_nsecs))
            if prev_kernel_ts is not None:
                kernel_deltas.append(kernel_ts - prev_kernel_ts)
            prev_kernel_ts = kernel_ts

            kernel_entry = next(kernel_iter, None)
            v4l2_entry = next(v4l2_iter, None)
            img_entry = next(img_iter, None)

        if kernel_entry or v4l2_entry or img_entry:
            print("[WARNING] Unmatched kernel_timestamp, image_raw or v4l2_timestamp messages at the end of the bag.")

        # Report errors and stats
        if errors:
            print("Validation errors found:")
            for error in errors:
                print(f"- {error}")
            print("Aborting processing due to validation errors.")
            return

        print("Validation passed.")
        mean_delta = np.mean(deltas) / 1_000_000  # Convert to milliseconds
        std_dev_delta = np.std(deltas) / 1_000_000  # Convert to milliseconds
        mean_kernel_delta = np.mean(kernel_deltas) / 1_000_000  # Convert to milliseconds
        print(f"\nStats: \nMean time between kernel trigger time and v4l2_ts: {mean_delta:.3f} ms")
        print(f"Std dev delta time: {std_dev_delta:.3f} ms")
        print(f"Trigger fps: {1_000 / mean_kernel_delta:.3f}")
        print(f"Number of messages: {len(deltas)} \n")

        # Process and overwrite timestamps
        with rosbag.Bag(input_bag_path, "r") as inbag, rosbag.Bag(output_bag_path, "w", compression="lz4") as outbag:
            total_messages = inbag.get_message_count()

            with tqdm(total=total_messages, desc=f"[2/2] Processing {Path(input_bag_path).name}", unit="msgs") as pbar:
                for topic, msg, t in inbag.read_messages():
                    if topic == f"/gt_box/{camera}/image_raw/compressed":
                        for k_secs, k_nsecs, v_secs, v_nsecs in paired_timestamps:
                            if (msg.header.stamp.secs, msg.header.stamp.nsecs) == (v_secs, v_nsecs):
                                updated_stamp = k_secs * 1_000_000_000 + k_nsecs + OFFSET_NSEC
                                msg.header.stamp.secs = updated_stamp // 1_000_000_000
                                msg.header.stamp.nsecs = updated_stamp % 1_000_000_000
                                break
                        outbag.write(topic, msg, t)
                    else:
                        outbag.write(topic, msg, t)
                    pbar.update(1)

    def run(self):
        for camera in self.cameras:
            bags = self.get_rosbags(camera)
            for bag in bags:
                output_bag = bag.replace(f"_{camera}.bag", f"_{camera}{self.output_suffix}.bag")
                self.validate_and_process_bag(bag, output_bag, camera)
                print(f"Processed {bag} -> {output_bag} \n")

                if os.environ.get("KLEINKRAM_ACTIVE", False) == "ACTIVE":
                    uuid = os.environ["MISSION_UUID"]
                    os.system(f"klein upload --mission {uuid} {output_bag}")


if __name__ == "__main__":
    cameras = ["hdr_front", "hdr_left", "hdr_right"]

    if os.environ.get("KLEINKRAM_ACTIVE", False) == "ACTIVE":
        uuid = os.environ["MISSION_UUID"]
        bag_names = " ".join([f"*_{c}.bag" for c in cameras])
        os.system(f"klein download --mission {uuid} --dest /mission_data '{bag_names}'")

    output_suffix = "_updated"
    processor = RosbagValidatorAndProcessor(MISSION_DATA, cameras, output_suffix)
    processor.run()
