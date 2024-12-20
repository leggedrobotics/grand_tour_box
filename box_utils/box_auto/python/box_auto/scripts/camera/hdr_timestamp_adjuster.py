import rosbag
from tqdm import tqdm
from pathlib import Path
import os
import numpy as np

from box_auto.utils import get_bag, upload_bag

"""
Exit Codes:

EXIT CODE 0: Successful Conversion
EXIT CODE 1: Empty HDR Bag
EXIT CODE 2: Timestamps could not be matched correctly
EXIT CODE 3: Non-Blocking Validation issues found, but conversion was completed.
"""


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
    def __init__(self, cameras, output_suffix):
        self.cameras = cameras
        self.output_suffix = output_suffix

    def validate_and_process_bag(self, input_bag_path, output_bag_path, camera):
        kernel_timestamps = []
        v4l2_timestamps = []
        image_stamps = []

        with rosbag.Bag(input_bag_path, "r") as inbag:
            total_messages = inbag.get_message_count()
            print("number of messages: ", total_messages)
            if total_messages == 0:
                print(f"Skipping empty bag: {Path(input_bag_path).name}")
                exit(1)

            # Read all relevant data first
            with tqdm(total=total_messages, desc=f"[1/2] Validating {Path(input_bag_path).name}", unit="msgs") as pbar:
                for topic, msg, _t in inbag.read_messages():
                    if topic == f"/gt_box/{camera}/kernel_timestamp":
                        kernel_timestamps.append((msg.header.seq, msg.time_ref.secs, msg.time_ref.nsecs))
                    elif topic == f"/gt_box/{camera}/v4l2_timestamp":
                        v4l2_timestamps.append((msg.header.seq, msg.time_ref.secs, msg.time_ref.nsecs))
                    elif topic == f"/gt_box/{camera}/image_raw/compressed":
                        image_stamps.append((msg.header.seq, msg.header.stamp.secs, msg.header.stamp.nsecs))
                    pbar.update(1)

        print(
            f"Found {len(kernel_timestamps)} kernel timestamps, {len(v4l2_timestamps)} v4l2 timestamps, "
            f"and {len(image_stamps)} images in {Path(input_bag_path).name}."
        )
        if len(kernel_timestamps) == 0 or len(v4l2_timestamps) == 0 or len(image_stamps) == 0:
            print(
                f"""Aborting because could not find topics: /gt_box/{camera}/kernel_timestamp, 
                  /gt_box/{camera}/v4l2_timestamp or /gt_box/{camera}/image_raw/compressed in {Path(input_bag_path).name}.
                  """
            )
            exit(1)

        # Validation
        errors = []
        warnings = []
        deltas = []
        kernel_deltas = []

        # Match v4l2 timestamps to the closest preceding kernel timestamps
        paired_timestamps = []

        # Initial pairing: find the first v4l2 timestamp and its preceding kernel timestamp
        kernel_index = 0
        v4l2_index = 0

        # Find the first valid pair
        first_pair_found = False
        while not first_pair_found and v4l2_index < len(v4l2_timestamps):
            v_seq, v_secs, v_nsecs = v4l2_timestamps[v4l2_index]
            v4l2_ts = v_secs * 1_000_000_000 + v_nsecs

            while (
                kernel_index < len(kernel_timestamps)
                and kernel_timestamps[kernel_index][1] * 1_000_000_000 + kernel_timestamps[kernel_index][2] <= v4l2_ts
            ):
                kernel_index += 1

            if kernel_index == 0:
                warnings.append(
                    f"""[WARNING] No kernel timestamp found before v4l2 timestamp secs={v_secs}, nsecs={v_nsecs}, 
                     seq={v_seq}. Advancing to next v4l2 timestamp, dropping this v4l2 timestamp.
                    """
                )
                v4l2_index += 1
            else:
                first_pair_found = True
                prev_kernel_ts = kernel_timestamps[kernel_index - 1]
                k_seq, k_secs, k_nsecs = prev_kernel_ts
                kernel_ts = k_secs * 1_000_000_000 + k_nsecs

                paired_timestamps.append((k_secs, k_nsecs, v_secs, v_nsecs))

                delta_ns = (v_secs - k_secs) * 1_000_000_000 + (v_nsecs - k_nsecs)
                deltas.append(delta_ns)

                v4l2_index += 1

        if not first_pair_found:
            print("Valid starting pair could not be found. Aborting processing.")
            if warnings:
                print("Warnings found:")
                for warning in warnings:
                    print(f"- {warning}")
            exit(2)

        # Proceed one-to-one for the rest of the timestamps
        while v4l2_index < len(v4l2_timestamps) and kernel_index < len(kernel_timestamps):
            v_seq, v_secs, v_nsecs = v4l2_timestamps[v4l2_index]
            k_seq, k_secs, k_nsecs = kernel_timestamps[kernel_index]

            v4l2_ts = v_secs * 1_000_000_000 + v_nsecs
            kernel_ts = k_secs * 1_000_000_000 + k_nsecs

            # Make sure the kernel timestamp is before the v4l2 timestamp
            if kernel_ts >= v4l2_ts:
                errors.append(
                    f"""Kernel timestamp seq={k_seq}, secs={k_secs}, nsecs={k_nsecs} appears after v4l2 timestamp 
                    seq={v_seq}, secs={v_secs}, nsecs={v_nsecs}.
                    """
                )
                break

            # Make sure the kernel timestamp is after the previous v4l2 timestamp.
            if len(paired_timestamps) > 0:
                prev_pair = paired_timestamps[-1]
                prev_v4l2_ts = prev_pair[2] * 1_000_000_000 + prev_pair[3]
                if kernel_ts < prev_v4l2_ts:
                    errors.append(
                        f"""Kernel timestamp seq={k_seq}, secs={k_secs}, nsecs={k_nsecs} appears before the previous 
                        v4l2 timestamp seq={prev_pair[0]}, secs={prev_pair[2]}, nsecs={prev_pair[3]}. FPS should not
                        be high enough to allow this.
                        """
                    )
                    break

            paired_timestamps.append((k_secs, k_nsecs, v_secs, v_nsecs))

            delta_ns = (v_secs - k_secs) * 1_000_000_000 + (v_nsecs - k_nsecs)

            # Make sure delta is close to the average delta
            if len(deltas) > 0:
                mean_delta = np.mean(deltas)
                if abs(delta_ns - mean_delta) / mean_delta > 0.2:
                    warnings.append(
                        f"""
                        Delta between kernel timestamp seq={k_seq}, secs={k_secs}, nsecs={k_nsecs} and v4l2 timestamp 
                        seq={v_seq}, secs={v_secs}, nsecs={v_nsecs} is too far (>20%) from the average delta 
                        {mean_delta/100_000} ms. 
                        """
                    )

            deltas.append(delta_ns)

            if len(paired_timestamps) > 1:
                prev_kernel_pair = paired_timestamps[-2]
                prev_kernel_ts = prev_kernel_pair[0] * 1_000_000_000 + prev_kernel_pair[1]
                kernel_deltas.append(kernel_ts - prev_kernel_ts)

            kernel_index += 1
            v4l2_index += 1

        if errors:
            print("Validation errors found:")
            for error in errors:
                print(f"- {error}")
            print("Aborting processing due to validation errors.")
            exit(2)

        print(f"Validation passed with {len(warnings)} warnings.")
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
                        match_found = False
                        for k_secs, k_nsecs, v_secs, v_nsecs in paired_timestamps:
                            if (msg.header.stamp.secs, msg.header.stamp.nsecs) == (v_secs, v_nsecs):
                                updated_stamp = k_secs * 1_000_000_000 + k_nsecs + OFFSET_NSEC
                                msg.header.stamp.secs = updated_stamp // 1_000_000_000
                                msg.header.stamp.nsecs = updated_stamp % 1_000_000_000
                                match_found = True
                                break
                        if not match_found:
                            warnings.append(
                                f"""[WARNING] No matching timestamp found for image timestamp seq={msg.header.seq},
                                secs={msg.header.stamp.secs}, nsecs={msg.header.stamp.nsecs}. Dropping this image.
                                """
                            )
                        else:
                            outbag.write(topic, msg, t)
                    else:
                        outbag.write(topic, msg, t)
                    pbar.update(1)
        # TODO: Check if this is intentional
        #os.system(f"rm {input_bag_path}")

        return warnings

    def run(self):
        warnings_total = 0
        for camera in self.cameras:
            print(f"Processing camera: {camera}")
            input_bag = get_bag(camera)
            output_bag = input_bag.replace(".bag", "_updated.bag")
            # Strip .bag and * from camera name
            camera = camera.split(".")[0][1:]
            warnings = self.validate_and_process_bag(input_bag, output_bag, camera)
            warnings_total += len(warnings)
            print(f"Processed {input_bag} -> {output_bag} with {len(warnings)} warnings: \n")
            for warning in warnings:
                print(f"- {warning}")
            upload_bag(output_bag)
        return warnings_total


if __name__ == "__main__":
    cameras = ["*hdr_front.bag", "*hdr_left.bag", "*hdr_right.bag"]
    output_suffix = "_updated"
    processor = RosbagValidatorAndProcessor(cameras, output_suffix)
    num_warnings = processor.run()
    if num_warnings > 0:
        exit(3)
    exit(0)
