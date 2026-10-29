import rosbag
from tqdm import tqdm
from pathlib import Path
import numpy as np
from box_auto.utils import get_bag, upload_bag
from box_auto.utils.atomic_file import atomic_output

"""
Exit Codes:

EXIT CODE 0: Successful Conversion
EXIT CODE 1: Empty HDR Bag / Missing required topics
EXIT CODE 2: Timestamps could not be matched correctly
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
    def __init__(self, cameras, output_suffix, delete_original=False):
        self.cameras = cameras
        self.output_suffix = output_suffix  # e.g. "_encoding"
        self.delete_original = delete_original

    @staticmethod
    def _safe_delete_file(path_str: str):
        try:
            p = Path(path_str)
            if p.exists():
                p.unlink()
                print(f"Deleted original bag: {p}")
            else:
                print(f"[WARNING] Original bag not found for deletion: {p}")
        except Exception as e:
            print(f"[WARNING] Failed to delete original bag '{path_str}': {e}")

    def _write_bag_with_encoding(
        self,
        input_bag_path,
        output_bag_path,
        camera,
        v4l2_to_kernel=None,
        apply_timestamp_update=False,
    ):
        """
        - Sets CompressedImage format to 'jpg' for the target topic
        - Uses msg.header.stamp as the rosbag timestamp (fallback to original t if missing or zero)
        - Optionally updates image header stamps using v4l2->kernel pairing + OFFSET_NSEC
        """
        image_topic = f"/gt_box/{camera}/image_raw/compressed"
        if Path(input_bag_path).resolve() == Path(output_bag_path).resolve():
            raise ValueError("HDR output must differ from the input bag")

        with atomic_output(output_bag_path) as temporary, rosbag.Bag(input_bag_path, "r") as inbag, rosbag.Bag(
            str(temporary), "w", compression="lz4"
        ) as outbag:
            total_messages = inbag.get_message_count()
            if inbag.get_message_count(topic_filters=[image_topic]) == 0:
                raise ValueError(f"No images on expected topic {image_topic} in {input_bag_path}")
            with tqdm(
                total=total_messages,
                desc=f"[2/2] Processing {Path(input_bag_path).name}",
                unit="msgs",
            ) as pbar:
                for topic, msg, t in inbag.read_messages():
                    if topic == image_topic:
                        # Update timestamp (header.stamp) if requested and mapping exists.
                        if apply_timestamp_update and v4l2_to_kernel is not None:
                            key = (msg.header.stamp.secs, msg.header.stamp.nsecs)
                            if key in v4l2_to_kernel:
                                k_secs, k_nsecs = v4l2_to_kernel[key]
                                updated_stamp_ns = k_secs * 1_000_000_000 + k_nsecs + OFFSET_NSEC
                                msg.header.stamp.secs = updated_stamp_ns // 1_000_000_000
                                msg.header.stamp.nsecs = updated_stamp_ns % 1_000_000_000
                            else:
                                # If an image has no match, treat this as a hard failure (consistent with strictness).
                                raise ValueError(
                                    f"Could not find matching kernel timestamp for image stamp "
                                    f"secs={msg.header.stamp.secs}, nsecs={msg.header.stamp.nsecs} in {Path(input_bag_path).name}."
                                )

                        # Force encoding label
                        msg.format = "jpg"

                    # TimeReference messages may have an unset (zero) header stamp.
                    # Retain their recording time instead of moving them to epoch zero.
                    stamp = msg.header.stamp if hasattr(msg, "header") else t
                    outbag.write(topic, msg, stamp if stamp.to_nsec() != 0 else t)
                    pbar.update(1)

    def validate_and_process_bag(self, input_bag_path, output_bag_path, camera):
        kernel_timestamps = []
        v4l2_timestamps = []
        image_stamps = []

        with rosbag.Bag(input_bag_path, "r") as inbag:
            total_messages = inbag.get_message_count()
            print("number of messages: ", total_messages)
            if total_messages == 0:
                print(f"\033[91m[ERROR] Skipping empty bag: {Path(input_bag_path).name}\033[0m")
                return False

            # Read all relevant data first
            with tqdm(
                total=total_messages,
                desc=f"[1/2] Validating {Path(input_bag_path).name}",
                unit="msgs",
            ) as pbar:
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
                f"\033[91m[ERROR] Could not find required topics: "
                f"/gt_box/{camera}/kernel_timestamp, /gt_box/{camera}/v4l2_timestamp or "
                f"/gt_box/{camera}/image_raw/compressed in {Path(input_bag_path).name}.\033[0m"
            )
            return False

        # Make sure that each image has a corresponding v4l2 timestamp, allowing for the first/last image or v4l2 ts to be missing
        if len(image_stamps) != len(v4l2_timestamps):
            print("Number of image messages and v4l2 timestamps do not match, running further validation.")
            img_idx = 0
            v4l2_idx = 0
            while img_idx < len(image_stamps) and v4l2_idx < len(v4l2_timestamps):
                img_ts = image_stamps[img_idx][1] * 1_000_000_000 + image_stamps[img_idx][2]
                v4l2_ts = v4l2_timestamps[v4l2_idx][1] * 1_000_000_000 + v4l2_timestamps[v4l2_idx][2]
                if img_ts == v4l2_ts:
                    break
                elif img_ts < v4l2_ts:
                    img_idx += 1
                else:
                    v4l2_idx += 1
            # Proceed one to one after the first matching pair
            while img_idx < len(image_stamps) and v4l2_idx < len(v4l2_timestamps):
                img_ts = image_stamps[img_idx][1] * 1_000_000_000 + image_stamps[img_idx][2]
                v4l2_ts = v4l2_timestamps[v4l2_idx][1] * 1_000_000_000 + v4l2_timestamps[v4l2_idx][2]
                if img_ts != v4l2_ts:
                    print(
                        f"Missing image message or v4l2 timestamp for image seq={image_stamps[img_idx][0]}, "
                        f"secs={image_stamps[img_idx][1]}, nsecs={image_stamps[img_idx][2]}. "
                        f"v4l2 timestamp seq={v4l2_timestamps[v4l2_idx][0]}, secs={v4l2_timestamps[v4l2_idx][1]}, "
                        f"nsecs={v4l2_timestamps[v4l2_idx][2]}. This should not happen as the messages are published in the same code block."
                    )
                    exit(2)
                img_idx += 1
                v4l2_idx += 1

        # Validation
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
                print(
                    f"[WARNING] No kernel timestamp found before v4l2 timestamp secs={v_secs}, nsecs={v_nsecs}, seq={v_seq}. "
                    f"Advancing to next v4l2 timestamp, dropping this v4l2 timestamp."
                )
                v4l2_index += 1
            else:
                first_pair_found = True
                prev_kernel_ts = kernel_timestamps[kernel_index - 1]
                k_seq, k_secs, k_nsecs = prev_kernel_ts

                paired_timestamps.append((k_secs, k_nsecs, v_secs, v_nsecs))

                delta_ns = (v_secs - k_secs) * 1_000_000_000 + (v_nsecs - k_nsecs)
                deltas.append(delta_ns)

                v4l2_index += 1

        if not first_pair_found:
            print(
            "Valid starting pair of kernel and v4l2 timestamps could not be found. "
            "Skipping timestamp update for this bag and continuing with encoding-only output."
            )
            bag_name = Path(input_bag_path).name
            print(f"\033[38;5;208mCurrent bag: {bag_name}\033[0m")

            return

        # Proceed one-to-one for the rest of the timestamps
        while v4l2_index < len(v4l2_timestamps) and kernel_index < len(kernel_timestamps):
            v_seq, v_secs, v_nsecs = v4l2_timestamps[v4l2_index]
            k_seq, k_secs, k_nsecs = kernel_timestamps[kernel_index]

            v4l2_ts = v_secs * 1_000_000_000 + v_nsecs
            kernel_ts = k_secs * 1_000_000_000 + k_nsecs

            # Make sure the kernel timestamp is before the v4l2 timestamp
            if kernel_ts >= v4l2_ts:
                print(
                    f"Kernel timestamp seq={k_seq}, secs={k_secs}, nsecs={k_nsecs} appears after v4l2 timestamp "
                    f"seq={v_seq}, secs={v_secs}, nsecs={v_nsecs}."
                )
                exit(2)

            # Make sure the kernel timestamp is after the previous v4l2 timestamp.
            if len(paired_timestamps) > 0:
                prev_pair = paired_timestamps[-1]
                prev_v4l2_ts = prev_pair[2] * 1_000_000_000 + prev_pair[3]
                if kernel_ts < prev_v4l2_ts:
                    print(
                        f"Kernel timestamp seq={k_seq}, secs={k_secs}, nsecs={k_nsecs} appears before the previous v4l2 timestamp "
                        f"seq=?, secs={prev_pair[2]}, nsecs={prev_pair[3]}. FPS should not be high enough to allow this."
                    )
                    exit(2)

            paired_timestamps.append((k_secs, k_nsecs, v_secs, v_nsecs))

            delta_ns = (v_secs - k_secs) * 1_000_000_000 + (v_nsecs - k_nsecs)

            # Make sure delta is close to the average delta
            if len(deltas) > 0:
                mean_delta = np.mean(deltas)
                if abs(delta_ns - mean_delta) / mean_delta > 0.25:
                    print(
                        f"Delta between kernel timestamp seq={k_seq}, secs={k_secs}, nsecs={k_nsecs} and v4l2 timestamp "
                        f"seq={v_seq}, secs={v_secs}, nsecs={v_nsecs} is too far (>25%) from the average delta "
                        f"{mean_delta/100_000} ms."
                    )
                    exit(2)

            deltas.append(delta_ns)

            if len(paired_timestamps) > 1:
                prev_kernel_pair = paired_timestamps[-2]
                prev_kernel_ts = prev_kernel_pair[0] * 1_000_000_000 + prev_kernel_pair[1]
                kernel_deltas.append(kernel_ts - prev_kernel_ts)

            kernel_index += 1
            v4l2_index += 1

        print("Validation passed.")
        mean_delta = np.mean(deltas) / 1_000_000  # ms
        std_dev_delta = np.std(deltas) / 1_000_000  # ms
        mean_kernel_delta = np.mean(kernel_deltas) / 1_000_000  # ms
        print(f"\nStats: \nMean time between kernel trigger time and v4l2_ts: {mean_delta:.3f} ms")
        print(f"Std dev delta time: {std_dev_delta:.3f} ms")
        print(f"Trigger fps: {1_000 / mean_kernel_delta:.3f}")
        print(f"Number of messages: {len(deltas)} \n")

        # Build fast lookup map: (v4l2_secs, v4l2_nsecs) -> (k_secs, k_nsecs)
        v4l2_to_kernel = {
            (v_secs, v_nsecs): (k_secs, k_nsecs)
            for (k_secs, k_nsecs, v_secs, v_nsecs) in paired_timestamps
        }

        # Single-pass output to *_encoding.bag (no intermediate *_updated.bag)
        self._write_bag_with_encoding(
            input_bag_path=input_bag_path,
            output_bag_path=output_bag_path,
            camera=camera,
            v4l2_to_kernel=v4l2_to_kernel,
            apply_timestamp_update=True,
        )
        return True

    def encode_only_bag(self, input_bag_path, output_bag_path, camera):
        # No timestamp adjustment, but still enforce msg.format="jpg" and bag time = header.stamp where present.
        self._write_bag_with_encoding(
            input_bag_path=input_bag_path,
            output_bag_path=output_bag_path,
            camera=camera,
            v4l2_to_kernel=None,
            apply_timestamp_update=False,
        )

    def run(self, skip_update=False):
        for camera_pattern in self.cameras:
            print(f"Processing camera: {camera_pattern}")
            input_bag = get_bag(camera_pattern)
            output_bag = input_bag.replace(".bag", f"{self.output_suffix}.bag")

            # Strip .bag and * from camera name for topic matching
            camera_name = camera_pattern.split(".")[0][1:]

            if skip_update:
                print(f"SKIP UPDATE: writing encoding bag without adjusting timestamps: {Path(output_bag).name}")
                self.encode_only_bag(input_bag, output_bag, camera_name)
            else:
                success = self.validate_and_process_bag(input_bag, output_bag, camera_name)
                if not success:
                    raise RuntimeError(f"HDR validation failed: {input_bag}")

            upload_bag(output_bag)

            # Delete original input bag after successful processing+upload (if enabled)
            if self.delete_original:
                self._safe_delete_file(input_bag)


def createRandomCollageFromEncodingBags(
    left_bag_path: str,
    front_bag_path: str,
    right_bag_path: str,
    output_png_path: str,
    dpi: int = 800,
):
    import random
    import os
    import numpy as np
    import cv2
    import rosbag

    left_topic = "/gt_box/hdr_left/image_raw/compressed"
    front_topic = "/gt_box/hdr_front/image_raw/compressed"
    right_topic = "/gt_box/hdr_right/image_raw/compressed"

    def bag_exists(path: str) -> bool:
        return bool(path) and os.path.exists(path)

    def collectStamps(bag_path: str, topic: str):
        stamps = set()
        if not bag_exists(bag_path):
            return stamps
        try:
            with rosbag.Bag(bag_path, "r") as bag:
                for _topic, msg, _t in bag.read_messages(topics=[topic]):
                    s = msg.header.stamp
                    stamps.add((int(s.secs), int(s.nsecs)))
        except Exception:
            # treat unreadable bag like missing
            return set()
        return stamps

    def loadImageAtStamp(bag_path: str, topic: str, key):
        if not bag_exists(bag_path):
            return None
        try:
            with rosbag.Bag(bag_path, "r") as bag:
                for _topic, msg, _t in bag.read_messages(topics=[topic]):
                    s = msg.header.stamp
                    if (int(s.secs), int(s.nsecs)) == key:
                        arr = np.frombuffer(bytes(msg.data), dtype=np.uint8)
                        img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
                        return img
        except Exception:
            return None
        return None

    def overlayLabel(img, text: str):
        font = cv2.FONT_HERSHEY_SIMPLEX
        scale = max(0.6, min(img.shape[0], img.shape[1]) / 800.0)
        thickness = max(1, int(round(scale * 2)))
        pad = max(4, int(round(scale * 8)))
        (tw, th), baseline = cv2.getTextSize(text, font, scale, thickness)
        x, y = pad, pad + th
        cv2.rectangle(img, (0, 0), (tw + 2 * pad, th + 2 * pad), (0, 0, 0), -1)
        cv2.putText(img, text, (x, y), font, scale, (255, 255, 255), thickness, cv2.LINE_AA)

    def redXPlaceholder(H: int, W: int):
        # white background + thick red X
        img = np.full((H, W, 3), 255, dtype=np.uint8)
        t = max(10, int(round(min(H, W) * 0.06)))
        margin = max(10, int(round(min(H, W) * 0.08)))
        p1 = (margin, margin)
        p2 = (W - margin - 1, H - margin - 1)
        p3 = (W - margin - 1, margin)
        p4 = (margin, H - margin - 1)
        cv2.line(img, p1, p2, (0, 0, 255), t, cv2.LINE_AA)
        cv2.line(img, p3, p4, (0, 0, 255), t, cv2.LINE_AA)
        return img

    # Collect stamps for whichever bags exist
    left_stamps = collectStamps(left_bag_path, left_topic)
    front_stamps = collectStamps(front_bag_path, front_topic)
    right_stamps = collectStamps(right_bag_path, right_topic)

    # Choose a timestamp:
    # - Prefer an actual common stamp among available bags (so real images line up)
    # - If only one bag exists (or no overlap), fall back to any stamp from any available bag
    available_stamp_sets = [s for s in [left_stamps, front_stamps, right_stamps] if len(s) > 0]

    selected_key = None
    if len(available_stamp_sets) >= 2:
        common = set.intersection(*available_stamp_sets)
        if common:
            # reservoir-sample from the (potentially large) common set without materializing a list
            # (iteration order of a set is fine for uniform sampling via reservoir)
            n = 0
            for k in common:
                n += 1
                if random.randint(1, n) == 1:
                    selected_key = k

    if selected_key is None and len(available_stamp_sets) >= 1:
        # fallback: sample from union of available stamps
        union = set.union(*available_stamp_sets)
        n = 0
        for k in union:
            n += 1
            if random.randint(1, n) == 1:
                selected_key = k

    if selected_key is None:
        # no bags (or none readable / no messages)
        # pick an arbitrary placeholder size
        H, W = 720, 1280
        img_left = redXPlaceholder(H, W)
        img_front = redXPlaceholder(H, W)
        img_right = redXPlaceholder(H, W)
        overlayLabel(img_left, "hdr_left (missing)")
        overlayLabel(img_front, "hdr_front (missing)")
        overlayLabel(img_right, "hdr_right (missing)")
    else:
        img_left = loadImageAtStamp(left_bag_path, left_topic, selected_key)
        img_front = loadImageAtStamp(front_bag_path, front_topic, selected_key)
        img_right = loadImageAtStamp(right_bag_path, right_topic, selected_key)

        # Determine target tile size from whichever images we actually have
        imgs = [im for im in [img_left, img_front, img_right] if im is not None and im.size > 0]
        if not imgs:
            H, W = 720, 1280
        else:
            H = int(min(im.shape[0] for im in imgs))
            W = int(min(im.shape[1] for im in imgs))
            if H <= 0 or W <= 0:
                H, W = 720, 1280

        # Replace missing ones with red X placeholders
        if img_left is None:
            img_left = redXPlaceholder(H, W)
            overlayLabel(img_left, "hdr_left (missing)")
        else:
            overlayLabel(img_left, "hdr_left")

        if img_front is None:
            img_front = redXPlaceholder(H, W)
            overlayLabel(img_front, "hdr_front (missing)")
        else:
            overlayLabel(img_front, "hdr_front")

        if img_right is None:
            img_right = redXPlaceholder(H, W)
            overlayLabel(img_right, "hdr_right (missing)")
        else:
            overlayLabel(img_right, "hdr_right")

        # Resize all to common size
        if img_left.shape[:2] != (H, W):
            img_left = cv2.resize(img_left, (W, H), interpolation=cv2.INTER_AREA)
        if img_front.shape[:2] != (H, W):
            img_front = cv2.resize(img_front, (W, H), interpolation=cv2.INTER_AREA)
        if img_right.shape[:2] != (H, W):
            img_right = cv2.resize(img_right, (W, H), interpolation=cv2.INTER_AREA)

    # Build collage
    collage = np.zeros((H, 3 * W, 3), dtype=np.uint8)
    collage[:, 0:W] = img_left
    collage[:, W:2 * W] = img_front
    collage[:, 2 * W:3 * W] = img_right

    # Save with DPI metadata
    try:
        from PIL import Image
    except Exception as e:
        raise RuntimeError(f"PIL is required to save PNG with DPI metadata: {e}")

    rgb = cv2.cvtColor(collage, cv2.COLOR_BGR2RGB)
    im = Image.fromarray(rgb)
    im.save(output_png_path, format="PNG", dpi=(int(dpi), int(dpi)))

if __name__ == "__main__":
    cameras = ["*hdr_front.bag", "*hdr_left.bag", "*hdr_right.bag"]
    output_suffix = "_encoding"

    import argparse

    parser = argparse.ArgumentParser(description="Perform verification and write encoding bags.")
    parser.add_argument(
        "--skip_update",
        action="store_true",
        help="If set, skip the validation and timestamp adjustment; still writes encoding bags.",
    )

    parser.add_argument(
        "--delete-original", action="store_true",
        help="Delete source bags after complete processing and the upload call (sources are retained by default).",
    )

    args = parser.parse_args()

    processor = RosbagValidatorAndProcessor(
        cameras=cameras,
        output_suffix=output_suffix,
        delete_original=args.delete_original,
    )
    processor.run(skip_update=args.skip_update)

    try:
        left_enc = get_bag(f"*hdr_left{output_suffix}.bag", return_upon_no_files=True)
        front_enc = get_bag(f"*hdr_front{output_suffix}.bag", return_upon_no_files=True)
        right_enc = get_bag(f"*hdr_right{output_suffix}.bag", return_upon_no_files=True)

        missing = []
        if not left_enc:
            missing.append("left_enc")
        if not front_enc:
            missing.append("front_enc")
        if not right_enc:
            missing.append("right_enc")

        if missing:
            print(f"\033[38;5;208m[WARNING] Missing encoding bag(s): {', '.join(missing)}\033[0m")

        base_bag = front_enc or left_enc or right_enc
        if not base_bag:
            print("\033[38;5;208m[WARNING] No encoding bags found; skipping HDR collage generation.\033[0m")
        else:
            collage_path = str(Path(base_bag).with_suffix(""))
            collage_path = (
                collage_path.replace("hdr_left", "hdr")
                .replace("hdr_right", "hdr")
                .replace("hdr_front", "hdr")
                + "_random_collage.png"
            )
            createRandomCollageFromEncodingBags(left_enc, front_enc, right_enc, collage_path, dpi=800)
    except Exception as e:
        print(f"\033[38;5;208m[WARNING] Failed to generate HDR collage: {e}\033[0m")

    exit(0)
