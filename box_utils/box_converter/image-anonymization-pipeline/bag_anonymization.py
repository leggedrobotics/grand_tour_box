"""Common ROS bag writer for local and deployed image anonymizers.

Detection is supplied by the caller; validation, encoding and publication follow
the same rules regardless of model generation or deployment environment.
"""

import os
from pathlib import Path
import tempfile

import rosbag
import tqdm
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Int32

from image_codec import decode_image_message, _encode_raw_image_msg, _reencode_compressed_image


def process_bag(
    input_path,
    output_path,
    *,
    image_topics,
    transform,
    head=None,
    on_frame=None,
    require_all_image_topics=False,
    jpeg_quality=95,
    png_compression=3,
):
    """Transform selected images and publish a completed bag without overwriting.

    ``transform(image, index)`` returns (redacted, annotated, detection_count).
    An optional ``on_frame`` callback may display/save previews or raise to abort.
    Local callers can intentionally select a subset of cameras. Cloud publication
    requires every image topic to be selected so raw images cannot pass through.
    """
    input_path, output_path = Path(input_path), Path(output_path)
    selected = set(image_topics)
    if not selected:
        raise ValueError("At least one image topic must be selected")
    if head is not None and head <= 0:
        raise ValueError("head must be positive when provided")
    if os.path.lexists(output_path):
        raise FileExistsError(f"Output bag already exists: {output_path}")

    with rosbag.Bag(str(input_path), "r") as source:
        topics = source.get_type_and_topic_info().topics
        active = selected & set(topics)
        if not active:
            raise ValueError(f"None of the selected image topics exist in {input_path}: {sorted(selected)}")
        if selected - active:
            raise ValueError(f"Selected image topics are missing: {sorted(selected - active)}")
        image_types = {"sensor_msgs/Image", "sensor_msgs/CompressedImage"}
        invalid = {topic: topics[topic].msg_type for topic in selected if topics[topic].msg_type not in image_types}
        if invalid:
            raise ValueError(f"Selected topics are not image messages: {invalid}")
        unselected = {topic for topic, info in topics.items() if info.msg_type in image_types} - selected
        if require_all_image_topics and unselected:
            raise ValueError(f"Unselected image topics would remain unanonymized: {sorted(unselected)}")

        output_path.parent.mkdir(parents=True, exist_ok=True)
        processed = {topic: 0 for topic in selected}
        image_index = 0
        # A sibling temporary directory keeps publication on one filesystem.
        # Linking the closed bag is atomic and fails if another writer won the race.
        with tempfile.TemporaryDirectory(prefix=".anonymizing-", dir=output_path.parent) as temporary:
            staged = Path(temporary) / "output.bag"
            with rosbag.Bag(str(staged), "w", compression="lz4") as destination:
                for index, (topic, msg, stamp) in tqdm.tqdm(
                    enumerate(source.read_messages()),
                    total=source.get_message_count(),
                    desc=f"processing {input_path.name} - {len(selected)} image topics",
                ):
                    if head is not None and index >= head:
                        break
                    if topic not in selected:
                        destination.write(topic, msg, stamp)
                        continue
                    image, grayscale, compressed = decode_image_message(msg, topics[topic].msg_type)
                    redacted, annotated, count = transform(image, image_index)
                    if redacted.shape != image.shape or redacted.dtype != image.dtype:
                        raise ValueError("Anonymization changed image dimensions or numeric depth")
                    if on_frame is not None:
                        on_frame(image, redacted, annotated, topic, stamp, count, image_index)
                    if compressed:
                        updated = CompressedImage(
                            header=msg.header,
                            format=msg.format,
                            data=_reencode_compressed_image(
                                redacted,
                                grayscale,
                                msg.format,
                                original_msg=msg,
                                jpeg_quality=jpeg_quality,
                                png_compression=png_compression,
                            ),
                        )
                    else:
                        updated = _encode_raw_image_msg(redacted, msg)
                    destination.write(topic, updated, stamp)
                    destination.write(f"{topic}/n_detections", Int32(data=int(count)), stamp)
                    processed[topic] += 1
                    image_index += 1
            if not image_index:
                raise ValueError("No images were processed; output bag was not published")
            if any(count == 0 for count in processed.values()):
                raise ValueError("Not every selected topic had a processed image; output was not published")
            if head is None and any(processed[topic] != topics[topic].message_count for topic in selected):
                raise ValueError("Processed image counts differ from the source; output was not published")
            os.link(staged, output_path)
