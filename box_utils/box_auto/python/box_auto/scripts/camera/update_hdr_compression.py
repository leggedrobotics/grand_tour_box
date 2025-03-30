import rosbag
from tqdm import tqdm
from pathlib import Path
from box_auto.utils import get_bag, upload_bag


def run(input_bag_path, output_bag_path, camera):
    # Process and overwrite timestamps
    with rosbag.Bag(input_bag_path, "r") as inbag, rosbag.Bag(output_bag_path, "w", compression="lz4") as outbag:
        total_messages = inbag.get_message_count()
        with tqdm(total=total_messages, desc=f"[2/2] Processing {Path(input_bag_path).name}", unit="msgs") as pbar:
            for topic, msg, t in inbag.read_messages():
                if topic == f"/gt_box/{camera}/image_raw/compressed":
                    msg.format = "jpg"

                outbag.write(topic, msg, t)
                pbar.update()


if __name__ == "__main__":
    cameras = ["*hdr_front_updated.bag", "*hdr_left_updated.bag", "*hdr_right_updated.bag"]
    for camera in cameras:
        input_bag = get_bag(camera)
        output_bag = input_bag.replace("_updated.bag", "_encoding.bag")
        print(camera[1:-4])
        run(input_bag, output_bag, (camera[1:]).replace("_updated.bag", ""))
        upload_bag(output_bag)
