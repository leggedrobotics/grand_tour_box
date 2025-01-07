import rosbag
from box_auto.utils import get_bag, upload_bag


if __name__ == "__main__":
    # Path to your rosbag file
    input_file_path = get_bag("*_nuc_alphasense.bag")
    output_file_path = input_file_path.replace("_nuc_alphasense.bag", "_nuc_alphasense_updated.bag")

    # Topics of interest
    frame_id_map = {
        "/gt_box/alphasense_driver_node/cam1/color/camera_info": "alphasense_front_center",
        "/gt_box/alphasense_driver_node/cam1/color/image/compressed": "alphasense_front_center",
        "/gt_box/alphasense_driver_node/cam3/color/camera_info": "alphasense_front_left",
        "/gt_box/alphasense_driver_node/cam3/color/image/compressed": "alphasense_front_left",
    }
    with rosbag.Bag(input_file_path, "r") as inbag:
        with rosbag.Bag(output_file_path, "w") as outbag:
            for topic, msg, t in inbag.read_messages():
                if topic in frame_id_map:
                    msg.header.frame_id = frame_id_map[topic]
                    outbag.write(topic, msg, t)
                else:
                    outbag.write(topic, msg, t)

    # Upload the processed bag file
    upload_bag(output_file_path)
