from box_auto.utils import WS
import rosbag
from tf_bag import BagTfTransformer
from pathlib import Path
import copy

p = Path(WS) / "src/grand_tour_box/box_calibration/box_calibration/calibration/"

for p in [str(s) for s in p.glob("*tf_static_metadata_new_*.bag")]:
    # This bags contains information about URDF
    # generated:  roscore
    #             rosbag record /tf_static
    #             roslaunch box_model box_model.launch rviz:=True
    tf_listener = BagTfTransformer("/home/jonfrey/2025-04-02-17-34-55.bag")
    with rosbag.Bag(p, "r") as bag:
        with rosbag.Bag(
            p.replace("tf_static_metadata_new_", "tf_static_metadata_final_"), "w", compression="lz4"
        ) as out_bag:
            for topic, msg, _t in bag.read_messages():
                if topic == "/tf_static":
                    delete_ids = []

                    for j, transform in enumerate(msg.transforms):
                        if "prism" in transform.child_frame_id:
                            delete_ids.append(j)
                    msg.transforms = [transform for i, transform in enumerate(msg.transforms) if i not in delete_ids]

                    tf = tf_listener.lookupTransform("box_base", "box_base_model", time=None, latest=True)
                    out = copy.deepcopy(msg.transforms[0])
                    out.header.frame_id = "box_base"
                    out.child_frame_id = "box_base_model"
                    out.transform.translation.x = tf[0][0]
                    out.transform.translation.y = tf[0][1]
                    out.transform.translation.z = tf[0][2]
                    out.transform.rotation.x = tf[1][0]
                    out.transform.rotation.y = tf[1][1]
                    out.transform.rotation.z = tf[1][2]
                    out.transform.rotation.w = tf[1][3]
                    msg.transforms.append(out)
                    print("working")

                    tf = tf_listener.lookupTransform("base", "cpt7_antenna_back_model", time=None, latest=True)
                    out = copy.deepcopy(msg.transforms[0])
                    out.header.frame_id = "base"
                    out.child_frame_id = "cpt7_antenna_back_model"
                    out.transform.translation.x = tf[0][0]
                    out.transform.translation.y = tf[0][1]
                    out.transform.translation.z = tf[0][2]
                    out.transform.rotation.x = tf[1][0]
                    out.transform.rotation.y = tf[1][1]
                    out.transform.rotation.z = tf[1][2]
                    out.transform.rotation.w = tf[1][3]
                    msg.transforms.append(out)

                    # Rename frame_id + flip tf
                    tf = tf_listener.lookupTransform("box_base", "prism_model", time=None, latest=True)
                    out = copy.deepcopy(msg.transforms[0])
                    out.header.frame_id = "box_base"
                    out.child_frame_id = "prism_model"
                    out.transform.translation.x = tf[0][0]
                    out.transform.translation.y = tf[0][1]
                    out.transform.translation.z = tf[0][2]
                    out.transform.rotation.x = tf[1][0]
                    out.transform.rotation.y = tf[1][1]
                    out.transform.rotation.z = tf[1][2]
                    out.transform.rotation.w = tf[1][3]
                    msg.transforms.append(out)

                    tf = tf_listener.lookupTransform("box_base", "prism", time=None, latest=True)
                    out = copy.deepcopy(msg.transforms[0])
                    out.header.frame_id = "box_base"
                    out.child_frame_id = "prism"
                    out.transform.translation.x = tf[0][0]
                    out.transform.translation.y = tf[0][1]
                    out.transform.translation.z = tf[0][2]
                    out.transform.rotation.x = tf[1][0]
                    out.transform.rotation.y = tf[1][1]
                    out.transform.rotation.z = tf[1][2]
                    out.transform.rotation.w = tf[1][3]
                    msg.transforms.append(out)

                out_bag.write(topic, msg, _t)
