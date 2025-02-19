import yaml
import rosbag
import tf2_msgs.msg
import geometry_msgs.msg
import numpy as np
import tf
import copy


def read_yaml(file_path):
    with open(file_path, "r") as file:
        return yaml.safe_load(file)


def create_tf_msg(frame_id, child_frame_id, matrix, secs):
    transform_stamped = geometry_msgs.msg.TransformStamped()
    transform_stamped.header.frame_id = frame_id
    transform_stamped.header.stamp.secs = secs
    transform_stamped.child_frame_id = child_frame_id

    # Extract translation
    transform_stamped.transform.translation.x = matrix[0, 3]
    transform_stamped.transform.translation.y = matrix[1, 3]
    transform_stamped.transform.translation.z = matrix[2, 3]

    # Extract rotation
    from tf.transformations import quaternion_from_matrix

    quaternion = quaternion_from_matrix(matrix)
    transform_stamped.transform.rotation.x = quaternion[0]
    transform_stamped.transform.rotation.y = quaternion[1]
    transform_stamped.transform.rotation.z = quaternion[2]
    transform_stamped.transform.rotation.w = quaternion[3]

    return transform_stamped


def invert(transform_msg):
    """Invert a TransformStamped message."""
    inverted_msg = geometry_msgs.msg.TransformStamped()
    inverted_msg.header = copy.deepcopy(transform_msg.header)  # Keep the same timestamp and frame info
    inverted_msg.header.frame_id = transform_msg.child_frame_id
    inverted_msg.child_frame_id = transform_msg.header.frame_id

    print(inverted_msg.header.frame_id, inverted_msg.child_frame_id, "Done")
    # Extract translation
    t = transform_msg.transform.translation
    translation = np.array([t.x, t.y, t.z])

    # Extract rotation
    q = transform_msg.transform.rotation
    quaternion = np.array([q.x, q.y, q.z, q.w])

    # Compute inverse rotation
    inv_quaternion = tf.transformations.quaternion_inverse(quaternion)

    # Compute inverse translation
    inv_translation = -tf.transformations.quaternion_matrix(inv_quaternion)[:3, :3] @ translation

    # Assign inverted values
    inverted_msg.transform.translation.x = inv_translation[0]
    inverted_msg.transform.translation.y = inv_translation[1]
    inverted_msg.transform.translation.z = inv_translation[2]

    inverted_msg.transform.rotation.x = inv_quaternion[0]
    inverted_msg.transform.rotation.y = inv_quaternion[1]
    inverted_msg.transform.rotation.z = inv_quaternion[2]
    inverted_msg.transform.rotation.w = inv_quaternion[3]

    return inverted_msg


def write_rosbag(yaml_data, output_bag):
    bag = rosbag.Bag(output_bag, "w")
    tf_msg = tf2_msgs.msg.TFMessage()

    for secs in [0, 1, 2]:
        for key, value in yaml_data.items():
            if key not in ["T_xsens_rgb0", "T_bmi088_rgb0"]:
                continue

            # "T_bmi088_rgb0",
            if isinstance(value, dict) and "data" in value:
                matrix = np.array(value["data"]).reshape(4, 4)
                frame_id, child_frame_id = key.split("_")[1], key.split("_")[2]

                msg = create_tf_msg(frame_id, child_frame_id, matrix, secs)

                inverted_msg = invert(msg)

                tf_msg.transforms.append(inverted_msg)
                # else:
                #     tf_msg.transforms.append(msg)

        bag.write("/tf_static", tf_msg, tf_msg.transforms[0].header.stamp)
    bag.close()


if __name__ == "__main__":

    yaml_file = "/media/jonfrey/Untitled/box_paper_dataset_v2/calib_chain.yaml"
    bag_file = "/media/jonfrey/Untitled/box_paper_dataset_v2/1006_03_LIO_tf_static.bag"
    yaml_data = read_yaml(yaml_file)
    write_rosbag(yaml_data, bag_file)
    print(f"ROS bag written to {bag_file}")
