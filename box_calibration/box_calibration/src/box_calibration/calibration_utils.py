import rosbag
import yaml


def fun1():
    return True


def extract_image_topics(rosbag_path):
    """
    Extract all sensor_msgs/Image topics from a ROS bag file.

    Args:
        rosbag_path (str): Path to the ROS bag file.

    Returns:
        list: A list of topics containing sensor_msgs/Image messages.
    """
    image_topics = []
    with rosbag.Bag(rosbag_path, 'r') as bag:
        topics_info = bag.get_type_and_topic_info().topics
        for topic, info in topics_info.items():
            if info.msg_type == 'sensor_msgs/Image':
                image_topics.append(topic)
    return image_topics


def filter_yaml_by_rostopics(input_yaml_path, required_rostopics, output_yaml_path):
    """
    :param input_yaml_path: Path to the input YAML file.
    :param required_rostopics: List of required ROS topics to filter the YAML content.
    :param output_yaml_path: Path to the output YAML file where filtered content will be saved.
    :return: Boolean indicating if the operation was successful.
    """
    with open(input_yaml_path, 'r') as file:
        yaml_content = yaml.safe_load(file)

    filtered_content = {k: v for k, v in yaml_content.items() if
                        'rostopic' in v and v['rostopic'] in required_rostopics}

    with open(output_yaml_path, 'w') as file:
        yaml.dump(filtered_content, file)
    return True
