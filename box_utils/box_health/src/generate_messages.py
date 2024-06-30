import yaml
import os


def read_yaml_file(file_path):
    with open(file_path, "r") as file:
        data = yaml.safe_load(file)
    return data


# Example usage
file_path = "box_utils/box_health/cfg/health_check_topics.yaml"  # Path to your YAML file
msg_file_path = "box_utils/box_health/msg"


yaml_data = read_yaml_file(file_path)
print(yaml_data)


def generate_ros_msg_definition(pc_name, info):
    msg_lines = []

    # Adding string definitions for offsets
    for t in info.get("ptp4l", []) + info.get("phc2sys", []):
        msg_lines.append(f"string offset_{t}")

    msg_lines.append("")  # Blank line for readability

    # Process each topic to generate the frequency variables
    topics = info["topics"]
    for topic in topics:
        # Convert the topic name into the required frequency variable name
        frequency_var_name = topic.replace("/", "_").strip("_") + "_hz"
        msg_lines.append(f"float64 {frequency_var_name}")

    msg_lines.append("")  # Blank line for readability

    # Add CPU and memory definitions
    msg_lines.append(f"float32 cpu_usage_{pc_name}")
    msg_lines.append(f"string avail_memory_{pc_name}")

    # Join all lines into a single string
    ros_msg_definition = "\n".join(msg_lines)
    return ros_msg_definition


for pc_name, info in yaml_data.items():
    # Generate the ROS message definition based on the YAML data
    ros_msg_content = generate_ros_msg_definition(pc_name, info)
    print(ros_msg_content)

    # Write the ROS message definition to a file
    file_path = os.path.join(msg_file_path, f"health_status_{pc_name}.msg")
    with open(file_path, "w") as file:
        file.write(ros_msg_content)
