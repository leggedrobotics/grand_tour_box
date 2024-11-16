import os.path

import yaml
from scipy.spatial.transform import Rotation
import argparse


def generate_urdf_xacro_with_rpy(yaml_file, topic_to_frame_mapping, output_xacro_file):
    # Load YAML data
    with open(yaml_file, 'r') as f:
        data = yaml.safe_load(f)
    with open(topic_to_frame_mapping, 'r') as f:
        topic_to_frame = yaml.safe_load(f)

    # Start building the xacro content
    xacro_content = '''<?xml version="1.0" ?>
<robot name="box" xmlns:xacro="http://www.ros.org/wiki/xacro">

    <xacro:property name="calibration_file" value="$(find box_calibration)/calibration/tf/calibration.yaml"/>
    <xacro:property name="calib" value="${xacro.load_yaml(calibration_file)}"/>
'''

    # Add links and joints for each camera
    for cam_name, cam_params in data.items():
        topic = cam_params['rostopic']
        link_name = topic_to_frame[topic]
        joint_name = f"box_base_to_{link_name}"

        # Extract translation and rotation from T_bundle_camera
        T = cam_params['T_bundle_camera']
        xyz = f"{T[0][3]:.6f} {T[1][3]:.6f} {T[2][3]:.6f}"

        # Convert rotation matrix to rpy
        rotation_matrix = [
            [T[0][0], T[0][1], T[0][2]],
            [T[1][0], T[1][1], T[1][2]],
            [T[2][0], T[2][1], T[2][2]]
        ]
        r = Rotation.from_matrix(rotation_matrix)
        rpy_angles = r.as_euler('xyz', degrees=True)  # Roll, Pitch, Yaw in degrees
        rpy = f"{rpy_angles[0]:.6f} {rpy_angles[1]:.6f} {rpy_angles[2]:.6f}"

        # Add link
        xacro_content += f'''
    <link name="{link_name}"/>
'''

        # Add joint
        xacro_content += f'''
    <joint name="{joint_name}" type="fixed">
        <origin xyz="{xyz}" rpy="{rpy}"/>
        <parent link="box_base"/>
        <child link="{link_name}"/>
    </joint>
'''

    # Close the robot tag
    xacro_content += '''
</robot>
'''

    if output_xacro_file is not None and os.path.exists(output_xacro_file):
        # Write to the output xacro file
        with open(output_xacro_file, 'w') as f:
            f.write(xacro_content)
        print(f"URDF xacro file generated: {output_xacro_file}")
    else:
        print(xacro_content)


def main():
    # Parse command-line arguments
    parser = argparse.ArgumentParser(description="Generate URDF xacro file from YAML.")
    parser.add_argument("-i", "--input_file", help="Path to the input YAML file.", required=True)
    parser.add_argument("-f", "--frame_mappings", help="Path to the frame mapping YAML file.", required=True)
    parser.add_argument("-o", "--output_file", help="Path to the output URDF xacro file.")
    args = parser.parse_args()

    # Generate the xacro file
    generate_urdf_xacro_with_rpy(args.input_file, args.frame_mappings, args.output_file)


if __name__ == "__main__":
    main()
