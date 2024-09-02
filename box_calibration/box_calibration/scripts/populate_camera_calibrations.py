import argparse
import os

import rospkg
import yaml
from yaml import MappingNode, SequenceNode


OUTPUT_FILE_MAPPINGS = {
    "/gt_box/alphasense_driver_node/cam1": "alphasense/cam1.yaml",
    "/gt_box/alphasense_driver_node/cam2": "alphasense/cam2.yaml",
    "/gt_box/alphasense_driver_node/cam3": "alphasense/cam3.yaml",
    "/gt_box/alphasense_driver_node/cam4": "alphasense/cam4.yaml",
    "/gt_box/alphasense_driver_node/cam5": "alphasense/cam5.yaml",
    "/gt_box/hdr_front/image_raw": "hdr/hdr_front.yaml",
    "/gt_box/hdr_left/image_raw": "hdr/hdr_left.yaml",
    "/gt_box/hdr_right/image_raw": "hdr/hdr_right.yaml",
    "/gt_box/zed2i_driver_node/left_raw/image_raw_gray": "zed2i/zed_left.yaml",
    "/gt_box/zed2i_driver_node/right_raw/image_raw_gray": "zed2i/zed_right.yaml",
}
FRAMEID_MAPPINGS = {
    "/gt_box/alphasense_driver_node/cam1": "cam1_sensor_frame",
    "/gt_box/alphasense_driver_node/cam2": "cam2_sensor_frame",
    "/gt_box/alphasense_driver_node/cam3": "cam3_sensor_frame",
    "/gt_box/alphasense_driver_node/cam4": "cam4_sensor_frame",
    "/gt_box/alphasense_driver_node/cam5": "cam5_sensor_frame",
    "/gt_box/hdr_front/image_raw": "hdr_front",
    "/gt_box/hdr_left/image_raw": "hdr_left",
    "/gt_box/hdr_right/image_raw": "hdr_right",
    "/gt_box/zed2i_driver_node/left_raw/image_raw_gray": "zed2i_left_camera_optical_frame",
    "/gt_box/zed2i_driver_node/right_raw/image_raw_gray": "zed2i_right_camera_optical_frame",
}


def write_raw_img_pipeline_format(camera_info_dict, camera_name, output_path, original_filename_short=""):
    data = dict()
    data["image_width"] = camera_info_dict['resolution'][0]
    data["image_height"] = camera_info_dict['resolution'][1]

    data["camera_name"] = FRAMEID_MAPPINGS[camera_name]

    data["camera_matrix"] = dict()
    data["camera_matrix"]["rows"] = 3
    data["camera_matrix"]["cols"] = 3
    fx = camera_info_dict['intrinsics'][0]
    fy = camera_info_dict['intrinsics'][1]
    cx = camera_info_dict['intrinsics'][2]
    cy = camera_info_dict['intrinsics'][3]
    data["camera_matrix"]["data"] = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]

    data["distortion_model"] = camera_info_dict['distortion_model']  # "equidistant"

    data["distortion_coefficients"] = dict()
    data["distortion_coefficients"]["rows"] = 1
    data["distortion_coefficients"]["cols"] = 4
    data["distortion_coefficients"]["data"] = camera_info_dict['distortion_coeffs']

    data["rectification_matrix"] = dict()
    data["rectification_matrix"]["rows"] = 3
    data["rectification_matrix"]["cols"] = 3
    data["rectification_matrix"]["data"] = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]

    data["projection_matrix"] = dict()
    data["projection_matrix"]["rows"] = 3
    data["projection_matrix"]["cols"] = 4
    data["projection_matrix"]["data"] = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]

    with open(output_path, 'w') as outfile:
        outfile.write('# Created using ' + original_filename_short + '\n')
        yaml_composition = yaml.compose(yaml.safe_dump(data))
        mutate_sequence_flowstyle_to_inline(yaml_composition)
        yaml.serialize(yaml_composition, outfile)  # , default_flow_style=False)


def mutate_sequence_flowstyle_to_inline(node):
    if isinstance(node, MappingNode):
        node.flow_style = False
        for _, child in node.value:
            mutate_sequence_flowstyle_to_inline(child)
    elif isinstance(node, SequenceNode):
        node.flow_style = True
    else:
        node.flow_style = False


def main(args):
    with open(args.input_calibration_file, "r") as f:
        kalib_data = yaml.safe_load(f)
    rospack = rospkg.RosPack()
    package_name = 'box_calibration'
    package_path = rospack.get_path(package_name)

    for camera_info in kalib_data.values():
        rostopic = camera_info["rostopic"]
        output_path = os.path.join(package_path, "calibration", OUTPUT_FILE_MAPPINGS[rostopic])
        print(f"{rostopic}->{output_path}")
        write_raw_img_pipeline_format(camera_info, camera_name=rostopic,
                                      output_path=output_path)


if __name__ == "__main__":
    parser = argparse.ArgumentParser("Populate the output of the camera bundle adjustment calibrations into"
                                     "the designated GT Box folder files")
    parser.add_argument("--input_calibration_file", "-c", help="Yaml file with the calibration output", required=True)
    main(parser.parse_args())
