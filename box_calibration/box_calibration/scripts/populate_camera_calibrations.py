import argparse
import os

import numpy as np
import rospkg
import yaml
from scipy.spatial.transform import Rotation
from yaml import MappingNode, SequenceNode
from collections import OrderedDict

OUTPUT_FILE_MAPPINGS = {
    "/gt_box/alphasense_driver_node/cam1": "alphasense/cam1.yaml",
    "/gt_box/alphasense_driver_node/cam2": "alphasense/cam2.yaml",
    "/gt_box/alphasense_driver_node/cam3": "alphasense/cam3.yaml",
    "/gt_box/alphasense_driver_node/cam4": "alphasense/cam4.yaml",
    "/gt_box/alphasense_driver_node/cam5": "alphasense/cam5.yaml",
    "/gt_box/hdr_front/image_raw": "hdr/hdr_front.yaml",
    "/gt_box/hdr_left/image_raw": "hdr/hdr_left.yaml",
    "/gt_box/hdr_right/image_raw": "hdr/hdr_right.yaml",
    "/gt_box/zed2i_driver_node/left_raw/image_raw_color": "zed2i/zed_left.yaml",
    "/gt_box/zed2i_driver_node/right_raw/image_raw_color": "zed2i/zed_right.yaml",
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
    "/gt_box/zed2i_driver_node/left_raw/image_raw_color": "zed2i_left_camera_optical_frame",
    "/gt_box/zed2i_driver_node/right_raw/image_raw_color": "zed2i_right_camera_optical_frame",
}

OUTPUT_BUNDLE_ORIGIN = "/gt_box/zed2i_driver_node/left_raw/image_raw_color"
ZED_LEFT_FRAME = "/gt_box/zed2i_driver_node/left_raw/image_raw_color"
ZED_RIGHT_FRAME = "/gt_box/zed2i_driver_node/right_raw/image_raw_color"
ALPHASENSE_FRONTLEFTCAM = "/gt_box/alphasense_driver_node/cam3"
ALPHASENSE_FRONTRIGHTCAM = "/gt_box/alphasense_driver_node/cam2"
ALPHASENSE_FRONTMIDDLECAM = "/gt_box/alphasense_driver_node/cam1"
ALPHASENSE_LEFTCAM = "/gt_box/alphasense_driver_node/cam4"
ALPHASENSE_RIGHTCAM = "/gt_box/alphasense_driver_node/cam5"
HDR_FRONTCAM = "/gt_box/hdr_front/image_raw"
HDR_LEFTCAM = "/gt_box/hdr_left/image_raw"
HDR_RIGHTCAM = "/gt_box/hdr_right/image_raw"


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


def load_xyzrpy_yaml(path):
    with open(path, "r") as f:
        pinned_constraints = yaml.safe_load(f)
    output_SE3 = OrderedDict()
    for k, v in pinned_constraints.items():
        xyz = [v["x"], v["y"], v["z"]]
        rpy = [v["roll"], v["pitch"], v["yaw"]]
        R = Rotation.from_euler("xyz", rpy).as_matrix()
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, -1] = xyz
        output_SE3[k] = T
    return output_SE3


# Custom YAML Dumper to avoid showing OrderedDict headers
class NoOrderedDumper(yaml.SafeDumper):
    def represent_ordereddict(self, data):
        return self.represent_dict(data.items())


# Register representer for OrderedDict to dump it as a normal dict
NoOrderedDumper.add_representer(OrderedDict, NoOrderedDumper.represent_ordereddict)


def save_xyzrpy_yaml(path, data):
    output_data = OrderedDict()
    for k, v in data.items():
        t = v[:3, -1].tolist()
        rpy = Rotation.from_matrix(v[:3, :3]).as_euler("xyz").tolist()
        output_data[k] = dict(x=t[0], y=t[1], z=t[2],
                              roll=rpy[0], pitch=rpy[1], yaw=rpy[2])
    with open(path, "w") as f:
        yaml.dump(output_data, f, Dumper=NoOrderedDumper, default_flow_style=False, sort_keys=False)


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

    pinned_constraints_file = os.path.join(package_path, "calibration/tf/pinned_constraints.yaml")
    pinned_constraints = load_xyzrpy_yaml(pinned_constraints_file)
    T_boxbase_zedbase = pinned_constraints["box_base_to_zed_base"]
    T_zedbase_zedcameracenter = pinned_constraints["zed2i_base_to_camera_center"]
    T_zedcameracenter_zedleft = pinned_constraints["zed_camera_center_to_zed_left"]
    T_zedleft_zedleftoptical = pinned_constraints["zed2i_camera_to_zed2i_optical"]
    T_boxbase_zedleft = (T_boxbase_zedbase @
                         T_zedbase_zedcameracenter @
                         T_zedcameracenter_zedleft @
                         T_zedleft_zedleftoptical)
    extrinsics = {v["rostopic"]: v["T_bundle_camera"] for v in kalib_data.values()}
    T_bundle_outputbundle = extrinsics[OUTPUT_BUNDLE_ORIGIN]
    T_outputbundle_bundle = np.linalg.inv(T_bundle_outputbundle)
    extrinsics_boxbase = dict()
    for name, T_bundle_camera in extrinsics.items():
        T_outputbundle_camera = T_outputbundle_bundle @ T_bundle_camera
        T_boxbase_camera = T_boxbase_zedleft @ T_outputbundle_camera
        extrinsics_boxbase[name] = T_boxbase_camera

    output_extrinsics = load_xyzrpy_yaml(os.path.join(package_path,
                                                      "calibration/tf/calibration_cad.yaml"))
    T_zedleftoptical_zedrightoptical = (
        np.linalg.inv(extrinsics[ZED_LEFT_FRAME]) @ extrinsics[ZED_RIGHT_FRAME]
    )
    output_extrinsics["zed_left_optical_to_zed_right_optical"] = (
        T_zedleftoptical_zedrightoptical
    )

    T_alphasensebase_alphasensefrontleft = pinned_constraints["alphasense_base_to_alphasense_front_left"]
    T_boxbase_alphasensefrontleft = extrinsics_boxbase[ALPHASENSE_FRONTLEFTCAM]
    T_boxbase_alphasensebase = (T_boxbase_alphasensefrontleft @
                                np.linalg.inv(T_alphasensebase_alphasensefrontleft))
    T_alphasensebase_alphasensefrontright = (
        T_alphasensebase_alphasensefrontleft @
        np.linalg.inv(extrinsics[ALPHASENSE_FRONTLEFTCAM]) @
        extrinsics[ALPHASENSE_FRONTRIGHTCAM]
    )
    T_alphasensebase_alphasensefrontmiddle = (
        T_alphasensebase_alphasensefrontleft @
        np.linalg.inv(extrinsics[ALPHASENSE_FRONTLEFTCAM]) @
        extrinsics[ALPHASENSE_FRONTMIDDLECAM]
    )
    T_alphasensebase_alphasenseleft = (
        T_alphasensebase_alphasensefrontleft @
        np.linalg.inv(extrinsics[ALPHASENSE_FRONTLEFTCAM]) @
        extrinsics[ALPHASENSE_LEFTCAM]
    )
    T_alphasensebase_alphasenseright = (
        T_alphasensebase_alphasensefrontleft @
        np.linalg.inv(extrinsics[ALPHASENSE_FRONTLEFTCAM]) @
        extrinsics[ALPHASENSE_RIGHTCAM]
    )
    output_extrinsics["box_base_to_alphasense_base"] = (
        T_boxbase_alphasensebase
    )
    output_extrinsics["alphasense_base_to_alphasense_front_middle"] = (
        T_alphasensebase_alphasensefrontmiddle
    )
    output_extrinsics["alphasense_base_to_alphasense_front_right"] = (
        T_alphasensebase_alphasensefrontright
    )
    output_extrinsics["alphasense_base_to_alphasense_left"] = (
        T_alphasensebase_alphasenseleft
    )
    output_extrinsics["alphasense_base_to_alphasense_right"] = (
        T_alphasensebase_alphasenseright
    )

    T_hdrbase_hdrfront = pinned_constraints["hdr_base_to_hdr_front"]
    T_boxbase_hdrfront = extrinsics_boxbase[HDR_FRONTCAM]
    T_boxbase_hdrbase = (T_boxbase_hdrfront @
                         np.linalg.inv(T_hdrbase_hdrfront))
    T_hdrbase_hdrleft = (
        T_hdrbase_hdrfront @
        np.linalg.inv(extrinsics[HDR_FRONTCAM]) @
        extrinsics[HDR_LEFTCAM]
    )
    T_hdrbase_hdrright = (
        T_hdrbase_hdrfront @
        np.linalg.inv(extrinsics[HDR_FRONTCAM]) @
        extrinsics[HDR_RIGHTCAM]
    )
    output_extrinsics["box_base_to_hdr_base"] = (
        T_boxbase_hdrbase
    )
    output_extrinsics["hdr_base_to_hdr_left"] = (
        T_hdrbase_hdrleft
    )
    output_extrinsics["hdr_base_to_hdr_right"] = (
        T_hdrbase_hdrright
    )
    calibration_cameras_path = os.path.join(
        package_path, "calibration/tf/calibration_latest.yaml")
    save_xyzrpy_yaml(calibration_cameras_path, output_extrinsics)


if __name__ == "__main__":
    parser = argparse.ArgumentParser("Populate the output of the camera bundle adjustment calibrations into"
                                     "the designated GT Box Camera Intrinsics and Extrinsics files")
    parser.add_argument("--input_calibration_file", "-c", help="Yaml file with the calibration output", required=True)
    main(parser.parse_args())
