import os.path
from collections import OrderedDict

import numpy as np
import argparse

import yaml
from bs4 import BeautifulSoup
from scipy.spatial.transform import Rotation
import rospkg

np.set_printoptions(suppress=True, precision=5)


def parse_html(file_path):
    with open(file_path, "r", encoding="utf-8") as file:
        soup = BeautifulSoup(file, "html.parser")

    feature_associativities = soup.find_all("div", id=lambda x: x and x.startswith("InfoObject"))
    results = {}

    for feature in feature_associativities:
        header = feature.find("div", class_="NXGroupHeader").get_text(strip=True)
        feature_name = header.split(":")[-1].strip()
        transform_matrix = np.eye(4)

        origin_table = feature.find("table", id=lambda x: x and "OriginLocationInAbsCoordSysandWCS" in x)
        if origin_table:
            rows = origin_table.find_all("tr")
            wcs_origin = [float(td.get_text(strip=True)) for td in rows[1].find_all("td")]
            transform_matrix[0:3, 3] = [x / 1000.0 for x in wcs_origin]

        for axis, idx in zip(["X", "Y", "Z"], range(3)):
            axis_table = feature.find("table", id=lambda x: x and f"{axis}axisVectorYZPlane" in x)
            if axis_table:
                rows = axis_table.find_all("tr")
                wcs_axis = [float(td.get_text(strip=True)) for td in rows[1].find_all("td")]
                transform_matrix[0:3, idx] = wcs_axis

        results[feature_name] = transform_matrix

    return results


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
        output_data[k] = dict(x=t[0], y=t[1], z=t[2], roll=rpy[0], pitch=rpy[1], yaw=rpy[2])
    with open(path, "w") as f:
        yaml.dump(output_data, f, Dumper=NoOrderedDumper, default_flow_style=False, sort_keys=False)


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


def main():
    parser = argparse.ArgumentParser(
        description="Parse HTML Assembly file and current STIM320 CAD" " and output new CAD based in CPT7."
    )
    parser.add_argument("--input_html", "-i", type=str, help="Path to the HTML file to parse.")
    args = parser.parse_args()
    html_transformations = parse_html(args.input_html)
    OLD_REFERENCE_FRAME = "STIM320 Coordinate System"
    NEW_REFERENCE_FRAME = "CPT7 Coordinate System"
    T_cadbase_oldref = html_transformations[OLD_REFERENCE_FRAME]
    T_cadbase_newref = html_transformations[NEW_REFERENCE_FRAME]
    T_newref_oldref = np.linalg.inv(T_cadbase_newref) @ T_cadbase_oldref

    rospack = rospkg.RosPack()
    package_name = "box_calibration"
    package_path = rospack.get_path(package_name)
    old_calibration_in_STIM320 = load_xyzrpy_yaml(os.path.join(package_path, "calibration/tf/calibration_latest.yaml"))

    # Update transforms of the type T_boxbase_XXX
    catch_string = "box_base_to_"
    len_catch_string = len(catch_string)
    new_calibration_in_cpt7 = old_calibration_in_STIM320.copy()
    for k, v in new_calibration_in_cpt7.items():
        if k[:len_catch_string] == catch_string:
            new_calibration_in_cpt7[k] = T_newref_oldref @ v

    # Manually update the prism
    del new_calibration_in_cpt7["prism_to_box_base"]
    T_prism_oldboxbase = old_calibration_in_STIM320["prism_to_box_base"]
    T_newboxbase_prism = T_newref_oldref @ np.linalg.inv(T_prism_oldboxbase)
    new_calibration_in_cpt7["box_base_to_prism"] = T_newboxbase_prism

    for k, v in new_calibration_in_cpt7.items():
        print(k)
        print(v)
        print()

    save_xyzrpy_yaml(
        os.path.join(package_path, "calibration/tf/calibration_latest_in_cpt7.yaml"), new_calibration_in_cpt7
    )


if __name__ == "__main__":
    main()
