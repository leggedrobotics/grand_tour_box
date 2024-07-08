"""
Standalone script that parses HTML files exported from CAD software and extracts transformation matrices in a yaml
format compliant with our calibration .xacro files.

This script relies on the presence of a html_tf_name_mappings.yaml file in a config directory, and will output to the 
calibration_default_data directory.

Example Usage:
python box_utils/box_calibration/cad_html_parser.py cad_tf.html
"""

import numpy as np
import argparse
from bs4 import BeautifulSoup
from scipy.spatial.transform import Rotation as R
import yaml


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
            transform_matrix[0:3, 3] = wcs_origin

        for axis, idx in zip(["X", "Y", "Z"], range(3)):
            axis_table = feature.find("table", id=lambda x: x and f"{axis}axisVectorYZPlane" in x)
            if axis_table:
                rows = axis_table.find_all("tr")
                wcs_axis = [float(td.get_text(strip=True)) for td in rows[1].find_all("td")]
                transform_matrix[0:3, idx] = wcs_axis

        results[feature_name] = transform_matrix

    return results


def convert_to_standard_types(data):
    if isinstance(data, np.ndarray):
        return data.tolist()
    elif isinstance(data, np.generic):
        return data.item()
    elif isinstance(data, dict):
        return {k: convert_to_standard_types(v) for k, v in data.items()}
    else:
        return data


def write_to_yaml(transformations, output_file, name_map):
    with open(output_file, "w") as file:
        for name, matrix in transformations.items():
            if name not in name_map:
                print(f"Warning: No mapping found for '{name}'. Using the original name.")
            new_name = name_map.get(name, name)  # Use mapped name if available, otherwise use original name
            rot = R.from_matrix(matrix[0:3, 0:3])
            rpy = rot.as_euler("xyz", degrees=True)  # Get RPY in degrees
            x, y, z = matrix[0:3, 3] / 1000  # Convert mm to meters
            data = {
                "x": float(x),
                "y": float(y),
                "z": float(z),
                "roll": float(rpy[0]),
                "pitch": float(rpy[1]),
                "yaw": float(rpy[2]),
            }
            yaml.dump({new_name: convert_to_standard_types(data)}, file, default_flow_style=False)
            file.write("\n")  # Add newline between entries


def load_name_map(name_map_file):
    with open(name_map_file, "r") as file:
        name_map = yaml.safe_load(file)
    return name_map


def main():
    parser = argparse.ArgumentParser(description="Parse HTML and extract transformations.")
    parser.add_argument("file_path", type=str, help="Path to the HTML file to parse.")
    parser.add_argument(
        "--output_file",
        type=str,
        help="Path to the output YAML file.",
        default="calibration_default_data/default_calibration.yaml",
    )
    parser.add_argument(
        "--name_map_file",
        type=str,
        help="Path to the YAML file containing name mappings.",
        default="config/html_tf_name_mappings.yaml",
    )
    args = parser.parse_args()
    name_map = load_name_map(args.name_map_file)
    transformations = parse_html(args.file_path)
    write_to_yaml(transformations, args.output_file, name_map)


if __name__ == "__main__":
    main()
