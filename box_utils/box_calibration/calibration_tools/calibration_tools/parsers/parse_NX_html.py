import os
from collections import OrderedDict
import argparse
import tempfile
from pathlib import Path

import numpy as np
import yaml
from bs4 import BeautifulSoup
from scipy.spatial.transform import Rotation

np.set_printoptions(suppress=True, precision=5)


def parse_html(file_path: str) -> "OrderedDict[str, np.ndarray]":
    with open(file_path, "r", encoding="utf-8") as file:
        soup = BeautifulSoup(file, "lxml")

    feature_associativities = soup.find_all("div", id=lambda x: x and x.startswith("InfoObject"))
    results: "OrderedDict[str, np.ndarray]" = OrderedDict()

    for feature in feature_associativities:
        header_div = feature.find("div", class_="NXGroupHeader")
        if header_div is None:
            continue

        header = header_div.get_text(strip=True)
        feature_name = header.split(":")[-1].strip()
        if not feature_name:
            continue

        if feature_name in results:
            raise ValueError(f"Duplicate calibration name: {feature_name}")
        transform_matrix = np.eye(4, dtype=float)

        def required_vector(table_id):
            table = feature.find("table", id=lambda value: value and table_id in value)
            rows = table.find_all("tr") if table else []
            cells = rows[1].find_all("td") if len(rows) >= 2 else []
            if len(cells) < 3:
                raise ValueError(f"{feature_name}: missing or incomplete {table_id}")
            vector = np.array([float(cell.get_text(strip=True)) for cell in cells[:3]])
            if not np.isfinite(vector).all():
                raise ValueError(f"{feature_name}: nonfinite calibration vector {table_id}")
            return vector

        # Translation (Origin) in mm -> convert to m
        transform_matrix[:3, 3] = required_vector("OriginLocationInAbsCoordSysandWCS") / 1000.0

        # Rotation axes as column vectors
        for axis, idx in zip(["X", "Y", "Z"], range(3)):
            transform_matrix[:3, idx] = required_vector(f"{axis}axisVectorYZPlane")

        # Rotation.from_matrix orthogonalizes its input. Validate first so a
        # missing, reflected or malformed CAD basis cannot become plausible RPY.
        rotation = transform_matrix[:3, :3]
        if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-5, rtol=0) or not np.isclose(
            np.linalg.det(rotation), 1.0, atol=1e-5, rtol=0
        ):
            raise ValueError(f"{feature_name}: axes must form an orthonormal right-handed basis")

        results[feature_name] = transform_matrix

    return results


class NoOrderedDumper(yaml.SafeDumper):
    def represent_ordereddict(self, data):
        return self.represent_dict(data.items())


NoOrderedDumper.add_representer(OrderedDict, NoOrderedDumper.represent_ordereddict)


def save_xyzrpy_yaml(path: str, data: "OrderedDict[str, np.ndarray]") -> None:
    output_data = OrderedDict()
    for k, v in data.items():
        t = v[:3, 3].tolist()
        rpy = Rotation.from_matrix(v[:3, :3]).as_euler("xyz").tolist()
        output_data[k] = dict(x=t[0], y=t[1], z=t[2], roll=rpy[0], pitch=rpy[1], yaw=rpy[2])

    destination = Path(path)
    with tempfile.TemporaryDirectory(prefix=".cad-calibration-", dir=destination.parent) as directory:
        temporary = Path(directory) / "calibration.yaml"
        with temporary.open("w", encoding="utf-8") as stream:
            yaml.dump(output_data, stream, Dumper=NoOrderedDumper, default_flow_style=False, sort_keys=False)
        os.replace(temporary, destination)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Parse HTML Assembly file and export all extracted transforms to YAML."
    )
    parser.add_argument(
        "--input_html",
        "-i",
        type=str,
        default="/abs/path/to/your/file.html",  # <-- hardcoded default
        help="Path to the HTML file to parse.",
    )
    args = parser.parse_args()

    input_html = args.input_html
    if not os.path.isfile(input_html):
        raise FileNotFoundError(f"Input HTML not found: {input_html}")

    output_path = os.path.join(os.path.dirname(os.path.abspath(input_html)), "CAD_vaues.yaml")

    html_transformations = parse_html(input_html)
    if not html_transformations:
        raise RuntimeError("No transforms found in the HTML (no InfoObject* sections parsed).")

    save_xyzrpy_yaml(output_path, html_transformations)
    print(f"Wrote {len(html_transformations)} transforms to: {output_path}")


if __name__ == "__main__":
    main()
