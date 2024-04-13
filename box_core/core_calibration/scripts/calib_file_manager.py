"""
Calibration File Manager. Allows for easy updated to the calibration.yaml file.

Example Usage:
from calibration_tools.calibration import CalibFileManager

def main():
    manager = CalibFileManager('calibration.yaml')
    # CalibName must match an exisitng calibration name in the .xarco files.
    manager.update_calibration(manager.CalibName.BOX_BASE_TO_CAMERAS_FRONT, 0.45, 0.0, 0.1, 0.0, 0.0)

if __name__ == "__main__":
    main()
"""

import yaml
import xml.etree.ElementTree as ET
import os
import glob
from enum import Enum
from dataclasses import dataclass, field
from typing import Dict, List

# Define the model directory (update this path to the actual directory containing XARCO files)
MODEL_DIRECTORY = "path_to_model_directory"


def find_xacro_files(directory: str) -> List[str]:
    """Recursively find all .xarco files in the given directory."""
    return glob.glob(os.path.join(directory, "**", "*.xarco"), recursive=True)


def extract_joint_names_from_xacro(file_path: str) -> List[str]:
    """Extract joint names from a given Xacro file."""
    joint_names = []
    tree = ET.parse(file_path)
    root = tree.getroot()
    for joint in root.findall(".//joint"):
        if "name" in joint.attrib:
            joint_names.append(joint.attrib["name"])
    return joint_names


def generate_calibration_enums(joint_names: List[str]) -> Enum:
    """Generate an Enum from a list of joint names for calibration."""
    return Enum("CalibName", {name.upper().replace(" ", "_"): name for name in joint_names})


@dataclass
class CalibrationData:
    data: Dict[str, Dict[str, float]] = field(default_factory=dict)

    def update_calibration(self, name, x: float, y: float, z: float, roll: float, pitch: float, yaw: float):
        """Update calibration data for a specified joint name."""
        self.data[name.value] = {"x": x, "y": y, "z": z, "roll": roll, "pitch": pitch, "yaw": yaw}


class CalibFileManager:
    def __init__(self, calibration_file: str):
        """Initialize the calibration manager, load data and generate enums."""
        self.calibration_file = calibration_file
        self.calib_data = self.load_calibration()
        self.CalibName = self.load_and_generate_enums()

    def load_calibration(self) -> CalibrationData:
        """Load calibration data from a YAML file. Creates a new file if not found."""
        try:
            with open(self.calibration_file, "r") as file:
                data = yaml.safe_load(file) or {}
        except FileNotFoundError:
            print(f"Calibration file not found. Creating a new calibration file at {self.calibration_file}.")
            data = {}
        return CalibrationData(data)

    def save_calibration(self):
        """Save the current calibration data back to the YAML file."""
        with open(self.calibration_file, "w") as file:
            yaml.safe_dump(self.calib_data.data, file)

    def load_and_generate_enums(self):
        """Load joint names from Xacro files and generate calibration enums."""
        xacro_files = find_xacro_files(MODEL_DIRECTORY)
        joint_names = set()
        for file_path in xacro_files:
            joint_names.update(extract_joint_names_from_xacro(file_path))
        return generate_calibration_enums(joint_names)

    def update_calibration(self, name, x: float, y: float, z: float, roll: float, pitch: float, yaw: float):
        """Update the calibration for a given enum name and transformation values. Checks if name is valid."""
        if not isinstance(name, self.CalibName):
            raise ValueError(f"{name} is not a valid calibration entry. Check or update .xarco files.")
        self.calib_data.update_calibration(name, x, y, z, roll, pitch, yaw)

    def __del__(self):
        """Destructor to ensure calibration data is saved when the instance is discarded."""
        self.save_calibration()
