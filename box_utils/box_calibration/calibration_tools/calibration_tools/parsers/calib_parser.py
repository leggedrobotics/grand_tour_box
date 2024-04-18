import math
import numpy as np
from typing import Dict


class Calibration:
    """
    Represents a calibration data object.
    """

    def __init__(self, matrix, xyz_rpy: dict):
        self.se3: np.NDArray = matrix
        self.x = xyz_rpy["x"]
        self.y = xyz_rpy["y"]
        self.z = xyz_rpy["z"]
        self.roll = xyz_rpy["roll"]
        self.pitch = xyz_rpy["pitch"]
        self.yaw = xyz_rpy["yaw"]

    def to_output_format(self) -> tuple:
        """Packages the calibration data into tuple."""
        return (float(self.x), float(self.y), float(self.z), float(self.roll), float(self.pitch), float(self.yaw))


class CalibParser:
    """
    Base class for parsing calibration files.
    """

    @staticmethod
    def row_major_se3_roll(T):
        try:
            return math.atan2(T[1, 0], T[0, 0])
        except ZeroDivisionError:
            print("[calib_parser] Error: gimbal lock in roll")

    @staticmethod
    def row_major_se3_pitch(T):
        try:
            return math.atan2(-T[2, 0], math.sqrt(T[2, 1] ** 2 + T[2, 2] ** 2))
        except ZeroDivisionError:
            print("[calib_parser] Error: gimbal lock in pitch")

    @staticmethod
    def row_major_se3_yaw(T):
        try:
            return math.atan2(T[2, 1], T[2, 2])
        except ZeroDivisionError:
            print("[calib_parser] Error: gimbal lock in yaw")

    @staticmethod
    def row_major_se3_to_xyz_rpy(transformation):
        # Ensure transformation is a numpy array
        transformation = np.array(transformation)

        xyz_rpy = {
            "x": float(transformation[0, 3]),
            "y": float(transformation[1, 3]),
            "z": float(transformation[2, 3]),
            "roll": float(CalibParser.row_major_se3_roll(transformation)),
            "pitch": float(CalibParser.row_major_se3_pitch(transformation)),
            "yaw": float(CalibParser.row_major_se3_yaw(transformation)),
        }
        return xyz_rpy

    def parse_calib_file(self, file_path: str) -> Dict[str, Calibration]:
        raise NotImplementedError("Subclasses must override parse_calib_file()")
