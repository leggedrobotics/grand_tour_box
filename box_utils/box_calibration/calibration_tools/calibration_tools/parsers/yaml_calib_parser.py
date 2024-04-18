import yaml
import numpy as np
from typing import Dict
from .calib_parser import CalibParser, Calibration


class YamlCalibParser(CalibParser):
    def parse_calib_file(self, file_path: str) -> Dict[str, Calibration]:
        with open(file_path, "r") as file:
            data = yaml.safe_load(file)

            calibrations = {}

            for name, matrix in data.items():
                np_matrix = np.array(matrix)
                calibrations[name] = Calibration(np_matrix, self.row_major_se3_to_xyz_rpy(np_matrix))

            return calibrations
