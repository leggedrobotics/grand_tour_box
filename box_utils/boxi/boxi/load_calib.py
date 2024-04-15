from boxi import BOX_ROOT_DIR, shell_run
import argparse

BOX_MODEL_DIR = "box_model/box_model/urdf"
CALIB_OUTPUT_FILE = "box_model/box_model/urdf/box/calibrations/test_calibration.yaml"


def add_arguments(parser):
    parser.set_defaults(main=main)
    return parser


def main(args):
    cmd = f"python {BOX_ROOT_DIR}/box_utils/box_calibration/calib_importer.py {BOX_ROOT_DIR}/{CALIB_OUTPUT_FILE} {BOX_ROOT_DIR}/{BOX_MODEL_DIR}"
    shell_run(cmd)