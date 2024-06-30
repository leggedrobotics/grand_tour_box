from boxi import BOX_ROOT_DIR, shell_run
import argparse

BOX_MODEL_DIR = "box_model/box_model/urdf"
CALIB_OUTPUT_FILE = "box_model/box_model/urdf/box/calibrations/calibration.yaml"
DEFAULT_CALIB_FILE = "box_utils/box_calibration/calibration_default_data/default_calibration.yaml"


def add_arguments(parser):
    parser.set_defaults(main=main)
    parser.add_argument("--default", action="store_true", help="Only load default CAD calibration.")
    return parser


def main(args):
    if args.default:
        cmd = f"cp {DEFAULT_CALIB_FILE} {CALIB_OUTPUT_FILE}"
    else:
        cmd = f"python {BOX_ROOT_DIR}/box_utils/box_calibration/calib_importer.py {BOX_ROOT_DIR}/{CALIB_OUTPUT_FILE} {BOX_ROOT_DIR}/{BOX_MODEL_DIR}"
    shell_run(cmd)
