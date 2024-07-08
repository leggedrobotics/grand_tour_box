from boxi import BOX_ROOT_DIR, shell_run
import argparse

BOX_MODEL_DIR = "box_model/box_model/urdf"
CALIB_OUTPUT_FILE = "box_model/box_model/urdf/box/calibrations/calibration.yaml"
DEFAULT_CALIB_FILE = "box_utils/box_calibration/calibration_default_data/default_calibration.yaml"


def add_arguments(parser):
    parser.set_defaults(main=main)
    parser.add_argument("--html_file", type=str, help="Load default CAD calib from given html file.")
    return parser


def main(args):
    if args.default:
        cmd = f"python {BOX_ROOT_DIR}/box_utils/box_calibration/cad_html_parser.py {args.html_file} --output_file={DEFAULT_CALIB_FILE}; cp {DEFAULT_CALIB_FILE} {CALIB_OUTPUT_FILE}"
    else:
        cmd = f"python {BOX_ROOT_DIR}/box_utils/box_calibration/calib_importer.py {BOX_ROOT_DIR}/{CALIB_OUTPUT_FILE} {BOX_ROOT_DIR}/{BOX_MODEL_DIR}"
    shell_run(cmd)
