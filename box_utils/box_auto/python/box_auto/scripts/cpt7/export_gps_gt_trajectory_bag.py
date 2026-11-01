import numpy as np
import pandas as pd
import rosbag
import rospy
import math
import os
import matplotlib.pyplot as plt

from nav_msgs.msg import Odometry
from geometry_msgs.msg import (
    PoseWithCovarianceStamped,
    TwistWithCovariance,
    Point,
    Quaternion,
    TransformStamped,
    Vector3,
    Vector3Stamped,
    PoseStamped,
)
from gnss_msgs.msg import GnssRaw
from sensor_msgs.msg import NavSatFix, NavSatStatus
from tf2_msgs.msg import TFMessage
from std_msgs.msg import Float64
from scipy.spatial.transform import Rotation
from pathlib import Path
from nav_msgs.msg import Path as PathRos
from box_auto.utils import MISSION_DATA, GPS_utils
from box_auto.utils.atomic_file import atomic_output
import xml.etree.ElementTree as ET
from html.parser import HTMLParser
import json
from tqdm import tqdm


FRAME_ID_WORLD = "enu_origin"
FRAME_ID_BODY = "cpt7_imu"

# -----------------------------------------------------------------------------
# IMPORTANT INTERPRETATION NOTES
#
# 1) IE velocity columns VX-ECEF, VY-ECEF, VZ-ECEF are interpreted as TRUE ECEF
#    linear velocity components (Earth-Centered, Earth-Fixed Cartesian).
#    - They are not ENU and not NED.
#    - For robotics, we rotate v_ecef into a local tangent plane.
#    - This script uses a single ENU tangent plane anchored at the chosen origin
#      (ENU@origin). This makes the published pose/odom consistent over the run.
#    - Conversion used throughout:
#         v_enu_origin = R_ecef2enu_origin * v_ecef
#
# 2) IE attitude columns (Heading, Roll, Pitch) are interpreted as an INS attitude
#    commonly expressed in NED navigation coordinates.
#    - IE_ATTITUDE_IN_NED=True applies a NED->ENU conversion so the published
#      quaternion maps BODY vectors into WORLD(ENU) vectors.
#    - IE_HEADING_IS_NEG_YAW can be used if the heading sign convention is opposite.
#
# 3) "Robot heading" exported by this script is the yaw of the BODY +X axis projected
#    into WORLD(ENU):
#    - Let rot_wb map BODY vectors into WORLD(ENU).
#    - f_w = rot_wb.apply([1,0,0]) is the WORLD forward direction.
#    - yaw_deg = atan2(East, North) in degrees, wrapped to [0, 360).
#    - This is the heading you typically want for robotics in an ENU world frame.
# -----------------------------------------------------------------------------

IE_HEADING_IS_NEG_YAW = False  # toggle if yaw sign is wrong vs COG
IE_ATTITUDE_IN_NED = True  # common for INS; converts NED->ENU for WORLD

# Additive correction to the IE compass heading before converting NED attitude
# to ENU.  Positive is clockwise from north; in standard ENU yaw this produces
# the opposite (clockwise) rotation.  Keep the default at zero so existing
# exports remain reproducible.  For the calibrated ALB-1 test, run with e.g.:
#
#   IE_HEADING_OFFSET_DEG=13.0 python export_gps_gt_trajectory_bag.py
#
# This is intentionally explicit: a mission-derived empirical bias must not be
# silently applied to every mission until its physical calibration is verified.
IE_HEADING_OFFSET_DEG = float(os.environ.get("IE_HEADING_OFFSET_DEG", "0.0"))

# If you want FRD->FLU conversion for body axes, enable:
CONVERT_BODY_FRD_TO_FLU = False

# Odometry.twist is expressed in child_frame_id in ROS.
PUBLISH_TWIST_IN_BODY_FRAME = True

# If you have no angular velocity covariance, publish "unknown" as large variances.
ANG_COV_UNKNOWN = 1e6

# Plotting
FIG_DPI = 800
BAND_SIGMA = 1.0  # confidence band = +/- BAND_SIGMA * std
SAVE_FIGS = True
SHOW_FIGS = False  # set True if you want interactive windows


# HTML parser to handle CDATA with <BR> and text
class DescriptionHTMLParser(HTMLParser):
    def __init__(self):
        super().__init__()
        self.lines = []

    def handle_data(self, data):
        line = data.strip()
        if line:
            self.lines.append(line)


# Parses description content into key-value pairs
def parse_solution_description(description_html):
    parser = DescriptionHTMLParser()
    parser.feed(description_html)

    result = {}
    current_key = None

    for line in parser.lines:
        if ":" in line:
            key, value = line.split(":", 1)
            current_key = key.strip()
            result[current_key] = value.strip()
        elif current_key:
            result[current_key] += " " + line.strip()
    return result


# Efficiently stream-parse large KML files
def extract_solution_data_stream(kml_file, output_json):
    ns = "http://earth.google.com/kml/2.2"
    target_tag = f"{{{ns}}}Placemark"
    name_tag = f"{{{ns}}}name"
    desc_tag = f"{{{ns}}}description"

    context = ET.iterparse(kml_file, events=("end",))
    for _event, elem in context:
        if elem.tag == target_tag:
            name_elem = elem.find(name_tag)
            if name_elem is not None and name_elem.text == "Solution":
                desc_elem = elem.find(desc_tag)
                if desc_elem is not None and desc_elem.text:
                    parsed = parse_solution_description(desc_elem.text)
                    with open(output_json, "w") as f:
                        json.dump(parsed, f, indent=2)
                    print(f"[OK] Extracted to {output_json}")
                    return parsed
            elem.clear()  # Free memory
    raise ValueError("No <Placemark> with <name>Solution</name> found.")


def read_sbet(filepath, num_fields=17):
    """
    Fast SBET reader: vectorized binary load.
    Equivalent record layout to struct.unpack loop, but much faster.
    NOTE: this truncates if the file size is not a multiple of num_fields.
    """
    data = np.fromfile(filepath, dtype="<f8")  # little-endian float64
    if data.size == 0:
        return np.empty((0, num_fields), dtype=np.float64)
    if data.size % num_fields != 0:
        data = data[: (data.size // num_fields) * num_fields]
    return data.reshape((-1, num_fields))


def read_sbet_checked(filepath, num_fields=17):
    """
    Strict SBET reader: fail if file size isn't an integer number of records.
    """
    filepath = Path(filepath)
    rec_bytes = num_fields * 8
    size = filepath.stat().st_size
    if size == 0:
        return np.empty((0, num_fields), dtype=np.float64)
    if size % rec_bytes != 0:
        raise ValueError(f"{filepath} size {size} not multiple of {rec_bytes} bytes; wrong format/corrupt?")
    return np.fromfile(str(filepath), dtype="<f8").reshape((-1, num_fields))


def write_sbet_to_txt(records, output_path, precision=8, include_header=True):
    header_fields = [
        "time",
        "latitude_rad",
        "longitude_rad",
        "altitude_m",
        "x_velocity_mps",
        "y_velocity_mps",
        "z_velocity_mps",
        "roll_rad",
        "pitch_rad",
        "heading_rad",
        "wander_rad",
        "x_accel_mps2",
        "y_accel_mps2",
        "z_accel_mps2",
        "x_ang_rate_rps",
        "y_ang_rate_rps",
        "z_ang_rate_rps",
    ]

    fmt = f"%.{precision}f"
    header = " ".join(header_fields) if include_header else ""
    np.savetxt(output_path, records, fmt=fmt, delimiter=" ", header=header, comments="")


def ecef_velocity_to_enu(vx, vy, vz, lat_rad, lon_rad):
    sin_lat = np.sin(lat_rad)
    cos_lat = np.cos(lat_rad)
    sin_lon = np.sin(lon_rad)
    cos_lon = np.cos(lon_rad)

    R = np.array(
        [
            [-sin_lon, cos_lon, 0],
            [-sin_lat * cos_lon, -sin_lat * sin_lon, cos_lat],
            [cos_lat * cos_lon, cos_lat * sin_lon, sin_lat],
        ]
    )

    v_ecef = np.array([vx, vy, vz])
    v_enu = R @ v_ecef
    return v_enu


def map_ambiguity_status(status_str):
    s = str(status_str).strip().lower()
    if s == "fixed":
        return GnssRaw.FIXED
    elif s == "float":
        return GnssRaw.FLOAT
    else:
        return 0


def ecef_to_enu_covariance(cov_ecef, lat_rad, lon_rad):
    sin_lat = np.sin(lat_rad)
    cos_lat = np.cos(lat_rad)
    sin_lon = np.sin(lon_rad)
    cos_lon = np.cos(lon_rad)

    R = np.array(
        [
            [-sin_lon, cos_lon, 0],
            [-sin_lat * cos_lon, -sin_lat * sin_lon, cos_lat],
            [cos_lat * cos_lon, cos_lat * sin_lon, sin_lat],
        ]
    )
    cov_enu = R.dot(cov_ecef).dot(R.T)
    return cov_enu


def dms_string_to_decimal(dms_str: str) -> float:
    parts = str(dms_str).strip().split()
    deg = float(parts[0])
    minutes = float(parts[1])
    seconds = float(parts[2])

    sign = math.copysign(1.0, deg)  # Preserve the sign of DMS values such as -0 30 0.
    deg = abs(deg)

    return sign * (deg + minutes / 60.0 + seconds / 3600.0)


def ecef_to_enu_rotation(lat_rad, lon_rad):
    sin_lat = math.sin(lat_rad)
    cos_lat = math.cos(lat_rad)
    sin_lon = math.sin(lon_rad)
    cos_lon = math.cos(lon_rad)

    return np.array(
        [
            [-sin_lon, cos_lon, 0.0],
            [-sin_lat * cos_lon, -sin_lat * sin_lon, cos_lat],
            [cos_lat * cos_lon, cos_lat * sin_lon, sin_lat],
        ],
        dtype=np.float64,
    )


def compact_cov_to_full(c6):
    # [c11, c21, c22, c31, c32, c33] -> symmetric 3x3
    c6 = np.asarray(c6, dtype=np.float64).reshape(
        6,
    )
    covariance = np.array(
        [
            [c6[0], c6[1], c6[3]],
            [c6[1], c6[2], c6[4]],
            [c6[3], c6[4], c6[5]],
        ],
        dtype=np.float64,
    )
    if not np.isfinite(covariance).all():
        raise ValueError("Covariance contains nonfinite values")
    tolerance = 1e-10 * max(1.0, float(np.max(np.abs(covariance))))
    if np.linalg.eigvalsh(covariance).min() < -tolerance:
        raise ValueError("Covariance is not positive semidefinite")
    return covariance


def _scalar(x):
    # robust scalar extraction for values that may come as 0-d or 1-elem arrays
    return float(np.asarray(x).squeeze())


def build_rot_wb_from_ie_hrp(heading_deg, roll_deg, pitch_deg):
    """
    Returns rot_wb: active rotation mapping BODY vectors -> WORLD(ENU) vectors.

    Interpretation details:
      - heading_deg is IE "Heading" (yaw-like angle).
      - roll_deg and pitch_deg are IE roll/pitch.
      - Euler sequence is "ZXY" with angles [yaw, roll, pitch] in degrees.
      - If IE attitude is in NED (common), we convert to ENU so WORLD is ENU.

    Configuration toggles:
      - IE_HEADING_IS_NEG_YAW: if the yaw sign convention is inverted.
      - IE_HEADING_OFFSET_DEG: additive correction in IE compass-heading degrees.
      - IE_ATTITUDE_IN_NED: if IE attitude is NED and must be converted to ENU.
      - CONVERT_BODY_FRD_TO_FLU: if body axes should be flipped FRD->FLU.
    """
    corrected_heading_deg = heading_deg + IE_HEADING_OFFSET_DEG
    yaw_deg = -corrected_heading_deg if IE_HEADING_IS_NEG_YAW else corrected_heading_deg
    rot_body_in_nav = Rotation.from_euler("ZXY", [yaw_deg, roll_deg, pitch_deg], degrees=True)

    if IE_ATTITUDE_IN_NED:
        # ENU->NED vector map: [E,N,U] -> [N,E,D]
        A_enu2ned = np.array([[0.0, 1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, -1.0]], dtype=np.float64)
        rot_ned2enu = Rotation.from_matrix(A_enu2ned.T)
        rot_wb = rot_ned2enu * rot_body_in_nav
    else:
        rot_wb = rot_body_in_nav

    if CONVERT_BODY_FRD_TO_FLU:
        # FRD -> FLU is (x, y, z) -> (x, -y, -z)
        rot_flu2frd = Rotation.from_matrix(np.diag([1.0, -1.0, -1.0]).astype(np.float64))
        rot_wb = rot_wb * rot_flu2frd

    return rot_wb


def orientation_covariance_enu(heading_deg, roll_deg, pitch_deg, std_hrp_deg):
    """Map independent IE ZXY angle errors to fixed ENU small-angle errors.

    ROS pose covariance describes rotations about fixed world axes. Merely
    reordering heading/roll/pitch variances is only valid at special attitudes.
    The Jacobian is for the same intrinsic ZXY composition used for the pose.
    A constant right-side body-axis conversion leaves this world Jacobian unchanged.
    """
    values = np.asarray([heading_deg, roll_deg, pitch_deg], dtype=float)
    std = np.asarray(std_hrp_deg, dtype=float)
    if std.shape != (3,) or not np.isfinite(values).all() or not np.isfinite(std).all() or np.any(std < 0):
        raise ValueError("Attitude and its standard deviations must be finite; deviations must be nonnegative")
    sign = -1.0 if IE_HEADING_IS_NEG_YAW else 1.0
    yaw = sign * (heading_deg + IE_HEADING_OFFSET_DEG)
    rz = Rotation.from_euler("z", yaw, degrees=True).as_matrix()
    rx = Rotation.from_euler("x", roll_deg, degrees=True).as_matrix()
    jacobian = np.column_stack((sign * np.array([0.0, 0.0, 1.0]), rz[:, 0], (rz @ rx)[:, 1]))
    if IE_ATTITUDE_IN_NED:
        jacobian = np.array([[0.0, 1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, -1.0]]) @ jacobian
    return (jacobian * np.square(np.deg2rad(std))) @ jacobian.T


def _save_fig(fig, out_path: Path):
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(str(out_path), dpi=FIG_DPI, bbox_inches="tight")
    plt.close(fig)


def plot_xyz_timeseries_with_bands(
    t_s: np.ndarray,
    xyz: np.ndarray,
    xyz_std: np.ndarray,
    title: str,
    ylabel: str,
    out_prefix: Path,
):
    """
    Saves three figures:
      out_prefix + "_x.png", "_y.png", "_z.png"
    with optional +/- BAND_SIGMA*std confidence band.
    """
    labels = ["x", "y", "z"]
    for k, lab in enumerate(labels):
        fig = plt.figure()
        ax = fig.add_subplot(111)
        y = xyz[:, k]
        ax.plot(t_s, y)
        if xyz_std is not None:
            s = xyz_std[:, k]
            s = np.sqrt(np.maximum(s * s, 0.0))  # ensure finite
            ax.fill_between(t_s, y - BAND_SIGMA * s, y + BAND_SIGMA * s, alpha=0.3)

        ax.set_xlabel("time [s]")
        ax.set_ylabel(f"{ylabel} {lab}")
        ax.set_title(f"{title} ({lab})")
        ax.grid(True)

        if SAVE_FIGS:
            _save_fig(fig, out_prefix.parent / f"{out_prefix.name}_{lab}.png")
        if SHOW_FIGS:
            fig.show()


def plot_biases(
    t_s: np.ndarray,
    acc_bias: np.ndarray,
    gyro_drift: np.ndarray,
    out_dir: Path,
    tag: str,
):
    # Acc bias XYZ
    fig1 = plt.figure()
    ax1 = fig1.add_subplot(111)
    ax1.plot(t_s, acc_bias[:, 0], label="x")
    ax1.plot(t_s, acc_bias[:, 1], label="y")
    ax1.plot(t_s, acc_bias[:, 2], label="z")
    ax1.set_xlabel("time [s]")
    ax1.set_ylabel("acc bias [m/s^2]")
    ax1.set_title(f"Accelerometer bias vs time ({tag})")
    ax1.grid(True)
    ax1.legend()
    if SAVE_FIGS:
        _save_fig(fig1, out_dir / f"acc_bias_xyz_{tag}.png")
    if SHOW_FIGS:
        fig1.show()

    # Gyro drift XYZ
    fig2 = plt.figure()
    ax2 = fig2.add_subplot(111)
    ax2.plot(t_s, gyro_drift[:, 0], label="x")
    ax2.plot(t_s, gyro_drift[:, 1], label="y")
    ax2.plot(t_s, gyro_drift[:, 2], label="z")
    ax2.set_xlabel("time [s]")
    ax2.set_ylabel("gyro drift [rad/s]")
    ax2.set_title(f"Gyroscope drift vs time ({tag})")
    ax2.grid(True)
    ax2.legend()
    if SAVE_FIGS:
        _save_fig(fig2, out_dir / f"gyro_drift_xyz_{tag}.png")
    if SHOW_FIGS:
        fig2.show()


def main():
    print(
        "IE attitude configuration: "
        f"heading_offset={IE_HEADING_OFFSET_DEG:+.3f} deg, "
        f"negative_yaw={IE_HEADING_IS_NEG_YAW}, "
        f"attitude_in_ned={IE_ATTITUDE_IN_NED}, "
        f"frd_to_flu={CONVERT_BODY_FRD_TO_FLU}"
    )
    patterns = ["*_nuc_livox.bag", "*_dlio.bag", "*_nuc_hesai.bag"]
    date = None
    for pattern in patterns:
        try:
            date = [(str(s.name)).split("_")[0] for s in Path(MISSION_DATA).glob(pattern)][0]
            break
        except IndexError:
            continue

    if date is None:
        print(
            f"Error: Could not find a suitable bag file in {MISSION_DATA} matching patterns: {patterns} to determine the date."
        )
        exit(1)

    mode_order = {mode: index for index, mode in enumerate(("ppp", "tc", "lc", "dgps"))}
    gps_files = sorted(
        (str(s) for s in (Path(MISSION_DATA) / "ie").rglob("*_GrandTour-LocalFrame-extended.txt")),
        key=lambda s: mode_order.get(Path(s).name.split("_")[1], len(mode_order)),
    )
    post_proc_modes = [s.split("/")[-1].split("_")[1] for s in gps_files]
    if not gps_files:
        raise FileNotFoundError("No GrandTour-LocalFrame-extended IE exports found")
    if len(set(post_proc_modes)) != len(post_proc_modes):
        raise ValueError("Multiple IE exports select the same output mode")
    print(post_proc_modes, MISSION_DATA)
    bag_paths = [
        str(Path(s).parent.parent / (date + f"_cpt7_ie_{mode}.bag")) for s, mode in zip(gps_files, post_proc_modes)
    ]

    frame_id = FRAME_ID_WORLD

    outer_iter = zip(gps_files, bag_paths, post_proc_modes)
    if tqdm is not None and len(gps_files) > 1:
        outer_iter = tqdm(outer_iter, total=len(gps_files), desc="Modes", unit="mode")

    for gps_file_path, bag_path, post_proc_mode in outer_iter:
        if post_proc_mode == "tc":
            kml_file_path = Path(MISSION_DATA) / "ie" / post_proc_mode / "Html" / f"ie_{post_proc_mode}.GE.kml"
            if not kml_file_path.exists() or not kml_file_path.is_file() or kml_file_path.stat().st_size == 0:
                print(f"Warning: No KML data found for {post_proc_mode}")
                raise ValueError(f"Warning: No KML data found for {post_proc_mode}. Meaning the processing failed.")
            else:
                print(f"Using KML file: {kml_file_path}")
                json_path = bag_path.replace(".bag", "_statistics.json")
                extract_solution_data_stream(kml_file_path, json_path)

        sbet_data_path = Path(MISSION_DATA) / "ie" / post_proc_mode / f"SBET_{post_proc_mode}.OUT"
        if not sbet_data_path.exists() or not sbet_data_path.is_file() or sbet_data_path.stat().st_size == 0:
            print(f"Warning: No SBET data found for {post_proc_mode}")
        else:
            sbet_data = read_sbet_checked(Path(MISSION_DATA) / "ie" / post_proc_mode / f"SBET_{post_proc_mode}.OUT")

            # You WERE reading SMRMSG before; you just weren't using it for ROS. Keep it, but guard mismatch.
            SMRMSG_path = Path(MISSION_DATA) / "ie" / post_proc_mode / f"SMRMSG_{post_proc_mode}.OUT"
            SMRMSG_data = None
            try:
                if SMRMSG_path.exists() and SMRMSG_path.stat().st_size > 0:
                    SMRMSG_data = read_sbet_checked(SMRMSG_path)
            except Exception as e:
                print(f"Warning: SMRMSG read skipped (layout mismatch): {e}")

            write_sbet_to_txt(
                sbet_data, Path(MISSION_DATA) / "ie" / post_proc_mode / f"SBET_text_{post_proc_mode}.txt", precision=5
            )
            if SMRMSG_data is not None:
                write_sbet_to_txt(
                    SMRMSG_data,
                    Path(MISSION_DATA) / "ie" / post_proc_mode / f"SMRMSG_text_{post_proc_mode}.txt",
                    precision=5,
                )

        gps_file = pd.read_csv(gps_file_path, sep=",", engine="python", skipinitialspace=True)
        gps_file.columns = gps_file.columns.str.strip()

        position_columns = ["X-ECEF", "Y-ECEF", "Z-ECEF"]
        positions_std_columns = ["SDX-ECEF", "SDY-ECEF", "SDZ-ECEF"]
        position_covariance_columns = ["Cx11", "Cx21", "Cx22", "Cx31", "Cx32", "Cx33"]
        orientation_columns = ["Heading", "Roll", "Pitch"]
        orientations_hrp_std_columns = ["HdngSD", "RollSD", "PitchSD"]
        lla_columns = ["Latitude", "Longitude", "H-Ell"]
        azimuth_column = ["Azimuth"]
        ecef_velocity_columns = ["VX-ECEF", "VY-ECEF", "VZ-ECEF"]
        ecef_velocity_covariance_columns = ["Vel-Cx11", "Vel-Cx21", "Vel-Cx22", "Vel-Cx31", "Vel-Cx32", "Vel-Cx33"]
        statistics_columns = ["AmbStatus", "iFlag", "NS", "PDOP", "Q"]
        acc_bias_columns = ["AccBiasX", "AccBiasY", "AccBiasZ"]
        gyro_drift_columns = ["GyroDriftX", "GyroDriftY", "GyroDriftZ"]
        COG_column = ["COG"]

        times = gps_file.iloc[:, 0]
        positions = gps_file[position_columns].to_numpy(dtype=np.float64, copy=False)
        orientations_hrp = gps_file[orientation_columns].to_numpy(dtype=np.float64, copy=False)
        positions_std = gps_file[positions_std_columns].to_numpy(dtype=np.float64, copy=False)
        orientations_hrp_std = gps_file[orientations_hrp_std_columns].to_numpy(dtype=np.float64, copy=False)

        position_covariance = gps_file[position_covariance_columns].to_numpy(dtype=np.float64, copy=False)
        lla = gps_file[lla_columns].to_numpy(copy=False)
        # IE writes latitude/longitude as DMS text. Validate the parsed numeric
        # values, not the object array (np.isfinite rejects valid DMS strings).
        lat_deg_ie_arr = np.array([dms_string_to_decimal(s) for s in lla[:, 0]], dtype=np.float64)
        lon_deg_ie_arr = np.array([dms_string_to_decimal(s) for s in lla[:, 1]], dtype=np.float64)
        alt_ie_arr = np.asarray(lla[:, 2], dtype=np.float64)
        lla_numeric = np.column_stack((lat_deg_ie_arr, lon_deg_ie_arr, alt_ie_arr))
        azimuth = gps_file[azimuth_column].to_numpy(copy=False)
        ecef_velocity = gps_file[ecef_velocity_columns].to_numpy(dtype=np.float64, copy=False)
        ecef_velocity_covariance = gps_file[ecef_velocity_covariance_columns].to_numpy(dtype=np.float64, copy=False)
        statistics = gps_file[statistics_columns].to_numpy(copy=False)
        acc_bias = gps_file[acc_bias_columns].to_numpy(dtype=np.float64, copy=False)
        gyro_drift = gps_file[gyro_drift_columns].to_numpy(dtype=np.float64, copy=False)
        cog = gps_file[COG_column].to_numpy(dtype=np.float64, copy=False)

        for name, values in (
            ("position", positions),
            ("attitude", orientations_hrp),
            ("position SD", positions_std),
            ("attitude SD", orientations_hrp_std),
            ("latitude/longitude/height", lla_numeric),
            ("velocity", ecef_velocity),
            ("acceleration bias", acc_bias),
            ("gyro drift", gyro_drift),
        ):
            if not np.isfinite(values).all():
                raise ValueError(f"IE {name} contains nonfinite values")
        if np.any(positions_std < 0) or np.any(orientations_hrp_std < 0):
            raise ValueError("IE standard deviations must be nonnegative")

        # Check: SD^2 vs diag(Cov) consistency (ECEF position)
        try:
            sd2 = np.square(positions_std.astype(np.float64, copy=False))
            cov_diag = np.stack(
                [
                    position_covariance[:, 0],
                    position_covariance[:, 2],
                    position_covariance[:, 5],
                ],
                axis=1,
            ).astype(np.float64, copy=False)
            max_abs = float(np.max(np.abs(sd2 - cov_diag)))
            print(f"[CHECK] {post_proc_mode}: max |SD^2 - diag(Cov)| = {max_abs:.6e}")
        except Exception as e:
            print(f"Warning: SD^2 vs Cov diag check failed: {e}")

        # Robust timestamp split (no string parsing)
        t_arr = times.to_numpy(dtype=np.float64, copy=False)
        if not len(t_arr) or not np.isfinite(t_arr).all() or np.any(np.diff(t_arr) <= 0):
            raise ValueError("IE timestamps must be finite, nonempty and strictly increasing")
        secs_arr = np.floor(t_arr).astype(np.int64)
        nsecs_arr = np.round((t_arr - secs_arr) * 1e9).astype(np.int64)
        carry = nsecs_arr // 1_000_000_000
        secs_arr += carry
        nsecs_arr -= carry * 1_000_000_000

        utils = GPS_utils()

        # Use the first N measurements for the origin calculation
        N = 10
        filtered_positions = []
        for pos, stat in zip(positions, statistics):
            quality0 = int(stat[4])
            num_satellites0 = int(stat[2])
            pdop0 = float(stat[3])
            if quality0 < 3 and num_satellites0 > 1 and pdop0 < 4:
                filtered_positions.append(pos)

        if len(filtered_positions) < N:
            print(f"Warning: Only {len(filtered_positions)} positions meet quality criteria. Need at least {N}.")
            if len(positions) < N:
                print(
                    f"Warning: Skipping {post_proc_mode}: found only {len(positions)} position measurements, "
                    f"but need at least {N}."
                )
                continue
            first_n_positions = positions[:N]
            print(f"Falling back to using first {N} positions without quality filtering.")
        else:
            first_n_positions = np.asarray(filtered_positions[:N], dtype=np.float64)
            print(f"Using {N} high-quality positions for origin calculation.")

        lat_long_h_list = [utils.ecef2geo(pos[0], pos[1], pos[2]) for pos in first_n_positions]
        avg_lat = float(np.mean([lla0[0] for lla0 in lat_long_h_list]))
        avg_long = float(np.mean([lla0[1] for lla0 in lat_long_h_list]))
        avg_h = float(np.mean([lla0[2] for lla0 in lat_long_h_list]))
        lat_long_h = (avg_lat, avg_long, avg_h)
        print(f"Using average of selected points for origin: {lat_long_h}")

        utils.setENUorigin(lat_long_h[0], lat_long_h[1], lat_long_h[2])
        start_time = None

        gps_path_msg = PathRos()
        gps_path_msg.header.frame_id = frame_id

        # Build a consistent ENU-at-origin frame for pose/odom AND for projecting ECEF velocity.
        origin_lat_rad = math.radians(float(lat_long_h[0]))
        origin_lon_rad = math.radians(float(lat_long_h[1]))
        R_ecef2enu_origin = ecef_to_enu_rotation(origin_lat_rad, origin_lon_rad)
        Rt_origin = R_ecef2enu_origin.T

        # CRITICAL FIX: force origin_ecef to be shape (3,), not (3,1) etc.
        ox, oy, oz = utils.geo2ecef(lat_long_h[0], lat_long_h[1], lat_long_h[2])
        origin_ecef = np.array([_scalar(ox), _scalar(oy), _scalar(oz)], dtype=np.float64).reshape(
            3,
        )

        topic_prefix = f"/gt_box/inertial_explorer/{post_proc_mode}"
        topic_acc_bias = f"{topic_prefix}/cpt7_imu_acc_bias"
        topic_gyro_drift = f"{topic_prefix}/cpt7_imu_gyro_drift"
        topic_origin = f"{topic_prefix}/origin"
        topic_navsat_origin = f"{topic_prefix}/navsatfix_origin"
        topic_raw = f"{topic_prefix}/raw"
        topic_navsat = f"{topic_prefix}/navsatfix"
        topic_odom = f"{topic_prefix}/odometry"
        topic_path = f"{topic_prefix}/gps_path"
        topic_pose_cov = f"{topic_prefix}/pose_with_covariance"

        # Export robot heading (degrees) as a separate topic in the output rosbag.
        topic_heading_deg = f"{topic_prefix}/robot_heading_deg"

        quality_times = []
        quality_values = []
        satellites_values = []
        ambiguity_values = []

        cov6_pose = np.zeros((6, 6), dtype=np.float64)
        cov6_vel = np.zeros((6, 6), dtype=np.float64)

        n = len(t_arr)
        t_rel = (t_arr - t_arr[0]).astype(np.float64, copy=False)

        pos_pub_arr = np.zeros((n, 3), dtype=np.float64)
        pos_std_arr = np.zeros((n, 3), dtype=np.float64)

        vel_pub_arr = np.zeros((n, 3), dtype=np.float64)
        vel_std_arr = np.zeros((n, 3), dtype=np.float64)

        # Minimal checks retained (extra diagnostics removed)
        yaw_cog_t = []
        yaw_cog_diff_deg = []
        heading_cog_diff_deg = []
        yaw_heading_diff_deg = []
        cogvel_cogcol_diff_deg = []
        reverse_moving_n = 0
        reverse_n = 0

        with atomic_output(bag_path) as temporary, rosbag.Bag(str(temporary), "w", compression="lz4") as bag:
            bag_write = bag.write

            iterable = zip(
                times.to_numpy(copy=False),
                positions,
                orientations_hrp,
                positions_std,
                orientations_hrp_std,
                position_covariance,
                azimuth[:, 0],
                ecef_velocity,
                ecef_velocity_covariance,
                statistics,
                acc_bias,
                gyro_drift,
                cog[:, 0],
            )
            if tqdm is not None:
                iterable = tqdm(iterable, total=len(times), desc=f"Writing {post_proc_mode}", unit="msg", leave=False)

            for i, (
                time_val,
                position,
                orientation_hrp,
                position_std,
                orientation_hrp_std,
                position_cov,
                _azimuth_val,
                ecef_vel,
                ecef_vel_cov,
                stat,
                acc_b,
                gyro_d,
                cog_value,
            ) in enumerate(iterable):
                timestamp = rospy.Time(secs=int(secs_arr[i]), nsecs=int(nsecs_arr[i]))
                num_satellites = int(stat[2])

                acc_bias_msg = Vector3Stamped()
                acc_bias_msg.header.seq = i
                acc_bias_msg.header.stamp = timestamp
                acc_bias_msg.header.frame_id = frame_id
                acc_bias_msg.vector = Vector3(x=float(acc_b[0]), y=float(acc_b[1]), z=float(acc_b[2]))

                gyro_drift_msg = Vector3Stamped()
                gyro_drift_msg.header.seq = i
                gyro_drift_msg.header.stamp = timestamp
                gyro_drift_msg.header.frame_id = frame_id
                gyro_drift_msg.vector = Vector3(x=float(gyro_d[0]), y=float(gyro_d[1]), z=float(gyro_d[2]))

                bag_write(topic=topic_acc_bias, msg=acc_bias_msg, t=timestamp)
                bag_write(topic=topic_gyro_drift, msg=gyro_drift_msg, t=timestamp)

                ambiguity_status = str(stat[0]).strip()

                _pdop = round(float(stat[3]))
                quality = int(stat[4])
                quality_times.append(timestamp.to_sec())
                quality_values.append(quality)
                satellites_values.append(num_satellites)
                ambiguity_values.append(map_ambiguity_status(ambiguity_status))

                # True ECEF velocities (vx,vy,vz in ECEF Cartesian)
                ecef_vx = float(ecef_vel[0])
                ecef_vy = float(ecef_vel[1])
                ecef_vz = float(ecef_vel[2])

                cov_ecef_vel = compact_cov_to_full(ecef_vel_cov)
                ecef_vel_std_x = math.sqrt(max(float(cov_ecef_vel[0, 0]), 0.0))
                ecef_vel_std_y = math.sqrt(max(float(cov_ecef_vel[1, 1]), 0.0))
                ecef_vel_std_z = math.sqrt(max(float(cov_ecef_vel[2, 2]), 0.0))
                velocity_ecef_std = Vector3(x=ecef_vel_std_x, y=ecef_vel_std_y, z=ecef_vel_std_z)

                msg = GnssRaw()
                msg.header.seq = i
                msg.header.stamp = timestamp
                msg.header.frame_id = frame_id

                msg.position_ecef.x = float(position[0])
                msg.position_ecef.y = float(position[1])
                msg.position_ecef.z = float(position[2])

                msg.position_ecef_std.x = float(position_std[0])
                msg.position_ecef_std.y = float(position_std[1])
                msg.position_ecef_std.z = float(position_std[2])

                msg.velocity_ecef.x = ecef_vx
                msg.velocity_ecef.y = ecef_vy
                msg.velocity_ecef.z = ecef_vz

                msg.velocity_ecef_std = velocity_ecef_std

                msg.orientation_hrp.x = float(orientation_hrp[0])
                msg.orientation_hrp.y = float(orientation_hrp[1])
                msg.orientation_hrp.z = float(orientation_hrp[2])

                msg.orientation_hrp_std.x = float(orientation_hrp_std[0])
                msg.orientation_hrp_std.y = float(orientation_hrp_std[1])
                msg.orientation_hrp_std.z = float(orientation_hrp_std[2])

                msg.number_of_satellites.data = num_satellites
                msg.quality_indicator.data = quality
                msg.position_dilution_of_precision.data = float(stat[3])
                msg.course_over_ground.data = float(cog_value)

                if ambiguity_status.lower() == "fixed":
                    msg.ambiguity_status.data = GnssRaw.FIXED
                elif ambiguity_status.lower() == "float":
                    msg.ambiguity_status.data = GnssRaw.FLOAT

                lat_deg, lon_deg, alt = utils.ecef2geo(float(position[0]), float(position[1]), float(position[2]))

                lat_deg_ie = float(lat_deg_ie_arr[i])
                lon_deg_ie = float(lon_deg_ie_arr[i])
                alt_ie = float(alt_ie_arr[i])

                try:
                    x2, y2, z2 = utils.geo2ecef(lat_deg_ie, lon_deg_ie, alt_ie)
                    x2s = _scalar(x2)
                    y2s = _scalar(y2)
                    z2s = _scalar(z2)
                    diff_x = float(position[0]) - x2s
                    diff_y = float(position[1]) - y2s
                    diff_z = float(position[2]) - z2s
                    distance = math.sqrt(diff_x**2 + diff_y**2 + diff_z**2)
                    if distance > 0.01:
                        raise ValueError(
                            f"ECEF to geographic conversion mismatch: {distance:.6f}m difference between original and IE data"
                        )
                except Exception as e:
                    print(f"Warning: Unable to verify consistency between ECEF and geographic coordinates: {e}")

                navsat_msg = NavSatFix()
                navsat_msg.header.seq = i
                navsat_msg.header.stamp = timestamp
                navsat_msg.header.frame_id = FRAME_ID_BODY

                navsat_msg.status.service = NavSatStatus.SERVICE_GPS
                if ambiguity_status.lower() == "fixed":
                    navsat_msg.status.status = NavSatStatus.STATUS_GBAS_FIX
                elif ambiguity_status.lower() == "float":
                    navsat_msg.status.status = NavSatStatus.STATUS_SBAS_FIX
                else:
                    navsat_msg.status.status = NavSatStatus.STATUS_FIX

                navsat_msg.latitude = float(lat_deg_ie)
                navsat_msg.longitude = float(lon_deg_ie)
                navsat_msg.altitude = float(alt_ie)

                cov_ecef = compact_cov_to_full(position_cov)

                # NavSatFix requires ENU at THIS fix, whereas pose/odometry use
                # one fixed ENU origin. These tangent frames differ along a route.
                rotation_at_fix = ecef_to_enu_rotation(math.radians(float(lat_deg_ie)), math.radians(float(lon_deg_ie)))
                cov_enu_fix = rotation_at_fix @ cov_ecef @ rotation_at_fix.T

                navsat_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_KNOWN
                navsat_msg.position_covariance = cov_enu_fix.flatten().tolist()

                if i == 0:
                    bag_write(topic=topic_origin, msg=msg, t=timestamp)
                    bag_write(topic=topic_navsat_origin, msg=navsat_msg, t=timestamp)

                bag_write(topic=topic_raw, msg=msg, t=timestamp)
                bag_write(topic=topic_navsat, msg=navsat_msg, t=timestamp)

                if start_time is None:
                    start_time = float(time_val)

                # Pose position in ENU at origin (consistent with frame_id name)
                p_ecef = np.asarray(position, dtype=np.float64).reshape(
                    3,
                )
                p_enu = R_ecef2enu_origin.dot(p_ecef - origin_ecef).reshape(
                    3,
                )
                pos_pub = p_enu.astype(np.float64).reshape(
                    3,
                )  # extra safety

                # Build BODY->WORLD(ENU) rotation from IE heading/roll/pitch (with NED->ENU if enabled).
                rot_wb = build_rot_wb_from_ie_hrp(
                    heading_deg=float(orientation_hrp[0]),
                    roll_deg=float(orientation_hrp[1]),
                    pitch_deg=float(orientation_hrp[2]),
                )
                quaternion_xyzw = rot_wb.as_quat()

                # Robot heading (degrees): yaw of BODY +X in WORLD(ENU), computed from the published quaternion.
                f_w = rot_wb.apply([1.0, 0.0, 0.0])
                yaw_est_deg = (math.degrees(math.atan2(float(f_w[0]), float(f_w[1]))) + 360.0) % 360.0

                heading_msg = Float64()
                heading_msg.data = float(yaw_est_deg)
                bag_write(topic=topic_heading_deg, msg=heading_msg, t=timestamp)

                output_msg = PoseWithCovarianceStamped()
                output_msg.header.seq = i
                output_msg.header.stamp = timestamp
                output_msg.header.frame_id = frame_id

                # Covariance in ENU-at-origin
                cov_enu_pos_origin = R_ecef2enu_origin.dot(cov_ecef).dot(Rt_origin)

                cov6_pose.fill(0.0)
                cov6_pose[:3, :3] = cov_enu_pos_origin.astype(np.float64, copy=False)
                cov6_pose[3:, 3:] = orientation_covariance_enu(*orientation_hrp, orientation_hrp_std)

                output_msg.pose.pose.position = Point(
                    x=float(pos_pub[0]),
                    y=float(pos_pub[1]),
                    z=float(pos_pub[2]),
                )
                output_msg.pose.pose.orientation = Quaternion(
                    x=float(quaternion_xyzw[0]),
                    y=float(quaternion_xyzw[1]),
                    z=float(quaternion_xyzw[2]),
                    w=float(quaternion_xyzw[3]),
                )

                pose_cov_list = cov6_pose.flatten().tolist()
                output_msg.pose.covariance = pose_cov_list

                bag_write(topic=topic_pose_cov, msg=output_msg, t=timestamp)

                # Velocity: interpret VX-ECEF/VY-ECEF/VZ-ECEF as TRUE ECEF, rotate to ENU@origin.
                v_ecef = np.array([ecef_vx, ecef_vy, ecef_vz], dtype=np.float64).reshape(
                    3,
                )
                v_enu = R_ecef2enu_origin.dot(v_ecef).reshape(
                    3,
                )
                cov_enu_vel_origin = R_ecef2enu_origin.dot(cov_ecef_vel).dot(Rt_origin)

                # Minimal consistency checks (kept)
                speed_h = math.hypot(float(v_enu[0]), float(v_enu[1]))
                if speed_h > 0.5:
                    diff = ((yaw_est_deg - float(cog_value) + 180.0) % 360.0) - 180.0
                    yaw_cog_t.append(float(t_rel[i]))
                    yaw_cog_diff_deg.append(float(diff))

                    heading_deg_col = float(orientation_hrp[0])
                    diff_a = ((heading_deg_col - float(cog_value) + 180.0) % 360.0) - 180.0
                    heading_cog_diff_deg.append(float(diff_a))

                    diff_b = ((yaw_est_deg - heading_deg_col + 180.0) % 360.0) - 180.0
                    yaw_heading_diff_deg.append(float(diff_b))

                    cog_from_vel_deg = (math.degrees(math.atan2(float(v_enu[0]), float(v_enu[1]))) + 360.0) % 360.0
                    diff_c = ((cog_from_vel_deg - float(cog_value) + 180.0) % 360.0) - 180.0
                    cogvel_cogcol_diff_deg.append(float(diff_c))

                    v_body_check = rot_wb.inv().apply(v_enu)
                    reverse_moving_n += 1
                    if float(v_body_check[0]) < 0.0:
                        reverse_n += 1

                velocity_msg = TwistWithCovariance()

                if PUBLISH_TWIST_IN_BODY_FRAME:
                    v_body = rot_wb.inv().apply(v_enu)
                    R_bw = rot_wb.inv().as_matrix()
                    cov_v_body = R_bw.dot(cov_enu_vel_origin).dot(R_bw.T)

                    velocity_msg.twist.linear = Vector3(x=float(v_body[0]), y=float(v_body[1]), z=float(v_body[2]))
                    velocity_msg.twist.angular = Vector3(x=0, y=0, z=0)

                    cov6_vel.fill(0.0)
                    cov6_vel[:3, :3] = cov_v_body.astype(np.float64, copy=False)

                    vel_pub = v_body
                    vel_var_diag = np.diag(cov_v_body)
                else:
                    velocity_msg.twist.linear = Vector3(x=float(v_enu[0]), y=float(v_enu[1]), z=float(v_enu[2]))
                    velocity_msg.twist.angular = Vector3(x=0, y=0, z=0)

                    cov6_vel.fill(0.0)
                    cov6_vel[:3, :3] = cov_enu_vel_origin.astype(np.float64, copy=False)

                    vel_pub = v_enu
                    vel_var_diag = np.diag(cov_enu_vel_origin)

                cov6_vel[3, 3] = float(ANG_COV_UNKNOWN)
                cov6_vel[4, 4] = float(ANG_COV_UNKNOWN)
                cov6_vel[5, 5] = float(ANG_COV_UNKNOWN)
                velocity_msg.covariance = cov6_vel.flatten().tolist()

                odometry_msg = Odometry()
                odometry_msg.header = output_msg.header
                odometry_msg.pose.pose = output_msg.pose.pose
                odometry_msg.pose.covariance = pose_cov_list
                odometry_msg.twist = velocity_msg
                odometry_msg.child_frame_id = FRAME_ID_BODY if PUBLISH_TWIST_IN_BODY_FRAME else frame_id
                bag_write(topic=topic_odom, msg=odometry_msg, t=timestamp)

                path_pose = PoseStamped()
                path_pose.header = output_msg.header
                path_pose.pose = output_msg.pose.pose
                gps_path_msg.poses.append(path_pose)
                if i % 200 == 0:
                    bag_write(topic=topic_path, msg=gps_path_msg, t=timestamp)

                def quat_normalize_xyzw(q):
                    n = math.sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3])
                    if n == 0.0:
                        raise ValueError("Zero-norm quaternion; cannot invert transform.")
                    return [q[0] / n, q[1] / n, q[2] / n, q[3] / n]

                def quat_conj_xyzw(q):
                    # for unit quaternion, conjugate == inverse
                    return [-q[0], -q[1], -q[2], q[3]]

                def quat_mul_xyzw(a, b):
                    ax, ay, az, aw = a
                    bx, by, bz, bw = b
                    return [
                        aw * bx + ax * bw + ay * bz - az * by,  # x
                        aw * by - ax * bz + ay * bw + az * bx,  # y
                        aw * bz + ax * by - ay * bx + az * bw,  # z
                        aw * bw - ax * bx - ay * by - az * bz,  # w
                    ]

                def quat_rotate_vec_xyzw(q, v):
                    # v' = q * (v,0) * conj(q)
                    q_conj = quat_conj_xyzw(q)
                    v_quat = [v[0], v[1], v[2], 0.0]
                    tmp = quat_mul_xyzw(q, v_quat)
                    out = quat_mul_xyzw(tmp, q_conj)
                    return [out[0], out[1], out[2]]

                # --- Invert TF: original is parent=frame_id, child=FRAME_ID_BODY ---
                t = [float(pos_pub[0]), float(pos_pub[1]), float(pos_pub[2])]
                q = quat_normalize_xyzw(
                    [
                        float(quaternion_xyzw[0]),
                        float(quaternion_xyzw[1]),
                        float(quaternion_xyzw[2]),
                        float(quaternion_xyzw[3]),
                    ]
                )

                q_inv = quat_conj_xyzw(q)  # inverse rotation
                t_inv = quat_rotate_vec_xyzw(q_inv, [-t[0], -t[1], -t[2]])  # -R^T t

                tf_message = TFMessage()
                tf_message.transforms = []

                inv_transform = TransformStamped()
                inv_transform.header.stamp = odometry_msg.header.stamp
                inv_transform.header.frame_id = FRAME_ID_BODY
                inv_transform.child_frame_id = frame_id

                inv_transform.transform.translation = Vector3(
                    x=t_inv[0],
                    y=t_inv[1],
                    z=t_inv[2],
                )
                inv_transform.transform.rotation = Quaternion(
                    x=q_inv[0],
                    y=q_inv[1],
                    z=q_inv[2],
                    w=q_inv[3],
                )

                tf_message.transforms.append(inv_transform)
                bag_write(topic="/tf", msg=tf_message, t=timestamp)

                # Store plotting data (matches what you publish)
                pos_pub_arr[i, :] = pos_pub
                pos_std_arr[i, :] = np.sqrt(np.maximum(np.diag(cov_enu_pos_origin), 0.0))

                vel_pub_arr[i, :] = vel_pub
                vel_std_arr[i, :] = np.sqrt(
                    np.maximum(
                        np.asarray(vel_var_diag).reshape(
                            3,
                        ),
                        0.0,
                    )
                )

        # Minimal summaries retained (extra diagnostics removed)
        try:
            if len(yaw_cog_diff_deg) > 0:
                d = np.asarray(yaw_cog_diff_deg, dtype=np.float64)
                abs_d = np.abs(d)
                p50 = float(np.percentile(abs_d, 50))
                p95 = float(np.percentile(abs_d, 95))
                mean_abs = float(np.mean(abs_d))
                max_abs = float(np.max(abs_d))
                print(
                    f"[CHECK] {post_proc_mode}: |yaw_est - COG| (deg), speed_h>0.5m/s: "
                    f"N={len(d)} mean={mean_abs:.3f} p50={p50:.3f} p95={p95:.3f} max={max_abs:.3f}"
                )
            else:
                print(f"[CHECK] {post_proc_mode}: COG vs yaw check skipped (no samples with speed_h>0.5 m/s)")
        except Exception as e:
            print(f"Warning: COG vs yaw check failed: {e}")

        try:
            if len(heading_cog_diff_deg) > 0:
                d = np.asarray(heading_cog_diff_deg, dtype=np.float64)
                abs_d = np.abs(d)
                p50 = float(np.percentile(abs_d, 50))
                p95 = float(np.percentile(abs_d, 95))
                mean_abs = float(np.mean(abs_d))
                max_abs = float(np.max(abs_d))
                print(
                    f"[CHECK-A] {post_proc_mode}: |Heading - COG| (deg), speed_h>0.5m/s: "
                    f"N={len(d)} mean={mean_abs:.3f} p50={p50:.3f} p95={p95:.3f} max={max_abs:.3f}"
                )
            else:
                print(f"[CHECK-A] {post_proc_mode}: Heading vs COG check skipped (no samples with speed_h>0.5 m/s)")
        except Exception as e:
            print(f"Warning: Heading vs COG check failed: {e}")

        try:
            if len(yaw_heading_diff_deg) > 0:
                d = np.asarray(yaw_heading_diff_deg, dtype=np.float64)
                abs_d = np.abs(d)
                p50 = float(np.percentile(abs_d, 50))
                p95 = float(np.percentile(abs_d, 95))
                mean_abs = float(np.mean(abs_d))
                max_abs = float(np.max(abs_d))
                print(
                    f"[CHECK-B] {post_proc_mode}: |yaw_est - Heading| (deg), speed_h>0.5m/s: "
                    f"N={len(d)} mean={mean_abs:.3f} p50={p50:.3f} p95={p95:.3f} max={max_abs:.3f}"
                )
            else:
                print(f"[CHECK-B] {post_proc_mode}: yaw_est vs Heading check skipped (no samples with speed_h>0.5 m/s)")
        except Exception as e:
            print(f"Warning: yaw_est vs Heading check failed: {e}")

        try:
            if len(cogvel_cogcol_diff_deg) > 0:
                d = np.asarray(cogvel_cogcol_diff_deg, dtype=np.float64)
                abs_d = np.abs(d)
                p50 = float(np.percentile(abs_d, 50))
                p95 = float(np.percentile(abs_d, 95))
                mean_abs = float(np.mean(abs_d))
                max_abs = float(np.max(abs_d))
                print(
                    f"[CHECK-C] {post_proc_mode}: |COG(v_enu) - COG(col)| (deg), speed_h>0.5m/s: "
                    f"N={len(d)} mean={mean_abs:.3f} p50={p50:.3f} p95={p95:.3f} max={max_abs:.3f}"
                )
            else:
                print(
                    f"[CHECK-C] {post_proc_mode}: COG(v_enu) vs COG(col) check skipped (no samples with speed_h>0.5 m/s)"
                )
        except Exception as e:
            print(f"Warning: COG(v_enu) vs COG(col) check failed: {e}")

        try:
            if reverse_moving_n > 0:
                frac = float(reverse_n) / float(reverse_moving_n)
                print(
                    f"[CHECK-D] {post_proc_mode}: reverse motion (v_body_x<0), speed_h>0.5m/s: "
                    f"N_moving={reverse_moving_n} N_reverse={reverse_n} frac={frac:.6f}"
                )
            else:
                print(f"[CHECK-D] {post_proc_mode}: reverse motion check skipped (no samples with speed_h>0.5 m/s)")
        except Exception as e:
            print(f"Warning: reverse motion check failed: {e}")

        # -----------------------------
        # Plot and save figures (dpi 800)
        # -----------------------------
        fig_dir = (Path(MISSION_DATA) / "ie" / post_proc_mode / "figures").resolve()
        fig_dir.mkdir(parents=True, exist_ok=True)

        plot_biases(
            t_s=t_rel,
            acc_bias=acc_bias,
            gyro_drift=gyro_drift,
            out_dir=fig_dir,
            tag=post_proc_mode,
        )

        plot_xyz_timeseries_with_bands(
            t_s=t_rel,
            xyz=pos_pub_arr,
            xyz_std=pos_std_arr,
            title=f"Trajectory components vs time (ENU @ origin) [{post_proc_mode}]",
            ylabel="pos [m]",
            out_prefix=fig_dir / f"pos_time_{post_proc_mode}",
        )

        vel_frame_tag = "body" if PUBLISH_TWIST_IN_BODY_FRAME else "enu"
        plot_xyz_timeseries_with_bands(
            t_s=t_rel,
            xyz=vel_pub_arr,
            xyz_std=vel_std_arr,
            title=f"Linear velocity vs time ({vel_frame_tag}) [{post_proc_mode}]",
            ylabel="vel [m/s]",
            out_prefix=fig_dir / f"vel_time_{vel_frame_tag}_{post_proc_mode}",
        )

        if len(quality_times) > 0:
            times_plot = np.asarray(quality_times, dtype=np.float64) - float(quality_times[0])

            fig = plt.figure()
            ax = fig.add_subplot(111)
            ax.plot(times_plot, quality_values, marker="o", linestyle="-")
            ax.set_xlabel("Time (s)")
            ax.set_ylabel("Quality Metric")
            ax.set_title(f"Quality Metric vs Time for {post_proc_mode}")
            ax.grid(True)
            if SHOW_FIGS:
                fig.show()
            if SAVE_FIGS:
                _save_fig(fig, fig_dir / f"quality_vs_time_{post_proc_mode}.png")

            fig = plt.figure()
            ax = fig.add_subplot(111)
            ax.plot(times_plot, satellites_values, marker="o", linestyle="-")
            ax.set_xlabel("Time (s)")
            ax.set_ylabel("Number of Satellites")
            ax.set_title(f"Number of Satellites vs Time for {post_proc_mode}")
            ax.grid(True)
            if SHOW_FIGS:
                fig.show()
            if SAVE_FIGS:
                _save_fig(fig, fig_dir / f"satellites_vs_time_{post_proc_mode}.png")

            fig = plt.figure()
            ax = fig.add_subplot(111)
            ax.plot(times_plot, ambiguity_values, marker="o", linestyle="-")
            ax.set_xlabel("Time (s)")
            ax.set_ylabel("Ambiguity Status (Numeric)")
            ax.set_title(f"Ambiguity Status vs Time for {post_proc_mode}")
            ax.grid(True)
            if SHOW_FIGS:
                fig.show()
            if SAVE_FIGS:
                _save_fig(fig, fig_dir / f"ambiguity_vs_time_{post_proc_mode}.png")
        else:
            print(f"Warning: No quality/satellites/ambiguity samples collected for {post_proc_mode}; skipping plots.")


if __name__ == "__main__":
    main()
    exit(0)
