import numpy as np
import pandas as pd
import rosbag
import struct
import rospy
import math
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
from scipy.spatial.transform import Rotation
from pathlib import Path
from nav_msgs.msg import Path as PathRos
from box_auto.utils import MISSION_DATA, GPS_utils

# Used to verify gpsUtils
"""
def ecef2geoLocal(x, y, z):
# pip install pyproj
    from pyproj.transformer import Transformer, AreaOfInterest
    # Convert ECEF coordinates (x, y, z) to geodetic coordinates (latitude, longitude, altitude)
    # using the WGS84 ellipsoid. This function uses PyProj to transform from EPSG:4978 to EPSG:4326.
    # An AreaOfInterest around Switzerland is specified (approximate bounds: west=5.95, south=45.75,
    # east=10.55, north=47.9) to optimize the transformation.
    # Returns:
    #   lat: Latitude in degrees.
    #   lon: Longitude in degrees.
    #   alt: Altitude in meters.
    # Define an area of interest around Switzerland
    aoi = AreaOfInterest(west_lon_degree=5.95, south_lat_degree=45.75, east_lon_degree=10.55, north_lat_degree=47.9)
    transformer = Transformer.from_crs("EPSG:4978", "EPSG:4326", area_of_interest=aoi, always_xy=True)
    lon, lat, alt = transformer.transform(x, y, z)
    return lat, lon, alt
"""


def read_sbet(filepath, num_fields=17):
    record_size = 8 * num_fields  # 8 bytes per double
    records = []

    with open(filepath, "rb") as f:
        while True:
            data = f.read(record_size)
            if not data or len(data) != record_size:
                break
            record = struct.unpack(f"{num_fields}d", data)
            records.append(record)

    return np.array(records)


def write_sbet_to_txt(records, output_path, precision=8, include_header=True):
    # Standard SBET field names (17 fields)
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

    fmt = f"{{:.{precision}f}}"

    with open(output_path, "w") as f:
        if include_header:
            f.write(" ".join(header_fields) + "\n")
        for row in records:
            line = " ".join(fmt.format(val) for val in row)
            f.write(line + "\n")


def ecef_velocity_to_enu(vx, vy, vz, lat_rad, lon_rad):
    sin_lat = np.sin(lat_rad)
    cos_lat = np.cos(lat_rad)
    sin_lon = np.sin(lon_rad)
    cos_lon = np.cos(lon_rad)

    # Rotation matrix from ECEF to ENU (east, north, up)
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
    # Map "Fixed" to 2, "Float" to 1, others to 0.
    s = status_str.lower()
    if s == "fixed":
        return 2
    elif s == "float":
        return 1
    else:
        return 0


def ecef_to_enu_covariance(cov_ecef, lat_rad, lon_rad):
    """
    Convert a 3x3 ECEF covariance matrix (in m^2) to an ENU covariance matrix (in m^2).

    Parameters:
      cov_ecef: 3x3 numpy array containing the covariance in ECEF coordinates.
      lat_rad: Latitude of the point (in radians).
      lon_rad: Longitude of the point (in radians).

    Returns:
      cov_enu: 3x3 numpy array containing the covariance in the ENU frame.
    """
    sin_lat = np.sin(lat_rad)
    cos_lat = np.cos(lat_rad)
    sin_lon = np.sin(lon_rad)
    cos_lon = np.cos(lon_rad)

    # Rotation matrix from ECEF to ENU (east, north, up)
    R = np.array(
        [
            [-sin_lon, cos_lon, 0],
            [-sin_lat * cos_lon, -sin_lat * sin_lon, cos_lat],
            [cos_lat * cos_lon, cos_lat * sin_lon, sin_lat],
        ]
    )
    cov_enu = R.dot(cov_ecef).dot(R.T)
    return cov_enu


def dms_string_to_decimal(dms_str):
    parts = dms_str.strip().split()
    degrees = float(parts[0])
    minutes = float(parts[1])
    seconds = float(parts[2])
    return degrees + minutes / 60 + seconds / 3600


def main():
    """
    Processes a GPS path and converts it into various ROS messages.
    Full Reference of the WGS84 ECEF Coordinate System used for export of the trajectory --- https://docs.novatel.com/OEM7/Content/Logs/BESTXYZ.htm#Figure_WGS84ECEFCoordinateSystem

    NOTE: For consistency across different post-processing methods, it is assumed that the enu_origin always starts at (0, 0, 0) based on the first pose.
    However, this assumption may be incorrect when considering a global ENU frame, as different post-processing methods might initialize at distinct ENU_COORDINATES
    but still treat them as (0, 0, 0) for the enu_origin.

    As a result, conflicting TFs (Transforms) might be published between the ROS bags.
    """
    try:
        date = [(str(s.name)).split("_")[0] for s in Path(MISSION_DATA).glob("*_nuc_livox.bag")][0]
    except:
        date = [(str(s.name)).split("_")[0] for s in Path(MISSION_DATA).glob("*_nuc_hesai.bag")][0]

    gps_files = [str(s) for s in (Path(MISSION_DATA) / "ie").rglob("*tc_GrandTour-LocalFrame-extended.txt")]
    post_proc_modes = [s.split("/")[-1].split("_")[1] for s in gps_files]
    print(post_proc_modes, MISSION_DATA)
    bag_paths = [
        str(Path(s).parent.parent / (date + f"_cpt7_ie_{mode}.bag")) for s, mode in zip(gps_files, post_proc_modes)
    ]

    for gps_file_path, bag_path, post_proc_mode in zip(gps_files, bag_paths, post_proc_modes):

        # Export the IMU SBET data.
        sbet_data_path = Path(MISSION_DATA) / "ie" / post_proc_mode / f"SBET_{post_proc_mode}.OUT"

        # Check if sbet_data file exists
        if not sbet_data_path.exists() or not sbet_data_path.is_file() or sbet_data_path.stat().st_size == 0:
            print(f"Warning: No SBET data found for {post_proc_mode}")
        else:

            sbet_data = read_sbet(Path(MISSION_DATA) / "ie" / post_proc_mode / f"SBET_{post_proc_mode}.OUT")
            SMRMSG_data = read_sbet(Path(MISSION_DATA) / "ie" / post_proc_mode / f"SMRMSG_{post_proc_mode}.OUT")
            write_sbet_to_txt(
                sbet_data, Path(MISSION_DATA) / "ie" / post_proc_mode / f"SBET_text_{post_proc_mode}.txt", precision=5
            )
            write_sbet_to_txt(
                SMRMSG_data,
                Path(MISSION_DATA) / "ie" / post_proc_mode / f"SMRMSG_text_{post_proc_mode}.txt",
                precision=5,
            )

        gps_file = pd.read_csv(gps_file_path)
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
        # AmbStatus: Ambiguity status, fixed or float
        # iFlag: 0 = no fix, 1 = fixed, 2 = float
        # NS: Number of satellites used in the solution
        # PDOP: Position Dilution of Precision
        # Q: Quality of the solution
        # Cog: Course Over Ground -> angle between the North and the direction of travel
        statistics_columns = ["AmbStatus", "iFlag", "NS", "PDOP", "Q"]
        acc_bias_columns = ["AccBiasX", "AccBiasY", "AccBiasZ"]
        gyro_drift_columns = ["GyroDriftX", "GyroDriftY", "GyroDriftZ"]
        COG_column = ["COG"]

        times = gps_file.iloc[:, 0]
        positions = np.array(gps_file[position_columns].to_numpy())
        orientations_hrp = np.array(gps_file[orientation_columns].to_numpy())
        positions_std = np.array(gps_file[positions_std_columns].to_numpy())
        orientations_hrp_std = np.array(gps_file[orientations_hrp_std_columns].to_numpy())

        # New columns
        position_covariance = np.array(gps_file[position_covariance_columns].to_numpy())
        lla = np.array(gps_file[lla_columns].to_numpy())
        azimuth = np.array(gps_file[azimuth_column].to_numpy())
        ecef_velocity = np.array(gps_file[ecef_velocity_columns].to_numpy())
        ecef_velocity_covariance = np.array(gps_file[ecef_velocity_covariance_columns].to_numpy())
        statistics = np.array(gps_file[statistics_columns].to_numpy())
        acc_bias = np.array(gps_file[acc_bias_columns].to_numpy())
        gyro_drift = np.array(gps_file[gyro_drift_columns].to_numpy())
        cog = np.array(gps_file[COG_column].to_numpy())

        utils = GPS_utils()

        # Use the first N measurements for the origin calculation
        N = 10
        filtered_positions = []
        for i, (pos, stat) in enumerate(zip(positions, statistics)):
            quality = int(stat[4])  # Q is at index 4
            num_satellites = int(stat[2])  # NS is at index 2
            pdop = float(stat[3])  # PDOP is at index 3

            if quality < 3 and num_satellites > 1 and pdop < 4:
                filtered_positions.append(pos)

        if len(filtered_positions) < N:
            print(f"Warning: Only {len(filtered_positions)} positions meet quality criteria. Need at least {N}.")
            if len(positions) < N:
                raise ValueError(f"Not enough position measurements. Found {len(positions)}, but need at least {N}.")
            first_n_positions = positions[:N]
            print(f"Falling back to using first {N} positions without quality filtering.")
        else:
            first_n_positions = filtered_positions[:N]
            print(f"Using {N} high-quality positions for origin calculation.")

        lat_long_h_list = [utils.ecef2geo(pos[0], pos[1], pos[2]) for pos in first_n_positions]
        avg_lat = np.mean([lla[0] for lla in lat_long_h_list])
        avg_long = np.mean([lla[1] for lla in lat_long_h_list])
        avg_h = np.mean([lla[2] for lla in lat_long_h_list])
        lat_long_h = (avg_lat, avg_long, avg_h)
        print(f"Using average of selected points for origin: {lat_long_h}")

        utils.setENUorigin(lat_long_h[0], lat_long_h[1], lat_long_h[2])
        start_time = None

        frame_id = "enu_origin"
        gps_path_msg = PathRos()
        gps_path_msg.header.frame_id = frame_id

        R_enu__ned = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]])

        quality_times = []
        quality_values = []
        satellites_values = []
        ambiguity_values = []

        with rosbag.Bag(bag_path, "w", compression="lz4") as bag:
            for i, (
                time,
                position,
                orientation_hrp,
                position_std,
                orientation_hrp_std,
                position_cov,
                lla_point,
                azimuth_val,
                ecef_vel,
                ecef_vel_cov,
                stat,
                acc_b,
                gyro_d,
                cog_value,
            ) in enumerate(
                zip(
                    times,
                    positions,
                    orientations_hrp,
                    positions_std,
                    orientations_hrp_std,
                    position_covariance,
                    lla,
                    azimuth,
                    ecef_velocity,
                    ecef_velocity_covariance,
                    statistics,
                    acc_bias,
                    gyro_drift,
                    cog,
                )
            ):
                secs = int(time)
                nsecs = str(time).split(".")[1]
                nsecs = int(nsecs + "0" * (9 - len(nsecs)))
                timestamp = rospy.Time(secs=secs, nsecs=nsecs)

                num_satellites = int(stat[2])

                if num_satellites < 1:
                    print("\033[31mError: Number of satellites is less than 1\033[0m")
                #     continue

                # Create Vector3Stamped messages for acceleration bias and gyro drift
                acc_bias_msg = Vector3Stamped()
                acc_bias_msg.header.seq = i
                acc_bias_msg.header.stamp = timestamp
                acc_bias_msg.header.frame_id = frame_id
                acc_bias_msg.vector = Vector3(x=acc_b[0], y=acc_b[1], z=acc_b[2])

                gyro_drift_msg = Vector3Stamped()
                gyro_drift_msg.header.seq = i
                gyro_drift_msg.header.stamp = timestamp
                gyro_drift_msg.header.frame_id = frame_id
                gyro_drift_msg.vector = Vector3(x=gyro_d[0], y=gyro_d[1], z=gyro_d[2])

                # Write acceleration bias to the bag
                bag.write(
                    topic=f"/gt_box/inertial_explorer/{post_proc_mode}/cpt7_imu_acc_bias",
                    msg=acc_bias_msg,
                    t=timestamp,
                )

                # Write gyro drift to the bag
                bag.write(
                    topic=f"/gt_box/inertial_explorer/{post_proc_mode}/cpt7_imu_gyro_drift",
                    msg=gyro_drift_msg,
                    t=timestamp,
                )

                # extract the statistics
                # AmbStatus: Ambiguity status, fixed or float
                # iFlag: 0 = no fix, 1 = fixed, 2 = float
                # NS: Number of satellites used in the solution
                # PDOP: Position Dilution of Precision
                # Q: Quality of the solution
                # statistics_columns = ["AmbStatus", "iFlag", "NS", "PDOP", "Q"]
                ambiguity_status = stat[0].strip()  # Remove leading/trailing whitespace
                # i_flag = int(stat[1]) # imu_status

                # TODO here we are rounding the PDOP value to an integer, but it should be a float
                pdop = round(float(stat[3]))
                quality = int(stat[4])
                quality_times.append(timestamp.to_sec())
                quality_values.append(quality)
                satellites_values.append(num_satellites)
                ambiguity_values.append(map_ambiguity_status(ambiguity_status))
                # PDOP is a unitless number which indicates how favorable the satellite geometry is to 3D
                # positioning accuracy. A strong satellite geometry, where the PDOP is low, occurs when
                # satellites are well distributed in each direction (north, south, east and west) as well as
                # directly overhead.
                # Values in the range of 1-2 indicate very good satellite geometry, 2-3 are adequate in the
                # sense that they do not generally, by themselves, limit positioning accuracy. Values
                # between 3-4 are considered marginal and values approaching or exceeding 5 are
                # considered poor

                # ambiguity_status:
                # This plot indicates where the processed solution is fixed (in one or both directions) or
                # float. If both forward and reverse solutions achieved a fix, the plot shows a value of 2 and
                # is plotted in bright green. If either the forward or reverse achieved a fix, but not both, a
                # value of 1 is plotted. The value will be plotted cyan if the fixed direction is forward and blue
                # if the fixed direction is reverse. If neither direction achieved a fix, a value of 0 is plotted
                # which appears red on the plot.

                # ------------------------------------------------------------------------------
                #
                # +----------+---------+----------------------------------------------+------------------+
                # | Quality  |  Color  | Description                                  | 3D Accuracy (m)  |
                # +----------+---------+----------------------------------------------+------------------+
                # | 1        | Green   | Fixed integer                                | 0.00 – 0.15      |
                # | 2        | Cyan    | Converged float or noisy fixed integer       | 0.05 – 0.40      |
                # | 3        | Blue    | Converging float                             | 0.20 – 1.00      |
                # | 4        | Purple  | Converging float                             | 0.50 – 2.00      |
                # | 5        | Magenta | DGPS                                         | 1.00 – 3.00      |
                # | 6        | Red     | DGPS                                         | 2.00 – 10.00     |
                # |          | Grey    | Has not been processed                       | N/A              |
                # +----------+---------+----------------------------------------------+------------------+
                #
                # Notes:
                # - Quality = 1 typically indicates an RTK fixed solution.
                # - Quality = 2 often indicates an RTK float or partially converged fixed solution.
                # - DGPS is differential GNSS (e.g., SBAS or local corrections).
                # ------------------------------------------------------------------------------

                # Extract ECEF velocity components
                ecef_vx = ecef_vel[0]
                ecef_vy = ecef_vel[1]
                ecef_vz = ecef_vel[2]

                # Build ECEF velocity covariance matrix
                cov_ecef_vel = np.array(
                    [
                        [ecef_vel_cov[0], ecef_vel_cov[1], ecef_vel_cov[3]],  # Vel-Cx11, Vel-Cx21, Vel-Cx31
                        [ecef_vel_cov[1], ecef_vel_cov[2], ecef_vel_cov[4]],  # Vel-Cx21, Vel-Cx22, Vel-Cx32
                        [ecef_vel_cov[3], ecef_vel_cov[4], ecef_vel_cov[5]],  # Vel-Cx31, Vel-Cx32, Vel-Cx33
                    ]
                )

                # Extract standard deviations from ECEF velocity covariance matrix
                # Standard deviation is the square root of the variance (diagonal elements of covariance matrix)
                ecef_vel_std_x = math.sqrt(cov_ecef_vel[0, 0])  # Standard deviation in x-direction
                ecef_vel_std_y = math.sqrt(cov_ecef_vel[1, 1])  # Standard deviation in y-direction
                ecef_vel_std_z = math.sqrt(cov_ecef_vel[2, 2])  # Standard deviation in z-direction

                # Store velocity standard deviations for later use in messages
                velocity_ecef_std = Vector3(x=ecef_vel_std_x, y=ecef_vel_std_y, z=ecef_vel_std_z)

                # --- Create GnssRaw message ---
                # This is our custom message type containing everything
                msg = GnssRaw()
                msg.header.seq = i
                msg.header.stamp = timestamp
                msg.header.frame_id = frame_id

                msg.position_ecef.x = position[0]
                msg.position_ecef.y = position[1]
                msg.position_ecef.z = position[2]

                msg.position_ecef_std.x = position_std[0]
                msg.position_ecef_std.y = position_std[1]
                msg.position_ecef_std.z = position_std[2]

                msg.velocity_ecef.x = ecef_vx
                msg.velocity_ecef.y = ecef_vy
                msg.velocity_ecef.z = ecef_vz

                msg.velocity_ecef_std = velocity_ecef_std

                msg.orientation_hrp.x = orientation_hrp[0]
                msg.orientation_hrp.y = orientation_hrp[1]
                msg.orientation_hrp.z = orientation_hrp[2]

                msg.orientation_hrp_std.x = orientation_hrp_std[0]
                msg.orientation_hrp_std.y = orientation_hrp_std[1]
                msg.orientation_hrp_std.z = orientation_hrp_std[2]

                msg.number_of_satellites.data = num_satellites
                msg.quality_indicator.data = quality
                msg.position_dilution_of_precision.data = float(stat[3])
                msg.course_over_ground.data = float(cog_value)
                msg.ambiguity_status.data = str(ambiguity_status)

                # --- Create NavSatFix message ---
                # Convert current ECEF position to geodetic (lat, lon, alt) using WGS84
                lat_deg, lon_deg, alt = utils.ecef2geo(position[0], position[1], position[2])

                lat_deg_ie = dms_string_to_decimal(lla_point[0])
                lon_deg_ie = dms_string_to_decimal(lla_point[1])
                alt_ie = lla_point[2]

                # Convert IE geographic coordinates back to ECEF for comparison
                try:
                    x2, y2, z2 = utils.geo2ecef(lat_deg_ie, lon_deg_ie, alt_ie)  # ECEF from IE data

                    # Calculate Euclidean distance between the two ECEF points
                    diff_x = position[0] - x2
                    diff_y = position[1] - y2
                    diff_z = position[2] - z2
                    distance = math.sqrt(diff_x**2 + diff_y**2 + diff_z**2)

                    if distance > 0.01:  # More than 1cm difference
                        raise ValueError(
                            f"ECEF to geographic conversion mismatch: {distance:.6f}m difference between original and IE data"
                        )
                except Exception as e:
                    print(f"Warning: Unable to verify consistency between ECEF and geographic coordinates: {e}")

                # Create NavSatFix message
                navsat_msg = NavSatFix()
                navsat_msg.header.seq = i
                navsat_msg.header.stamp = timestamp
                navsat_msg.header.frame_id = frame_id

                # Create a custom NavSatFix message with PDOP and quality in its status field
                # Assign geodetic coordinates (latitude and longitude in degrees, altitude in meters)
                navsat_msg.status.service = quality  # Using quality as service
                navsat_msg.latitude = lat_deg
                navsat_msg.longitude = lon_deg
                navsat_msg.altitude = alt

                if ambiguity_status == "Fixed":
                    navsat_msg.status.status = NavSatStatus.STATUS_GBAS_FIX
                elif ambiguity_status == "Float":
                    navsat_msg.status.status = NavSatStatus.STATUS_SBAS_FIX

                navsat_msg.position_covariance_type = pdop

                # Initialize covariance matrix as 3x3 identity matrix
                cov_ecef = np.eye(3)

                # --- Covariance Transformation ---
                # Build the ECEF covariance matrix from the position_covariance elements
                # The position_covariance vector contains the 6 unique elements of a symmetric 3x3 matrix
                # ["Cx11", "Cx21", "Cx22", "Cx31", "Cx32", "Cx33"]
                cov_ecef = np.array(
                    [
                        [position_cov[0], position_cov[1], position_cov[3]],  # Cx11, Cx21, Cx31
                        [position_cov[1], position_cov[2], position_cov[4]],  # Cx21, Cx22, Cx32
                        [position_cov[3], position_cov[4], position_cov[5]],  # Cx31, Cx32, Cx33
                    ]
                )

                # For the transformation, convert latitude and longitude to radians
                lat_rad = math.radians(lat_deg)
                lon_rad = math.radians(lon_deg)
                # Transform the covariance from ECEF to ENU (in m^2)
                cov_enu = ecef_to_enu_covariance(cov_ecef, lat_rad, lon_rad)

                # NavSatFix expects a 9-element covariance (row-major order) relative to ENU axes.
                navsat_msg.position_covariance = cov_enu.flatten().tolist()

                # Convert velocity covariance from ECEF to ENU
                velocity_enu = ecef_velocity_to_enu(ecef_vx, ecef_vy, ecef_vz, lat_rad, lon_rad)

                # Convert velocity covariance from ECEF to ENU
                cov_enu_vel = ecef_to_enu_covariance(cov_ecef_vel, lat_rad, lon_rad)

                # Set the Origin.
                if i == 0:
                    bag.write(
                        topic=f"/gt_box/inertial_explorer/{post_proc_mode}/origin",
                        msg=msg,
                        t=timestamp,
                    )

                    bag.write(
                        topic=f"/gt_box/inertial_explorer/{post_proc_mode}/navsatfix_origin",
                        msg=navsat_msg,
                        t=timestamp,
                    )

                # Write to gnss_raw
                bag.write(
                    topic=f"/gt_box/inertial_explorer/{post_proc_mode}/raw",
                    msg=msg,
                    t=timestamp,
                )

                # Write to gnss_raw
                bag.write(topic=f"/gt_box/inertial_explorer/{post_proc_mode}/navsatfix", msg=navsat_msg, t=timestamp)

                if start_time is None:
                    start_time = time

                # Position
                position_ned = utils.ecef2enu(position[0], position[1], position[2])
                position_enu = R_enu__ned @ position_ned
                # position_enu_std = np.array(R_enu__ned @ utils.R @ position_std)
                # position_enu_var = np.square(position_enu_std)[0]

                # Covariance is diagonal - 6x6 matrix ( dx, dy, dz, droll, dpitch, dyaw)
                # TO ROS convention: fixed axis https://www.ros.org/reps/rep-0103.html
                quaternion_xyzw = Rotation.from_matrix(
                    R_enu__ned @ Rotation.from_euler("ZXY", orientation_hrp, degrees=True).as_matrix()
                ).as_quat()
                rot_std = np.deg2rad(orientation_hrp_std)
                rot_std = np.array([rot_std[2], rot_std[1], rot_std[0]])
                rot_var = np.square(rot_std)

                covariance = np.eye(6)
                # Set the position covariance (top-left 3x3 block)
                covariance[:3, :3] = cov_enu
                # Set the rotation covariance (bottom-right 3x3 block)
                covariance[3, 3] = rot_var[0]
                covariance[4, 4] = rot_var[1]
                covariance[5, 5] = rot_var[2]

                # Create PoseWithCovarianceStamped message
                output_msg = PoseWithCovarianceStamped()
                output_msg.header.seq = i
                output_msg.header.stamp = timestamp
                output_msg.header.frame_id = frame_id
                output_msg.pose.pose.position = Point(x=position_enu[1], y=position_enu[0], z=-position_enu[2])
                output_msg.pose.covariance = covariance.astype(np.float32).flatten().tolist()
                ###########################
                output_msg.pose.pose.orientation = Quaternion(
                    x=quaternion_xyzw[0], y=quaternion_xyzw[1], z=quaternion_xyzw[2], w=quaternion_xyzw[3]
                )

                # orig_quat = [quaternion_xyzw[0], quaternion_xyzw[1], quaternion_xyzw[2], quaternion_xyzw[3]]

                # # Convert the original quaternion into a Rotation object.
                # orig_rotation = Rotation.from_quat(orig_quat)

                # cog_rad = math.radians(cog_value)

                # # Create a rotation about the z-axis by the azimuth angle.
                # azimuth_rotation = Rotation.from_euler("z", cog_rad)

                # # Compose the rotations.
                # # The order here means: apply the original rotation, then apply the azimuth rotation.
                # # This results in a new rotation that includes the azimuth correction.
                # new_rotation = azimuth_rotation * orig_rotation

                # # Convert the composed rotation back to a quaternion (in [x, y, z, w] order).
                # new_quat = new_rotation.as_quat()

                # # Now update your PoseWithCovarianceStamped message with the new orientation.
                # output_msg.pose.pose.orientation = Quaternion(
                #     x=new_quat[0], y=new_quat[1], z=new_quat[2], w=new_quat[3]
                # )

                # Create a TwistWithCovariance message for velocity
                velocity_msg = TwistWithCovariance()
                velocity_msg.twist.linear = Vector3(
                    x=velocity_enu[1].item(), y=velocity_enu[0].item(), z=-velocity_enu[2].item()
                )
                velocity_msg.twist.angular = Vector3(x=0, y=0, z=0)  # not the best, but we don't have the data
                velocity_cov = np.eye(6)
                velocity_cov[:3, :3] = cov_enu_vel

                # not the best, but we don't have the data
                velocity_cov[3, 3] = rot_var[0]
                velocity_cov[4, 4] = rot_var[1]
                velocity_cov[5, 5] = rot_var[2]

                velocity_msg.covariance = velocity_cov.astype(np.float32).flatten().tolist()

                # Write to the bag as a combined odometry message
                odometry_msg = Odometry()
                odometry_msg.header = output_msg.header
                odometry_msg.pose.pose = output_msg.pose.pose
                odometry_msg.twist = velocity_msg
                odometry_msg.child_frame_id = "cpt7_imu"
                bag.write(topic=f"/gt_box/inertial_explorer/{post_proc_mode}/odometry", msg=odometry_msg, t=timestamp)

                path_pose = PoseStamped()
                path_pose.header = output_msg.header
                path_pose.pose = output_msg.pose.pose
                gps_path_msg.poses.append(path_pose)
                if i % 200 == 0:
                    bag.write(
                        topic=f"/gt_box/inertial_explorer/{post_proc_mode}/gps_path", msg=gps_path_msg, t=timestamp
                    )

                # --- Create TF message ---
                tf_message = TFMessage()
                tf_message.transforms = []
                box_transform = TransformStamped()
                box_transform.header = odometry_msg.header
                box_transform.header.frame_id = "cpt7_imu"
                box_transform.child_frame_id = frame_id

                position_enu = np.array(position_enu)[:, 0]

                SE3 = np.eye(4)
                SE3[:3, :3] = Rotation.from_quat(quaternion_xyzw).as_matrix()

                # ENU to NED, so we need to invert the z-axis
                SE3[:3, 3] = [position_enu[1], position_enu[0], -position_enu[2]]

                # Inversion is needed as cpt7_imu is the parent, and enu_origin is the child
                SE3 = np.linalg.inv(SE3)

                q = Rotation.from_matrix(SE3[:3, :3]).as_quat()
                box_transform.transform.rotation = Quaternion(q[0], q[1], q[2], q[3])
                box_transform.transform.translation = Vector3(x=SE3[0, 3], y=SE3[1, 3], z=SE3[2, 3])

                tf_message.transforms.append(box_transform)

                # cog_transform = TransformStamped()
                # # Use the same header as your odometry message, or create a new one as needed.
                # cog_transform.header = odometry_msg.header
                # cog_transform.header.frame_id = "cpt7_imu"      # Parent frame
                # cog_transform.child_frame_id = "cpt7_imu_aligned"     # Child frame

                # # No translation: pure rotation (set translation to zero)
                # cog_transform.transform.translation.x = 0.0
                # cog_transform.transform.translation.y = 0.0
                # cog_transform.transform.translation.z = 0.0

                # # Compute the rotation quaternion from the COG angle.
                # # Assume cog_value is given in degrees.
                # cog_rad = math.radians(cog_value)
                # cog_quat = Rotation.from_euler('z', cog_rad).as_quat()  # returns [x, y, z, w]

                # cog_transform.transform.rotation = Quaternion(
                #     x=cog_quat[0],
                #     y=cog_quat[1],
                #     z=cog_quat[2],
                #     w=cog_quat[3]
                # )

                # tf_message.transforms.append(cog_transform)

                # bag.write(topic="/tf", msg=tf_message, t=timestamp)

        import matplotlib.pyplot as plt

        times = np.array(quality_times) - quality_times[0]
        plt.figure("Quality vs Time")
        plt.plot(times, quality_values, marker="o", linestyle="-")
        plt.xlabel("Time (s)")
        plt.ylabel("Quality Metric")
        plt.title(f"Quality Metric vs Time for {post_proc_mode}")
        plt.grid(True)
        # Save the figure in the same directory as the bag file
        output_fig = Path(bag_path).parent / "ie" / post_proc_mode / f"quality_vs_time_{post_proc_mode}.png"
        plt.savefig(str(output_fig))
        plt.close()
        print(f"Saved quality vs time figure to: {output_fig}")

        # Number of Satellites vs Time
        plt.figure("Satellites vs Time")
        plt.plot(times, satellites_values, marker="o", linestyle="-")
        plt.xlabel("Time (s)")
        plt.ylabel("Number of Satellites")
        plt.title(f"Number of Satellites vs Time for {post_proc_mode}")
        plt.grid(True)
        output_fig = Path(bag_path).parent / "ie" / post_proc_mode / f"satellites_vs_time_{post_proc_mode}.png"
        plt.savefig(str(output_fig))
        plt.close()
        print(f"Saved satellites vs time figure to: {output_fig}")

        # Ambiguity Status vs Time
        plt.figure("Ambiguity vs Time")
        plt.plot(times, ambiguity_values, marker="o", linestyle="-")
        plt.xlabel("Time (s)")
        plt.ylabel("Ambiguity Status (Numeric)")
        plt.title(f"Ambiguity Status vs Time for {post_proc_mode}")
        plt.grid(True)
        output_fig = Path(bag_path).parent / "ie" / post_proc_mode / f"ambiguity_vs_time_{post_proc_mode}.png"
        plt.savefig(str(output_fig))
        plt.close()
        print(f"Saved ambiguity vs time figure to: {output_fig}")


if __name__ == "__main__":
    main()
    exit(0)
