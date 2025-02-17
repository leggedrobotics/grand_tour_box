import rosbag
import gpxpy

from box_auto.utils import get_bag
from datetime import datetime
import numpy as np


def ecef_to_lla(x, y, z):
    # WGS84 ellipsoid constants
    a = 6378137.0  # Semi-major axis (meters)
    e2 = 6.69437999014e-3  # Square of first eccentricity
    f = 1 / 298.257223563  # Flattening

    # Compute longitude
    lon = np.arctan2(y, x)

    # Iterative computation for latitude and altitude
    b = a * (1 - f)
    ep2 = (a**2 - b**2) / b**2
    p = np.sqrt(x**2 + y**2)
    theta = np.arctan2(z * a, p * b)

    sin_theta = np.sin(theta)
    cos_theta = np.cos(theta)

    lat = np.arctan2(z + ep2 * b * sin_theta**3, p - e2 * a * cos_theta**3)

    # Compute altitude
    N = a / np.sqrt(1 - e2 * np.sin(lat) ** 2)
    alt = p / np.cos(lat) - N

    # Convert radians to degrees
    lat = np.degrees(lat)
    lon = np.degrees(lon)

    return lat, lon, alt


if __name__ == "__main__":

    valid_topics = "/gt_box/inertial_explorer/tc/raw/position_ecef"

    bag_path = get_bag("*_cpt7_ie_tc.bag")

    gpx = gpxpy.gpx.GPX()
    # Create first track in our GPX:
    gpx_track = gpxpy.gpx.GPXTrack()
    gpx.tracks.append(gpx_track)

    # Create first segment in our GPX track:
    gpx_segment = gpxpy.gpx.GPXTrackSegment()
    gpx_track.segments.append(gpx_segment)
    # Create points

    with rosbag.Bag(bag_path, "r") as bag:
        for topic, msg, ts in bag.read_messages(topics=[valid_topics]):
            lat, lon, alt = ecef_to_lla(msg.x, msg.y, msg.z)

            # convert msg.header to datetime
            timestamp = datetime.fromtimestamp(ts.secs + ts.nsecs * 1e-9)

            # Add points to the GPS path
            gpx_segment.points.append(
                gpxpy.gpx.GPXTrackPoint(latitude=lat, longitude=lon, elevation=alt, time=timestamp)
            )
    out = bag_path.replace(".bag", ".gpx")
    with open(out, "w") as f:
        f.write(gpx.to_xml())
