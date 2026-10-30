#!/usr/bin/env python3
"""Export one complete box-model TF snapshot and its calibration metadata."""
import os
from datetime import datetime
from pathlib import Path
import tempfile

import rosbag
import rospkg
import rospy
import yaml
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage

DATE_FORMAT = "%Y-%m-%d-%H-%M-%S"
EXPECTED_FILES = {
    "alphasense": ["cam1.yaml", "cam2.yaml", "cam3.yaml", "cam4.yaml", "cam5.yaml"],
    "hdr": ["hdr_front.yaml", "hdr_left.yaml", "hdr_right.yaml"],
    "zed2i": ["zed_left.yaml", "zed_right.yaml"],
}


def parse_date(value):
    if not isinstance(value, str):
        raise ValueError(f"Expected a date in {DATE_FORMAT} format, got {value!r}")
    date = datetime.strptime(value, DATE_FORMAT)
    if date.strftime(DATE_FORMAT) != value:
        raise ValueError(f"Expected a date in {DATE_FORMAT} format, got {value!r}")
    return date


def load_calibration_records(calibration_path, validity_start="", validity_end=""):
    """Validate every input before opening an output bag."""
    calibration_path = Path(calibration_path)
    with (calibration_path / "tf/calibration_latest.yaml").open() as stream:
        metadata = yaml.safe_load(stream)["calibration_metadata"]
    if not isinstance(metadata, dict) or not metadata:
        raise ValueError("Calibration metadata must be a nonempty mapping")
    summary = dict(metadata)
    dates = [parse_date(value) for key, value in metadata.items()
             if key not in ("validity_start", "validity_end") and str(value).lower() != "cad"]
    if validity_start:
        start = parse_date(validity_start)
    elif dates:
        start = min(dates)
    else:
        raise ValueError("An explicit validity_start is required for an all-CAD calibration")
    end = parse_date(validity_end) if validity_end else datetime.now().replace(microsecond=0)
    if start > end:
        raise ValueError("validity_start must not be after validity_end")
    summary.update(validity_start=start.strftime(DATE_FORMAT), validity_end=end.strftime(DATE_FORMAT))
    records = [("/calibration_metadata", yaml.safe_dump(summary))]
    topics = set()
    for folder, names in EXPECTED_FILES.items():
        for name in names:
            path = calibration_path / folder / name
            content = path.read_text()
            data = yaml.safe_load(content)
            topic = data.get("rostopic") if isinstance(data, dict) else None
            if not isinstance(topic, str) or not topic.startswith("/") or topic in topics:
                raise ValueError(f"Missing, relative or duplicate camera rostopic in {path}")
            topics.add(topic)
            records.append((topic.rstrip("/") + "/calibration_data", content))
    return summary, records


def write_calibration_bags(bag_file, tf_message, calibration_path, validity_start="", validity_end=""):
    if not tf_message.transforms:
        raise ValueError("Cannot export an empty /tf_static snapshot")
    summary, records = load_calibration_records(calibration_path, validity_start, validity_end)
    output = Path(bag_file).absolute()
    dated = output.with_name(f"tf_static_metadata_{summary['validity_start']}__{summary['validity_end']}.bag")
    if dated.exists() and dated != output:
        raise FileExistsError(f"Dated calibration already exists: {dated}")
    fd, temporary = tempfile.mkstemp(prefix=f".{output.name}.", suffix=".bag", dir=output.parent)
    os.close(fd)
    dated_created = False
    try:
        with rosbag.Bag(temporary, "w") as bag:
            stamp = rospy.Time.from_sec(datetime.now().timestamp())
            bag.write("/tf_static", tf_message, stamp)
            for topic, content in records:
                bag.write(topic, String(data=content), stamp)
        # A closed rosbag is already indexed. Publish only complete bags, and
        # refuse to replace a dated calibration that another run created.
        if dated != output:
            os.link(temporary, dated)
            dated_created = True
        os.replace(temporary, output)
    except Exception:
        if dated_created:
            dated.unlink()
        raise
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)
    return output, dated


def main():
    rospy.init_node("store_tf_static")
    calibration_path = Path(rospkg.RosPack().get_path("box_calibration")) / "calibration"
    start = rospy.get_param("~validity_start", "")
    end = rospy.get_param("~validity_end", "")
    load_calibration_records(calibration_path, start, end)
    # The shipped launch has one robot_state_publisher containing the box model.
    # Wait synchronously so shutdown cannot race an in-progress bag callback.
    snapshot = rospy.wait_for_message("/tf_static", TFMessage, timeout=rospy.get_param("~timeout", 30.0))
    outputs = write_calibration_bags(rospy.get_param("~bag_file", "output.bag"), snapshot,
                                    calibration_path, start, end)
    rospy.loginfo("Calibration bags written: %s", ", ".join(map(str, outputs)))


if __name__ == "__main__":
    try:
        main()
    except Exception as error:
        rospy.logerr("Calibration export failed: %s", error)
        raise SystemExit(1) from error
