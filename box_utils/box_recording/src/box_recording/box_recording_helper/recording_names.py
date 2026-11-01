"""ROS graph names derived from hostnames and recording profile keys."""

import re


def recording_node_name(host, bag=None):
    # Hostnames may contain '-' and profile keys may contain other punctuation.
    # Sanitize only ROS graph names; preserve host/profile identity in filenames.
    host = re.sub(r"[^a-zA-Z0-9_]", "_", host)
    if bag is None:
        return "rosbag_record_node_" + host
    return "record_" + host + "_" + re.sub(r"[^a-zA-Z0-9_]", "_", bag)
