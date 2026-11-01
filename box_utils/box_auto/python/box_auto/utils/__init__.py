"""Public utility API, loaded on demand.

Local file/geometry tools must not need ROS, cloud credentials or video SDKs
merely to import the atomic-output helper. Existing public imports retain their
names; each implementation is loaded when that attribute is requested.
"""

from importlib import import_module

_EXPORTS = {
    "WS": "utils",
    "PRE": "utils",
    "MISSION_DATA": "utils",
    "BOX_AUTO_DIR": "utils",
    "BOX_AUTO_SCRIPTS_DIR": "utils",
    "ARTIFACT_FOLDER": "utils",
    "get_uuid_mapping": "utils",
    "run_ros_command": "utils",
    "check_duplicate_timestamps": "utils",
    "find_and_extract_non_matching": "utils",
    "start_roscore": "utils",
    "kill_roscore": "utils",
    "get_bag": "utils",
    "get_file": "utils",
    "upload_bag": "utils",
    "GPS_utils": "gps_utils",
    "create_github_issue": "utils",
    "RosbagMessageGenerator": "utils",
    "okviz_trajectory_to_bag": "okviz_trajectory_to_bag",
    "execute_command_per_mission": "run_local",
    "deployments": "run_local",
    "read_sheet_data": "google_sheet",
    "upload_simple": "kleinkram_helper",
    "upload_video": "youtube",
}
__all__ = list(_EXPORTS)


def __getattr__(name):
    if name not in _EXPORTS:
        raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
    value = getattr(import_module("." + _EXPORTS[name], __name__), name)
    globals()[name] = value
    return value


def __dir__():
    return sorted(set(globals()) | set(__all__))
