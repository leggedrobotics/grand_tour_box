import os
import rospy
import shutil


def store_debug_logs_to_folder(start_time, end_time=None, directory=None, copy_to=None):
    # Respect ROS overrides as well as the account running the recorder.
    if directory is None:
        directory = os.environ.get("ROS_LOG_DIR") or os.environ.get("ROS_HOME") or os.path.expanduser("~/.ros")
    modified_files = []
    if end_time is None:
        end_time = rospy.Time.now()

    for root, _, files in os.walk(directory):
        for file in files:
            if file.endswith(".log"):
                file_path = os.path.join(root, file)
                mod_time = os.path.getmtime(file_path)
                if start_time.to_sec() <= mod_time <= end_time.to_sec():
                    modified_files.append(file_path)
                    if copy_to:
                        try:
                            os.makedirs(copy_to, exist_ok=True)
                            shutil.copy2(file_path, os.path.join(copy_to, file))
                            print(f"Copied {file} to {copy_to}")
                        except Exception as e:
                            print(f"Error copying {file}: {str(e)}")
    return modified_files
