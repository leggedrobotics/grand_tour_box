import time
from pathlib import Path


def cleanup_ros_folder_on_startup():
    for i in range(240):
        current_time = time.time()
        if current_time > 1704063600:  # Unix timestamp for 2024-01-01 00:00:00
            break
        else:
            time.sleep(1)

    if current_time < 1704063600:
        raise ValueError("rospy.time is not set to a value greater than 2024 January 1st and waited for 4 minutes")

    # Deleting files older than 7 days in .ros folder
    ros_folder = Path.home() / ".ros"
    now = time.time()
    for file_path in ros_folder.rglob("*"):
        if file_path.is_file():
            # Get the file's last modification time
            file_time = file_path.stat().st_mtime

            # Calculate the file's age
            file_age = now - file_time

            # If the file is older than 7 days, delete it
            if file_age > 3600 * 24:
                try:
                    file_path.unlink()
                    print(f"Deleted: {file_path}")
                except Exception as e:
                    print(f"Error deleting {file_path}: {e}")


if __name__ == "__main__":
    cleanup_ros_folder_on_startup()
