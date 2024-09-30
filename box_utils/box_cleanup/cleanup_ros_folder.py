import time
from pathlib import Path

def cleanup_ros_folder():
    for i in range(240):
        current_time = time.time()
        if current_time > 1704063600:  # Unix timestamp for 2024-01-01 00:00:00
            break
        else:
            time.sleep(1)

    if current_time < 1704063600:
        raise ValueError("rospy.time is not set to a value greater than 2024 January 1st and waited for 4 minutes")

    # Set the path to the .ros folder
    ros_folder = Path.home() / ".ros"

    # Get the current time
    now = time.time()

    # Iterate through all files in the .ros folder and its subfolders
    for file_path in ros_folder.rglob("*"):
        # Check if it's a file (not a directory)
        if file_path.is_file():
            # Get the file's last modification time
            file_time = file_path.stat().st_mtime

            # Calculate the file's age
            file_age = now - file_time

            # If the file is older than 1 day, delete it
            if file_age > 3600 * 24:
                try:
                    file_path.unlink()  # Uncomment this line to actually delete the file
                    print(f"Deleted: {file_path}")
                except Exception as e:
                    print(f"Error deleting {file_path}: {e}")


if __name__ == "__main__":
    cleanup_ros_folder()
