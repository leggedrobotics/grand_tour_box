import os
import signal
import subprocess
import time


def run_non_blocking_command(command):
    """
    Runs a non-blocking command using subprocess.Popen.
    """
    return os.system("bash -c '" + command + " & " + "' ")


def run_blocking_command(command):
    """
    Runs a blocking command using subprocess.run.
    """
    return os.system("bash -c '" + command + "' ")


class RoscoreEnvironment:
    def __init__(self, use_sim_time=None):
        self.roscore_process = None
        self.use_sim_time = use_sim_time

    def shutdown_roscore(self):
        """
        Shuts down any running roscore instances by finding the process and killing it.
        """
        print("Shutting down any existing roscore instances...")
        try:
            # Use pgrep to find roscore processes
            result = subprocess.run(["pgrep", "-f", "roscore"], capture_output=True, text=True)
            pids = result.stdout.splitlines()
            roslaunch_results = subprocess.run(["pgrep", "-f", "roslaunch"], capture_output=True, text=True)
            pids += roslaunch_results.stdout.splitlines()

            if not pids:
                print("No running roscore instances found.")
                return

            for pid in pids:
                print(f"Terminating roscore process with PID: {pid}")
                os.kill(int(pid), signal.SIGTERM)

            time.sleep(2)  # Give time for processes to terminate
            print("All roscore instances shut down successfully.")
        except Exception as e:
            print(f"Error during shutdown: {e}")

    def start_roscore(self):
        """
        Starts a new instance of roscore.
        """
        print("Starting roscore...")
        self.roscore_process = subprocess.Popen(
            ["roscore"],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            preexec_fn=os.setsid  # Start a new process group for better control
        )
        time.sleep(2)  # Allow time for roscore to initialize
        if self.roscore_process.poll() is None:  # Check if roscore is still running
            print("roscore started successfully.")
            time.sleep(5)
            if self.use_sim_time is not None:
                import rospy
                rospy.set_param('use_sim_time', self.use_sim_time)
                print(f"Set 'use_sim_time' to {self.use_sim_time}.")
        else:
            print("Failed to start roscore. Check the environment setup.")

    def __enter__(self):
        """
        Enter the environment: restart roscore.
        """
        self.shutdown_roscore()  # Shutdown any existing roscore instances
        self.start_roscore()  # Start a new roscore instance
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        """
        Exit the environment: shut down roscore.
        """
        print("Exiting environment...")
        self.shutdown_roscore()  # Shutdown the current roscore instance


def main():
    # Define different -cc (camera calibration) and -cl (lidar calibration) paths
    camera_lidar_calibrations = [
        ("OXFORD_CALIBRATIONS/cam_cam_sunday_evening/2025-09-14-21-53-37_calibration",
         "OXFORD_CALIBRATIONS/cam_lidar_tuesday/2025-09-17-00-15-01_calibration",
         "tf_static_metadata_oxford.bag",
         "2026-10-10-00-00-00"),]

    # Fixed paths
    prism_calibration = "/media/fu/cloudster31/GT-spires/calibrations/OXFORD_CALIBRATIONS/cam_prism_only"
    imu_calibration = "/media/fu/cloudster31/grand_tour_calibrations/2024-12-06_calibration_imu"

    # Iterate through all camera and lidar calibration pairs
    for i, (cc_path, cl_path, output_bag_path, validity_end) in enumerate(camera_lidar_calibrations):
        cc_path = os.path.join("/media/fu/cloudster31/GT-spires/calibrations/", cc_path)
        cl_path = os.path.join("/media/fu/cloudster31/GT-spires/calibrations/", cl_path)
        output_bag_path = os.path.join("/media/fu/cloudster31/GT-spires/calibrations/", output_bag_path)
        with RoscoreEnvironment(use_sim_time=True):
            command = [
                "rosrun", "box_calibration", "process_all_calibrations.py",
                "-cc", cc_path, "--skip_camera",
                "-cl", cl_path, "--skip_lidar",
                "-cp", prism_calibration,
                "-ci", imu_calibration
            ]

            print(f"Running command:\n{' '.join(command)}")
            subprocess.run(command)

            run_non_blocking_command("roslaunch box_model box_model.launch rviz:=false")
            time.sleep(2)
            if i > 0:
                run_blocking_command(
                    f"rosrun box_calibration store_tf_static.py _bag_file:={output_bag_path} _validity_end:={validity_end}")
            else:
                run_blocking_command(
                    f"rosrun box_calibration store_tf_static.py _bag_file:={output_bag_path} _validity_end:={validity_end} _validity_start:=0")
            time.sleep(10)


if __name__ == "__main__":
    main()
