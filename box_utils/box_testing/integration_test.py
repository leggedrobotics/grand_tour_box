from boxi import shell_run
import time

USERNAME = "rsl"


def main():
    shell_run("boxi push --jetson --nuc")
    shell_run("boxi launch -m=recording --restart --all")

    # Wait for 30 seconds
    time.sleep(30)

    # Start recording
    print("Starting recording...")
    shell_run("rosservice call /gt_box/rosbag_record_coordinator/start_recording \"yaml_file: 'box_default'\" ")

    # Wait for 30 seconds
    time.sleep(30)

    # Stop recording
    print("Stopping recording...")
    shell_run('rosservice call /gt_box/rosbag_record_coordinator/stop_recording "verbose: false" ')


if __name__ == "__main__":
    main()
