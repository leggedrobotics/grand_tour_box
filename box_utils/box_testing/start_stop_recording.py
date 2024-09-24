from boxi import shell_run
import time

USERNAME = "rsl"


def main():
    for i in range(50):
        print(f"Iteration {i + 1}/50")

        # Start recording
        print("Starting recording...")
        shell_run("rosservice call /gt_box/rosbag_record_coordinator/start_recording \"yaml_file: 'box_default'\" ")

        # Wait for 30 seconds
        time.sleep(30)

        # Stop recording
        print("Stopping recording...")
        shell_run('rosservice call /gt_box/rosbag_record_coordinator/stop_recording "verbose: false" ')

        # Wait for another 30 seconds
        print("Waiting 30 seconds before next iteration...")
        time.sleep(30)

    print("All 50 iterations completed.")


if __name__ == "__main__":
    main()
