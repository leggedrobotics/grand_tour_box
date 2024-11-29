import os
import requests
import time

SERVER_IP = "192.168.X.X"  # Replace with the laptop's static IP
SERVER_PORT = 5000
MAX_RETRIES = 5  # Maximum retries for contacting the server
RETRY_DELAY = 10  # Delay between retries in seconds
DEVICE_CHECK_DELAY = 5  # Delay before checking devices after boot, in seconds


def get_device_list():
    """Fetch the list of devices (e.g., /dev/video*)"""
    return " ".join(device for device in os.listdir("/dev") if device.startswith("video"))


def main():
    retries = 0
    time.sleep(DEVICE_CHECK_DELAY)  # Wait before checking devices
    device_list = get_device_list()

    while retries < MAX_RETRIES:
        try:
            response = requests.get(f"http://{SERVER_IP}:{SERVER_PORT}/restart", params={"device_list": device_list})
            response_data = response.json()
            if response_data.get("restart"):
                print("Restarting Jetson...")
                os.system("sudo reboot")
            else:
                print("No restart required. Experiment finished.")
                return  # Exit script gracefully
        except Exception as e:
            retries += 1
            print(f"Error contacting server: {e}. Retrying ({retries}/{MAX_RETRIES})...")
            time.sleep(RETRY_DELAY)

    print("Max retries reached. Exiting experiment.")
    return  # Exit script without restarting


if __name__ == "__main__":
    main()
