import ntplib
import datetime
import socket
import time


def add_arguments(parser):
    parser.set_defaults(main=main)
    parser.add_argument("--local", action="store_true", help="Use local time")
    parser.add_argument("--ros", action="store_true", help="Use local time")

    parser.add_argument("--manual", action="store_true", help="Do not use internet time")
    parser.add_argument("--year", default=1996, help="Year to set")

    return parser


def set_internet_time():
    client = ntplib.NTPClient()

    # Request the current time from the NTP server (here we use pool.ntp.org as an example)
    response = client.request("pool.ntp.org")
    response.tx_time

    # Calculate the current GPS week and seconds since the start of GPS epoch (January 6, 1980 00:00:00 UTC)
    gps_epoch_start = datetime.datetime(1980, 1, 6, 0, 0, 0)

    print(response.tx_time)
    delta_seconds = response.tx_time - datetime.datetime.timestamp(gps_epoch_start) + 18 - 3600
    gps_weeks = int(delta_seconds / 604800)  # 604800 seconds in a GPS week
    gps_seconds = delta_seconds % 604800  # 18 leap seconds since 1980
    return gps_weeks, gps_seconds


def set_local_pc_time():
    now = datetime.datetime.now().timestamp()
    print(now)
    # Calculate the current GPS week and seconds since the start of GPS epoch (January 6, 1980 00:00:00 UTC)
    gps_epoch_start = datetime.datetime(1980, 1, 6, 0, 0, 0)

    delta_seconds = now - datetime.datetime.timestamp(gps_epoch_start) + 18 - 3600
    gps_weeks = int(delta_seconds / 604800)  # 604800 seconds in a GPS week
    gps_seconds = delta_seconds % 604800  # 18 leap seconds since 1980
    return gps_weeks, gps_seconds


def main(args):
    if args.local:
        gps_weeks, gps_seconds = set_local_pc_time()
    elif args.manual:
        gps_weeks, gps_seconds = (int(args.year) - 1980) * 52 + 26, 0
    else:
        gps_weeks, gps_seconds = set_internet_time()

    # Define the IP address and port
    IP = "192.168.2.98"
    PORT = 3001

    # Define the command to be sent
    command = f"SETAPPROXTIME {gps_weeks} {round(gps_seconds)}"

    if args.ros:
        import rospy
        from novatel_oem7_msgs.srv import (
            Oem7AbasciiCmd,
            Oem7AbasciiCmdRequest,
        )  # Replace 'your_package' with the actual package name

        rospy.init_node("cpt7_recording_node")

        def call_service(cmd, max_attempts=5):
            print("Calling ros service")
            service_name = "/gt_box/cpt7/receivers/main/Oem7Cmd"
            rospy.wait_for_service(service_name)
            oem_service = rospy.ServiceProxy(service_name, Oem7AbasciiCmd)

            for attempt in range(max_attempts):
                try:
                    req = Oem7AbasciiCmdRequest()
                    req.cmd = cmd
                    response = oem_service(req)
                    print("Response received: ", response.rsp)
                    if response.rsp == "OK":
                        return True
                    rospy.loginfo(f"{cmd}: {response.rsp}")
                except rospy.ServiceException as e:
                    rospy.logerr(f"Service call failed: {e}")
            return False

        call_service(command)
    else:

        def send_cmd(command):
            try:
                # Create a socket object
                with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                    # Connect to the server
                    s.connect((IP, PORT))
                    print(f"Connected to {IP}:{PORT}")
                    # Send the command
                    s.sendall(command.encode("utf-8"))
                    print(f"Command sent: {command}")
                    # Receive response (if any)
                    response = s.recv(1024)
                    # Try to decode the response as UTF-8
                    print(response[:7].decode("utf-8"))
            except Exception as e:
                print(f"An error occurred: {e}")

        time.sleep(0.5)
        send_cmd(command)
        time.sleep(0.5)
