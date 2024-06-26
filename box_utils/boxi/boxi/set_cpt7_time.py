import ntplib
import datetime
import socket

def add_arguments(parser):
    parser.set_defaults(main=main)
    return parser


def main(args):
    def fetch_current_time_for_zurich():
        client = ntplib.NTPClient()

        # Request the current time from the NTP server (here we use pool.ntp.org as an example)
        response = client.request('pool.ntp.org')
        response.tx_time
        # Calculate the current GPS week and seconds since the start of GPS epoch (January 6, 1980 00:00:00 UTC)
        gps_epoch_start = datetime.datetime(1980, 1, 6, 0, 0, 0)

        delta_seconds = response.tx_time - datetime.datetime.timestamp(gps_epoch_start) + 18 - 3600
        gps_weeks = int(delta_seconds / 604800)  # 604800 seconds in a GPS week
        gps_seconds = delta_seconds % 604800 # 18 leap seconds since 1980
        return gps_weeks, gps_seconds

    gps_weeks, gps_seconds = fetch_current_time_for_zurich()

    # Define the IP address and port
    IP = "192.168.2.98"
    PORT = 3001

    # Define the command to be sent
    command = f"SETAPPROXTIME {gps_weeks} {round(gps_seconds)}"

    def send_cmd(command):
        try:
            # Create a socket object
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                # Connect to the server
                s.connect((IP, PORT))
                print(f"Connected to {IP}:{PORT}")
                # Send the command
                s.sendall(command.encode('utf-8'))
                print(f"Command sent: {command}")
                # Receive response (if any)
                response = s.recv(1024)        
                # Try to decode the response as UTF-8
                print(response[:7].decode('utf-8'))
        except Exception as e:
            print(f"An error occurred: {e}")

    send_cmd(command)
    send_cmd("LOG TIME")
