import socket
import binascii

from time import sleep
from datetime import datetime
import gps_time
import codecs


def add_arguments(parser):
    parser.set_defaults(main=main)
    parser.add_argument("--time", default=None, help="Time of the ptp master - Format: Day/Month/Year Hours:Minutes:Seconds")
    parser.add_argument("--rtc", action="store_true", help="Use the time of the battery powered real-time clock")
    parser.add_argument("--gps", action="store_true", help="Let CPT7 use GPS time.")

    return parser

def send_command(ip, port, command):
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
            
            sock.settimeout(10)
            sock.connect((ip, port))
            enc = command.encode('utf-8')
            print(enc)
            sock.sendall(enc)
  
            """
            while True:
                try:
                    sleep(0.5)
                    reply = sock.recv(10)
                    if not reply:
                        break
                    print("recvd: ", reply)
                    #print("reply ascii: ", reply.decode('ascii'))
                    print("hexreply: ", reply.hex())
                    
                except KeyboardInterrupt:
                    print("bye")
                    break
            """
            sock.close()
            
    except Exception as e:
        print(f"Error: {e}")


def main(args):
    cpt7_ip = "192.168.2.98" 
    cpt7_port = 3002


    if args.rtc and args.gps:
        print("Specify only one, '--rtc' or '--gps'")
        exit()
    if not args.rtc and not args.time and not args.gps:
        print("Either specify '--rtc' , '--gps' or '--time Day/Month/Year-Hours:Minutes:Seconds'")
        exit()
    if args.rtc and args.time:
        print("Specify only one, '--rtc' or '--time Day/Month/Year-Hours:Minutes:Seconds'")
        exit()
        
    if args.gps:
        send_command(cpt7_ip, cpt7_port, f"SETTIMESOURCE 3 gps beidou galileo ")
        send_command(cpt7_ip, cpt7_port, f"SAVECONFIG")
        print(f"Set CPT-7 time to GPS Time.")
        exit()
        
    # If we are here, we want to set a specific time not gps time.
    # Don't use any satellite system as a timesource
    send_command(cpt7_ip, cpt7_port, f"SETTIMESOURCE 0")
    send_command(cpt7_ip, cpt7_port, f"SAVECONFIG")
            
    if args.rtc:
        input_time = datetime.now()
    if args.time:
        input_time = datetime.strptime(args.time, '%d/%m/%Y-%H:%M:%S')

    gpstime = gps_time.core.GPSTime.from_datetime(input_time)
    approxTimeCommand = f"SETAPPROXTIME {gpstime.week_number} {int(gpstime.time_of_week)}"
    
    # Set the approximate time of the CPT-7
    send_command(cpt7_ip, cpt7_port, approxTimeCommand)
    send_command(cpt7_ip, cpt7_port, f"SAVECONFIG")
    print(f"Set CPT-7 time to {input_time} with command ' {approxTimeCommand} ' ")
    
    # Currently cant parse
    #send_command(cpt7_ip, cpt7_port, f"log time")
