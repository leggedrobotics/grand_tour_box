import rospy


START_RECORDING = ["FILECONFIG OPEN"]
STOP_RECORDING = ["FILECONFIG CLOSE"]
SIGNAL_TO_RECORD = [
    # Inertial Explorer.
    # Requires binary logs: https://docs.novatel.com/Waypoint/Content/AppNotes/SPAN_Logging_for_IE.htm#Logging
    # These fields are populated by inspecting the Logging panel in the Novatel Application Suite app on OPC (Ubuntu)
    # Select "inertial explorer", then click "Next", and press "Edit Optional Settings"
    # NOTE! The fields will be shown without the binary binary postfix "B" which is required by Inertial Explorer,
    # so "B" is appended to the end of each topic name.
    "LOG FILE BDSEPHEMERISB ONNEW",
    "LOG FILE BESTGNSSPOSB ONTIME 1",
    "LOG FILE BESTPOSB ONTIME 1",
    "LOG FILE DMICONFIGB ONCHANGED",  # (if using a DMI sensor)
    "LOG FILE GALFNAVEPHEMERISB ONNEW",
    "LOG FILE GALINAVEPHEMERISB ONNEW",
    "LOG FILE GPSEPHEMB ONNEW",
    "LOG FILE HEADING2B ONNEW",
    "LOG FILE INSCONFIGB ONCHANGED",
    "LOG FILE INSPVAXB ONTIME 1",
    "LOG FILE INSUPDATESTATUSB ONCHANGED",
    "LOG FILE QZSSEPHEMERISB ONNEW",
    "LOG FILE RANGECMPB ONTIME 1",
    "LOG FILE RAWDMIB ONNEW",
    "LOG FILE RAWIMUSXB ONNEW",
    "LOG FILE RXCONFIGB ONCE",
    "LOG FILE RXCONFIG ONCE",  # Logged once in ASCII for debug purposes
    "LOG FILE RXSTATUSB ONCE",
    "LOG FILE THISANTENNATYPEB ONCE",
    "LOG FILE TIMEB ONTIME 1",
    "LOG FILE VERSIONB ONCE",
    "LOG FILE RANGECMP_1 ONTIME 1",  # NEEDS BINARY
    # # GraphNav
    # "LOG FILE BDSEPHEMERIS",
    # "LOG FILE BESTPOS",
    # "LOG FILE GALFNAVEPHEMERIS",
    # "LOG FILE GALINAVEPHEMERIS",
    # "LOG FILE GPSEPHEM",
    # "LOG FILE QZSSEPHEMERIS",
    # "LOG FILE RANGECMP",
    # "LOG FILE RXSTATUS",
    # "LOG FILE THISANTENNATYPE",
    # # ALIGN
    # "LOG FILE ALIGNBSLNENU",
    # "LOG FILE ALIGNBSLNXYZ",
    # "LOG FILE ALIGNDOP",
    # "LOG FILE BDSEPHEMERIS",
    # "LOG FILE BESTPOS",
    # "LOG FILE DUALANTENNAHEADING",
    # "LOG FILE GALINAVRAWEPHEMERIS",
    # "LOG FILE HEADING2",
    # "LOG FILE HEADINGSATS",
    # "LOG FILE MASTERPOS",
    # "LOG FILE PASSTHROUGH",
    # "LOG FILE PORTSTATS",
    # "LOG FILE RANGE",
    # "LOG FILE RANGE_1",
    # "LOG FILE RAWEPHEM",
    # "LOG FILE ROVERPOS",
    # "LOG FILE RXSTATUS",
    # "LOG FILE TRACKSTAT_1",
    # # General
    # "LOG FILE BESTPOS",
    # "LOG FILE HWMONITOR",
    # "LOG FILE PORTSTATS",
    # "LOG FILE PROFILEINFO",
    # "LOG FILE RANGE",
    # "LOG FILE RAWEPHEM",
    # "LOG FILE RXSTATUS",
    # # Interference Detection
    # "LOG FILE BDSALMANAC",
    # "LOG FILE BDSCLOCK",
    # "LOG FILE BDSEPHEMERIS",
    # "LOG FILE BESTPOS",
    # "LOG FILE CLOCKSTEERING",
    # "LOG FILE GALALMANAC",
    # "LOG FILE GALCLOCK",
    # "LOG FILE GALINAVRAWEPHEMERIS",
    # "LOG FILE IONUTC",
    # "LOG FILE ITDETECTSTATUS",
    # "LOG FILE ITPSDDETECT",
    # "LOG FILE ITPSDFINAL",
    # "LOG FILE NAVICALMANAC",
    # "LOG FILE NAVICEPHEMERIS",
    # "LOG FILE PASSTHROUGH",
    # "LOG FILE QZSSALMANAC",
    # "LOG FILE QZSSEPHEMERIS",
    # "LOG FILE RANGE",
    # "LOG FILE RANGE_1",
    # "LOG FILE RAWALM",
    # "LOG FILE RAWEPHEM",
    # "LOG FILE RXSTATUS",
    # "LOG FILE SATVIS2",
    # "LOG FILE TRACKSTAT",
    # "LOG FILE TRACKSTAT_1",
    # # RTK ROVER
    # "LOG FILE BDSRAWNAVSUBFRAME",
    # "LOG FILE BESTPOS",
    # "LOG FILE CLOCKMODEL",
    # "LOG FILE GALFNAVRAWEPHEMERIS",
    # "LOG FILE GALINAVRAWEPHEMERIS",
    # "LOG FILE IONUTC",
    # "LOG FILE MATCHEDPOS",
    # "LOG FILE MATCHEDSATS",
    # "LOG FILE PASSTHROUGH",
    # "LOG FILE PSRPOS",
    # "LOG FILE QZSSRAWEPHEM",
    # "LOG FILE RANGE",
    # "LOG FILE RAWALM",
    # "LOG FILE RAWEPHEM",
    # "LOG FILE REFSTATION",
    # "LOG FILE RTKPOS",
    # "LOG FILE RTKSATS",
    # "LOG FILE RTKVEL",
    # "LOG FILE RXSTATUS",
    # "LOG FILE TRACKSTAT",
    # # ?
    # "LOG FILE ALMANAC",
    # "LOG FILE BDSALMANAC",
    # "LOG FILE BDSBCNAV1EPHEMERIS",
    # "LOG FILE BDSBCNAV2EPHEMERIS",
    # "LOG FILE BDSBCNAV3EPHEMERIS",
    # "LOG FILE BDSEPHEMERIS",
    # "LOG FILE BESTPOS",
    # "LOG FILE GALALMANAC",
    # "LOG FILE GALFNAVEPHEMERIS",
    # "LOG FILE GALINAVEPHEMERIS",
    # "LOG FILE GPSEPHEM",
    # "LOG FILE IONUTC",
    # "LOG FILE LBANDBEAMTABLE",
    # "LOG FILE LBANDTRACKSTAT",
    # "LOG FILE PPPPOS",
    # "LOG FILE PPPSATS",
    # "LOG FILE PSRPOS",
    # "LOG FILE QZSSALMANAC",
    # "LOG FILE QZSSEPHEMERIS",
    # "LOG FILE RANGE",
    # "LOG FILE RXSTATUS",
    # "LOG FILE TERRASTARINFO",
    # "LOG FILE TERRASTARSTATUS",
    # "LOG FILE TRACKSTAT",
    # # TERRASTAR
    # "LOG FILE ALMANAC",
    # "LOG FILE BDSALMANAC",
    # "LOG FILE BDSBCNAV1EPHEMERIS",
    # "LOG FILE BDSBCNAV2EPHEMERIS",
    # "LOG FILE BDSBCNAV3EPHEMERIS",
    # "LOG FILE BDSEPHEMERIS",
    # "LOG FILE BESTPOS",
    # "LOG FILE GALALMANAC",
    # "LOG FILE GALFNAVEPHEMERIS",
    # "LOG FILE GALINAVEPHEMERIS",
    # "LOG FILE GPSEPHEM",
    # "LOG FILE IONUTC",
    # "LOG FILE LBANDBEAMTABLE",
    # "LOG FILE LBANDTRACKSTAT",
    # "LOG FILE PPPPOS",
    # "LOG FILE PPPSATS",
    # "LOG FILE PSRPOS",
    # "LOG FILE QZSSALMANAC",
    # "LOG FILE QZSSEPHEMERIS",
    # "LOG FILE RANGE",
    # "LOG FILE RXSTATUS",
    # "LOG FILE TERRASTARINFO",
    # "LOG FILE TERRASTARSTATUS",
    # "LOG FILE TRACKSTAT",
    # # SPAN
    # "LOG FILE BDSEPHEMERIS",
    # "LOG FILE BESTGNSSPOS",
    # "LOG FILE BESTPOS",
    # "LOG FILE GALINAVRAWEPHEMERIS",
    # "LOG FILE INSCONFIG",
    # "LOG FILE INSPVAX",
    # "LOG FILE INSUPDATESTATUS",
    # "LOG FILE IONUTC",
    # "LOG FILE RANGECMP4",
    # "LOG FILE RAWEPHEM",
    # "LOG FILE RAWIMUSX",
    # "LOG FILE RXSTATUS",
]
# import time
# import os
# #import rospy
#
# #from novatel_oem7_msgs.srv import Oem7AbasciiCmd, Oem7AbasciiCmdRequest
#
# def cpt7_start_recording():
#     # rospy.wait_for_service('/gt_box/cpt7/receivers/main/Oem7Cmd')
#     # oem = rospy.ServiceProxy('/gt_box/cpt7/receivers/main/Oem7Cmd', Oem7AbasciiCmd)
#     # oem()
#     # stop_recording_srv = rospy.ServiceProxy(service_name, StopRecordingInternal)
#     # req = Oem7AbasciiCmdRequest()
#     # req.cmd = START_RECORDING[0]
#     # response = stop_recording_srv(req)
#
#
#     os.system(f"""rosservice call /gt_box/cpt7/receivers/main/Oem7Cmd \"cmd: '{START_RECORDING[0]}'\" """)
#     for command in SIGNAL_TO_RECORD:
#         os.system(f"""rosservice call /gt_box/cpt7/receivers/main/Oem7Cmd \"cmd: '{command}'\" """)
#
#
# def cpt7_stop_recording():
#     for command in STOP_RECORDING:
#         os.system(f"""rosservice call /gt_box/cpt7/receivers/main/Oem7Cmd \"cmd: '{command}'\" """)
#
# if __name__ == "__main__":
#     cpt7_stop_recording()

# import socket
# import os


def call_service(cmd, max_attempts=10):
    from novatel_oem7_msgs.srv import (
        Oem7AbasciiCmd,
        Oem7AbasciiCmdRequest,
    )  # Replace 'your_package' with the actual package name

    service_name = "/gt_box/cpt7/receivers/main/Oem7Cmd"
    rospy.wait_for_service(service_name)
    oem_service = rospy.ServiceProxy(service_name, Oem7AbasciiCmd)

    for attempt in range(max_attempts):
        try:
            req = Oem7AbasciiCmdRequest()
            req.cmd = cmd
            response = oem_service(req)
            if response.rsp == "OK":
                return True
            rospy.loginfo(f"{cmd}: {response.rsp}")
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
    return False


def cpt7_start_recording():
    # First, stop any ongoing recording
    cpt7_stop_recording()

    suc = True
    # Start recording
    suc = suc and call_service(START_RECORDING[0])

    # Send signal to record commands
    for command in SIGNAL_TO_RECORD:
        suc = suc and call_service(command)

    return suc


def cpt7_stop_recording():
    return call_service(STOP_RECORDING[0])


if __name__ == "__main__":
    rospy.init_node("cpt7_recording_node")

    # Example usage
    if cpt7_start_recording():
        rospy.loginfo("Recording started successfully")
    else:
        rospy.logerr("Failed to start recording")

    rospy.spin()
