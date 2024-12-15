import os
from pathlib import Path

print("THIS SCRIPT NEVER WORKED")

LOGFILE = "/media/jonfrey/Data/deployment_day_16/2024-11-25-14-57-08/NMZT24120004N_2024-11-25_14-03-31.LOG"
PROJ = str(Path(LOGFILE).parent / "IE.proj")
OUTPUT = str(Path(LOGFILE).parent / "output.txt")
IE_API_KEY = os.environ["IE_API_KEY"]

cmd = "/home/jonfrey/Downloads/waypoint_ie_10_00_1206/bin/WPGCMDIMU "

cmd += f'-remfile "{LOGFILE}" '
cmd += "-procmode PPP "
cmd += f"-proccfg {PROJ} "
cmd += '-procprofile "GNSS Pedestrian" '  # SPAN Pedestrian (CPT-HG4930)
# cmd += "-downloadbase 1 50 "
# cmd += "-snserver ch.nrtk.eu "
cmd += "-procdatum WGS84 "
cmd += "-snusername RSL_01 "
cmd += f"-snpassword {IE_API_KEY} "
cmd += "-expprofile1 GPX "  # GrandTour-LocalFrame-minimal
cmd += "-expsrc1 epochs "
cmd += f"-expfile1 {OUTPUT} "

print(cmd)
os.system(cmd)


###
