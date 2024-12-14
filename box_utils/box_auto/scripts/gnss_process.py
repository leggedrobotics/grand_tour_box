import os
from pathlib import Path

LOGFILE = "/media/jonfrey/BoxiS4-2TB/deployment_day_15/2024-11-21-16-36-19/NMZT24120004N_2024-11-21_15-51-22.LOG"
PROJ = str(Path(LOGFILE).parent / "IE.proj")
OUTPUT = str(Path(LOGFILE).parent / "output.txt")
IE_API_KEY = os.environ["IE_API_KEY"]

cmd = "/home/jonfrey/Downloads/waypoint_ie_10_00_1206/bin/WPGCMDIMU "

cmd += f"-remfile {LOGFILE} "
cmd += "-procmode PPP "
cmd += f"-proccfg {PROJ} "
cmd += '-procprofile "SPAN Pedestrian (CPT-HG4930)" '
cmd += "-downloadbase 1 50 "
cmd += "-snserver ch.nrtk.eu "
cmd += "-procdatum WGS84 "
cmd += "-snusername RSL_01 "
cmd += f"-snpassword {IE_API_KEY} "
cmd += "-expprofile1 GrandTour-LocalFrame-minimal "  # GrandTour-LocalFrame-minimal
cmd += "-expsrc1 epochs "
cmd += f"-expfile1 {OUTPUT} "

print(cmd)
os.system(cmd)


###
