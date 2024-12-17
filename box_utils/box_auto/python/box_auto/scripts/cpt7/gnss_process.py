import os
from pathlib import Path

print("CLI REFERENCE: https://docs.novatel.com/Waypoint/Content/Appendix/WPGCMD.htm")

for PROCMODE in ["tc", "lc", "ppp", "dgps"]:

    for EXP_PROFILE in ["GrandTour-LocalFrame-minimal", "GPX"]:
        LOGFILE = "/media/jonfrey/Data/deployment_day_16/2024-11-25-14-57-08/NMZT24120004N_2024-11-25_14-03-31.LOG"
        PROJ = str(Path(LOGFILE).parent / "ie" / PROCMODE / f"ie_{PROCMODE}.proj")
        OUTPUT = str(Path(LOGFILE).parent / "ie" / f"output_{PROCMODE}_{EXP_PROFILE}.txt")
        IE_API_KEY = os.environ["IE_API_KEY"]
        Path(PROJ).parent.mkdir(exist_ok=True, parents=True)

        cmd = "cd /home/jonfrey/Downloads/waypoint_ie_10_00_1206/bin; ./WPGCMDIMU "

        cmd += f'-remfile "{LOGFILE}" '
        cmd += f"-procmode {PROCMODE} "
        cmd += f"-proccfg {PROJ} "
        cmd += '-procprofile "SPAN Pedestrian (CPT7-HG4930)" '
        cmd += "-downloadbase 1 50 "
        cmd += "-snserver ch-xpos.nrtk.eu "  # https://ch.nrtk.eu/sbc/ -> This needs to be inferred
        cmd += "-procdatum WGS84 "
        cmd += "-snusername RSL_02 "
        cmd += f"-snpassword {IE_API_KEY} "
        cmd += f"-expprofile1 {EXP_PROFILE} "
        cmd += "-expsrc1 epochs "
        cmd += f"-expfile1 {OUTPUT} "

        print(cmd)
        os.system(cmd)
