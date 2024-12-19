import os
from pathlib import Path
from box_auto.utils import MISSION_DATA, get_file
import time
import subprocess
import shlex


def main():
    print("CLI REFERENCE: https://docs.novatel.com/Waypoint/Content/Appendix/WPGCMD.htm")
    for PROCMODE in ["tc", "lc", "ppp", "dgps"]:
        for EXP_PROFILE in ["GrandTour-LocalFrame-minimal"]:
            log_files = [str(s) for s in Path(MISSION_DATA).rglob("*.LOG") if "/ie/" not in str(s)]
            suc = True if len(log_files) == 1 else False

            if len(log_files) == 0:
                log_files = [str(s) for s in Path(MISSION_DATA).rglob("*.LOG")]

                if len(log_files) == 1:
                    log_file = log_files[0]
                    print(f"cp {log_file} {MISSION_DATA}")
                    exit(1)

                log_files = [str(s) for s in Path(MISSION_DATA).rglob("*.LOG") if "/ie/" not in str(s)]
                suc = True if len(log_files) == 1 else False

            if not suc:
                raise ValueError("Failed to find LOG file.")

            log_file = log_files[0]

            PROJ = str(Path(MISSION_DATA) / "ie" / PROCMODE / f"ie_{PROCMODE}.proj")
            OUTPUT = str(Path(MISSION_DATA) / "ie" / f"output_{PROCMODE}_{EXP_PROFILE}.txt")
            IE_API_KEY = os.environ["IE_API_KEY"]
            Path(PROJ).parent.mkdir(exist_ok=True, parents=True)

            os.system(f"cp {log_file} " + str(Path(MISSION_DATA) / "ie"))

            # Updated moved log_file path
            log_file, suc = get_file("*.LOG", str(Path(MISSION_DATA) / "ie"))

            cmd = "./WPGCMDIMU "
            cmd += f'-remfile "{log_file}" '
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

            os.chdir("/home/jonfrey/Downloads/waypoint_ie_10_00_1206/bin")
            results = subprocess.run(
                shlex.split(cmd),
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,  # Ensure outputs are returned as strings
            )
            time.sleep(2)

            if results.returncode == 1:
                if "_ERROR_ No records were decoded" in results.stdout:
                    exit(1)
                elif "_ERROR_ TBD" in results.stdout:
                    exit(2)
                else:
                    exit(3)
            exit(0)


if __name__ == "__main__":
    main()
