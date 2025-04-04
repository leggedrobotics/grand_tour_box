import os
from pathlib import Path
from box_auto.utils import MISSION_DATA, get_file
import time
import subprocess
import shlex
import shutil


def main():
    print("CLI REFERENCE: https://docs.novatel.com/Waypoint/Content/Appendix/WPGCMD.htm")
    for PROCMODE in ["tc", "lc", "ppp", "dgps"]:

        for EXP_PROFILE in ["GrandTour-LocalFrame-extended"]:
            # Ensure the /ie/ directory exists
            ie_path = Path(MISSION_DATA) / "ie"
            ie_path.mkdir(parents=True, exist_ok=True)
            print(f"Using directory: {ie_path}")

            # Look for a *.LOG file in the /ie/ directory
            ie_log_files = list(ie_path.glob("*.LOG"))
            if len(ie_log_files) == 1:
                log_file = ie_log_files[0]
                print(f"Using existing LOG file in /ie/ directory: {log_file}")
            elif len(ie_log_files) > 1:
                raise ValueError(f"Multiple LOG files found in /ie/ directory: {ie_log_files}")
            else:
                # No LOG file found in /ie/, so search in MISSION_DATA excluding the /ie/ directory
                external_log_files = [s for s in Path(MISSION_DATA).rglob("*.LOG") if ie_path not in s.parents]
                if len(external_log_files) == 1:
                    log_file = external_log_files[0]
                    print(f"Found LOG file outside /ie/ directory: {log_file}")
                elif len(external_log_files) == 0:
                    raise ValueError("No LOG file found in MISSION_DATA.")
                else:
                    raise ValueError(f"Multiple LOG files found outside /ie/ directory: {external_log_files}")

                # Copy the found LOG file to the /ie/ directory
                destination = ie_path / log_file.name
                shutil.copy(str(log_file), str(destination))
                print(f"Copied LOG file to /ie/ directory: {destination}")
                log_file = destination

            # Set up project and output file paths
            PROJ = Path(MISSION_DATA) / "ie" / PROCMODE / f"ie_{PROCMODE}.proj"
            OUTPUT = Path(MISSION_DATA) / "ie" / f"output_{PROCMODE}_{EXP_PROFILE}.txt"
            IE_API_KEY = os.environ["IE_API_KEY"]

            # Ensure the project directory exists
            PROJ.parent.mkdir(parents=True, exist_ok=True)

            # Optionally, update the log_file path using get_file
            log_file, success = get_file("*.LOG", str(ie_path))
            if not success:
                raise ValueError("Failed to retrieve LOG file from /ie/ directory.")

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
            cmd += "-expsrc1 epochs "  # Print the processed output in the epcoch rate.
            cmd += "-expkml on "  # export GPS measurements to KML file
            cmd += "-expsbet on "  # Export the SBET file which contains IMU exchange data.
            # https://gis.stackexchange.com/questions/175026/open-a-sbet-file-format-in-gis-software
            cmd += "-expsbetimuframe on "  # Export sbet data in IMU frame.
            cmd += "-procmsg on "  # Export all data to  "<project name>_ProcMsg.log" file in addition.
            cmd += f"-expsbetkernel {PROCMODE} "
            cmd += f"-expfile1 {OUTPUT} "

            os.chdir("/home/tutuna/Documents/IE_CLI_10/Inertial Explorer/waypoint_ie_10_00_1206/bin")
            results = subprocess.run(
                shlex.split(cmd),
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,  # Ensure outputs are returned as strings
            )
            time.sleep(2)
            print(results.returncode, results.stdout)
            if results.returncode == 1:
                print(results.returncode, results.stdout)
                if "_ERROR_ No records were decoded" in results.stdout:
                    exit(3)
                elif "_ERROR_ Failed to download base station data from " in results.stdout:
                    exit(4)
                else:
                    exit(5)
    exit(0)


if __name__ == "__main__":
    main()
