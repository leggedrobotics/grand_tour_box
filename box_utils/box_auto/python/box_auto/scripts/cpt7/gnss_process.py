import os
from pathlib import Path
from box_auto.utils import MISSION_DATA, get_file
import time
import subprocess
import shlex
import logging
import shutil

# Set up logging to a file (or configure as needed)
logging.basicConfig(filename="process_output.log", level=logging.INFO)


def main():
    print("CLI REFERENCE: https://docs.novatel.com/Waypoint/Content/Appendix/WPGCMD.htm")

    execution_results = []

    for PROCMODE in ["ppptc", "tc", "lc", "ppp", "dgps"]:
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
                external_log_files = [s for s in Path(MISSION_DATA).rglob("*.LOG") if ie_path not in s.parents]
                if len(external_log_files) == 1:
                    log_file = external_log_files[0]
                    print(f"Found LOG file outside /ie/ directory: {log_file}")
                elif len(external_log_files) == 0:
                    raise ValueError("No LOG file found in MISSION_DATA.")
                else:
                    raise ValueError(f"Multiple LOG files found outside /ie/ directory: {external_log_files}")

                destination = ie_path / log_file.name
                shutil.copy(str(log_file), str(destination))
                print(f"Copied LOG file to /ie/ directory: {destination}")
                log_file = destination

            log_file, success = get_file("*.LOG", str(ie_path))
            if not success:
                raise ValueError("Failed to retrieve LOG file from /ie/ directory.")

            max_retries = 8 if PROCMODE == "tc" else 1
            procInterval = 0.05
            attempt = 0
            success = False

            while attempt < max_retries:
                attempt += 1

                # Set up paths again in case needed per retry
                PROC_PATH = Path(MISSION_DATA) / "ie" / PROCMODE
                PROJ = PROC_PATH / f"ie_{PROCMODE}.proj"
                OUTPUT = Path(MISSION_DATA) / "ie" / f"output_{PROCMODE}_{EXP_PROFILE}.txt"
                IE_API_KEY = os.environ["IE_API_KEY"]
                PROJ.parent.mkdir(parents=True, exist_ok=True)

                cmd = "./WPGCMDIMU "
                cmd += f'-remfile "{log_file}" '
                cmd += f"-procmode {PROCMODE} "
                cmd += f"-proccfg {PROJ} "
                cmd += '-procprofile "SPAN Pedestrian (CPT7-HG4930)" '
                cmd += "-downloadbase 1 50 "
                cmd += "-snserver ch-xpos.nrtk.eu "
                cmd += "-procdatum WGS84 "
                cmd += f"-procint {procInterval} "
                cmd += "-procdata L1L2 "
                cmd += "-snusername RSL_02 "
                cmd += f"-snpassword {IE_API_KEY} "
                cmd += f"-expprofile1 {EXP_PROFILE} "
                cmd += "-expsrc1 epochs "
                cmd += "-expkml on "
                if PROCMODE == "tc":
                    cmd += '-procdir "multi" '
                    cmd += "-expsbet on "
                    cmd += "-expsbetimuframe on "
                    cmd += "-expsbetsmrmsg on "
                cmd += "-procmsg on "
                cmd += f"-expsbetkernel {PROCMODE} "
                cmd += f"-expfile1 {OUTPUT} "

                print(f"\033[32m[{PROCMODE}] Attempt {attempt} | procint = {procInterval:.5f}\033[0m")

                # os.chdir(Path(WS) / "src/grand_tour_box/box_applications/waypoint_ie_10_00_1206/bin")
                # os.chdir("/home/tutuna/Documents/IE_CLI_10/Inertial Explorer/waypoint_ie_10_00_1206/bin")
                os.chdir("/home/jonfrey/ie_turcan/waypoint_ie_10_00_1206/bin")
                print(cmd)
                results = subprocess.run(
                    shlex.split(cmd),
                    input="y\n",
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    text=True,
                )

                # Log output
                logging.info("STDOUT:\n%s", results.stdout)
                logging.info("STDERR:\n%s", results.stderr)

                log_filename = Path(MISSION_DATA) / "ie" / PROCMODE / f"{PROCMODE}_terminal_logs_attempt{attempt}.txt"
                with open(log_filename, "w") as f:
                    f.write("STDOUT:\n")
                    f.write(results.stdout)
                    f.write("\n\nSTDERR:\n")
                    f.write(results.stderr)

                time.sleep(2)

                status = {
                    "mode": PROCMODE,
                    "profile": EXP_PROFILE,
                    "returncode": results.returncode,
                    "message": "Success",
                    "attempt": attempt,
                    "procint": procInterval,
                }

                if results.returncode == 0:
                    success = True
                    execution_results.append(status)
                    break
                else:
                    if "_ERROR_ No records were decoded" in results.stdout:
                        status["message"] = "Failed: No records were decoded"
                    elif "_ERROR_ Failed to download base station data from " in results.stdout:
                        status["message"] = "Failed: Base station download error"
                    else:
                        status["message"] = "Failed: Unknown error"

                    if PROCMODE == "tc":
                        procInterval *= 2

                    if attempt == max_retries:
                        execution_results.append(status)

    # Final Report
    print("\n=== PROCESSING REPORT ===")
    for result in execution_results:
        print(
            f"[{result['mode']} | {result['profile']}] → {result['message']} "
            f"(code: {result['returncode']}, procint: {result.get('procint')}, attempt: {result['attempt']})"
        )


if __name__ == "__main__":
    main()
