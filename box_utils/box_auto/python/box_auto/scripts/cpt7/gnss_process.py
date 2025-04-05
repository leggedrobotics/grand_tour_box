import os
from pathlib import Path
from box_auto.utils import MISSION_DATA, get_file
import time
import subprocess
import shlex
import logging
import shutil
from xml.etree import ElementTree as ET

# Set up logging to a file (or configure as needed)
logging.basicConfig(filename="process_output.log", level=logging.INFO)


def convert_points_to_linestring(input_kml, output_kml):
    tree = ET.parse(input_kml)
    root = tree.getroot()
    ns = root.tag.split("}")[0].strip("{")
    nsmap = {"kml": ns}

    document = root.find("kml:Document", nsmap)

    # Extract coordinates from all <Point> placemarks
    coords = []
    for placemark in document.findall("kml:Placemark", nsmap):
        point = placemark.find("kml:Point", nsmap)
        if point is not None:
            coord_elem = point.find("kml:coordinates", nsmap)
            if coord_elem is not None and coord_elem.text:
                coords.append(coord_elem.text.strip())
            document.remove(placemark)  # Remove old point placemark

    # Create new LineString Placemark
    if coords:
        linestring_placemark = ET.Element(f"{{{ns}}}Placemark")
        name = ET.SubElement(linestring_placemark, f"{{{ns}}}name")
        name.text = "GPS Track"

        style = ET.SubElement(linestring_placemark, f"{{{ns}}}Style")
        linestyle = ET.SubElement(style, f"{{{ns}}}LineStyle")
        color = ET.SubElement(linestyle, f"{{{ns}}}color")
        color.text = "ff0000ff"  # Red line in ABGR format
        width = ET.SubElement(linestyle, f"{{{ns}}}width")
        width.text = "3"

        linestring = ET.SubElement(linestring_placemark, f"{{{ns}}}LineString")
        tessellate = ET.SubElement(linestring, f"{{{ns}}}tessellate")
        tessellate.text = "1"
        coord_elem = ET.SubElement(linestring, f"{{{ns}}}coordinates")
        coord_elem.text = "\n".join(coords)

        document.append(linestring_placemark)

    tree.write(output_kml, encoding="UTF-8", xml_declaration=True)


def main():
    print("CLI REFERENCE: https://docs.novatel.com/Waypoint/Content/Appendix/WPGCMD.htm")
    for PROCMODE in ["tc", "lc", "ppp", "dgps"]:  # ppptc #ppplc

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
            PROC_PATH = Path(MISSION_DATA) / "ie" / PROCMODE
            PROJ = PROC_PATH / f"ie_{PROCMODE}.proj"
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
            cmd += "-procint 0.05 "
            # cmd += "-procdata L1L2 "
            cmd += "-snusername RSL_02 "
            cmd += f"-snpassword {IE_API_KEY} "
            cmd += f"-expprofile1 {EXP_PROFILE} "
            cmd += "-expsrc1 epochs "  # Print the processed output in the epcoch rate.
            cmd += "-expkml on "  # export GPS measurements to KML file
            if PROCMODE == "tc":
                # cmd += '-procdir "multi" '
                cmd += "-expsbet on "  # Export the SBET file which contains IMU exchange data.
                # https://gis.stackexchange.com/questions/175026/open-a-sbet-file-format-in-gis-software
                cmd += "-expsbetimuframe on "  # Export sbet data in IMU frame.
                cmd += "-expsbetsmrmsg on "
            cmd += "-procmsg on "  # Export all data to  "<project name>_ProcMsg.log" file in addition.
            cmd += f"-expsbetkernel {PROCMODE} "
            cmd += f"-expfile1 {OUTPUT} "

            # Print PROCMODE in green
            print(f"\033[32mProcessing mode: {PROCMODE}\033[0m")

            os.chdir("/home/tutuna/Documents/IE_CLI_10/Inertial Explorer/waypoint_ie_10_00_1206/bin")
            results = subprocess.run(
                shlex.split(cmd),
                input="y\n",  # Automatically sends 'y' followed by a newline
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,  # Ensures outputs are returned as strings
            )

            # Log the output
            logging.info("STDOUT:\n%s", results.stdout)
            logging.info("STDERR:\n%s", results.stderr)

            log_filename = Path(MISSION_DATA) / "ie" / PROCMODE / f"{PROCMODE}_terminal_logs.txt"
            with open(log_filename, "w") as f:
                f.write("STDOUT:\n")
                f.write(results.stdout)
                f.write("\n\nSTDERR:\n")
                f.write(results.stderr)

            # convert_points_to_linestring(PROC_PATH / "Html" / f"ie_{PROCMODE}.GE.kml", PROC_PATH / "Html" / f"ie_{PROCMODE}_processed.GE.kml")

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
