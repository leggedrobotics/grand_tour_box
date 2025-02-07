import pathlib
import os
import sys
from box_auto.utils import create_github_issue, MISSION_DATA, ARTIFACT_FOLDER, WS
from box_auto.scripts.verification.health_check_mission import validate_bags, LOG_FILE
from box_auto.scripts.verification.topic_freq import read_rosbag_and_generate_histograms


def analyze_mission_and_report():
    """
    Analyze the raw data from a mission in the MISSION_DATA directory and report the results.

    Returns:
        bool: True if the validation passed, False otherwise.
    """

    # TODO(kappi): generate new default reference data
    yaml_path = str(pathlib.Path(WS) / "src/grand_tour_box/box_utils/box_auto/cfg/health_check_reference_raw_data.yaml")
    validation_passed = validate_bags(
        reference_folder=None,
        yaml_file=yaml_path,
        mission_folder=MISSION_DATA,
        time_tolerance=20,
    )

    # Generate histograms of the topics
    output_dir = pathlib.Path(ARTIFACT_FOLDER) / "topic_frequency_histograms"
    for bag_file in pathlib.Path(MISSION_DATA).rglob("*.bag"):
        name = bag_file.stem
        read_rosbag_and_generate_histograms(bag_file, output_dir, name)

    if not validation_passed:
        print("Validation failed. Reading error logs.")

        # Read error logs
        with open(LOG_FILE, "r") as f:
            logs = f.read()

        # Get a string with only ERROR logs
        errors = [line for line in logs.split("\n") if "ERROR" in line]

        # Create a GitHub issue with the results
        name = list(pathlib.Path(MISSION_DATA).rglob("*.bag"))[0].name
        mission_name = name.split("_")[0]
        issue_body = (
            f"## Raw Data Verification Results for `{mission_name}`\n\n"
            f"### Validation Failed\n\n"
            f"For topic frequency histograms, check the artifacts folder using the link below.\n\n"
            f"### Error Logs\n```{errors}```\n\n"
            f"<details>\n"
            f"<summary>Click to see all logs</summary>\n\n"
            f"### All Logs:\n```{logs}```"
            f"</details>\n\n"
        )
        create_github_issue(
            title=f"Raw Data Verification Failed for {mission_name}",
            body=issue_body,
        )
        return False
    else:
        return True


if __name__ == "__main__":
    
    if os.environ.get("KLEINKRAM_ACTIVE", False) == "ACTIVE":
        project_uuid = os.environ.get("PROJECT_UUID")
        mission_uuid = os.environ["MISSION_UUID"]
        print(f"Dowloading Mission from KleinKram UUID: {mission_uuid}")
        os.system(f"klein download --project {project_uuid} --mission {mission_uuid} --dest {MISSION_DATA} '*.bag'")

    validation_passed = analyze_mission_and_report()
    return_code = 1
    if not validation_passed:
        return_code = -1

    sys.exit(return_code)

