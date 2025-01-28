import pathlib
import os
import sys
from box_auto.utils import create_github_issue, MISSION_DATA, GRAND_TOUR_UUID
from box_auto.scripts.verification.health_check_mission import validate_bags, LOG_FILE


def analyze_mission_and_report():
    """
    Analyze the raw data from a mission in the MISSION_DATA directory and report the results.

    Returns:
        bool: True if the validation passed, False otherwise.
    """

    # Comment in to generate new reference data
    validation_passed = validate_bags(
        reference_folder=None,
        yaml_file="default",
        mission_folder=MISSION_DATA,
        time_tolerance=20,
    )

    if not validation_passed:
        print("Validation failed. Reading error logs.")

        # Read error logs
        with open(LOG_FILE, "r") as f:
            logs = f.read()

        # Create a GitHub issue with the results
        # TODO(kappi): Get this in a neater way, using kleinkram API
        name = list(pathlib.Path(MISSION_DATA).rglob("*.bag"))[0].name
        mission_name = name.split("_")[0]
        issue_body = (
            f"## Raw Data Verification Results for `{mission_name}`\n\n"
            f"### Validation Failed\n\n"
            f"### Logs:\n```{logs}```"
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
        project_uuid = os.environ.get("PROJECT_UUID", GRAND_TOUR_UUID)
        mission_uuid = os.environ["MISSION_UUID"]
        print(f"Dowloading Mission from KleinKram UUID: {mission_uuid}")
        os.system(f"klein download --project {project_uuid} --mission {mission_uuid} --dest {MISSION_DATA} '*.bag'")

    validation_passed = analyze_mission_and_report()
    return_code = 1
    if not validation_passed:
        return_code = -1

    sys.exit(return_code)

