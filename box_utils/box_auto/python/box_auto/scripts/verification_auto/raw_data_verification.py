import pathlib
import json
import os
import requests
import sys
from box_auto.utils import create_github_issue, MISSION_DATA, ARTIFACT_FOLDER, WS, get_bag
from box_auto.scripts.verification.health_check_mission import validate_bags, LOG_FILE

# from box_auto.scripts.verification.topic_freq import read_rosbag_and_generate_histograms


def fetch_mission_metadata_and_append():
    """
    Fetch metadata for the current mission and append tags (keys and values)
    to /out/health_check_mission/result.txt.
    """
    file_path = os.path.expanduser("~/.kleinkram.json")

    if os.environ.get("APIKEY", False):
        cookie = f"clikey={os.environ['APIKEY']}"
    else:
        try:
            with open(file_path, "r") as file:
                data = json.load(file)
                refresh_token = data["credentials"]["https://api.datasets.leggedrobotics.com"]["refresh_token"]
                auth_token = data["credentials"]["https://api.datasets.leggedrobotics.com"]["auth_token"]
                cookie = f"refreshtoken={refresh_token}; authtoken={auth_token}"
        except (FileNotFoundError, json.JSONDecodeError):
            print("Error: KleinKram credentials not found.")
            return ""

    mission_uuid = os.environ.get("MISSION_UUID")
    if not mission_uuid:
        print("Error: MISSION_UUID environment variable is not set.")
        return ""

    mission_url = f"https://api.datasets.leggedrobotics.com/mission/one?uuid={mission_uuid}"
    headers = {
        "accept": "application/json",
        "cookie": cookie,
    }

    response = requests.get(mission_url, headers=headers)
    if response.status_code != 200:
        print(f"Error {response.status_code}: {response.text}")
        return ""

    mission_data = response.json()
    tags = mission_data.get("tags", [])

    # Process tags into key-value format
    tag_entries = []
    for tag in tags:
        tag_value = None
        for key in ["STRING", "NUMBER", "BOOLEAN", "LOCATION"]:
            if tag[key] is not None:
                tag_value = str(tag[key])
                break
        if tag_value:
            tag_entries.append(f"{tag['tagType']['name']}: {tag_value}")

    # Prepend results to the output file
    output_path = pathlib.Path(ARTIFACT_FOLDER) / "health_check_mission" / "result.txt"
    output_path.parent.mkdir(parents=True, exist_ok=True)
    new_content = f"Mission: {mission_uuid}\nTags:\n" + "\n".join(f"  {entry}" for entry in tag_entries) + "\n\n"

    return new_content


def analyze_mission_and_report():
    """
    Analyze the raw data from a mission in the MISSION_DATA directory and report the results.

    Returns:
        bool: True if the validation passed, False otherwise.
    """

    yaml_path = str(pathlib.Path(WS) / "src/grand_tour_box/box_utils/box_auto/cfg/health_check_reference_raw_data.yaml")
    validation_passed = validate_bags(
        reference_folder=None,
        yaml_file=yaml_path,
        mission_folder=MISSION_DATA,
        time_tolerance=20,
    )

    # Fetch metadata for the current mission and append to the health check results
    metadata = fetch_mission_metadata_and_append()

    # Generate histograms of the topics
    # output_dir = pathlib.Path(ARTIFACT_FOLDER) / "topic_frequency_histograms"
    # for bag_file in pathlib.Path(MISSION_DATA).rglob("*.bag"):
    #     name = bag_file.stem
    #     read_rosbag_and_generate_histograms(bag_file, output_dir, name)

    if not validation_passed:
        print("Validation failed. Reading error logs.")

        # Read error logs
        with open(LOG_FILE, "r") as f:
            logs = f.read()

        # Get a string with only ERROR logs
        errors = [line for line in logs.split("\n") if "ERROR" in line]
        error_string = "\n".join(errors)

        # Create a GitHub issue with the results
        name = list(pathlib.Path(MISSION_DATA).rglob("*.bag"))[0].name
        mission_name = name.split("_")[0]
        issue_body = (
            f"## Raw Data Verification Results for `{mission_name}`\n\n"
            f"### Mission Metadata\n{metadata}\n"
            f"### Validation Failed\n\n"
            f"For topic frequency histograms and full logs, check the artifacts folder using the link below.\n\n"
            f"### Error Logs\n``` \n"
            f"{error_string}```\n\n"
        )
        create_github_issue(
            title=f"Raw Data Verification Failed for {mission_name}",
            body=issue_body,
        )
        return False
    else:
        return True


if __name__ == "__main__":
    get_bag("*.bag")

    validation_passed = analyze_mission_and_report()
    return_code = 1
    if not validation_passed:
        return_code = -1

    sys.exit(return_code)
