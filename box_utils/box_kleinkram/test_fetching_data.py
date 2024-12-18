import requests
import gdown
import os
import json
from pathlib import Path

# Resolve the full path of the JSON file
file_path = os.path.expanduser("~/.kleinkram.json")
# Open and read the JSON file
try:
    with open(file_path, "r") as file:
        data = json.load(file)
        print("JSON data:", data)  # Print or process the data
except FileNotFoundError:
    print(f"The file '{file_path}' does not exist.")
except json.JSONDecodeError as e:
    print(f"Error decoding JSON: {e}")

login_using_cli_info = True
if not login_using_cli_info:
    print(
        "Please open the following URL manually to authenticate: https://api.datasets.leggedrobotics.com/auth/google?state=cli-no-redirect"
    )
    print("Enter the authentication token provided after logging in:")
    from getpass import getpass

    auth_token = getpass("Authentication Token: ")
    refresh_token = getpass("Refresh Token: ")
else:
    refresh_token = data["credentials"]["https://api.datasets.leggedrobotics.com"]["refresh_token"]
    auth_token = data["credentials"]["https://api.datasets.leggedrobotics.com"]["auth_token"]


project_uuid = "3c97a27e-4180-4e40-b8af-59714de54a87"  # input("Project UUID:")


url = "https://api.datasets.leggedrobotics.com/action/listActions"
metadata_folder = "/data/GrandTour/kleinkram_actions"
artifacts_folder = Path(metadata_folder) / "artifacts"
artifacts_folder.mkdir(exist_ok=True, parents=True)
download = True

params = {"project_uuid": f"{project_uuid}", "take": 20, "skip": 0, "sortBy": "createdAt", "sortDirection": "DESC"}

headers = {
    "accept": "application/json",
    "cookie": f"refreshtoken={refresh_token}; authtoken={auth_token}",
}
response = requests.get(url, headers=headers, params=params)

if response.status_code == 200:
    actions = response.json()
    for action in actions[0]:
        action_uuid = action["uuid"]
        artifact_url = action["artifact_url"]

        if artifact_url is None:
            print(f"No artifact found for action {action_uuid}")
            continue

        print(f"Downloading artifact for action {action_uuid}: {artifact_url}")

        if download:
            action_artifact_folder = artifacts_folder / action_uuid
            action_artifact_folder.mkdir(exist_ok=True, parents=True)
            gdown.download_folder(artifact_url, output=str(action_artifact_folder))
else:
    print(f"Error {response.status_code}: {response.text}")
