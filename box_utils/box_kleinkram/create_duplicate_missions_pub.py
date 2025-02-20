import requests
import os
import json

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


project_uuid = "3c97a27e-4180-4e40-b8af-59714de54a87"  # GrandTour
# project_uuid = "e5a38ae3-78b9-45c1-b682-1cc956f70791"  # GrandTourDev

url = f"https://api.datasets.leggedrobotics.com/mission/filtered?uuid={project_uuid}&take=999"

headers = {
    "accept": "application/json",
    "cookie": f"refreshtoken={refresh_token}; authtoken={auth_token}",
}
response = requests.get(url, headers=headers)
overview = {}

# Create new mission with _pub
if response.status_code == 200:
    missions = response.json()
    for m in missions[0]:
        ori_name = m["name"]
        if "pub" not in ori_name:
            overview[ori_name] = {"uuid": m["uuid"], "uuid_pub": "nan"}
            new_name = "pub_" + ori_name
            os.system(
                f"klein upload --mission {new_name} --project GrandTour --create /home/jonfrey/git/grand_tour_box/box_calibration/box_calibration/calibration/tf_static.bag"
            )

# Fetch the mapping from uuid to _uuid
response = requests.get(url, headers=headers)
if response.status_code == 200:
    missions = response.json()
    for m in missions[0]:
        name = m["name"]
        if "pub_" in name:
            ori_name = m["name"].replace("pub_", "")
            overview[ori_name]["uuid_pub"] = m["uuid"]

print(overview)
