import os
import yaml


def create_mission_yaml(directory, missions):
    # Ensure the directory exists
    if not os.path.exists(directory):
        os.makedirs(directory)

    # Define common values
    robot_name = "Democracy"
    location = "Wangen / ARCHE"
    gnss = "Working"
    deployment_day = 14

    # Loop over the missions and create YAML files
    for mission in missions:
        mission_filename = f"{mission}.yaml"
        mission_path = os.path.join(directory, mission, mission_filename)

        mission_data = {
            "Description": "",
            "Google Drive Files": "",
            "Robot Name": robot_name,
            "Location": location,
            "GNSS": gnss,
            "Deployment Day": deployment_day,
        }

        # Write the data to a YAML file
        with open(mission_path, "w") as file:
            yaml.dump(mission_data, file, sort_keys=False)

        print(f"Created YAML file: {mission_path}")


# Input: directory and missions
directory = "/data/deployment_day_14"  # Replace with your directory path
missions = [
    "2024-11-18-11-42-04",
    "2024-11-18-12-05-01",
    "2024-11-18-13-22-14",
    "2024-11-18-13-48-19",
    "2024-11-18-15-46-05",
    "2024-11-18-16-45-27",
    "2024-11-18-16-59-23",
    "2024-11-18-17-13-09",
    "2024-11-18-17-31-36",
]

# Create the YAML files
create_mission_yaml(directory, missions)
