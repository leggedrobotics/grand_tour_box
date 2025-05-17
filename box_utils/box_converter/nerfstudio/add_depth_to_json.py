import json
import os
import shutil

# Path to the original JSON file
json_file_path = "/data/GrandTour/nerfstudio_euler/nerfstudio_scratch/2024-11-04-10-57-34_nerfstudio/transforms.json"

# Create a backup of the original JSON file
backup_file_path = json_file_path.replace(".json", "_old.json")
shutil.copy(json_file_path, backup_file_path)
print(f"Backup created: {backup_file_path}")

# Load the JSON data
with open(json_file_path, "r") as file:
    data = json.load(file)

# Add "depth_file_path" to each frame
for frame in data.get("frames", []):
    # Extract the file name from the existing "file_path"
    file_name = os.path.basename(frame["file_path"])
    # Construct the depth file path
    frame["depth_file_path"] = f"./depth/{file_name}"

# Save the modified JSON data back to the original file
with open(json_file_path, "w") as file:
    json.dump(data, file, indent=4)
print(f"Updated JSON file saved: {json_file_path}")
