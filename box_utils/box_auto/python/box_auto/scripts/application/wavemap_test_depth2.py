import json
import numpy as np
import open3d as o3d
import cv2
from pathlib import Path

# Load the JSON file
json_file_path = "/data/GrandTour/nerfstudio_euler/nerfstudio_scratch/2024-10-01-11-29-55_nerfstudio/transforms.json"
with open(json_file_path, "r") as f:
    data = json.load(f)

# Extract the first frame
frame = data["frames"][0]

# Load the depth image
depth_image_path = (
    str(Path(json_file_path).parent / frame["file_path"])
    .replace("nerfstudio_scratch", "depth_scratch")
    .replace("rgb", "depth")
)
depth = cv2.imread(depth_image_path, cv2.IMREAD_UNCHANGED)

# Convert camera intrinsics
fl_x = float(frame["fl_x"])
fl_y = float(frame["fl_y"])
cx = float(frame["cx"])
cy = float(frame["cy"])
width = int(frame["w"])
height = int(frame["h"])

# Create Open3D camera intrinsic
intrinsic = o3d.camera.PinholeCameraIntrinsic(width, height, fl_x, fl_y, cx, cy)

# Convert the depth to Open3D format
depth_o3d = o3d.geometry.Image(depth)

rgb_image = cv2.imread(str(Path(json_file_path).parent / frame["file_path"]))
rgb_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2RGB)
color_o3d = o3d.geometry.Image(rgb_image)


# rgb_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2RGB)  # Convert BGR to RGB
# Convert to Open3D format


rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(
    color_o3d,
    depth_o3d,
    convert_rgb_to_intensity=False,
    depth_scale=1000.0,  # Adjust if your depth is in mm
    depth_trunc=25.0,  # Truncate depths beyond 3 meters
)


# Create point cloud
pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd_image, intrinsic)

# Apply the transform matrix to the point cloud
transform_matrix = np.array(frame["transform_matrix"])
pcd.transform(transform_matrix)

# Visualize
o3d.visualization.draw_geometries([pcd])
