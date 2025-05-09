import os
import cv2
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path

MISSION_DATA = Path(os.environ.get("MISSION_DATA", "/tmp_disk"))

# Paths
rgb_folder = MISSION_DATA / "nerf_studio/rgb"
depth_folder = MISSION_DATA / "wavemap"
output_video = depth_folder / "depth_video.mp4"

# Get sorted list of RGB files
rgb_files = sorted(f for f in os.listdir(rgb_folder) if f.endswith(".png"))

# Initialize video writer
fourcc = cv2.VideoWriter_fourcc(*"mp4v")
out = None

# Process images
for rgb_file in rgb_files:
    idx = os.path.splitext(rgb_file)[0].split("_")[-1]  # Extract idx
    depth_file = f"front_{idx}.png"

    rgb_path = os.path.join(rgb_folder, rgb_file)
    depth_path = os.path.join(depth_folder, depth_file)

    if not os.path.exists(depth_path):
        continue  # Skip if depth image doesn't exist

    # Load the RGB image
    rgb_image = cv2.imread(rgb_path, cv2.IMREAD_COLOR)
    rgb_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2RGB)  # Convert BGR to RGB

    # Load the depth image
    depth_image = cv2.imread(depth_path, cv2.IMREAD_UNCHANGED)

    # Normalize depth for visualization
    norm_depth = (depth_image.astype(np.float32) / 1000) / 10  # 10m is max distnace

    # Apply colormap
    colormap = plt.cm.viridis
    colored_depth = colormap(norm_depth)[:, :, :3]  # Ignore alpha channel
    colored_depth = (colored_depth * 255).astype(np.uint8)  # Convert to uint8

    # Blend RGB and colored depth
    alpha = 0.6
    overlay = cv2.addWeighted(rgb_image, 1 - alpha, colored_depth, alpha, 0)

    # Convert back to BGR for saving
    overlay_bgr = cv2.cvtColor(overlay, cv2.COLOR_RGB2BGR)

    # Initialize video writer if not done
    if out is None:
        h, w, _ = overlay_bgr.shape
        out = cv2.VideoWriter(str(output_video), fourcc, 30, (w, h))

    # Write frame to video
    out.write(overlay_bgr)

# Release video writer
if out:
    out.release()

print(f"Video saved to {output_video}")
