import time
import zarr
from PIL import Image
import numpy as np

# Path to the ZAR file
zar_file_path = "/media/jonfrey/BoxiS4-2TB/deployment_day_1_subfolder/2024-10-01-11-29-55_nuc_alphasense_cor.zarr"

# Open the ZAR file
store = zarr.open(zar_file_path, mode="r")

# Select an image dataset in the archive (replace 'image_dataset' with actual key)
image_dataset = store["gt_box_alphasense_driver_node_cam5_color_corrected_image_compressed"]["image"]

# Benchmark the read time
start_time = time.time()

num = 50
for i in range(num):
    # Read the image into memory
    image_data = np.array(image_dataset[i])  # Load the entire image array

elapsed_time = time.time() - start_time

# Convert to a PIL image if necessary
image = Image.fromarray(image_data)
image.show()
# Print the time taken
d = elapsed_time / num
print(f"Time to read the image: {elapsed_time:.4f} seconds - {d:.4f} ")
