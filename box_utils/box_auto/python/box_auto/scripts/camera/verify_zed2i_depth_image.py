import rosbag
import cv2
import numpy as np

bag_new = rosbag.Bag("2024-10-01-11-29-55_zed2i_depth_new.bag")
for topic, msg, t in bag_new.read_messages(topics=["/boxi/zed2i/depth/image_raw/compressedDepth"]):
    # Skip the first 12 bytes (header)
    arr = np.frombuffer(msg.data, dtype=np.uint8)
    png_bytes = arr[12:]  # skip 12-byte header
    img = cv2.imdecode(png_bytes, cv2.IMREAD_UNCHANGED)
    if img is None:
        raise RuntimeError("PNG decode failed")
    print("dtype:", img.dtype)
    print("shape:", img.shape)
    print("min:", img.min())
    print("max:", img.max())
    print("format:", msg.format)
    flat_img = img.flatten()
    idx = np.random.choice(flat_img.size, 100, replace=False)
    print("random 100 pixel values:", flat_img[idx])
    cv2.imwrite("depth_image.png", img)
    break
