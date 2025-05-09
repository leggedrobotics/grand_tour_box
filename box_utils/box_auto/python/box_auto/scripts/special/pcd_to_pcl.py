import rosbag
import tf
import sensor_msgs.point_cloud2 as pc2
import open3d as o3d
import numpy as np
from tf import transformations
from tf_bag import BagTfTransformer
from pytictac import CpuTimer
from tqdm import tqdm


# Function to extract points from a PointCloud2 message
def point_cloud2_to_numpy(msg):
    # Unpack PointCloud2 data to a numpy array
    pc_data = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
    pc_array = np.array(list(pc_data))
    return pc_array


# Function to transform points from the source frame to the target frame
def transform_points(points, transform):
    # Apply the transformation (rotation and translation)
    rotation_matrix = transformations.quaternion_matrix(transform[1])
    translation = np.array(transform[0])
    points_transformed = np.dot(rotation_matrix[:3, :3], points.T).T + translation
    return points_transformed


# Set the path to the bag file
tf_bag_file_path = "/data/DigiForest/2024-07-11-11-03-48/2024-07-11-11-03-48_hesai_dlio_tf.bag"
bag_file_path = "/data/DigiForest/2024-07-11-11-03-48/2024-07-11-11-03-48_hesai_dlio.bag"

with CpuTimer("tf_lookup"):
    tf_lookup = BagTfTransformer(tf_bag_file_path)

step = 600
# Initialize the bag reader to read the bag file
with rosbag.Bag(bag_file_path, "r") as bag:

    global_pcd = o3d.geometry.PointCloud()
    voxel_size = 0.02
    total_messages = bag.get_message_count("/dlio/deskewed_point_cloud")
    count = 0
    # Iterate through the messages in the bag
    for topic, msg, t in tqdm(
        bag.read_messages(topics=["/dlio/deskewed_point_cloud"]),
        desc="Processing messages",
        total=total_messages,  # Set total for the progress bar
        unit="message",
    ):
        points = point_cloud2_to_numpy(msg)

        try:
            # Get the transform from hesai_lidar to dlio_map frame
            transform = tf_lookup.lookupTransform("dlio_map", msg.header.frame_id, t)
            # Transform the point cloud from hesai_lidar frame to dlio_map frame
            transformed_points = transform_points(points, transform)

            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(transformed_points)

            # Append the transformed points to the global point cloud
            global_pcd += pcd

            # global_pcd = voxel_pcd

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            print(f"Error looking up transform: {e}")

        count += 1

        if count % step == 0:
            voxel_pcd = global_pcd.voxel_down_sample(voxel_size)
            global_pcd = voxel_pcd
            o3d.io.write_point_cloud(
                f"/data/DigiForest/2024-07-11-11-03-48/2024-07-11-11-03-48_hesai_dlio_{count}.pcd", global_pcd
            )
            global_pcd = o3d.geometry.PointCloud()
            print("Saved point cloud at count: ", count)

global_pcd = o3d.geometry.PointCloud()
for i in range(0, count, step):
    pcd = o3d.io.read_point_cloud(f"/data/DigiForest/2024-07-11-11-03-48/2024-07-11-11-03-48_hesai_dlio_{i}.pcd")
    global_pcd += pcd
    voxel_pcd = global_pcd.voxel_down_sample(voxel_size)
    o3d.io.write_point_cloud(
        "/data/DigiForest/2024-07-11-11-03-48/2024-07-11-11-03-48_hesai_dlio_merged.pcd", global_pcd
    )
# Convert the accumulated global map into a single numpy array
# global_map = np.concatenate(global_map, axis=0)

# Visualize the global map using Open3D
# pcd = o3d.geometry.PointCloud()
# pcd.points = o3d.utility.Vector3dVector(global_map)
# o3d.visualization.draw_geometries([pcd])
