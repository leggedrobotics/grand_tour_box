import json
import numpy as np
import open3d as o3d

def post_process_mesh(mesh, cluster_to_keep=1):
    """
    Post-process a mesh to filter out floaters and disconnected parts
    """
    mesh_new = mesh
    with o3d.utility.VerbosityContextManager(o3d.utility.VerbosityLevel.Debug) as cm:
            triangle_clusters, cluster_n_triangles, cluster_area = (mesh_new.cluster_connected_triangles())

    triangle_clusters = np.asarray(triangle_clusters)
    cluster_n_triangles = np.asarray(cluster_n_triangles)
    cluster_area = np.asarray(cluster_area)
    n_cluster = np.sort(cluster_n_triangles.copy())[-cluster_to_keep]
    n_cluster = max(n_cluster, 30)
    triangles_to_remove = cluster_n_triangles[triangle_clusters] < n_cluster
    mesh_new.remove_triangles_by_mask(triangles_to_remove)
    mesh_new.remove_degenerate_triangles()
    mesh_new.remove_unreferenced_vertices()
    return mesh_new

# Configuration
base_path = ""
depth_prefix = base_path + "/depth"
color_prefix = base_path + "/frame"
depth_ext = ".png"
# depth_ext = ".exr"
color_ext = ".jpg"
num_frames = 90  # number of frames saved previously
camera_info_file = base_path + "/camera_info.json"
poses_file = base_path + "/traj.txt"

# 1. Load camera intrinsics from JSON
with open(camera_info_file, 'r') as f:
    cam_info = json.load(f)

w = cam_info["camera"]["w"]
h = cam_info["camera"]["h"]
fx = cam_info["camera"]["fx"]
fy = cam_info["camera"]["fy"]
cx = cam_info["camera"]["cx"]
cy = cam_info["camera"]["cy"]
scale = cam_info["camera"]["scale"]

intrinsics = o3d.camera.PinholeCameraIntrinsic(
    width=w,
    height=h,
    fx=fx,
    fy=fy,
    cx=cx,
    cy=cy
)

# 2. Load poses
poses = []
with open(poses_file, 'r') as f:
    for line in f:
        # Each line has 16 values
        vals = list(map(float, line.strip().split()))
        if len(vals) == 16:
            pose = np.array(vals, dtype=np.float64).reshape((4, 4))
            poses.append(pose)
        else:
            print("Warning: Invalid line in poses file:", line)


if len(poses) < num_frames:
    print("Warning: Found fewer poses than expected frames.")

# 3. Prepare TSDF Volume
# Choose appropriate parameters for your use-case
voxel_length = 0.04  # 5 mm voxel size
sdf_trunc = 0.05      # truncation distance
color_type = o3d.pipelines.integration.TSDFVolumeColorType.RGB8

volume = o3d.pipelines.integration.ScalableTSDFVolume(
    voxel_length=voxel_length,
    sdf_trunc=sdf_trunc,
    color_type=color_type
)

# 4. Integrate frames
for i in range(num_frames):
    depth_path = f"{depth_prefix}{i}{depth_ext}"
    color_path = f"{color_prefix}{i}{color_ext}"

    depth_img = o3d.io.read_image(depth_path)
    color_img = o3d.io.read_image(color_path)

    if depth_img is None or color_img is None:
        print(f"Skipping frame {i}: could not load images.")
        continue

    if i >= len(poses):
        print(f"No pose for frame {i}, skipping.")
        continue

    pose = poses[i]

    # Create an RGBD image for integration
    # Depth is assumed to be in millimeters converted to the depth units
    # If your depth is in meters already, ensure your scaling matches how you saved the depth.
    rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
        color_img, 
        depth_img, 
        depth_scale=scale,  # Adjust if your depth needs scaling. If saved as mm and Open3D expects meters, set accordingly.
        depth_trunc=15.0,  # max depth distance
        convert_rgb_to_intensity=False
    )

    # Integrate the frame into the TSDF volume
    # The pose is the camera extrinsic: converts points from camera frame to world (map) frame
    volume.integrate(rgbd, intrinsics, np.linalg.inv(pose)) #np.linalg.inv(pose)

print("Integration complete.")

# 5. Extract a mesh or point cloud from the volume
# mesh = volume.extract_triangle_mesh()
# mesh.compute_vertex_normals()


with o3d.utility.VerbosityContextManager(o3d.utility.VerbosityLevel.Debug) as cm:
    try:
        mesh = volume.extract_triangle_mesh()
    except Exception as e:
        print(e)
    mesh.remove_unreferenced_vertices()
    mesh.remove_degenerate_triangles()


# cleaned_mesh = post_process_mesh(mesh, 1)
# cleaned_mesh.compute_vertex_normals()

# Save the mesh if desired
o3d.io.write_triangle_mesh(base_path + "/integrated_mesh.ply", mesh, print_progress=True) # ,  write_triangle_uvs=True, write_vertex_colors=True, write_vertex_normals=True)
print("Saved integrated_mesh.ply")

# pcd = volume.extract_point_cloud()
# o3d.io.write_point_cloud("integrated_pointcloud.ply", pcd)
# print("Saved integrated_pointcloud.ply")

print("Validation and integration finished.")
