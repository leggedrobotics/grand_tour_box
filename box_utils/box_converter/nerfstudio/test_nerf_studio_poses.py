import json
import numpy as np


# Function to extract XYZ coordinates from the transformation matrix
def extract_coordinates(transform_matrix):
    """Extracts the x, y, z coordinates from a transformation matrix."""
    return [transform_matrix[0][3], transform_matrix[1][3], transform_matrix[2][3]]


def rotate_points_180_x(points):
    """Rotates a list of points 180 degrees around the x-axis."""
    rotation_matrix = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]])
    return [np.dot(rotation_matrix, point).tolist() for point in points]


# Function to write points to a .ply file
def write_ply(points, filename):
    """Writes a list of points to a PLY file."""
    with open(filename, "w") as f:
        # PLY header
        f.write("ply\n")
        f.write("format ascii 1.0\n")
        f.write(f"element vertex {len(points)}\n")
        f.write("property float x\n")
        f.write("property float y\n")
        f.write("property float z\n")
        f.write("end_header\n")
        # Write point data
        for point in points:
            f.write(f"{point[0]} {point[1]} {point[2]}\n")


# Main function to process the input JSON and generate the point cloud
def main(input_json_path, output_ply_path):
    # Read the input JSON file
    with open(input_json_path, "r") as f:
        data = json.load(f)

    # Extract camera poses and convert to a point cloud
    points = []
    for frame in data["frames"]:
        transform_matrix = frame["transform_matrix"]
        coordinates = extract_coordinates(transform_matrix)
        points.append(coordinates)

    points = rotate_points_180_x(points)
    # Write the point cloud to a PLY file
    write_ply(points, output_ply_path)
    print(f"Point cloud saved to {output_ply_path}")


# Example usage
if __name__ == "__main__":
    input_json = "/media/jonfrey/BoxiS4-2TB/deployment_day_1_subfolder/nerf_studio/transforms.json"  # Replace with your input JSON file path
    output_ply = "/media/jonfrey/BoxiS4-2TB/deployment_day_1_subfolder/nerf_studio/camera_points.ply"  # Replace with your desired output PLY file path
    main(input_json, output_ply)
