#!/usr/bin/python3
import argparse
from collections import OrderedDict

import networkx as nx
import numpy as np
import yaml
from matplotlib import pyplot as plt
from scipy.spatial.transform import Rotation as R


# Custom YAML Dumper to handle OrderedDict cleanly
class OrderedDumper(yaml.Dumper):
    def represent_dict(self, data):
        return super().represent_dict(data.items())


OrderedDumper.add_representer(OrderedDict, OrderedDumper.represent_dict)


class Graph:
    def __init__(self):
        self.input_edges = {}  # Edges: {node1: {node2: transform_matrix}}
        self.input_edge_tags = {}  # Edge tags: {(node1, node2): "tag"}
        self.traversal_edges = {}

    def _to_se3(self, rpy=None, xyz=None, se3_matrix=None):
        """
        Convert input to an SE(3) transformation matrix.

        Parameters:
        - rpy (tuple): A tuple of (roll, pitch, yaw) in radians (optional).
        - xyz (tuple): A tuple of (x, y, z) translation (optional).
        - se3_matrix (np.ndarray): A 4x4 SE(3) transformation matrix (optional).

        Returns:
        - np.ndarray: A 4x4 SE(3) transformation matrix.
        """
        if se3_matrix is not None:
            if se3_matrix.shape == (4, 4):
                return se3_matrix
            else:
                raise ValueError("Provided SE(3) matrix must be a 4x4 array.")
        else:
            # Default to identity if RPY or XYZ is not provided
            rotation = np.eye(3)
            translation = [0, 0, 0]

            # Apply RPY if provided
            if rpy is not None:
                if len(rpy) == 3:
                    rotation = R.from_euler('xyz', rpy).as_matrix()
                else:
                    raise ValueError("RPY must be a tuple of (roll, pitch, yaw).")

            # Apply XYZ if provided
            if xyz is not None:
                if len(xyz) == 3:
                    translation = xyz
                else:
                    raise ValueError("XYZ must be a tuple of (x, y, z).")

            # Construct the SE(3) matrix
            transform = np.eye(4)
            transform[:3, :3] = rotation
            transform[:3, 3] = translation
            return transform

    def add_edge(self, node1, node2, transform=None, rpy=None, xyz=None, tag=None):
        """
        Adds a bidirectional edge between two nodes with the given SE(3) transform.
        Accepts either an SE(3) matrix or separate RPY and XYZ components.

        Parameters:
        - node1, node2 (str): Names of the nodes to connect.
        - transform (np.ndarray): 4x4 SE(3) transformation matrix (optional).
        - rpy (tuple): A tuple of (roll, pitch, yaw) in radians (optional).
        - xyz (tuple): A tuple of (x, y, z) translation (optional).
        - tag (str): A descriptive tag for the edge (optional).
        """
        transform = self._to_se3(rpy=rpy, xyz=xyz, se3_matrix=transform)

        if node1 not in self.traversal_edges:
            self.traversal_edges[node1] = {}
        if node2 not in self.traversal_edges:
            self.traversal_edges[node2] = {}

        if node1 not in self.input_edges:
            self.input_edges[node1] = {}
        self.input_edges[node1][node2] = transform
        self.traversal_edges[node1][node2] = transform
        self.traversal_edges[node2][node1] = np.linalg.inv(transform)
        # Add tag to the edge
        if tag:
            self.input_edge_tags[(node1, node2)] = tag

    def get_transform(self, from_node, to_node):
        """
        Retrieve the transform matrix between two directly connected nodes.
        """
        return self.traversal_edges.get(from_node, {}).get(to_node, None)

    def extract_rotation_as_euler(self, transform, order="xyz"):
        """
        Extract the rotation component of an SE(3) transform as Euler angles.

        Parameters:
        - transform (np.ndarray): 4x4 SE(3) transformation matrix.
        - order (str): The order of axes for Euler angles (default: "xyz").

        Returns:
        - np.ndarray: Euler angles in radians.
        """
        rotation_matrix = transform[:3, :3]
        rotation = R.from_matrix(rotation_matrix)
        return rotation.as_euler(order)

    def extract_translation(self, transform):
        """
        Extract the translation vector from an SE(3) transform.

        Parameters:
        - transform (np.ndarray): 4x4 SE(3) transformation matrix.

        Returns:
        - np.ndarray: Translation vector as a 1D array (x, y, z).
        """
        return transform[:3, 3]

    def compute_transform(self, from_node, to_node, log=False):
        """
        Compute the transform matrix between two nodes by traversing the graph.
        """

        def dfs(current, target, path, transform):
            if current == target:
                return path, transform
            for neighbor, edge_transform in self.traversal_edges.get(current, {}).items():
                if neighbor not in path:
                    returned_path, result = dfs(neighbor, target, path + [neighbor], transform @ edge_transform)
                    if returned_path[-1] == target:
                        return returned_path, result
            return path, None

        path, transform = dfs(from_node, to_node, [from_node], np.eye(4))
        if path[-1] != to_node:
            raise ValueError(f"No path found from {from_node} to {to_node}")
        if log:
            print(" -> ".join(path))
        return transform

    def build_new_graph(self, new_structure):
        """
        Build a new graph with the given structure. The new_structure should
        be a list of tuples (parent, child) defining the new edges.
        """
        new_graph = Graph()
        for edge in new_structure:
            if len(edge) == 2:  # If no tag is provided
                parent, child = edge
                tag = None
            elif len(edge) == 3:  # If tag is provided
                parent, child, tag = edge
            else:
                raise ValueError(f"Invalid edge definition: {edge}")

            transform = self.compute_transform(parent, child, log=True)
            new_graph.add_edge(parent, child, transform, tag=tag)
        return new_graph

    def find_root(self):
        """
        Identify the root node of the graph. A root node is a node with no parents.
        """
        # Collect all nodes with parents
        all_children = set(child for edges in self.input_edges.values() for child in edges)
        all_nodes = set(self.input_edges.keys()).union(all_children)
        root_candidates = all_nodes - all_children

        if len(root_candidates) == 0:
            raise ValueError("Graph has no root. It may be cyclic or disconnected.")
        elif len(root_candidates) > 1:
            raise ValueError(f"Graph has multiple roots: {root_candidates}")

        return next(iter(root_candidates))

    def validate_single_root(self):
        """
        Validate that the graph has a single root node.
        """
        try:
            root = self.find_root()
            print(f"The graph has a single root: {root}")
        except ValueError as e:
            print(f"Validation Error: {e}")

    def visualize_by_depth(self, root=None, title="Graph Visualization by Depth"):
        """
        Visualize the graph with nodes ordered by depth in the tree.
        Nodes are arranged in layers corresponding to their depth from the root.

        Parameters:
        - root (str): The root node of the tree. If None, the method will find the root automatically.
        - title (str): Title for the visualization.
        """
        if root is None:
            root = self.find_root()  # Automatically determine the root if not provided

        # Compute depth for each node using BFS
        depths = {}
        queue = [(root, 0)]  # (node, depth)
        while queue:
            node, depth = queue.pop(0)
            if node not in depths:
                depths[node] = depth
                for neighbor in self.input_edges.get(node, {}):
                    if neighbor not in depths:
                        queue.append((neighbor, depth + 1))

        # Organize nodes by depth
        layers = {}
        for node, depth in depths.items():
            if depth not in layers:
                layers[depth] = []
            layers[depth].append(node)

        # Assign positions for nodes based on depth
        pos = {}
        y_spacing = 1.5  # Vertical spacing between layers
        x_spacing = 2.0  # Horizontal spacing within a layer
        for depth, nodes in layers.items():
            for i, node in enumerate(nodes):
                pos[node] = (i * x_spacing, -depth * y_spacing)

        # Create a directed graph for visualization
        G = nx.DiGraph()
        edge_labels = {}
        for (node1, neighbors) in self.input_edges.items():
            for node2, transform in neighbors.items():
                tag = self.input_edge_tags.get((node1, node2), "")  # Get tag if exists
                rpy = self.extract_rotation_as_euler(transform)
                xyz = self.extract_translation(transform)
                G.add_edge(node1, node2)
                edge_labels[(node1, node2)] = f"{tag}\nxyz: {xyz}\nrpy: {rpy}"

        # Draw the graph
        plt.figure(figsize=(12, 8))
        nx.draw(G, pos, with_labels=True, node_color="lightblue", node_size=2000, font_size=12, font_weight="bold",
                arrows=True)

        # Draw edge labels (tags)
        nx.draw_networkx_edge_labels(G, pos, edge_labels=edge_labels, font_size=8)

        plt.title(title)
        plt.show()

    def save_tagged_edges_to_yaml(self, output_file):
        """
        Save all edges with tags into a YAML file with fields for X, Y, Z, roll, pitch, and yaw.
        The key in the YAML file is the edge tag.

        Parameters:
        - output_file (str): Path to the YAML file where edges will be saved.
        """
        tagged_edges = OrderedDict()

        for (node1, node2), tag in self.input_edge_tags.items():
            transform = self.get_transform(node1, node2)
            if transform is None:
                continue

            xyz = self.extract_translation(transform).tolist()
            rpy = self.extract_rotation_as_euler(transform, order="xyz").tolist()

            # Save fields in a dictionary using the tag as the key
            tagged_edges[tag] = OrderedDict([
                ('x', xyz[0]),
                ('y', xyz[1]),
                ('z', xyz[2]),
                ('roll', rpy[0]),
                ('pitch', rpy[1]),
                ('yaw', rpy[2])
            ])

        # Write to a YAML file
        with open(output_file, 'w') as f:
            yaml.dump(tagged_edges, f, Dumper=OrderedDumper, default_flow_style=False)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(prog="Converts calibration output yaml into a YAML used by the URDF.")
    parser.add_argument("-i", "--input_calibration_yaml", help="Path to the input calibration yaml file.",
                        required=True)
    parser.add_argument("-o", "--output_urdf_compatible_yaml",
                        help="Path to the output URDF compatible calibration yaml file.", )
    parser.add_argument("-imu", "--input_imu_calibration_yaml", help="Path to the input IMU calibration yaml file.",
                        required=False, default="")
    args = parser.parse_args()
    with open(args.input_calibration_yaml, "r") as f:
        calibration_data = yaml.safe_load(f)
    calibration_data_by_topic = {v["rostopic"]: v for v in calibration_data.values()}

    # Define the old graph
    calibration_graph = Graph()
    identity = np.eye(4)

    camera_bundle_topic = "/gt_box/alphasense_driver_node/cam1"

    calibration_graph.add_edge("cam1_sensor_frame", "cam2_sensor_frame",
                               transform=np.array(
                                   calibration_data_by_topic["/gt_box/alphasense_driver_node/cam2"]["T_bundle_camera"]))
    calibration_graph.add_edge("cam1_sensor_frame", "cam3_sensor_frame",
                               transform=np.array(
                                   calibration_data_by_topic["/gt_box/alphasense_driver_node/cam3"]["T_bundle_camera"]))
    calibration_graph.add_edge("cam1_sensor_frame", "cam4_sensor_frame",
                               transform=np.array(
                                   calibration_data_by_topic["/gt_box/alphasense_driver_node/cam4"]["T_bundle_camera"]))
    calibration_graph.add_edge("cam1_sensor_frame", "cam5_sensor_frame",
                               transform=np.array(
                                   calibration_data_by_topic["/gt_box/alphasense_driver_node/cam5"]["T_bundle_camera"]))
    calibration_graph.add_edge("cam1_sensor_frame", "hdr_front",
                               transform=np.array(
                                   calibration_data_by_topic["/gt_box/hdr_front/image_raw"]["T_bundle_camera"]))
    calibration_graph.add_edge("cam1_sensor_frame", "hdr_left",
                               transform=np.array(
                                   calibration_data_by_topic["/gt_box/hdr_left/image_raw"]["T_bundle_camera"]))
    calibration_graph.add_edge("cam1_sensor_frame", "hdr_right",
                               transform=np.array(
                                   calibration_data_by_topic["/gt_box/hdr_right/image_raw"]["T_bundle_camera"]))
    calibration_graph.add_edge("cam1_sensor_frame", "zed2i_left_camera_optical_frame",
                               transform=np.array(
                                   calibration_data_by_topic["/gt_box/zed2i_driver_node/left_raw/image_raw_color"]
                                   ["T_bundle_camera"]))
    calibration_graph.add_edge("cam1_sensor_frame", "zed2i_right_camera_optical_frame",
                               transform=np.array(
                                   calibration_data_by_topic["/gt_box/zed2i_driver_node/right_raw/image_raw_color"]
                                   ["T_bundle_camera"]))

    if "T_bundle_hesai" in calibration_data_by_topic[camera_bundle_topic]:
        calibration_graph.add_edge("cam1_sensor_frame", "hesai_lidar",
                                   transform=np.array(calibration_data_by_topic[camera_bundle_topic]["T_bundle_hesai"]))
    else:
        calibration_graph.add_edge("box_base", "hesai_lidar",
                                   xyz=[0.303281493778, -0.036500000014, -0.005456477156000006],
                                   rpy=[3.141592653589793, 0.000, -1.571])

    if "T_bundle_livox" in calibration_data_by_topic[camera_bundle_topic]:
        calibration_graph.add_edge("cam1_sensor_frame", "livox_lidar",
                                   transform=np.array(calibration_data_by_topic[camera_bundle_topic]["T_bundle_livox"]))
    else:
        calibration_graph.add_edge("box_base", "livox_lidar",
                                   xyz=[0.303281493778, -0.036500000014, -0.100456477156],
                                   rpy=[0.0, 0.0, 0.0])

    has_imu_calibration = args.input_imu_calibration_yaml != ""
    if has_imu_calibration:
        with open(args.input_imu_calibration_yaml, "r") as f:
            imu_frame_data = yaml.safe_load(f)

        if "cpt7_imu" in imu_frame_data:
            calibration_graph.add_edge("cam1_sensor_frame", "cpt7_imu",
                                       transform=np.array(imu_frame_data["cpt7_imu"]["T_camerabundle_imu"]))
        else:
            calibration_graph.add_edge("cpt7_imu", "cam1_sensor_frame",
                                       xyz=[0.369, -0.046, 0.068],
                                       rpy=[1.397, -0.001, 1.571])
        if "livox_imu" in imu_frame_data:
            calibration_graph.add_edge("cam1_sensor_frame", "livox_imu",
                                       transform=np.array(imu_frame_data["livox_imu"]["T_camerabundle_imu"]))
        else:
            calibration_graph.add_edge("cpt7_imu", "livox_imu",
                                       xyz=[0.321, -0.023, -0.151],
                                       rpy=[0.00, 0.0, 0.0])
        if "imu_sensor_frame" in imu_frame_data:
            calibration_graph.add_edge("cam1_sensor_frame", "imu_sensor_frame",
                                       transform=np.array(imu_frame_data["imu_sensor_frame"]["T_camerabundle_imu"]))
        else:
            calibration_graph.add_edge("cpt7_imu", "imu_sensor_frame",
                                       xyz=[0.297, -0.068, 0.157],
                                       rpy=[0.0, 0.0, 0.0])
        if "zed2i_imu_link" in imu_frame_data:
            calibration_graph.add_edge("cam1_sensor_frame", "zed2i_imu_link",
                                       transform=np.array(imu_frame_data["zed2i_imu_link"]["T_camerabundle_imu"]))
        else:
            calibration_graph.add_edge("cpt7_imu", "zed2i_imu_link",
                                       xyz=[0.358, -0.009, 0.142],
                                       rpy=[0.000, -0.174, -0.013])
        if "adis16475_imu" in imu_frame_data:
            calibration_graph.add_edge("cam1_sensor_frame", "adis16475_imu",
                                       transform=np.array(imu_frame_data["adis16475_imu"]["T_camerabundle_imu"]))
        else:
            calibration_graph.add_edge("cpt7_imu", "adis16475_imu",
                                       xyz=[0.329, -0.012, 0.155],
                                       rpy=[3.141, 0.001, -1.571])
        if "zed2i_imu_link" in imu_frame_data:
            calibration_graph.add_edge("cam1_sensor_frame", "zed2i_imu_link",
                                       transform=np.array(imu_frame_data["zed2i_imu_link"]["T_camerabundle_imu"]))
        else:
            calibration_graph.add_edge("cpt7_imu", "zed2i_imu_link",
                                       xyz=[0.358, -0.009, 0.142],
                                       rpy=[0.000, -0.174, -0.013])
        if "ap20_imu" in imu_frame_data:
            calibration_graph.add_edge("cam1_sensor_frame", "ap20_imu",
                                       transform=np.array(imu_frame_data["ap20_imu"]["T_camerabundle_imu"]))
        else:
            calibration_graph.add_edge("cpt7_imu", "ap20_imu",
                                       xyz=[0.004, -0.102, 0.002],
                                       rpy=[-1.569, 0.000, -0.000])
        if "stim320_imu" in imu_frame_data:
            calibration_graph.add_edge("cam1_sensor_frame", "stim320_imu",
                                       transform=np.array(imu_frame_data["stim320_imu"]["T_camerabundle_imu"]))
        else:
            calibration_graph.add_edge("cpt7_imu", "stim320_imu",
                                       xyz=[0.297, -0.068, 0.157],
                                       rpy=[-3.140, -0.000, -0.000])
    else:
        calibration_graph.add_edge("cpt7_imu", "cam1_sensor_frame",
                                   xyz=[0.369, -0.046, 0.068],
                                   rpy=[1.397, -0.001, 1.571])
        calibration_graph.add_edge("cpt7_imu", "livox_imu",
                                   xyz=[0.321, -0.023, -0.151],
                                   rpy=[0.00, 0.0, 0.0])
        calibration_graph.add_edge("cpt7_imu", "imu_sensor_frame",
                                   xyz=[0.297, -0.068, 0.157],
                                   rpy=[0.0, 0.0, 0.0])
        calibration_graph.add_edge("cpt7_imu", "adis16475_imu",
                                   xyz=[0.329, -0.012, 0.155],
                                   rpy=[3.141, 0.001, -1.571])
        calibration_graph.add_edge("cpt7_imu", "zed2i_imu_link",
                                   xyz=[0.358, -0.009, 0.142],
                                   rpy=[0.000, -0.174, -0.013])
        calibration_graph.add_edge("cpt7_imu", "ap20_imu",
                                   xyz=[0.004, -0.102, 0.002],
                                   rpy=[-1.569, 0.000, -0.000])
        calibration_graph.add_edge("cpt7_imu", "stim320_imu",
                                   xyz=[0.297, -0.068, 0.157],
                                   rpy=[-3.140, -0.000, -0.000])

    has_prism_calibration = "t_cam_prism" in calibration_data_by_topic[camera_bundle_topic]
    if has_prism_calibration:
        t_cam1_prism = calibration_data_by_topic[camera_bundle_topic]["t_cam_prism"]
        calibration_graph.add_edge("cam1_sensor_frame", "prism",
                                   xyz=t_cam1_prism,
                                   rpy=[0.000, -1.397, -1.572])
    else:
        calibration_graph.add_edge("cam1_sensor_frame", "prism",
                                   xyz=[-0.009, 0.347, -0.128],
                                   rpy=[0.000, -1.397, -1.572])

    calibration_graph.add_edge("cam1_sensor_frame", "alphasense_base", transform=identity)
    calibration_graph.add_edge("cam1_sensor_frame", "alphasense_front_center", transform=identity)
    calibration_graph.add_edge("cam2_sensor_frame", "alphasense_front_right", transform=identity)
    calibration_graph.add_edge("cam3_sensor_frame", "alphasense_front_left", transform=identity)
    calibration_graph.add_edge("cam4_sensor_frame", "alphasense_left", transform=identity)
    calibration_graph.add_edge("cam5_sensor_frame", "alphasense_right", transform=identity)
    calibration_graph.add_edge("hdr_front", "hdr_base", transform=identity)
    calibration_graph.add_edge("zed2i_left_camera_optical_frame", "zed2i_left_camera_frame",
                               xyz=[0, 0, 0],
                               rpy=[1.570796325, -1.570796325, 0.0])
    calibration_graph.add_edge("zed2i_left_camera_frame", "zed2i_camera_center",
                               xyz=[0.01, -0.06, 0.0],
                               rpy=[0, 0, 0])
    calibration_graph.add_edge("zed2i_camera_center", "zed2i_base_link",
                               xyz=[0, 0, -0.015],
                               rpy=[0, 0, 0])
    calibration_graph.add_edge("zed2i_base_link", "zed_base_link", transform=identity)
    calibration_graph.add_edge("zed2i_right_camera_optical_frame", "zed2i_right_camera_frame",
                               xyz=[0, 0, 0],
                               rpy=[1.570796325, -1.570796325, 0.0])
    calibration_graph.add_edge("box_base", "box_base_model",
                               xyz=[0.297, -0.068, 0.157],
                               rpy=[-3.1415, -0.000, -0.000])
    calibration_graph.add_edge("livox_lidar", "livox_model", transform=identity)
    calibration_graph.add_edge("cpt7_imu", "cpt7_antenna_front",
                               xyz=[0.13924890003, -0.0361, -0.193298],
                               rpy=[0, 0, 0])
    calibration_graph.add_edge("cpt7_imu", "cpt7_antenna_back",
                               xyz=[-0.4404038, -0.0361, -0.048868],
                               rpy=[0, 0, 0])
    calibration_graph.add_edge("prism", "prism_model", transform=identity)
    calibration_graph.add_edge("hesai_lidar", "hesai_model", transform=identity)
    calibration_graph.add_edge("cpt7_imu", "box_base", transform=identity)
    calibration_graph.add_edge("box_base", "base",
                               xyz=[0.0764038, -0.0361, -0.2802790],
                               rpy=[-3.1415126, 0.0, 0.0])
    # Validate the root of the graph
    calibration_graph.validate_single_root()

    # # Define the desired structure for the new graph
    new_structure = [
        ("base", "box_base", "base_to_boxbase"),
        ("box_base", "alphasense_base", "box_base_to_alphasense_base"),
        ("alphasense_base", "cam1_sensor_frame", "alphasense_base_to_alphasense_front_center"),
        ("alphasense_base", "cam2_sensor_frame", "alphasense_base_to_alphasense_front_right"),
        ("alphasense_base", "cam3_sensor_frame", "alphasense_base_to_alphasense_front_left"),
        ("alphasense_base", "cam4_sensor_frame", "alphasense_base_to_alphasense_left"),
        ("alphasense_base", "cam5_sensor_frame", "alphasense_base_to_alphasense_right"),
        ("cam1_sensor_frame", "alphasense_front_center", "alphasense_cam1_helper_joint"),
        ("cam2_sensor_frame", "alphasense_front_right", "alphasense_cam2_helper_joint"),
        ("cam3_sensor_frame", "alphasense_front_left", "alphasense_cam3_helper_joint"),
        ("cam4_sensor_frame", "alphasense_left", "alphasense_cam4_helper_joint"),
        ("cam5_sensor_frame", "alphasense_right", "alphasense_cam5_helper_joint"),
        ("box_base", "ap20_imu", "box_base_to_ap20_imu_frame"),
        ("box_base", "cpt7_imu", "box_base_to_cpt7_imu_frame"),
        ("box_base", "imu_sensor_frame", "box_base_to_imu_alphasense"),
        ("cpt7_imu", "cpt7_antenna_back", "cpt7_imu_frame_to_cpt7_antenna_back"),
        ("cpt7_imu", "cpt7_antenna_front", "cpt7_imu_frame_to_cpt7_antenna_front"),
        ("box_base", "adis16475_imu", "box_base_to_adis16475_imu"),
        ("box_base", "stim320_imu", "box_base_to_imu_stim320"),
        ("stim320_imu", "box_base_model", "stim320_imu_to_box_base_model"),
        ("box_base", "hdr_base", "box_base_to_hdr_base"),
        ("hdr_base", "hdr_front", "hdr_base_to_hdr_front"),
        ("hdr_base", "hdr_left", "hdr_base_to_hdr_left"),
        ("hdr_base", "hdr_right", "hdr_base_to_hdr_right"),
        ("box_base", "hesai_lidar", "box_base_to_hesai_lidar"),
        ("hesai_lidar", "hesai_model", "hesai_lidar_to_hesai_model"),
        ("box_base", "livox_lidar", "box_base_to_livox_lidar"),
        ("livox_lidar", "livox_imu", "livox_lidar_to_livox_imu"),
        ("livox_lidar", "livox_model", "livox_lidar_to_livox_model"),
        ("box_base", "prism", "box_base_to_prism"),
        ("prism", "prism_model", "prism_to_prism_model"),
        ("box_base", "zed_base_link", "box_base_to_zed_base"),
        ("zed_base_link", "zed2i_base_link", "zed2i_base_helper_joint"),
        ("zed2i_base_link", "zed2i_camera_center", "zed2i_base_to_camera_center"),
        ("zed2i_camera_center", "zed2i_left_camera_frame", "zed_camera_center_to_zed_left"),
        ("zed2i_left_camera_frame", "zed2i_imu_link", "zed_left_camera_frame_to_zed_imu"),
        ("zed2i_left_camera_frame", "zed2i_left_camera_optical_frame", "zed_left_to_left_optical"),
        (
            "zed2i_left_camera_optical_frame", "zed2i_right_camera_optical_frame",
            "zed_left_optical_to_zed_right_optical"),
        ("zed2i_right_camera_optical_frame", "zed2i_right_camera_frame", "zed_right_to_right_optical")]

    # Build the new graph
    new_graph = calibration_graph.build_new_graph(new_structure)

    # Validate the new graph's root
    new_graph.validate_single_root()

    # Print the new graph edges
    # print("\nNew Graph with Bidirectional Edges:")
    # for node, neighbors in new_graph.input_edges.items():
    #     for neighbor, transform in neighbors.items():
    #         print(f"{node} -> {neighbor}: \n{transform}")

    # calibration_graph.visualize_by_depth(title="Old Graph with Transformations")
    # new_graph.visualize_by_depth(title="New Graph with Transformations")
    new_graph.save_tagged_edges_to_yaml(args.output_urdf_compatible_yaml)
