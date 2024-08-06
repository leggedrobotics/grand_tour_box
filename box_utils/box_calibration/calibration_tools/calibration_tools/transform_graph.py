from __future__ import annotations

import argparse
import copy
import json
from collections import deque
from io import StringIO
from typing import Dict

import networkx as nx
import numpy as np
import rospy
import tf
from networkx.readwrite import json_graph
from scipy.spatial.transform import Rotation


# Custom JSON Encoder to handle numpy ndarrays
class NumpyEncoder(json.JSONEncoder):
    def default(self, obj):
        if isinstance(obj, np.ndarray):
            return obj.tolist()
        return super(NumpyEncoder, self).default(obj)



class TransformGraph(object):
    def __init__(self, G, transforms_data: Dict):
        self.G = G
        self.transform_data = transforms_data
        # Add edges with SE3 4x4 matrix transforms and store them as edge properties
        for target_frame in list(self.transform_data.keys()):
            for source_frame in list(self.transform_data[target_frame].keys()):
                transform = self.transform_data[target_frame][source_frame]
                T = np.array(transform)
                G.add_edge(target_frame, source_frame, transform=T)
                if not G.has_edge(source_frame, target_frame):
                    T_inv = np.linalg.inv(T)
                    G.add_edge(source_frame, target_frame, transform=T_inv)

        self.nodes = list(self.G.nodes().keys())

    @classmethod
    def from_tf_listener(cls, listener:tf.TransformListener) -> TransformGraph:
        rospy.sleep(2)  # Sleep for a bit to ensure listener has received some transforms
        try:
            frames_dot = listener.allFramesAsDot()
        except tf.Exception as e:
            rospy.logerr(f"Error getting transform frames: {e}")
            raise ValueError

        # Create a graph from the DOT format string
        dot_stream = StringIO(frames_dot)
        G = nx.drawing.nx_pydot.read_dot(dot_stream)

        # Add transform data to the graph
        transform_data = {}
        for edge in G.edges:
            target_frame = edge[0].strip('"')
            source_frame = edge[1].strip('"')
            try:
                (trans, rot) = listener.lookupTransform(target_frame, source_frame, rospy.Time(0))
                if target_frame not in transform_data:
                    transform_data[target_frame] = dict()
                T_target_source = np.eye(4)
                R = Rotation.from_quat(rot).as_matrix()
                T_target_source[:3, :3] = R
                T_target_source[:3, -1] = trans
                transform_data[target_frame][source_frame] = T_target_source.tolist()
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.logwarn(f"Could not lookup transform from {target_frame} to {source_frame}")
                continue
        return cls(G=G, transforms_data=transform_data)

    @classmethod
    def from_json_file(cls, json_path) -> TransformGraph:
        with open(json_path, 'r') as f:
            graph_dict = json.load(f)
        # Reconstruct the graph
        G = json_graph.node_link_graph(graph_dict)
        # Extract transform data
        transforms_data = graph_dict.get('transforms', {})
        # Load the TF graph and transform data
        return cls(G=G, transforms_data=transforms_data)

    def serialise(self, output_file):

        # Convert the graph to a dictionary format
        graph_dict = nx.readwrite.json_graph.node_link_data(self.G)
        graph_dict['transforms'] = self.transform_data

        # Write the graph dictionary to a JSON file
        with open(output_file, 'w') as f:
            json.dump(graph_dict, f, indent=4, cls=NumpyEncoder)

        rospy.loginfo(f"Serialized TF graph and transforms to {output_file}")

    def find_T_target_source(self, source, target):
        path = self._bfs_path(source, target)
        if not path:
            return path

        T_target_source = np.eye(4)
        for x in path:
            T_target_source = x[1] @ T_target_source
        return T_target_source

    def _bfs_path(self, source, target):
        visited = set()
        queue = deque([(source, [(source, np.eye(4))])])

        while queue:
            current_node, path = queue.popleft()
            if current_node == target:
                return path
            visited.add(current_node)
            for neighbor in self.G.neighbors(current_node):
                if neighbor not in visited:
                    T_target_source = self._get_T_target_source(target_frame=neighbor, source_frame=current_node)
                    queue.append((neighbor, path + [(neighbor, T_target_source)]))

        print(f"No path found from {source} to {target}")
        return None

    def _get_T_target_source(self, target_frame, source_frame):
        edge_data = self.G.get_edge_data(target_frame, source_frame)
        for edge in edge_data.values():
            transform = edge.get('transform')
            if transform is not None:
                return transform
        print(f"No edge data available from {target_frame} to {source_frame}")
        return None
