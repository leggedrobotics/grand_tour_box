#! /usr/bin/python3
import os.path
import uuid

import numpy as np
import rerun as rr
import rerun.blueprint as rrb
import rospkg
import rospy
import yaml

from grand_tour_camera_detection_msgs.msg import (CameraCameraAdjacency, CameraCameraCalibrationState,
                                                  CameraIntrinsicsExtrinsicsSigma, CameraIntrinsicsExtrinsics)
from grand_tour_calibration_viewers.srv import FetchRecordingIDFromBlueprintNode, FetchRecordingIDFromBlueprintNodeResponse
from std_msgs.msg import String
from scipy.spatial.transform import Rotation


def load_yaml(path: str):
    with open(path, "r") as f:
        return yaml.safe_load(f)

CAMERA_BUNDLE_ORIGIN_NAME = "cameras_base"
def plot_frame(source_frame, T_dest_source):
    angle_axis = Rotation.from_matrix(T_dest_source[:3, :3]).as_rotvec()
    angle = np.linalg.norm(angle_axis)
    axis = angle_axis / angle if np.abs(angle) > 0 else angle_axis
    rr.log(
        f"{CAMERA_BUNDLE_ORIGIN_NAME}/{source_frame}",
        rr.Transform3D(translation=T_dest_source[:3, -1], rotation=rr.RotationAxisAngle(axis=axis, radians=angle)),
        static=False
    )

def generate_random_uuid():
    return str(uuid.uuid4())

class CameraCameraViewerBlueprintNode:
    def __init__(self):
        rospy.init_node('calibration_viewer_launch')
        self.recording_id = generate_random_uuid()
        self.recording_id_service = rospy.Service(
            'camera_camera_calibration_viewer/recording_id_service',
            FetchRecordingIDFromBlueprintNode,
            self.handle_fetch_recording)

        rospack = rospkg.RosPack()
        package_path = rospack.get_path('grand_tour_calibration_viewers')
        config_root = os.path.join(package_path, "config")
        application_id = load_yaml(os.path.join(config_root, "viewer.yaml"))["application_id"]

        alphasense_front_center_topic = load_yaml(os.path.join(config_root, "alphasense_front_center.yaml"))[
            "image_topic"]
        alphasense_front_left_topic = load_yaml(os.path.join(config_root, "alphasense_front_left.yaml"))["image_topic"]
        alphasense_front_right_topic = load_yaml(os.path.join(config_root, "alphasense_front_right.yaml"))[
            "image_topic"]
        alphasense_left_topic = load_yaml(os.path.join(config_root, "alphasense_left.yaml"))["image_topic"]
        alphasense_right_topic = load_yaml(os.path.join(config_root, "alphasense_right.yaml"))["image_topic"]
        zed_left_topic = load_yaml(os.path.join(config_root, "zed_left.yaml"))["image_topic"]
        zed_right_topic = load_yaml(os.path.join(config_root, "zed_right.yaml"))["image_topic"]

        hdr_left_topic = load_yaml(os.path.join(config_root, "hdr_left.yaml"))["image_topic"]
        hdr_front_topic = load_yaml(os.path.join(config_root, "hdr_front.yaml"))["image_topic"]
        hdr_right_topic = load_yaml(os.path.join(config_root, "hdr_right.yaml"))["image_topic"]

        # Initialize Rerun blueprint views via ROS params
        blueprint = rrb.Horizontal(rrb.BarChartView(name="percentage_data",
                                                    origin="percent_till_solve"),
            rrb.Vertical(
            rrb.Horizontal(
                rrb.Vertical(
                    rrb.Horizontal(
                        rrb.Vertical(
                            rrb.Spatial2DView(name="alphasense_left", origin=f"{alphasense_left_topic}"),
                            rrb.Spatial2DView(name="hdr_left", origin=f"{hdr_left_topic}")
                        ),
                        rrb.Vertical(
                            rrb.Horizontal(rrb.Spatial2DView(name="alphasense_front_left_image",
                                                             origin=f"{alphasense_front_left_topic}"),
                                           rrb.Spatial2DView(name="alphasense_front_center_image",
                                                             origin=f"{alphasense_front_center_topic}"),
                                           rrb.Spatial2DView(name="alphasense_front_right_image",
                                                             origin=f"{alphasense_front_right_topic}"),
                                           ),
                            rrb.Spatial2DView(name="hdr_front", origin=f"{hdr_front_topic}"),
                            rrb.Horizontal(
                                rrb.Spatial2DView(name="zed_stereo_left", origin=f"{zed_left_topic}"),
                                rrb.Spatial2DView(name="zed_stereo_right", origin=f"{zed_right_topic}")
                            )
                        ),
                        rrb.Vertical(
                            rrb.Spatial2DView(name="alphasense_right", origin=f"{alphasense_right_topic}"),
                            rrb.Spatial2DView(name="hdr_right", origin=f"{hdr_right_topic}")
                        ),
                        column_shares=[1, 1, 1], name="Camera Detections"
                    ),
                    rrb.Horizontal(
                        rrb.Vertical(
                            rrb.Spatial2DView(name="alphasense_left", origin=f"{alphasense_left_topic}_residuals"),
                            rrb.Spatial2DView(name="hdr_left", origin=f"{hdr_left_topic}_residuals")
                        ),
                        rrb.Vertical(
                            rrb.Horizontal(
                                rrb.Spatial2DView(name="alphasense_front_left_residual",
                                                  origin=f"{alphasense_front_left_topic}_residuals"),

                                rrb.Spatial2DView(name="alphasense_front_center_residual",
                                                  origin=f"{alphasense_front_center_topic}_residuals"),
                                rrb.Spatial2DView(name="alphasense_front_right_residual",
                                                  origin=f"{alphasense_front_right_topic}_residuals")
                            ),
                            rrb.Spatial2DView(name="hdr_front", origin=f"{hdr_front_topic}_residuals"),
                            rrb.Horizontal(
                                rrb.Spatial2DView(name="zed_stereo_left", origin=f"{zed_left_topic}_residuals"),
                                rrb.Spatial2DView(name="zed_stereo_right", origin=f"{zed_right_topic}_residuals")
                            )
                        ),
                        rrb.Vertical(
                            rrb.Spatial2DView(name="alphasense_right", origin=f"{alphasense_right_topic}_residuals"),
                            rrb.Spatial2DView(name="hdr_right", origin=f"{hdr_right_topic}_residuals")
                        ),
                        column_shares=[1, 1, 1], name="Intrinsics Reprojection Error"
                    )),
                rrb.Spatial3DView(name="Extrinsics", origin=f"{CAMERA_BUNDLE_ORIGIN_NAME}"),
            column_shares=[2,1]),
            rrb.Horizontal(
                rrb.Horizontal(
                    rrb.Spatial2DView(name="camera_camera_adjacency", origin="adjacency"),
                    rrb.TextDocumentView(name="camera_names", origin="camera_name_mappings")),
                rrb.BarChartView(name="camera_sigmas", origin="sigma_position"),
                rrb.BarChartView(name="camera_sigmas", origin="sigma_rotation"),
                rrb.BarChartView(name="camera_sigmas", origin="sigma_focal"),
                name="Camera Optimisation Stats"
            ), row_shares=[2,1]
        ), column_shares=[1, 9])
        blueprint = rrb.Blueprint(blueprint, collapse_panels=True)
        rr.init(application_id=application_id,
                recording_id=self.recording_id, spawn=True, default_blueprint=blueprint)
        rr.spawn(memory_limit="5000MB")
        rr.send_blueprint(blueprint)
        self.node_positions = dict()
        self.adjacency_subscriber = rospy.Subscriber("camera_camera_online_calibration/adjacency",
                                                     CameraCameraAdjacency, self.adjacency_callback)
        self.data_accumulation_subscriber = rospy.Subscriber("camera_camera_online_calibration/data_accumulation_state",
                                                     CameraCameraCalibrationState, self.data_accumulation_callback)
        self.sigma_subscriber = rospy.Subscriber("camera_camera_online_calibration/sigma",
                                                     CameraIntrinsicsExtrinsicsSigma,
                                                     self.sigma_callback)
        self.camera_extrinsics_subscriber = rospy.Subscriber("camera_camera_online_calibration/intrinsics_extrinsics",
                                                             CameraIntrinsicsExtrinsics,
                                                             self.camera_extrinsics_callback)
        rr.log(f"{CAMERA_BUNDLE_ORIGIN_NAME}",
               rr.ViewCoordinates.RIGHT_HAND_Y_UP, static=True)
        self.projection_sigma = dict()
        self.camera_position_sigma = dict()
        self.rotation_sigma = dict()

    def handle_fetch_recording(self, req):
        recording_string = self.recording_id
        recording_msg = String()
        recording_msg.data = recording_string
        return FetchRecordingIDFromBlueprintNodeResponse(
            recording_id=recording_msg)

    def run(self):
        # Keep the node running and listen for callbacks
        rospy.spin()

    def camera_extrinsics_callback(self, msg:CameraIntrinsicsExtrinsics):
        rr.set_time_nanos("camera_calibration_time", msg.header.stamp.to_nsec())
        q = msg.T_bundle_camera.rotation
        t = msg.T_bundle_camera.translation
        R = Rotation.from_quat(np.array([q.x, q.y, q.z, q.w])).as_matrix()
        translation = np.array([t.x, t.y, t.z])
        T_bundle_camera = np.eye(4)
        T_bundle_camera[:3, :3] = R
        T_bundle_camera[:3, -1] = translation
        plot_frame(msg.header.frame_id, T_bundle_camera)
        print(msg)

    def data_accumulation_callback(self, msg: CameraCameraCalibrationState):
        stamp = msg.header.stamp.to_nsec()
        rr.set_time_nanos("camera_calibration_time", stamp)
        percentage = msg.percentage_progress.data
        rr.log(f"percent_till_solve",
               rr.BarChart([percentage, 1.0], color=[[255, 255, 255] if percentage < 1.0
                                                else [255, 0, 0]]))

    def adjacency_callback(self, msg: CameraCameraAdjacency):
        name_a = msg.camera_a.data
        name_b = msg.camera_b.data
        if name_a not in self.node_positions:
            self.node_positions[name_a] = None
            self.compute_new_node_positions()
        if name_b not in self.node_positions:
            self.node_positions[name_b] = None
            self.compute_new_node_positions()
        rr.set_time_nanos("camera_calibration_time", msg.header.stamp.to_nsec())
        rr.log(f"adjacency/{name_a}_{name_b}",
               rr.LineStrips2D([[self.node_positions[name_a], self.node_positions[name_b]]],
                               labels=[f"{str(msg.capacity.data)}"]))

    def sigma_callback(self, msg: CameraIntrinsicsExtrinsicsSigma):
        stamp = msg.header.stamp.to_nsec()
        rr.set_time_nanos("camera_calibration_time", stamp)
        self.rotation_sigma[msg.header.frame_id] = np.array(msg.rvectvec_sigma[:3])
        self.camera_position_sigma[msg.header.frame_id] = np.array(msg.rvectvec_sigma[-3:])
        self.projection_sigma[msg.header.frame_id] = np.array(msg.fxfycxcy_sigma)

        header = f"| **Index**       | **Camera Name** |\n| ----------------- | --------------- |\n"
        camera_names = "\n".join([f"| {k}  | {v} |" for k, v in enumerate(self.rotation_sigma.keys())])
        rr.log("camera_name_mappings", rr.TextDocument((header + camera_names).strip(),
                                                       media_type=rr.MediaType.MARKDOWN))
        red = (255, 0, 0)
        green = (0, 255, 0)

        rotation_sigmas = np.rad2deg(np.array([np.max(x) for x in self.rotation_sigma.values()
                                               if len(x)]))
        if len(rotation_sigmas):
            rr.log(f"sigma_rotation",
                   rr.BarChart(rotation_sigmas, color=[red if rotation_sigmas.max() > 0.1 else green]))
        position_sigmas = np.array([np.max(x) for x in self.camera_position_sigma.values()
                                    if len(x)])
        if len(position_sigmas):
            rr.log(f"sigma_position",
                   rr.BarChart(position_sigmas, color=[red if position_sigmas.max() > 0.001 else green]))

        projection_sigmas = np.array([np.max(x) for x in self.projection_sigma.values()])
        if len(projection_sigmas):
            rr.log(f"sigma_focal",
                   rr.BarChart(projection_sigmas, color=[red if projection_sigmas.max() > 2.0 else green]))

    def compute_new_node_positions(self):
        n = len(self.node_positions)
        angles = [i / n * np.pi * 2 for i in range(n)]
        scale = 3
        for i, (name) in enumerate(self.node_positions):
            position = np.array([np.cos(angles[i]) * scale, np.sin(angles[i]) * scale])
            self.node_positions[name] = position


if __name__ == '__main__':
    visualizer = CameraCameraViewerBlueprintNode()
    visualizer.run()
