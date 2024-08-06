import rospy
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster

from box_utils.box_calibration.calibration_tools.calibration_tools.transform_graph import TransformGraph
from scipy.spatial.transform import Rotation


class StaticTFPublisher:
    @classmethod
    def publish_transform_graph(cls, tg: TransformGraph, root_frame):
        broadcaster = StaticTransformBroadcaster()
        parent_frame = root_frame
        nodes = tg.nodes

        # Define transformations
        transforms = []
        for node in [x for x in nodes if x != parent_frame]:
            T_parent_child = tg.find_T_target_source(source=node, target=parent_frame)
            transform_msg = cls.create_transform_msg(T_parent_child, node, parent_frame)
            transforms.append(transform_msg)

        # Broadcast all transformations
        broadcaster.sendTransform(transforms)

    @staticmethod
    def create_transform_msg(T_parent_child, child_frame, parent_frame):
        quat_xyzw = Rotation.from_matrix(T_parent_child[:3, :3]).as_quat()
        t = T_parent_child[:3, -1]
        transform_msg = TransformStamped()
        # base_link to link1
        transform_msg.header.stamp = rospy.Time.now()
        transform_msg.header.frame_id = parent_frame
        transform_msg.child_frame_id = child_frame
        transform_msg.transform.translation.x = t[0]
        transform_msg.transform.translation.y = t[1]
        transform_msg.transform.translation.z = t[2]
        transform_msg.transform.rotation.x = quat_xyzw[0]
        transform_msg.transform.rotation.y = quat_xyzw[1]
        transform_msg.transform.rotation.z = quat_xyzw[2]
        transform_msg.transform.rotation.w = quat_xyzw[3]
        return transform_msg
