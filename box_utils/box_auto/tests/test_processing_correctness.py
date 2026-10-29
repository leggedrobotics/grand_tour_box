"""Numerical contracts and failure preservation for standalone dataset tools.

Fixtures are synthetic and self-cleaning; no cloud or real mission writes occur.
"""

import importlib.util
import logging
from pathlib import Path
import struct
import types
import unittest

import cv2
import numpy as np
import pandas as pd
import rosbag
import rospy
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import CameraInfo, CompressedImage, PointCloud2, PointField
from scipy.spatial.transform import Rotation
from std_msgs.msg import Header, String
from tf2_msgs.msg import TFMessage

from test_dataset_reliability import TemporaryCase, SCRIPTS, PACKAGE, extracted_function, load_script

rotation = load_script("camera/alphasense_correct_front_left_rotated_edge_case.py")
livox = load_script("special/livoxToHesai.py")
tf_repair = load_script("special/correct_box_to_box_base_transform_tf_static.py")
ap20 = load_script("ap20/ap20_python_online_matcher.py")
rawimu = load_script("cpt7/export_raw_imu_bag.py")
parser_path = PACKAGE.parent / "box_calibration/calibration_tools/calibration_tools/parsers/parse_NX_html.py"
spec = importlib.util.spec_from_file_location("nx_calibration_test", parser_path)
nx = importlib.util.module_from_spec(spec)
spec.loader.exec_module(nx)


def cloud(big_endian=False):
    msg = PointCloud2()
    msg.header = Header(seq=7, stamp=rospy.Time(1700000000, 123), frame_id="livox")
    msg.height, msg.width = 2, 1
    msg.is_bigendian = big_endian
    msg.is_dense = True
    msg.point_step, msg.row_step = 25, 29
    msg.fields = [
        PointField(name=n, offset=o, datatype=t, count=1)
        for n, o, t in (
            ("x", 0, 7),
            ("y", 4, 7),
            ("z", 8, 7),
            ("intensity", 12, 7),
            ("line", 16, 2),
            ("timestamp", 17, 8),
        )
    ]
    fmt = (">" if big_endian else "<") + "ffffBd"
    msg.data = b"".join(
        struct.pack(fmt, 1.0, 2.0, 3.0, 4.0, ring, 1700000000000000000.0 + ring * 1000000) + b"pad!" for ring in (1, 2)
    )
    return msg


class ConversionCorrectnessTests(TemporaryCase):
    def test_rotation_pixels_headers_and_unrelated_messages(self):
        pixels = np.arange(18, dtype=np.uint8).reshape(2, 3, 3)
        msg = CompressedImage(header=Header(seq=4, stamp=rospy.Time(20), frame_id="camera"), format="png")
        msg.data = cv2.imencode(".png", pixels)[1].tobytes()
        source = self.directory / "input.bag"
        with rosbag.Bag(str(source), "w") as bag:
            bag.write("/image", msg, rospy.Time(21))
            bag.write("/info", CameraInfo(width=3, height=2), rospy.Time(21))
            bag.write("/metadata", String(data="unchanged"), rospy.Time(22))
        self.assertEqual(rotation.rotate_bag(source, self.output, "/image"), 1)
        with rosbag.Bag(str(self.output)) as bag:
            messages = list(bag.read_messages())
        out = messages[0].message
        np.testing.assert_array_equal(cv2.imdecode(np.frombuffer(out.data, np.uint8), 1), pixels[::-1, ::-1])
        self.assertEqual(
            (out.header.seq, out.header.stamp.to_nsec(), out.header.frame_id),
            (msg.header.seq, msg.header.stamp.to_nsec(), msg.header.frame_id),
        )
        self.assertEqual(messages[0].timestamp, rospy.Time(21))
        self.assertEqual(messages[1].message.width, 3)
        self.assertEqual(messages[2].message.data, "unchanged")

    def test_failed_rotation_and_missing_topic_preserve_output(self):
        source = self.directory / "input.bag"
        self.output.write_bytes(b"previous output")
        with rosbag.Bag(str(source), "w") as bag:
            bag.write("/image", CompressedImage(format="jpeg", data=b"bad image"), rospy.Time(2))
        for topic in ("/image", "/missing"):
            with self.assertRaises((ValueError, cv2.error)):
                rotation.rotate_bag(source, self.output, topic, overwrite=True)
            self.assertEqual(self.output.read_bytes(), b"previous output")
        self.assertFalse(list(self.directory.glob(".output.bag.*")))

    def test_livox_layout_padding_endianness_and_acquisition_time(self):
        for big_endian in (False, True):
            original = cloud(big_endian)
            out = livox.convert_cloud_to_hesai(original, rospy.Time(1700000001))
            self.assertEqual(out.header, original.header)
            self.assertEqual((out.width, out.height, out.point_step, out.row_step), (1, 2, 26, 26))
            fmt = (">" if big_endian else "<") + "ffffdH"
            for row in range(2):
                values = struct.unpack(fmt, out.data[row * 26 : (row + 1) * 26])
                self.assertEqual(values[:4], (1.0, 2.0, 3.0, 4.0))
                self.assertEqual(values[5], row + 1)
                self.assertAlmostEqual(values[4], 1700000000.0 + (row + 1) * 0.001, delta=3e-7)
            changed = livox.convert_cloud_to_hesai(original, rospy.Time(25), header_time="bag")
            self.assertEqual(changed.header.stamp, rospy.Time(25))

    def test_missing_livox_topic_preserves_existing_output(self):
        source = self.directory / "input.bag"
        with rosbag.Bag(str(source), "w") as bag:
            bag.write("/metadata", String(data="only metadata"), rospy.Time(1))
        self.output.write_bytes(b"previous output")
        with self.assertRaises(ValueError):
            livox.convert_bag(source, self.output, overwrite=True)
        self.assertEqual(self.output.read_bytes(), b"previous output")

    def test_livox_rejects_invalid_timestamps_and_layout(self):
        for invalid in (float("nan"), float("inf"), -1.0, float(2**63)):
            with self.assertRaises(ValueError):
                livox._ns_float_to_int64(np.array([invalid]))
        msg = cloud()
        msg.row_step = 2
        with self.assertRaises(ValueError):
            livox.convert_cloud_to_hesai(msg, rospy.Time(1))

    def test_tf_repair_is_idempotent_and_preserves_other_frames(self):
        source = self.directory / "input.bag"
        base = TransformStamped()
        base.header.frame_id = "base"
        base.child_frame_id = "box_base"
        base.transform.rotation.w = 1
        spider = TransformStamped()
        spider.header.frame_id = "box_base"
        spider.child_frame_id = "spider"
        spider.transform.rotation.w = 1
        other = TransformStamped()
        other.header.frame_id = "box_base"
        other.child_frame_id = "sensor"
        other.transform.translation.x = 3
        other.transform.rotation.w = 1
        with rosbag.Bag(str(source), "w") as bag:
            bag.write("/tf_static", TFMessage([base, spider, spider, other]), rospy.Time(5))
            bag.write("/metadata", String(data="unchanged"), rospy.Time(6))
        tf_repair.repair_bag(source, self.output, [1, 2, 3, 0, 0, np.pi / 2])
        for _ in range(2):
            tf_repair.repair_bag(self.output, self.output, [1, 2, 3, 0, 0, np.pi / 2], overwrite=True)
            with rosbag.Bag(str(self.output)) as bag:
                messages = list(bag.read_messages())
            transforms = messages[0].message.transforms
            self.assertEqual([t.child_frame_id for t in transforms], ["box_base", "spider", "sensor"])
            self.assertEqual(transforms[2].header.frame_id, other.header.frame_id)
            self.assertEqual(transforms[2].transform.translation.x, 3.0)
            np.testing.assert_allclose(
                [transforms[0].transform.rotation.z, transforms[0].transform.rotation.w], [2**-0.5, 2**-0.5]
            )
            self.assertEqual(messages[1].message.data, "unchanged")

    def test_tf_absent_target_preserves_previous_output(self):
        source = self.directory / "input.bag"
        with rosbag.Bag(str(source), "w") as bag:
            bag.write("/tf_static", TFMessage(), rospy.Time(1))
        self.output.write_bytes(b"previous output")
        with self.assertRaises(ValueError):
            tf_repair.repair_bag(source, self.output, [0] * 6, overwrite=True)
        self.assertEqual(self.output.read_bytes(), b"previous output")

    def test_cad_requires_complete_right_handed_calibration(self):
        source = self.directory / "cad.html"
        start = '<div id="InfoObject1"><div class="NXGroupHeader">CSYS: sensor</div>'
        source.write_text(start + "</div>")
        with self.assertRaises(ValueError):
            nx.parse_html(str(source))

        def table(name, values):
            return (
                '<table id="'
                + name
                + '"><tr></tr><tr>'
                + "".join("<td>" + str(v) + "</td>" for v in values)
                + "</tr></table>"
            )

        body = (
            table("OriginLocationInAbsCoordSysandWCS", [1000, 2000, 3000])
            + table("XaxisVectorYZPlane", [0, 1, 0])
            + table("YaxisVectorYZPlane", [-1, 0, 0])
        )
        source.write_text(start + body + table("ZaxisVectorYZPlane", [0, 0, 1]) + "</div>")
        result = nx.parse_html(str(source))["sensor"]
        np.testing.assert_allclose(result[:3, 3], [1, 2, 3])
        np.testing.assert_allclose(result[:3, :3], [[0, -1, 0], [1, 0, 0], [0, 0, 1]])
        source.write_text(start + body + table("ZaxisVectorYZPlane", [0, 0, -1]) + "</div>")
        with self.assertRaises(ValueError):
            nx.parse_html(str(source))

    def test_rawimu_time_offset_and_first_sample_preservation(self):
        # Headerless NovAtel CSV: field name, clock fields, IMU type, week, SOW,
        # followed by six sensor counts. Conversion must retain the first row.
        source = self.directory / "RAWIMUSX.ASCII"
        source.write_text("%RAWIMUSXA,0,0,68,2300,10.01,1,2,3,4,5,6\n%RAWIMUSXA,0,0,68,2300,10.02,1,2,3,4,5,6\n")
        times = []
        for offset in (-18.0, 19.0):
            parser = rawimu.RAWIMUDataParser(logging.getLogger("test"), "imu", offset)
            frame = parser.fetch_dataframe_from_csv(str(source))
            self.assertEqual(len(frame), 2)
            self.assertTrue(parser.load_imu_times(frame))
            times.append(parser.ros_times)
        self.assertEqual([(b - a).to_sec() for a, b in zip(*times)], [37.0, 37.0])
        self.assertAlmostEqual((times[0][1] - times[0][0]).to_sec(), 0.01)


class FrameCovarianceTests(unittest.TestCase):
    def test_ie_covariance_matches_attitude_perturbations(self):
        path = SCRIPTS / "cpt7/export_gps_gt_trajectory_bag.py"
        covariance_fn = extracted_function(path, "orientation_covariance_enu", dict(np=np, Rotation=Rotation))
        ned_to_enu = np.array([[0., 1., 0.], [1., 0., 0.], [0., 0., -1.]])
        for angles in (np.array([0., 0., 0.]), np.array([34., 17., -9.]), np.array([-80., 30., 15.])):
            def rotation_fn(values):
                return Rotation.from_matrix(ned_to_enu @ Rotation.from_euler("ZXY", values, degrees=True).as_matrix())
            std = np.array([2., 3., 5.])
            base = rotation_fn(angles)
            jacobian = []
            step = 1e-5
            for i in range(3):
                perturbed = angles.copy()
                perturbed[i] += step
                jacobian.append((rotation_fn(perturbed) * base.inv()).as_rotvec() / np.deg2rad(step))
            j = np.array(jacobian).T
            expected = j @ np.diag(np.deg2rad(std) ** 2) @ j.T
            np.testing.assert_allclose(covariance_fn(*angles, std), expected, atol=1e-9, rtol=1e-6)

    def test_ie_rejects_invalid_covariance(self):
        fn = extracted_function(SCRIPTS / "cpt7/export_gps_gt_trajectory_bag.py", "compact_cov_to_full", dict(np=np))
        np.testing.assert_array_equal(fn([1, 0, 2, 0, 0, 3]), np.diag([1, 2, 3]))
        for values in ([1, 2, 1, 0, 0, 1], [float("nan"), 0, 1, 0, 0, 1]):
            with self.assertRaises(ValueError):
                fn(values)


if __name__ == "__main__":
    unittest.main()
