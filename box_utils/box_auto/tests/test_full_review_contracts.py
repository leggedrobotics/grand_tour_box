"""Cross-group contracts for the complete 1–10 integration candidate."""

import contextlib
from collections import deque
import csv
from datetime import timezone
import importlib.util
import io
from pathlib import Path
import struct
import sys
import types
from unittest.mock import patch
import xml.etree.ElementTree as ET

import numpy as np
import rosbag
import rospy
from geometry_msgs.msg import TransformStamped, PoseWithCovarianceStamped
from sensor_msgs.msg import Image, CameraInfo, NavSatFix, PointCloud2, PointField
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from tf2_msgs.msg import TFMessage
from test_dataset_reliability import TemporaryCase, PACKAGE, SCRIPTS, load_script

sys.path.insert(0, str(PACKAGE / "python"))
from box_auto.utils.atomic_file import atomic_directory
from box_auto.utils.gpx_data import acquisition_time, decimate_coordinates


def load(relative):
    spec = importlib.util.spec_from_file_location("full_review_" + Path(relative).stem, SCRIPTS / relative)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


class FullReviewContracts(TemporaryCase):
    def test_directory_publication_preserves_existing_and_cleans_failure(self):
        target = self.directory / "report"
        with self.assertRaises(RuntimeError):
            with atomic_directory(target) as pending:
                (pending / "partial").write_text("partial")
                raise RuntimeError("render failed")
        self.assertEqual(list(self.directory.iterdir()), [])
        with atomic_directory(target) as pending:
            (pending / "complete").write_text("complete")
        with self.assertRaises(FileExistsError):
            with atomic_directory(target):
                self.fail("Existing directory must be rejected")
        self.assertEqual((target / "complete").read_text(), "complete")

    def test_gpx_clock_is_acquisition_utc_and_invalid_header_is_rejected(self):
        message = NavSatFix(header=Header(stamp=rospy.Time(1700000000, 125000000)))
        date = acquisition_time(message, rospy.Time(1700000010))
        self.assertEqual(date.tzinfo, timezone.utc)
        self.assertEqual(date.timestamp(), 1700000000.125)
        self.assertEqual(acquisition_time(object(), rospy.Time(1700000010)).timestamp(), 1700000010)
        message.header.stamp = rospy.Time(0)
        with self.assertRaises(ValueError):
            acquisition_time(message, rospy.Time(1700000010))

    def test_gpx_decimation_bounds_count_and_preserves_both_endpoints(self):
        for count in (0, 1, 2, 4999, 5000, 5001, 9999, 10001):
            points = [(index, -index, 100, None) for index in range(count)]
            selected = decimate_coordinates(points, 5000)
            self.assertEqual(len(selected), min(count, 5000))
            if count:
                self.assertEqual(selected[0], points[0][:2])
                self.assertEqual(selected[-1], points[-1][:2])
                self.assertEqual(len(set(selected)), len(selected))
        with self.assertRaises(ValueError):
            decimate_coordinates([], 0)

    def test_gpx_report_pair_and_failure_exit_without_network_or_overwrite(self):
        module = load("cpt7/gpx_converter_automated.py")
        bag_path = self.directory / "mission_cpt7_ie_tc.bag"
        with rosbag.Bag(str(bag_path), "w") as bag:
            for index in range(12):
                message = NavSatFix(
                    header=Header(stamp=rospy.Time(1700000000 + index)),
                    latitude=47.0 + index * 0.00001,
                    longitude=8.0,
                    altitude=450.0,
                    position_covariance=[0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.04],
                    position_covariance_type=2,
                )
                bag.write(module.VALID_TOPIC, message, rospy.Time(1700000100 + index))
        with contextlib.redirect_stdout(io.StringIO()), patch.object(
            module, "read_gspread_row", side_effect=AssertionError("network")
        ), patch.object(module, "load_stats_optional", return_value={}):
            target = self.directory / "reports"
            saved = module.process_one_mission("fixture", bag_path, target, False, [])
            self.assertIsNotNone(saved)
            root = ET.fromstring((target / "fixture_tc.gpx").read_text())
            times = [node.text for node in root.iter() if node.tag.endswith("}time")]
            self.assertTrue(times[0].startswith("2023-11-14T22:13:20"))
            self.assertTrue(times[0].endswith("Z"))
            before = {p.name: p.read_bytes() for p in target.iterdir()}
            self.assertIsNone(module.process_one_mission("fixture", bag_path, target, False, []))
            self.assertEqual(before, {p.name: p.read_bytes() for p in target.iterdir()})
            with patch.object(module, "build_folium_map", side_effect=ValueError("map failed")):
                self.assertIsNone(module.process_one_mission("failed", bag_path, target, False, []))
            self.assertFalse(list(target.glob("failed*")))
            with patch.object(module, "process_one_mission", return_value=None), patch.object(
                module, "get_bag", return_value=str(bag_path)
            ), patch.object(module, "find_and_extract_non_matching", return_value="fixture"):
                self.assertEqual(module.main(["--mission-data", str(self.directory), "--output-dir", str(target)]), 1)
        self.assertFalse(module.ENABLE_REVERSE_GEOCODING)
        self.assertFalse(module.SHEET_UPDATE)
        self.assertFalse(module.OPEN_HTML_INTERACTIVE)

    def test_ap20_candidate_window_loss_duplicate_and_reinsertion_order(self):
        module = load_script("ap20/ap20_python_online_matcher.py")
        ns = types.SimpleNamespace

        def imu(clock, arrival, index):
            return ns(
                header=Header(stamp=rospy.Time.from_sec(arrival), seq=index),
                imu=ns(header=Header(stamp=rospy.Time.from_sec(clock), seq=index)),
            )

        def tick(clock):
            return ns(timestamp=ns(data=rospy.Time.from_sec(clock)))

        last = module.LastMatch(imu(50, 100.001, 1), tick(100))
        candidates = [tick(100.005), tick(100.010), tick(100.010), tick(100.015)]
        queue = deque(candidates)
        chosen, index = module.select_timestamp_for_imu(imu(50.010, 100.011, 3), queue, last, window_size=3)
        self.assertIs(chosen, candidates[1])
        self.assertEqual(index, 1)
        module.push_deque_index(queue, chosen, index)
        self.assertEqual(list(queue), candidates)
        chosen, index = module.select_timestamp_for_imu(imu(50.010, 100.011, 3), queue, last)
        self.assertEqual(module.TIMESTAMP_MATCH_WINDOW_SIZE, 1)
        self.assertIs(chosen, candidates[0])
        queue = deque([tick(101), tick(100.010)])
        chosen, index = module.select_timestamp_for_imu(imu(50.010, 100.011, 3), queue, None, window_size=2)
        self.assertEqual(chosen.timestamp.data, rospy.Time.from_sec(100.010))
        self.assertEqual(module.select_timestamp_for_imu(imu(1, 1, 1), deque(), None), (None, None))

    def test_overlay_preflight_requires_transforms_and_sensor_data(self):
        module = load("verification/overlay_verification.py")
        transform = TransformStamped(header=Header(frame_id="base"), child_frame_id="camera")
        transform.transform.rotation.w = 1
        with rosbag.Bag(str(self.output), "w") as bag:
            bag.write("/tf_static", TFMessage([transform]), rospy.Time(1))
        with self.assertRaises(ValueError):
            module.prepare_inputs([self.output])
        with rosbag.Bag(str(self.output), "a") as bag:
            bag.write("/boxi/alphasense/front_center/image_raw", Image(), rospy.Time(1))
        paths, flags = module.prepare_inputs([self.output, self.output])
        self.assertEqual(paths, [self.output.resolve()])
        self.assertTrue(flags["enable_alphasense"])
        self.assertFalse(flags["enable_zed"])
        with rosbag.Bag(str(self.output), "w") as bag:
            bag.write("/camera/image", Image(), rospy.Time(1))
        with self.assertRaises(ValueError):
            module.prepare_inputs([self.output])

    def test_replay_preflight_for_both_pose_modes_and_invalid_pose(self):
        module = load_script("application/colorless_mapper.py")
        for kind in (PoseWithCovarianceStamped, Odometry):
            pose = kind(header=Header(stamp=rospy.Time(1), frame_id="map"))
            pose.pose.pose.orientation.w = 1
            from sensor_msgs import point_cloud2

            fields = [
                PointField(name=name, offset=i * 4, datatype=7, count=1)
                for i, name in enumerate(("x", "y", "z", "intensity"))
            ]
            cloud = point_cloud2.create_cloud(Header(stamp=rospy.Time(1), frame_id="lidar"), fields, [(1, 2, 3, 1)])
            tf = TransformStamped(header=Header(frame_id="base"), child_frame_id="lidar")
            tf.transform.rotation.w = 1
            with rosbag.Bag(str(self.output), "w") as bag:
                bag.write("/pose", pose, rospy.Time(1))
                bag.write("/dlio/deskewed_point_cloud", cloud, rospy.Time(1))
                bag.write("/tf_static", TFMessage([tf]), rospy.Time(1))
            module.validate_replay_inputs([self.output], "/pose")
            with self.assertRaises(ValueError):
                module.validate_replay_inputs([self.output], "/missing")
            pose.pose.pose.orientation.w = 0
            with rosbag.Bag(str(self.output), "a") as bag:
                bag.write("/pose", pose, rospy.Time(2))
            with self.assertRaises(ValueError):
                module.validate_replay_inputs([self.output], "/pose")
            with rosbag.Bag(str(self.output), "a") as bag:
                unsupported = point_cloud2.create_cloud_xyz32(
                    Header(stamp=rospy.Time(1), frame_id="lidar"), [(1, 2, 3)]
                )
                bag.write("/dlio/deskewed_point_cloud", unsupported, rospy.Time(1))
            with self.assertRaisesRegex(ValueError, "DLIO cloud layout"):
                module.validate_replay_inputs([self.output], "/pose")

    def test_viewer_padded_endian_depth_and_exact_packed_colors(self):
        module = load("special/sun_in_my_palms_point_clouds.py")
        for endian, big in ((">", True), ("<", False)):
            image = Image(
                height=2,
                width=2,
                step=5,
                encoding="16UC1",
                is_bigendian=big,
                data=struct.pack(endian + "HH", 1000, 2000) + b"x" + struct.pack(endian + "HH", 1500, 0) + b"x",
            )
            np.testing.assert_allclose(module._image_msg_to_depth_meters(image), [[1, 2], [1.5, 0]])
            image.data = image.data[:-1]
            with self.assertRaises(RuntimeError):
                module._image_msg_to_depth_meters(image)
            for datatype in (PointField.UINT32, PointField.FLOAT32):
                cloud = PointCloud2(
                    height=2,
                    width=1,
                    point_step=16,
                    row_step=20,
                    is_bigendian=big,
                    fields=[
                        PointField(name=name, offset=offset, datatype=dtype, count=1)
                        for name, offset, dtype in [("rgba", 12, datatype), ("z", 8, 7), ("x", 0, 7), ("y", 4, 7)]
                    ],
                    data=(struct.pack(endian + "fffI", 1, 2, 3, 0xFFFF1122) + b"pad!") * 2,
                )
                xyz, rgb = module._pc2_to_numpy_ros1(cloud)
                np.testing.assert_array_equal(xyz, [[1, 2, 3]] * 2)
                np.testing.assert_allclose(rgb, [[1, 17 / 255, 34 / 255]] * 2)
        info = CameraInfo(K=[float("nan"), 0, 0, 0, 1, 0, 0, 0, 1])
        with self.assertRaises(RuntimeError):
            module._camera_info_to_intrinsics(info)
        info = CameraInfo(
            K=[1, 0, 0, 0, 1, 0, 0, 0, 1], P=[4, 0, 10, 0, 0, 8, 12, 0, 0, 0, 1, 0], binning_x=2, binning_y=2
        )
        info.roi.x_offset, info.roi.y_offset = 2, 4
        self.assertEqual(module._camera_info_to_intrinsics(info), (2, 4, 4, 4))
        xyz = module._depth_to_pointcloud(np.ones((2, 2)), 2, 2, 0, 0, stride=1)
        np.testing.assert_allclose(xyz, [[0, 0, 1], [0.5, 0, 1], [0, 0.5, 1], [0.5, 0.5, 1]])

    def test_bcc_word_boundaries_deduplication_and_sparse_rows(self):
        from box_auto.utils.extract_grandtour_bcc import extract_emails

        path = self.directory / "responses.csv"
        with path.open("w", newline="") as stream:
            writer = csv.writer(stream)
            writer.writerow(list("ABCDEFG"))
            for address, answer in [
                ("one@example.com", "Yes"),
                ("ONE@example.com", "yes"),
                ("two@example.com", "yesterday"),
                ("three@example.com", "Yes and No"),
                ("four@example.com", "No"),
                ("a@b, c@d", "Yes"),
            ]:
                writer.writerow([""] * 5 + [address, answer])
            writer.writerow(["incomplete"])
        with contextlib.redirect_stderr(io.StringIO()):
            addresses, counts = extract_emails(path)
        self.assertEqual(addresses, ["one@example.com"])
        self.assertEqual(counts["ambiguous"], 1)

    def test_scatter_rejects_bad_metrics_and_preserves_report(self):
        module = load("special/scatter_plot_ate_rte.py")
        source, output = self.directory / "results.csv", self.directory / "results.png"
        source.write_text("name,ate,rte\nmethod,0.2,0.1\n")
        args = ["--csv", str(source), "--output", str(output), "--ate-unit", "m", "--rte-unit", "m"]
        module.main(args)
        original = output.read_bytes()
        self.assertTrue(original.startswith(b"\x89PNG"))
        with self.assertRaises(FileExistsError):
            module.main(args)
        self.assertEqual(output.read_bytes(), original)
        source.write_text("name,ate,rte\nmethod,nan,0.1\n")
        with self.assertRaises(ValueError):
            module.main(args)
