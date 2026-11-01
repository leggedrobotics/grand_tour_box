"""Input, covariance, codec and evaluation contracts using disposable fixtures."""

import importlib.util
import sys
from pathlib import Path
import unittest
from unittest.mock import patch
import numpy as np
import cv2
import rosbag
import rospy
import yaml
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import Header
from test_dataset_reliability import TemporaryCase, PACKAGE, load_script, extracted_function


def load_file(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


utils = PACKAGE / "python/box_auto/utils"
stats = load_file(utils / "trajectory_statistics.py", "test_trajectory_statistics")
config = load_file(utils / "evaluation_config.py", "test_evaluation_config")
codec = load_file(PACKAGE.parent / "box_converter/image-anonymization-pipeline/image_codec.py", "test_image_codec")
heading = load_file(PACKAGE / "python/box_auto/scripts/verification/alb1_heading_deep_dive.py", "test_heading_analysis")
posecov = load_script("cpt7/Not_needed_generatePoseStampedWithCov.py")
table = load_script("verification/generate_ijrr_kio_table.py")


class SupportCorrectnessTests(TemporaryCase):
    def odometry_bag(self, samples):
        path = self.directory / "odom.bag"
        with rosbag.Bag(str(path), "w") as bag:
            for i, (stamp, xyz) in enumerate(samples):
                msg = Odometry(header=Header(seq=i, stamp=rospy.Time.from_sec(stamp), frame_id="map"))
                msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z = xyz
                msg.pose.pose.orientation.w = 1
                msg.pose.covariance = np.eye(6).reshape(-1).tolist()
                bag.write("/odom", msg, rospy.Time(10 + i))
        return path

    def test_distance_and_partial_coverage(self):
        path = self.odometry_bag([(1, (0, 0, 0)), (2, (3, 4, 0)), (3, (3, 4, 12))])
        distance, edges = stats.compute_odom_edges_and_length(path, "/odom")
        self.assertEqual(distance, 17)
        self.assertEqual(stats.covered_distance_from_edges(edges, [(1.5, 2.5)]), 8.5)
        self.assertEqual(stats.compute_odom_edges_and_length(path, "/missing"), (None, []))

    def test_invalid_odometry_fails_instead_of_underreporting(self):
        for samples in [
            [(1, (0, 0, 0)), (1, (1, 0, 0))],
            [(0, (0, 0, 0))],
            [(1, (float("nan"), 0, 0))],
            [(1, (0, 0, 0)), (2, (1000, 0, 0))],
        ]:
            with self.assertRaises(ValueError):
                stats.compute_odom_edges_and_length(self.odometry_bag(samples), "/odom")

    def test_availability_gap_policy(self):
        intervals = stats.build_intervals_from_times([1, 1.1, 1.2, 2, 2.1])
        np.testing.assert_allclose(intervals, [(0.95, 1.25), (1.95, 2.15)])
        for times in ([1, 1], [2, 1], [1, float("nan")]):
            with self.assertRaises(ValueError):
                stats.build_intervals_from_times(times)

    def test_big_endian_mono16_padding_roundtrip(self):
        values = np.array([[1, 256, 65535], [1024, 40000, 7]], dtype=">u2")
        data = b"".join(row.tobytes() + b"xx" for row in values)
        msg = Image(
            header=Header(seq=7, stamp=rospy.Time(2), frame_id="camera"),
            height=2,
            width=3,
            encoding="mono16",
            is_bigendian=1,
            step=8,
            data=data,
        )
        image, gray, compressed = codec.decode_image_message(msg, "sensor_msgs/Image")
        out = codec._encode_raw_image_msg(image, msg)
        self.assertTrue(gray)
        self.assertFalse(compressed)
        np.testing.assert_array_equal(np.frombuffer(out.data, ">u2").reshape(2, 3), values)
        self.assertEqual(out.header, msg.header)
        self.assertEqual(out.is_bigendian, 1)

    def test_alpha_preserved_raw_and_png(self):
        pixels = np.arange(24, dtype=np.uint8).reshape(2, 3, 4)
        for raw in (True, False):
            if raw:
                msg = Image(height=2, width=3, encoding="rgba8", step=12, data=pixels.tobytes())
            else:
                msg = CompressedImage(format="png", data=cv2.imencode(".png", pixels)[1].tobytes())
            image, gray, _ = codec.decode_image_message(
                msg, "sensor_msgs/Image" if raw else "sensor_msgs/CompressedImage"
            )
            image[:] = 0
            if raw:
                out = codec._encode_raw_image_msg(image, msg)
                result = np.frombuffer(out.data, np.uint8).reshape(2, 3, 4)
            else:
                result = cv2.imdecode(
                    np.frombuffer(codec._reencode_compressed_image(image, gray, msg.format, msg), np.uint8),
                    cv2.IMREAD_UNCHANGED,
                )
            np.testing.assert_array_equal(result[..., 3], pixels[..., 3])
            self.assertTrue((result[..., :3] == 0).all())

    def test_codec_rejects_stride_corruption_and_lossy_16bit(self):
        with self.assertRaises(ValueError):
            codec._decode_raw_image_with_step(b"123", 2, 2, 2, np.uint8, 1)
        with self.assertRaises(ValueError):
            codec._reencode_compressed_image(np.ones((2, 2, 3), np.uint16), False, "jpeg")

    def test_pose_view_preserves_input_clock_pose_covariance(self):
        source = self.odometry_bag([(1, (1, 2, 3)), (2, (4, 5, 6))])
        self.assertEqual(posecov.write_bag_with_posecov(source, self.output, "/odom", "/pose"), 2)
        with rosbag.Bag(str(self.output)) as bag:
            messages = list(bag.read_messages())
        self.assertEqual([x.timestamp.to_sec() for x in messages], [10, 10, 11, 11])
        self.assertEqual(messages[0].message.header.seq, messages[1].message.header.seq)
        self.assertEqual(messages[1].message.pose.covariance, messages[0].message.pose.covariance)
        with self.assertRaises(ValueError):
            posecov.write_bag_with_posecov(self.output, self.directory / "second.bag", "/odom", "/pose")

    def test_all_shipped_evaluation_presets(self):
        paths = list((PACKAGE / "cfg").glob("evo*.yaml")) + [PACKAGE / "cfg/IJRR_revision.yaml"]
        for path in paths:
            with self.subTest(path=path.name):
                result = config.validate_evaluation_config(yaml.safe_load(path.read_text()))
                self.assertIn("ap20_velocity_rejection", result)
        bad = yaml.safe_load(paths[0].read_text())
        bad["bag_topic_pairs"].append(bad["bag_topic_pairs"][0])
        with self.assertRaises(ValueError):
            config.validate_evaluation_config(bad)

    def test_heading_position_alignment_and_degeneracy(self):
        from scipy.spatial.transform import Rotation

        source = np.array([[0.0, 0, 0], [1, 0, 0], [0, 2, 0], [0, 0, 3]])
        rotation = Rotation.from_euler("xyz", [0.2, -0.4, 0.6]).as_matrix()
        translation = np.array([4.0, -2.0, 8.0])
        target = source @ rotation.T + translation
        actual, offset, scale, _ = heading.umeyama_similarity(source, target, False)
        np.testing.assert_allclose(actual, rotation, atol=1e-12)
        np.testing.assert_allclose(offset, translation, atol=1e-12)
        self.assertEqual(scale, 1)
        line = np.array([[0.0, 0, 0], [1, 0, 0], [2, 0, 0]])
        with self.assertRaises(ValueError):
            heading.umeyama_similarity(line, line, False)
        np.testing.assert_allclose(heading.wrap_deg(np.array([181.0, -181.0, 360.0])), [-179.0, 179.0, 0.0])

    def test_statistics_append_preserves_invalid_existing_output(self):
        # Load the shared writer with the real atomic publisher and no cloud imports.
        append = extracted_function(utils / "trajectory_statistics.py", "append_summary", {"Path": Path})
        import types

        utilities = types.ModuleType("box_auto.utils")
        utilities.__path__ = [str(utils)]
        parent = types.ModuleType("box_auto")
        parent.__path__ = [str(PACKAGE / "python/box_auto")]
        with patch.dict(sys.modules, {"box_auto": parent, "box_auto.utils": utilities}):
            self.output.write_text("not: a list")
            with self.assertRaises(ValueError):
                append(self.output, {"distance_m": 5.0})
            self.assertEqual(self.output.read_text(), "not: a list")
            self.output.unlink()
            append(self.output, {"distance_m": 5.0})
            append(self.output, {"distance_m": 7.0})
            self.assertEqual(yaml.safe_load(self.output.read_text()), [{"distance_m": 5.0}, {"distance_m": 7.0}])

    def test_sparse_sheet_rows_and_duplicate_identifiers(self):
        parse_topics = extracted_function(utils / "google_sheet.py", "parse_topic_rows", {})
        parse_missions = extracted_function(utils / "google_sheet.py", "parse_mission_rows", {})
        topics = parse_topics([[], ["Bag: 2024-11-11-12-42-47_camera.bag"], ["/image", "Image", "cam", "Yes"]])
        self.assertEqual(topics[0]["bag_name_orig"], "camera.bag")
        self.assertEqual(topics[0]["bag_name_out"], "")
        self.assertEqual(
            parse_missions([["id", "name", "value"], [], ["mission", "test"]]),
            {"mission": {"name": "test", "value": ""}},
        )
        with self.assertRaises(ValueError):
            parse_missions([["id", "name"], ["same", "a"], ["same", "b"]])
        with self.assertRaises(ValueError):
            parse_topics([["/image", "Image", "cam", "Yes"]])

    def test_covariance_plot_requires_matching_full_covariance(self):
        get_std = extracted_function(
            PACKAGE / "python/box_auto/scripts/verification/plot_with_covariance.py", "get_position_std", {"np": np}
        )
        path = self.directory / "covariance.tum"
        covariance = np.diag([1.0, 4.0, 9.0, 0.1, 0.1, 0.1])
        data = np.column_stack(([100.0, 101.0], np.tile(covariance.reshape(-1), (2, 1))))
        np.savetxt(path, data, header="timestamp full_covariance")
        np.testing.assert_allclose(get_std(path, np.array([100.0, 101.0])), [[1, 2, 3], [1, 2, 3]])
        with self.assertRaises(ValueError):
            get_std(path, np.array([100.0, 102.0]))
        data[0, 1] = -1
        np.savetxt(path, data, header="timestamp full_covariance")
        with self.assertRaises(ValueError):
            get_std(path, np.array([100.0, 101.0]))

    def test_output_protection_and_explicit_replacement(self):
        missions = self.directory / "missions"
        workspace = self.directory / "workspace"
        missions.mkdir()
        workspace.mkdir()
        for path in (self.directory, missions, missions / "mission"):
            with self.assertRaises(ValueError):
                table.ensure_safe_output_dir(path, missions, workspace)
        out = self.directory / "results"
        out.mkdir()
        (out / "keep").write_text("precious")
        with self.assertRaises(ValueError):
            table.prepare_output_dir(out, missions, workspace, True, False)
        self.assertEqual((out / "keep").read_text(), "precious")
        fresh = self.directory / "fresh"
        table.prepare_output_dir(fresh, missions, workspace, False, False)
        table.prepare_output_dir(fresh, missions, workspace, True, False)
        self.assertTrue((fresh / ".box_auto_kio_output").exists())

    def test_missing_methods_have_no_expected_zips(self):
        self.assertEqual(table.expected_result_paths(self.directory, {"bag_topic_pairs": []}), [])
        self.assertFalse(table.all_expected_zips_exist(self.directory, {"bag_topic_pairs": []}))


if __name__ == "__main__":
    unittest.main()
