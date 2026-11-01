"""Exercise the selected CPT7 exports from CSV through serialized ROS bags."""

import ast
import contextlib
import importlib.util
import io
import logging
import sys
import types
from unittest.mock import patch

import numpy as np
import pandas as pd
import rosbag
from scipy.spatial.transform import Rotation

from test_dataset_reliability import TemporaryCase, PACKAGE, SCRIPTS, load_script


def load_ie(mission):
    spec = importlib.util.spec_from_file_location("contract_gps", PACKAGE / "python/box_auto/utils/gps_utils.py")
    gps = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(gps)
    utils = types.ModuleType("box_auto.utils")
    utils.__path__ = [str(PACKAGE / "python/box_auto/utils")]
    utils.GPS_utils = gps.GPS_utils
    utils.MISSION_DATA = str(mission)
    parent = types.ModuleType("box_auto")
    parent.__path__ = [str(PACKAGE / "python/box_auto")]
    spec = importlib.util.spec_from_file_location("contract_ie", SCRIPTS / "cpt7/export_gps_gt_trajectory_bag.py")
    module = importlib.util.module_from_spec(spec)
    with patch.dict(sys.modules, {"box_auto": parent, "box_auto.utils": utils}):
        spec.loader.exec_module(module)
    return module


class CPT7ExportContracts(TemporaryCase):
    def test_ie_dms_preserves_negative_zero_degrees(self):
        module = load_ie(self.directory)
        self.assertEqual(module.dms_string_to_decimal("-0 30 0"), -0.5)
        self.assertEqual(module.dms_string_to_decimal("0 30 0"), 0.5)

    def test_rawimu_default_batch_samples_units_and_failure_preservation(self):
        module = load_script("cpt7/export_raw_imu_bag.py")
        self.assertEqual(module.RAWIMUDataParser("/imu").expected_utc_offset, -18.0)
        for folder in ("first", "second"):
            directory = self.directory / folder
            directory.mkdir()
            source = directory / "data_RAWIMUSX.ASCII"
            source.write_text("%RAWIMUSXA,0,0,68,2300,10.01,1,2,3,4,5,6\n%RAWIMUSXA,0,0,68,2300,10.02,1,2,3,4,5,6\n")
        outputs = module.convert_all_rawimu(self.directory, logging.getLogger("rawimu_contract"))
        self.assertEqual(len(outputs), 2)
        for output in outputs:
            with rosbag.Bag(str(output)) as bag:
                messages = list(bag.read_messages())
            self.assertEqual(len(messages), 2)
            first = messages[0]
            self.assertEqual(first.message.header.stamp, first.timestamp)
            self.assertAlmostEqual(first.timestamp.to_sec(), 315964800 + 2300 * 604800 + 10.01 - 18, places=6)
            self.assertEqual(first.message.orientation_covariance[0], -1.0)
            np.testing.assert_allclose(
                [
                    first.message.linear_acceleration.x,
                    first.message.linear_acceleration.y,
                    first.message.linear_acceleration.z,
                ],
                np.array([3.0, -2.0, 1.0]) * 100 * 2**-29,
            )
            np.testing.assert_allclose(
                [first.message.angular_velocity.x, first.message.angular_velocity.y, first.message.angular_velocity.z],
                np.array([6.0, -5.0, 4.0]) * 100 * 2**-33,
            )
        before = outputs[0].read_bytes()
        bad = self.directory / "first/data_RAWIMUSX.ASCII"
        bad.write_text(bad.read_text().replace(",68,", ",99,"))
        with self.assertRaises(RuntimeError):
            module.convert_all_rawimu(self.directory, logging.getLogger("rawimu_contract"))
        self.assertEqual(outputs[0].read_bytes(), before)

    def test_ie_pose_covariance_reaches_both_topics_and_failure_preserves_bag(self):
        module = load_ie(self.directory)
        (self.directory / "2024_nuc_livox.bag").touch()
        directory = self.directory / "ie/ppp"
        directory.mkdir(parents=True)
        source = directory / "ie_ppp_GrandTour-LocalFrame-extended.txt"
        # Discover the CSV schema, then supply independent physical/numeric fixtures.
        columns = []
        for node in ast.walk(ast.parse((SCRIPTS / "cpt7/export_gps_gt_trajectory_bag.py").read_text())):
            if isinstance(node, ast.Assign) and isinstance(node.value, ast.List):
                if any(
                    isinstance(t, ast.Name) and (t.id.endswith("_columns") or t.id.endswith("_column"))
                    for t in node.targets
                ):
                    columns.extend(ast.literal_eval(node.value))
        data = pd.DataFrame({name: np.zeros(10) for name in ["Time"] + columns})
        data["Time"] = 1700000000.0 + np.arange(10) * 0.1
        data[["X-ECEF", "Y-ECEF", "Z-ECEF"]] = [6378137.0, 0.0, 0.0]
        data[["Heading", "Roll", "Pitch"]] = [34.0, 17.0, -9.0]
        data[["HdngSD", "RollSD", "PitchSD"]] = [2.0, 3.0, 5.0]
        data[["Cx11", "Cx22", "Cx33", "Vel-Cx11", "Vel-Cx22", "Vel-Cx33"]] = [1.0, 2.0, 3.0, 0.1, 0.2, 0.3]
        data["Latitude"] = "0 0 0"
        data["Longitude"] = "0 0 0"
        data["AmbStatus"] = "Fixed"
        data[["Q", "NS", "PDOP"]] = [1, 10, 2]
        data.to_csv(source, index=False)
        with contextlib.redirect_stdout(io.StringIO()):
            module.main()
        output = self.directory / "ie/2024_cpt7_ie_ppp.bag"
        with rosbag.Bag(str(output)) as bag:
            messages = list(bag.read_messages())
        poses = [m for t, m, stamp in messages if t.endswith("/pose_with_covariance")]
        odometry = [m for t, m, stamp in messages if t.endswith("/odometry")]
        self.assertEqual((len(poses), len(odometry)), (10, 10))
        angles = np.array([34.0, 17.0, -9.0])
        transform = np.array([[0.0, 1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, -1.0]])
        base = Rotation.from_matrix(transform @ Rotation.from_euler("ZXY", angles, degrees=True).as_matrix())
        jacobian = []
        step = 1e-5
        for axis in range(3):
            perturbed = angles.copy()
            perturbed[axis] += step
            rotation = Rotation.from_matrix(transform @ Rotation.from_euler("ZXY", perturbed, degrees=True).as_matrix())
            jacobian.append((rotation * base.inv()).as_rotvec() / np.deg2rad(step))
        jacobian = np.array(jacobian).T
        expected = jacobian @ np.diag(np.deg2rad([2.0, 3.0, 5.0]) ** 2) @ jacobian.T
        for pose, odom in zip(poses, odometry):
            # rosbag can generate distinct Python classes for equivalent nested
            # types; compare their wire representation rather than class identity.
            odom_bytes, pose_bytes = io.BytesIO(), io.BytesIO()
            odom.pose.serialize(odom_bytes)
            pose.pose.serialize(pose_bytes)
            self.assertEqual(odom_bytes.getvalue(), pose_bytes.getvalue())
            q = odom.pose.pose.orientation
            np.testing.assert_allclose([q.x, q.y, q.z, q.w], base.as_quat())
            covariance = np.asarray(pose.pose.covariance).reshape(6, 6)
            np.testing.assert_allclose(covariance[3:, 3:], expected, atol=1e-9, rtol=1e-6)
            np.testing.assert_allclose(covariance[:3, :3], np.diag([2.0, 3.0, 1.0]))
        previous = output.read_bytes()
        data.loc[5, "Cx11"] = -1
        data.to_csv(source, index=False)
        with contextlib.redirect_stdout(io.StringIO()), self.assertRaises(ValueError):
            module.main()
        self.assertEqual(output.read_bytes(), previous)
        self.assertFalse(list(output.parent.glob("." + output.name + ".*")))
