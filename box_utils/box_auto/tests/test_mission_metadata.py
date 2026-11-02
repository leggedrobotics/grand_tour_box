"""Partial-deployment metadata tests: synthetic bags, isolated output, no cloud access."""

import contextlib
import importlib.util
import io
from pathlib import Path
import sys
import tempfile
import types
import unittest
from unittest.mock import Mock, patch

import numpy as np
import rosbag
import rospy
import yaml
from geometry_msgs.msg import PoseWithCovarianceStamped

PACKAGE = Path(__file__).parents[1]


def load_metadata(root):
    parent = types.ModuleType("box_auto")
    parent.__path__ = [str(PACKAGE / "python/box_auto")]
    utils = types.ModuleType("box_auto.utils")
    utils.__path__ = [str(PACKAGE / "python/box_auto/utils")]
    utils.BOX_AUTO_DIR = str(root)
    utils.deployments = []
    utils.get_bag = Mock()
    utils.get_file = Mock(return_value=(None, False))
    ap20 = types.ModuleType("box_auto.scripts.ap20")
    ap20.process_prism_position_bag = Mock(return_value=(None, np.zeros(3)))
    spec = importlib.util.spec_from_file_location(
        "metadata_under_test", PACKAGE / "python/box_auto/scripts/verification/create_mission_metadata.py"
    )
    module = importlib.util.module_from_spec(spec)
    with patch.dict(sys.modules, {"box_auto": parent, "box_auto.utils": utils, "box_auto.scripts.ap20": ap20}):
        spec.loader.exec_module(module)
    return module


class MissionMetadata(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name)
        self.output = self.root / "cfg/mission_metadata.yaml"
        self.output.parent.mkdir()
        self.old = {"unmounted": {"distance_walked": 42.0, "prism": "trusted old metadata"}}
        self.output.write_text(yaml.safe_dump(self.old))
        self.original = self.output.read_bytes()
        self.module = load_metadata(self.root)

    def run_metadata(self):
        with contextlib.redirect_stdout(io.StringIO()):
            return self.module.create_metadata()

    def add_mission(self, name, moving=True):
        folder = self.root / name
        folder.mkdir()
        bag = folder / "test_lpc_state_estimator.bag"
        with rosbag.Bag(str(bag), "w") as output:
            for i, position in enumerate([0.0, 1.0, 2.0, 2.0] if moving else [0.0, 0.0, 0.0, 0.0]):
                msg = PoseWithCovarianceStamped()
                msg.header.stamp = rospy.Time(1700000000 + i)
                msg.pose.pose.position.x = position
                output.write("/state_estimator/pose_in_odom", msg, msg.header.stamp)
        self.module.deployments.append({"data_folder": str(self.root), "mission_names": [name]})
        self.module.get_bag.side_effect = lambda pattern, directory: str(
            Path(directory)
            / ("test_lpc_state_estimator.bag" if "state_estimator" in pattern else "test_jetson_ap20_synced.bag")
        )
        return folder

    def test_empty_selection_preserves_existing_file(self):
        self.assertEqual(self.run_metadata(), {})
        self.assertEqual(self.output.read_bytes(), self.original)
        self.module.get_bag.assert_not_called()

    def test_missing_directory_skips_download_and_preserves_file(self):
        self.module.deployments = [{"data_folder": str(self.root), "mission_names": ["missing"]}]
        self.assertEqual(self.run_metadata(), {})
        self.module.get_bag.assert_not_called()
        self.assertEqual(self.output.read_bytes(), self.original)

    def test_partial_collection_merges_and_applies_only_present_overrides(self):
        name = "2024-10-29-09-08-34"
        self.add_mission(name)
        self.module.deployments.append({"data_folder": str(self.root), "mission_names": ["missing"]})
        result = self.run_metadata()
        self.assertEqual(list(result), [name])
        written = yaml.safe_load(self.output.read_text())
        self.assertEqual(written["unmounted"], self.old["unmounted"])
        self.assertFalse(written[name]["ap20_bag_available"])
        self.assertEqual(written[name]["prism"], "Not at start and end")
        self.assertEqual(written[name]["distance_walked"], 2.0)
        self.assertGreater(written[name]["mission_duration"], 0)

    def test_new_mission_without_historical_overrides_succeeds(self):
        self.add_mission("new_mission")
        self.run_metadata()
        written = yaml.safe_load(self.output.read_text())
        self.assertEqual(set(written), {"new_mission", "unmounted"})

    def test_stationary_second_mission_cannot_reuse_first_mission_times(self):
        self.add_mission("moving")
        self.add_mission("stationary", moving=False)
        with self.assertRaisesRegex(ValueError, "movement interval"):
            self.run_metadata()
        self.assertEqual(self.output.read_bytes(), self.original)

    def test_invalid_previous_metadata_preserved(self):
        self.add_mission("new_mission")
        for original in ("- invalid-list\n", "[]\n", "false\n"):
            with self.subTest(original=original):
                self.output.write_text(original)
                with self.assertRaisesRegex(ValueError, "mission mapping"):
                    self.run_metadata()
                self.assertEqual(self.output.read_text(), original)

    def test_write_failure_preserves_previous_file_and_cleans_temporary(self):
        self.add_mission("new_mission")
        with patch.object(self.module.yaml, "safe_dump", side_effect=OSError("disk full")):
            with self.assertRaises(OSError):
                self.run_metadata()
        self.assertEqual(self.output.read_bytes(), self.original)
        self.assertEqual(list(self.output.parent.iterdir()), [self.output])


if __name__ == "__main__":
    unittest.main()
