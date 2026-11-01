#!/usr/bin/env python3
"""Run with: python3 test_merge.py /path/to/merge. TMPDIR selects the test filesystem."""

import hashlib
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

import rosbag
import rospy
from std_msgs.msg import String


MERGE = str(Path(sys.argv.pop(1)).resolve())


class MergeContracts(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix="merge_contract_")
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)

    def write(self, name, value, stamp=1):
        path = self.root / name
        path.parent.mkdir(parents=True, exist_ok=True)
        with rosbag.Bag(str(path), "w") as bag:
            bag.write("/payload", String(value), rospy.Time(stamp))
        return path

    def run_merge(self):
        return subprocess.run([MERGE, str(self.root)], capture_output=True, text=True, timeout=30)

    def test_records_and_repeat_output_are_preserved(self):
        self.write("mission_sensor_10.bag", "ten", 10)
        self.write("mission_sensor_2.bag", "two", 2)
        result = self.run_merge()
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        output = self.root / "mission_sensor.bag"
        with rosbag.Bag(str(output)) as bag:
            records = [(topic, msg.data, stamp.to_sec()) for topic, msg, stamp in bag.read_messages()]
        self.assertEqual(records, [("/payload", "two", 2), ("/payload", "ten", 10)])
        digest = hashlib.sha256(output.read_bytes()).digest()
        self.assertEqual(self.run_merge().returncode, 0)
        self.assertEqual(hashlib.sha256(output.read_bytes()).digest(), digest)

    def test_cross_directory_groups_fail_before_any_output(self):
        # Also include a valid earlier group: preflight must catch ambiguity
        # before publishing an unrelated result from the same invocation.
        self.write("aaa_sensor_0.bag", "valid")
        self.write("one/mission_sensor_0.bag", "one")
        self.write("two/mission_sensor_1.bag", "two")
        result = self.run_merge()
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("spans multiple directories", result.stderr)
        self.assertFalse((self.root / "aaa_sensor.bag").exists())
        self.assertFalse((self.root / "mission_sensor.bag").exists())

    def test_duplicate_numeric_split_indices_fail(self):
        self.write("mission_sensor_0.bag", "one")
        self.write("mission_sensor_00.bag", "duplicate")
        result = self.run_merge()
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("Duplicate split index", result.stderr)
        self.assertFalse((self.root / "mission_sensor.bag").exists())

    def test_corrupt_input_leaves_no_output_or_staging(self):
        self.write("mission_sensor_0.bag", "valid")
        (self.root / "mission_sensor_1.bag").write_bytes(b"corrupt")
        self.assertNotEqual(self.run_merge().returncode, 0)
        self.assertFalse((self.root / "mission_sensor.bag").exists())
        self.assertFalse(list(self.root.glob(".*")))

    def test_existing_output_is_preserved(self):
        self.write("mission_sensor_0.bag", "input")
        output = self.root / "mission_sensor.bag"
        output.write_bytes(b"precious existing output")
        self.assertEqual(self.run_merge().returncode, 0)
        self.assertEqual(output.read_bytes(), b"precious existing output")

    def test_empty_directory_fails(self):
        self.assertNotEqual(self.run_merge().returncode, 0)


if __name__ == "__main__":
    unittest.main()
