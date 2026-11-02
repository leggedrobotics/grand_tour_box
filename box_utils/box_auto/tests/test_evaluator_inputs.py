"""Exercise the compiled converter with disposable bags and a private ROS master.

Run in a ROS Noetic Python environment with EVO_CONVERSION_NODE pointing to the
newly built executable. EVO_BASELINE_NODE optionally compares a previous build.
No dataset or existing ROS master is used; all generated files are removed.
"""

import io
import os
from pathlib import Path
import signal
import socket
import subprocess
import tempfile
import time
import unittest
import xmlrpc.client

import rosbag
import rospy
import yaml
from geometry_msgs.msg import PointStamped, TransformStamped
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage


PACKAGE = Path(__file__).resolve().parents[1]


@unittest.skipUnless(os.environ.get("EVO_CONVERSION_NODE"), "Set EVO_CONVERSION_NODE to a fresh build")
class EvaluatorInputs(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.executable = str(Path(os.environ["EVO_CONVERSION_NODE"]).resolve(strict=True))
        cls.temporary = tempfile.TemporaryDirectory(prefix="evaluator-inputs-")
        cls.addClassCleanup(cls.temporary.cleanup)
        cls.root = Path(cls.temporary.name)
        with socket.socket() as listener:
            listener.bind(("127.0.0.1", 0))
            port = listener.getsockname()[1]
        cls.environment = dict(os.environ)
        cls.environment.update(
            ROS_MASTER_URI=f"http://127.0.0.1:{port}",
            ROS_IP="127.0.0.1",
            ROS_HOSTNAME="127.0.0.1",
            ROS_HOME=str(cls.root / "ros-home"),
            ROS_LOG_DIR=str(cls.root / "ros-logs"),
        )
        log = (cls.root / "roscore.log").open("w")
        cls.addClassCleanup(log.close)
        cls.roscore = subprocess.Popen(
            ["roscore", "-p", str(port)], env=cls.environment,
            stdout=log, stderr=subprocess.STDOUT, start_new_session=True,
        )
        cls.addClassCleanup(cls.stop_roscore)
        cls.master = xmlrpc.client.ServerProxy(cls.environment["ROS_MASTER_URI"])
        deadline = time.monotonic() + 15
        while time.monotonic() < deadline:
            try:
                if cls.master.getPid("/evaluator_test")[0] == 1:
                    break
            except OSError:
                pass
            if cls.roscore.poll() is not None:
                raise RuntimeError((cls.root / "roscore.log").read_text())
            time.sleep(0.05)
        else:
            raise RuntimeError("Private ROS master failed to start")
        with (PACKAGE / "cfg/evo_evaluations.yaml").open() as stream:
            cls.config = yaml.safe_load(stream)

    @classmethod
    def stop_roscore(cls):
        if cls.roscore.poll() is None:
            os.killpg(cls.roscore.pid, signal.SIGINT)
            try:
                cls.roscore.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(cls.roscore.pid, signal.SIGKILL)
                cls.roscore.wait(timeout=5)

    def setUp(self):
        self.directory = self.root / self._testMethodName
        self.directory.mkdir()
        self.inputs = self.directory / "inputs"
        self.inputs.mkdir()

    def write_bags(self, record_stamps=None):
        if record_stamps is None:
            record_stamps = [rospy.Time(1700000000 + i) for i in range(3)]
        entries = self.config["bag_topic_pairs"]
        for entry in entries:
            with rosbag.Bag(str(self.inputs / ("sample" + entry["bag"])), "w") as bag:
                if not entry["topic"]:
                    transform = TransformStamped()
                    transform.header.frame_id = "hesai_lidar"
                    transform.child_frame_id = "prism"
                    transform.header.stamp = rospy.Time(1700000000)
                    transform.transform.translation.x = 1
                    transform.transform.rotation.w = 1
                    bag.write("/tf_static", TFMessage(transforms=[transform]), transform.header.stamp)
                    continue
                for i, stamp in enumerate(record_stamps):
                    if entry["test_name"] == "dlio":
                        message = Odometry()
                        message.child_frame_id = "hesai_lidar"
                        message.pose.pose.position.x = i
                        message.pose.pose.orientation.w = 1
                        message.pose.covariance = [0.0] * 36
                        for axis in range(6):
                            message.pose.covariance[axis * 7] = axis + 1
                    else:
                        message = PointStamped()
                        message.point.x = i + 1
                    message.header.frame_id = "odom"
                    message.header.stamp = rospy.Time(1700000000 + i)
                    bag.write(entry["topic"], message, stamp)

    def convert(self, name="output", simulated_time=False, executable=None):
        for key, value in (
            ("/use_sim_time", simulated_time),
            ("/target_frame", self.config["target_frame"]),
            ("/bag_topic_pairs", self.config["bag_topic_pairs"]),
        ):
            self.assertEqual(self.master.setParam("/evaluator_test", key, value)[0], 1)
        output = self.directory / name
        result = subprocess.run(
            [executable or self.executable, "__name:=evaluator_test",
             "_bag_file_directory:=" + str(self.inputs), "_prefix:=sample",
             "_enable_interpolation:=false", "_output_folder_path:=" + str(output)],
            env=self.environment, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
            text=True, timeout=15,
        )
        return result, output

    def assert_valid_output(self, output):
        for name in ("dlio", "ap20_robot"):
            rows = [list(map(float, line.split())) for line in
                    (output / (name + ".tum")).read_text().splitlines() if not line.startswith("#")]
            self.assertEqual(rows, [[1700000000 + i, i + 1, 0, 0, 0, 0, 0, 1] for i in range(3)])
        with rosbag.Bag(str(output / "sample_evaluation_pose_collections.bag")) as bag:
            messages = list(bag.read_messages())
        self.assertEqual(len(messages), 3)
        for i, (topic, message, stamp) in enumerate(messages):
            self.assertEqual(topic, "/dlio")
            self.assertEqual(stamp, rospy.Time(1700000000 + i))
            self.assertEqual(message.header.stamp, stamp)
            self.assertEqual(message.header.frame_id, "odom")
            self.assertEqual(message.child_frame_id, "prism")
            self.assertEqual(message.pose.pose.position.x, i + 1)
            # Unit x lever arm adds yaw/pitch uncertainty to y/z and retains
            # their cross-covariances. These are independent analytic values.
            self.assertEqual([message.pose.covariance[j * 7] for j in range(6)], [1, 8, 8, 4, 5, 6])
            self.assertEqual(message.pose.covariance[1 * 6 + 5], 6)
            self.assertEqual(message.pose.covariance[2 * 6 + 4], -5)

    def test_shipped_preset_converts_pose_covariance_and_ap20(self):
        self.write_bags()
        result, output = self.convert()
        self.assertEqual(result.returncode, 0, result.stdout)
        self.assert_valid_output(output)

    def test_offline_simulated_time_needs_no_clock_publisher(self):
        self.write_bags()
        result, output = self.convert(simulated_time=True)
        self.assertEqual(result.returncode, 0, result.stdout)
        self.assert_valid_output(output)

    def test_invalid_record_time_spans_fail_before_writing_trajectories(self):
        cases = (
            ("empty", [], "contains no messages"),
            ("single", [rospy.Time(1700000000)], "non-increasing timestamps"),
            ("equal", [rospy.Time(1700000000)] * 2, "non-increasing timestamps"),
            ("zero", [rospy.Time(0), rospy.Time(1)], "zero"),
            ("minimum", [rospy.Time(0, 1), rospy.Time(1)], "TIME_MIN or TIME_MAX"),
            ("maximum", [rospy.Time(4294967294), rospy.Time(4294967295, 999999999)], "TIME_MIN or TIME_MAX"),
        )
        for name, stamps, diagnostic in cases:
            with self.subTest(name=name):
                self.write_bags(stamps)
                result, output = self.convert(name=name)
                self.assertNotEqual(result.returncode, 0, result.stdout)
                self.assertIn(diagnostic, result.stdout)
                self.assertEqual(list(output.iterdir()), [])

    def test_existing_results_are_preserved(self):
        self.write_bags()
        output = self.directory / "output"
        output.mkdir()
        sentinel = output / "dlio.tum"
        sentinel.write_text("existing result\n")
        result, _ = self.convert()
        self.assertNotEqual(result.returncode, 0, result.stdout)
        self.assertEqual(sentinel.read_text(), "existing result\n")
        self.assertEqual(list(output.iterdir()), [sentinel])

    @unittest.skipUnless(os.environ.get("EVO_BASELINE_NODE"), "Optional previous-build comparison")
    def test_valid_outputs_match_previous_build(self):
        self.write_bags()
        result, current = self.convert()
        self.assertEqual(result.returncode, 0, result.stdout)
        result, baseline = self.convert("baseline", executable=os.environ["EVO_BASELINE_NODE"])
        self.assertEqual(result.returncode, 0, result.stdout)
        self.assertEqual(sorted(p.name for p in current.iterdir()), sorted(p.name for p in baseline.iterdir()))
        for path in current.glob("*.tum"):
            self.assertEqual(path.read_bytes(), (baseline / path.name).read_bytes(), path.name)

        def bag_contents(directory):
            messages = []
            with rosbag.Bag(str(directory / "sample_evaluation_pose_collections.bag")) as bag:
                for topic, message, stamp in bag.read_messages():
                    serialized = io.BytesIO()
                    message.serialize(serialized)
                    messages.append((topic, stamp.to_nsec(), serialized.getvalue()))
            return messages

        self.assertEqual(bag_contents(current), bag_contents(baseline))


if __name__ == "__main__":
    unittest.main()
