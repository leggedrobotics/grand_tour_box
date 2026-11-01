"""Recorder boundary contracts; no ROS master, devices, subprocesses or datasets."""

import ast
import importlib.util
from pathlib import Path
import signal
import threading
import types
import unittest
from unittest.mock import Mock
import xml.etree.ElementTree as ET

import yaml

ROOT = Path(__file__).resolve().parents[1]
REPO = ROOT.parents[1]


def definitions(path, names, namespace):
    tree = ast.parse(path.read_text())
    nodes = [n for n in tree.body if isinstance(n, (ast.ClassDef, ast.FunctionDef)) and n.name in names]
    module = ast.Module(
        body=[ast.ImportFrom(module="__future__", names=[ast.alias(name="annotations")], level=0), *nodes],
        type_ignores=[],
    )
    exec(compile(ast.fix_missing_locations(module), str(path), "exec"), namespace)
    return namespace


def response():
    return types.SimpleNamespace(suc=True, message="", result="")


class RecorderContracts(unittest.TestCase):
    def test_real_hostname_is_safe_for_ros_without_changing_deployed_names(self):
        path = ROOT / "src/box_recording/box_recording_helper/recording_names.py"
        spec = importlib.util.spec_from_file_location("recording_names", path)
        names = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(names)
        self.assertEqual(
            names.recording_node_name("tutuna-MS-7D25", "zed_sensors"), "record_tutuna_MS_7D25_zed_sensors"
        )
        self.assertEqual(names.recording_node_name("tutuna-MS-7D25"), "rosbag_record_node_tutuna_MS_7D25")
        self.assertEqual(names.recording_node_name("jetson", "zed_sensors"), "record_jetson_zed_sensors")

    def setUp(self):
        self.ros = Mock()
        self.ros.ROSException = type("ROSException", (Exception,), {})
        self.ros.ServiceException = type("ServiceException", (Exception,), {})
        ns = {
            "rospy": self.ros,
            "StartRecordingSVO": object,
            "StartRecordingSVORequest": types.SimpleNamespace,
            "StartRecordingInternalResponse": response,
            "StopRecordingInternalResponse": response,
            "Trigger": object,
            "signal": signal,
        }
        definitions(ROOT / "src/box_recording/rosbag_record_node.py", {"RosbagRecordNode"}, ns)
        self.node = ns["RosbagRecordNode"].__new__(ns["RosbagRecordNode"])
        self.node.__dict__.update(
            namespace="/gt_box/",
            node="jetson",
            bag_base_path="/unused",
            recording_zed=False,
            zed_recording_error="",
            bag_running=False,
            _recording_lock=threading.RLock(),
        )
        self.ros.ServiceProxy.return_value = Mock(return_value=types.SimpleNamespace(success=True))

    def coordinator(self):
        path = ROOT / "src/box_recording/box_recording_helper/recording_names.py"
        spec = importlib.util.spec_from_file_location("recording_names", path)
        names = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(names)
        ns = {
            "rospy": self.ros,
            "StartRecordingInternal": object,
            "StartRecordingInternalRequest": types.SimpleNamespace,
            "StopRecordingInternal": object,
            "StopRecordingInternalRequest": types.SimpleNamespace,
            "StopRecordingResponse": response,
            "recording_node_name": names.recording_node_name,
        }
        definitions(ROOT / "src/box_recording/rosbag_record_coordinator.py", {"RosbagRecordCoordinator"}, ns)
        coord = ns["RosbagRecordCoordinator"].__new__(ns["RosbagRecordCoordinator"])
        coord.namespace = "/gt_box/"
        coord.cfg = {"jetson": {"bags": {"zed2i": ["svo"]}}}
        coord.timestamp = "test"
        return coord

    def test_coordinator_uses_normalized_recorder_service_names(self):
        coord = self.coordinator()
        coord.cfg = {"desktop-host": {"bags": {"radar": ["/radar"]}}}
        self.ros.ServiceProxy.return_value.return_value = response()
        coord.send_request_ros("desktop-host", "radar----/radar", "test", response())
        coord.stop_recording(types.SimpleNamespace(verbose=False))
        requested = [call.args[0] for call in self.ros.ServiceProxy.call_args_list]
        self.assertEqual(
            requested,
            [
                "/gt_box/rosbag_record_node_desktop_host/start_recording",
                "/gt_box/rosbag_record_node_desktop_host/stop_recording",
            ],
        )

    def test_rejected_svo_start_is_not_reported_as_running(self):
        self.ros.ServiceProxy.return_value.return_value.success = False
        result = self.node.toggle_zed_recording(True, "test", response())
        self.assertFalse(result.suc)
        self.assertFalse(self.node.recording_zed)
        self.assertIn("FAILED", result.message)
        self.assertTrue(self.node.zed_recording_error)

    def test_rejected_stop_preserves_state_for_retry(self):
        self.node.recording_zed = True
        self.ros.ServiceProxy.side_effect = lambda name, kind: (
            (lambda: types.SimpleNamespace(success=True, message="Recording"))
            if name.endswith("get_recording_status")
            else (lambda request: types.SimpleNamespace(success=False))
        )
        self.assertFalse(self.node.toggle_zed_recording(False, "", response()).suc)
        self.assertTrue(self.node.recording_zed)

    def test_failed_capture_at_stop_reports_failure_but_releases_closed_recording(self):
        self.node.recording_zed = True
        self.ros.ServiceProxy.side_effect = lambda name, kind: (
            (lambda: types.SimpleNamespace(success=False, message="SDK write failed"))
            if name.endswith("get_recording_status")
            else (lambda request: types.SimpleNamespace(success=False))
        )
        self.assertFalse(self.node.toggle_zed_recording(False, "", response()).suc)
        self.assertFalse(self.node.recording_zed)
        self.assertEqual(self.node.zed_recording_error, "SDK write failed")

    def test_successful_new_start_clears_previous_failure(self):
        self.node.zed_recording_error = "previous failure"
        self.assertTrue(self.node.toggle_zed_recording(True, "test", response()).suc)
        self.assertTrue(self.node.recording_zed)
        self.assertEqual(self.node.zed_recording_error, "")

    def test_svo_rejection_reaches_coordinator_response(self):
        def service(name, kind):
            if name.endswith("start_recording_svo"):
                return lambda request: types.SimpleNamespace(success=False)
            return lambda request: self.node.toggle_zed_recording(True, request.timestamp, response())

        self.ros.ServiceProxy.side_effect = service
        out = response()
        self.coordinator().send_request_ros("jetson", "zed2i----svo", "test", out)
        self.assertFalse(out.suc)
        self.assertIn("jetson-[FAILED]", out.message)

    def test_later_node_success_does_not_clear_earlier_failure(self):
        self.ros.ServiceProxy.return_value.return_value = response()
        out = response()
        out.suc = False
        self.coordinator().send_request_ros("nuc", "radar----/ti_mmwave/radar_scan_pcl_0", "test", out)
        self.assertFalse(out.suc)

    def test_stop_failure_reaches_coordinator(self):
        out = response()
        out.suc = False
        self.ros.ServiceProxy.return_value.return_value = out
        result = self.coordinator().stop_recording(types.SimpleNamespace(verbose=False))
        self.assertFalse(result.suc)
        self.assertNotIn("boxi get_data", result.message)

    def test_mid_recording_failure_is_latched(self):
        self.node.recording_zed = True
        self.ros.ServiceProxy.return_value.return_value = types.SimpleNamespace(
            success=False, message="SVO write failed"
        )
        self.node.check_zed_recording()
        self.assertFalse(self.node.recording_zed)
        self.assertEqual(self.node.zed_recording_error, "SVO write failed")

    def test_missing_status_service_does_not_claim_success(self):
        self.node.recording_zed = True
        self.ros.wait_for_service.side_effect = self.ros.ROSException("unreachable")
        self.node.check_zed_recording()
        self.assertTrue(self.node.recording_zed)  # Retain stop/retry capability.
        self.assertIn("Cannot verify", self.node.zed_recording_error)

    def test_double_start_is_rejected_before_any_output_changes(self):
        self.node.bag_running = True
        result = self.node.start_recording(types.SimpleNamespace())
        self.assertFalse(result.suc)
        self.assertEqual(self.node.bag_base_path, "/unused")
        self.ros.ServiceProxy.assert_not_called()

    def test_writer_failure_is_returned_after_waiting_for_exit(self):
        process = Mock()
        process.wait.return_value = 1
        self.assertEqual(self.node.stop_bag_process(process), 1)
        self.assertEqual(process.mock_calls, [unittest.mock.call.send_signal(signal.SIGINT), unittest.mock.call.wait()])

    def test_grpc_stop_waits_for_all_writers_and_reports_failure(self):
        class Abort(Exception):
            pass

        ns = {
            "rospy": self.ros,
            "signal": signal,
            "grpc": types.SimpleNamespace(StatusCode=types.SimpleNamespace(INTERNAL="internal")),
            "stop_recording_pb2_grpc": types.SimpleNamespace(StopRecordingServicer=object),
        }
        definitions(ROOT / "src/box_recording/rosbag_record_node_grpc.py", {"StopRecordingServicer"}, ns)
        writers = [Mock(), Mock()]
        writers[0].wait.return_value = 1
        writers[1].wait.return_value = 0
        node = types.SimpleNamespace(
            recording_lock=threading.RLock(), store_debug_logs=False, processes=writers, bag_running=True
        )
        context = types.SimpleNamespace(abort=Mock(side_effect=Abort))
        with self.assertRaises(Abort):
            ns["StopRecordingServicer"](node).SendMessage(types.SimpleNamespace(), context)
        for writer in writers:
            writer.send_signal.assert_called_once_with(signal.SIGINT)
            writer.wait.assert_called_once_with()
        self.assertFalse(node.bag_running)
        self.assertEqual(node.processes, [])
        context.abort.assert_called_once()

    def test_grpc_double_start_uses_transport_failure(self):
        class Abort(Exception):
            pass

        ns = {
            "rospy": self.ros,
            "grpc": types.SimpleNamespace(StatusCode=types.SimpleNamespace(FAILED_PRECONDITION="failed")),
            "start_recording_pb2_grpc": types.SimpleNamespace(StartRecordingServicer=object),
        }
        definitions(ROOT / "src/box_recording/rosbag_record_node_grpc.py", {"StartRecordingServicer"}, ns)
        context = types.SimpleNamespace(abort=Mock(side_effect=Abort))
        obj = ns["StartRecordingServicer"](types.SimpleNamespace(bag_running=True, recording_lock=threading.RLock()))
        with self.assertRaises(Abort):
            obj.SendMessage(types.SimpleNamespace(), context)
        context.abort.assert_called_once_with("failed", "Recording already running")


class ProfileContracts(unittest.TestCase):
    def profile(self, name):
        return yaml.safe_load((ROOT / "cfg" / (name + ".yaml")).read_text())

    def assert_superset(self, expanded, base):
        for host, config in base.items():
            self.assertEqual(expanded[host].get("grpc_cfg"), config.get("grpc_cfg"))
            for bag, topics in config["bags"].items():
                self.assertLessEqual(set(topics), set(expanded[host]["bags"][bag]))

    def test_diagnostics_preserve_every_existing_robot_topic_and_endpoint(self):
        robot, diagnostics = self.profile("robot"), self.profile("robot_diagnostics")
        self.assert_superset(diagnostics, robot)
        self.assertIn("/state_estimator/twist_throttle", diagnostics["lpc"]["bags"]["state_estimator"])
        self.assertIn("/state_estimator/twist", diagnostics["lpc"]["bags"]["state_estimator"])
        self.assertIn("/chrony_monitor_npc/status", diagnostics["npc"]["bags"]["general"])

    def test_raw_hesai_packets_have_an_active_producer(self):
        topic = "/gt_box/hesai/packets"
        self.assertIn(topic, self.profile("calib_lidar")["nuc"]["bags"]["hesai"])
        driver = yaml.safe_load((REPO / "box_bringup/bringup_hesai/config/hesaiXT32_config.yaml").read_text())
        ros = driver["lidar"][0]["ros"]
        self.assertTrue(ros["send_packet_ros"])
        self.assertEqual(ros["ros_send_packet_topic"], topic)

    def test_robot_calibration_is_an_explicit_extension(self):
        self.assert_superset(self.profile("calib_lidar_robot"), self.profile("calib_lidar"))
        self.assertNotIn("npc", self.profile("calib_lidar"))
        self.assertIn("/lidar/point_cloud", self.profile("calib_lidar_robot")["npc"]["bags"]["velodyne"])

    def test_radar_profile_preserves_default_capture(self):
        self.assert_superset(self.profile("box_radar"), self.profile("box_default"))
        self.assertEqual(self.profile("box_radar")["nuc"]["bags"]["radar"], ["/ti_mmwave/radar_scan_pcl_0"])
        config = ET.parse(REPO / "box_bringup/bringup_fkie/launch/master_sync_opc.launch")
        topics = yaml.safe_load(config.find(".//rosparam[@param='sync_topics']").text)
        self.assertIn("/ti_mmwave/radar_scan_pcl_0", topics)

    def test_lightweight_recorder_and_visualizer_remain_separate(self):
        recording = (REPO / "box_launch/tmux/jetson_recording.yaml").read_text()
        visualization = (REPO / "box_launch/tmux/jetson_visualization.yaml").read_text()
        self.assertIn("bringup_zed2i_recording", recording)
        self.assertNotIn("bringup_zed2i_visualization", recording)
        self.assertIn("bringup_zed2i_visualization", visualization)
        self.assertNotIn("bringup_zed2i_recording", visualization)

    def test_svo_sensor_sidecar_topics_match_driver_publishers(self):
        expected = {"/gt_box/zed2i_recording_driver/" + sensor for sensor in ("imu", "mag", "baro")}
        for name in ("box_default", "robot", "box_radar", "robot_diagnostics"):
            self.assertEqual(set(self.profile(name)["jetson"]["bags"]["zed_sensors"]), expected)

    def test_stim_plots_use_numeric_fields_on_the_same_sensor(self):
        tree = ET.parse(REPO / "box_utils/box_rviz/cfg/rqt_multiplot.xml")
        curves = [node for node in tree.iter() if node.tag.startswith("curve_") and node.findtext("title") == "STIM320"]
        self.assertEqual(len(curves), 2)
        fields = set()
        for curve in curves:
            self.assertEqual(curve.findtext("axes/x_axis/field"), "header/stamp")
            self.assertEqual(curve.findtext("axes/x_axis/topic"), "/gt_box/stim320/imu")
            self.assertEqual(curve.findtext("axes/y_axis/topic"), "/gt_box/stim320/imu")
            fields.add(curve.findtext("axes/y_axis/field"))
        self.assertEqual(fields, {"linear_acceleration/x", "angular_velocity/z"})


if __name__ == "__main__":
    unittest.main()
