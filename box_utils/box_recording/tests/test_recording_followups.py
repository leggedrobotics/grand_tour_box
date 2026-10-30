import importlib.util
from pathlib import Path
import os
import sys
import tempfile
import types
import unittest
from unittest.mock import Mock, patch

ROOT = Path(__file__).resolve().parents[1] / "src/box_recording/box_recording_helper"


def load(name):
    spec = importlib.util.spec_from_file_location(name, ROOT / f"{name}.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class RecordingTests(unittest.TestCase):
    def test_selected_directory_and_failure(self):
        module = load("recording_directory")
        with tempfile.TemporaryDirectory() as temporary:
            selected = Path(temporary) / "disk/session"
            self.assertEqual(module.prepare_recording_directory(selected), str(selected))
            self.assertTrue(selected.is_dir())
            file = Path(temporary) / "file"
            file.write_text("preserve")
            with self.assertRaises(OSError):
                module.prepare_recording_directory(file / "child")
            self.assertEqual(file.read_text(), "preserve")

    def test_current_ros_logs_and_time_window(self):
        with patch.dict(sys.modules, {"rospy": Mock()}):
            module = load("store_debug_logs_to_folder")
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "node.log"
            path.write_text("log")
            os.utime(path, (15, 15))
            time = lambda value: types.SimpleNamespace(to_sec=lambda: value)
            with patch.dict(os.environ, {"ROS_LOG_DIR": temporary}):
                self.assertEqual(module.store_debug_logs_to_folder(time(10), time(20)), [str(path)])
                self.assertEqual(module.store_debug_logs_to_folder(time(16), time(20)), [])

    def cpt7(self):
        with patch.dict(sys.modules, {"rospy": Mock(get_param=Mock(return_value=True))}):
            return load("cpt7_helper")

    def test_required_failure_does_not_suppress_later_logs(self):
        module = self.cpt7()
        module.call_service = Mock(side_effect=lambda cmd, **kw: cmd != module.SIGNAL_TO_RECORD[0])
        self.assertFalse(module.cpt7_start_recording())
        self.assertEqual([call.args[0] for call in module.call_service.call_args_list],
                         module.STOP_RECORDING + module.START_RECORDING + module.SIGNAL_TO_RECORD + module.DIAGNOSTIC_SIGNALS)

    def test_optional_failures_preserve_required_success(self):
        module = self.cpt7()
        module.call_service = Mock(side_effect=lambda cmd, **kw: cmd not in module.DIAGNOSTIC_SIGNALS)
        self.assertTrue(module.cpt7_start_recording())
        self.assertEqual(module.rospy.logwarn.call_count, 10)

    def test_open_failure_stops_setup(self):
        module = self.cpt7()
        module.call_service = Mock(side_effect=[True, False])
        self.assertFalse(module.cpt7_start_recording())
        self.assertEqual(module.call_service.call_count, 2)

    def test_close_rejection_still_attempts_open(self):
        module = self.cpt7()
        module.call_service = Mock(side_effect=lambda cmd, **kw: cmd != module.STOP_RECORDING[0])
        self.assertTrue(module.cpt7_start_recording())
        self.assertEqual(module.call_service.call_args_list[1].args[0], module.START_RECORDING[0])

    def test_diagnostics_can_be_disabled(self):
        module = self.cpt7()
        module.rospy.get_param.return_value = False
        module.call_service = Mock(return_value=True)
        self.assertTrue(module.cpt7_start_recording())
        self.assertEqual(module.call_service.call_count, 2 + len(module.SIGNAL_TO_RECORD))


if __name__ == "__main__":
    unittest.main()
