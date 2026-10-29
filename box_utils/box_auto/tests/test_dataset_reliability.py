"""Local regression fixtures; run with the ROS 1 Python environment.

python3 -m unittest discover -s box_utils/box_auto/tests -v
Cloud uploads, downloads and ROS launch commands are stubbed; bag I/O is real.
"""
import argparse
import ast
import contextlib
import importlib.util
import io
from pathlib import Path
import sys
import tempfile
import types
import unittest
import re
import shlex
import subprocess
import typing

import numpy as np
import tqdm
import yaml
from unittest.mock import Mock, patch

import rosbag
import rospy
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import CompressedImage, CameraInfo
from std_msgs.msg import String

PACKAGE = Path(__file__).resolve().parents[1]
SCRIPTS = PACKAGE / 'python/box_auto/scripts'
sys.dont_write_bytecode = True


def load_script(relative):
    utilities = types.ModuleType('box_auto.utils')
    utilities.__path__ = [str(PACKAGE / 'python/box_auto/utils')]
    for name in ('upload_bag', 'get_bag', 'run_ros_command', 'kill_roscore'):
        setattr(utilities, name, Mock())
    utilities.MISSION_DATA = '/unused/test/mission'
    utilities.WS = '/unused/test/workspace'
    parent = types.ModuleType('box_auto')
    parent.__path__ = [str(PACKAGE / 'python/box_auto')]
    spec = importlib.util.spec_from_file_location('reliability_' + Path(relative).stem, SCRIPTS / relative)
    module = importlib.util.module_from_spec(spec)
    overrides = {'box_auto': parent, 'box_auto.utils': utilities, spec.name: module}
    previous = {name: sys.modules.get(name) for name in overrides}
    sys.modules.update(overrides)
    try:
        with contextlib.redirect_stdout(io.StringIO()):
            spec.loader.exec_module(module)
    finally:
        for name, old in previous.items():
            if old is None:
                sys.modules.pop(name, None)
            else:
                sys.modules[name] = old
    return module


merge = load_script('general/merge_mission.py')
ap20 = load_script('ap20/ap20_validate.py')
hdr = load_script('camera/combined_hdr_camera_processing.py')
lidar = load_script('lidar/filter_pointclouds.py')
calibration = load_script('calibration/update_calibration.py')


def string_bag(path, messages=3):
    with rosbag.Bag(str(path), 'w') as bag:
        for i in range(messages):
            bag.write('/message', String(data=str(i)), rospy.Time(1700000000 + i))


class TemporaryCase(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.directory = Path(self.temp.name)
        self.output = self.directory / 'output.bag'


class MergeTests(TemporaryCase):
    def test_same_path_moves_and_outputs_preserve_file(self):
        self.output.write_bytes(b'original')
        self.assertFalse(merge.move_file(self.output, self.output, True, False))
        self.assertFalse(merge.create_output_from_single_bag(self.output, self.output, True, False))
        self.assertEqual(self.output.read_bytes(), b'original')

    def test_cleanup_preserves_conflicts_and_same_prefix(self):
        for overwrite in (False, True):
            with self.subTest(overwrite=overwrite):
                old = self.directory / 'mission_nuc_livox.bag'
                new = self.directory / '2025-01-01-00-00-00_nuc_livox.bag'
                old.write_bytes(b'original')
                new.write_bytes(b'different output')
                args = (self.directory, self.directory/'ORIGINAL_FILES')
                merge.cleanup_old_named_merged_bags(*args, 'mission', '2025-01-01-00-00-00', overwrite, 'No', False)
                merge.cleanup_old_named_merged_bags(*args, '2025-01-01-00-00-00', '2025-01-01-00-00-00', overwrite, 'No', False)
                self.assertEqual(old.read_bytes(), b'original')
                self.assertEqual(new.read_bytes(), b'different output')

    def test_corrupt_merge_preserves_existing_output(self):
        good, bad = self.directory/'good.bag', self.directory/'bad.bag'
        string_bag(good)
        bad.write_bytes(b'not a bag')
        self.output.write_bytes(b'previous complete output')
        with self.assertRaises(RuntimeError):
            merge.merge_bags_single([str(good), str(bad)], str(self.output))
        self.assertEqual(self.output.read_bytes(), b'previous complete output')
        self.assertTrue(good.exists())
        self.assertTrue(bad.exists())
        self.assertFalse(list(self.directory.glob('*.tmp')))

    def test_failed_copy_preserves_source_and_destination(self):
        source = self.directory/'source.bag'
        source.write_bytes(b'input')
        self.output.write_bytes(b'old output')
        with patch.object(merge.os, 'link', side_effect=OSError('links unavailable')), patch.object(merge.shutil, 'copy2', side_effect=OSError('disk full')):
            with self.assertRaises(OSError):
                merge.create_output_from_single_bag(source, self.output, True, False)
        self.assertEqual(source.read_bytes(), b'input')
        self.assertEqual(self.output.read_bytes(), b'old output')

    def test_atomic_no_overwrite_race(self):
        with self.assertRaises(FileExistsError):
            with merge.atomic_output(self.output, overwrite=False) as temporary:
                temporary.write_bytes(b'ours')
                self.output.write_bytes(b'other writer')
        self.assertEqual(self.output.read_bytes(), b'other writer')

    def test_main_merges_and_reruns_without_originals_in_root(self):
        for i in range(2):
            string_bag(self.directory/f'mission_nuc_livox_{i}.bag')
        with patch.object(merge, 'MISSION_DATA', str(self.directory)), patch.object(sys, 'argv', ['merge_mission']):
            merge.main()
            output = self.directory/'2023-11-14-22-13-20_nuc_livox.bag'
            with rosbag.Bag(str(output)) as bag:
                self.assertEqual(bag.get_message_count(), 6)
            original_bytes = output.read_bytes()
            merge.main()
            self.assertEqual(output.read_bytes(), original_bytes)
            self.assertEqual(len(list((self.directory/'ORIGINAL_FILES').glob('*.bag'))), 2)

    def test_main_does_not_archive_after_failed_merge(self):
        good = self.directory/'mission_nuc_livox_0.bag'
        bad = self.directory/'mission_nuc_livox_1.bag'
        string_bag(good)
        bad.write_bytes(b'corrupt')
        with patch.object(merge, 'MISSION_DATA', str(self.directory)), patch.object(sys, 'argv', ['merge_mission']):
            with self.assertRaises(RuntimeError):
                merge.main()
        self.assertTrue(good.exists())
        self.assertTrue(bad.exists())
        self.assertFalse(list((self.directory/'ORIGINAL_FILES').glob('*.bag')))


class Ap20Tests(TemporaryCase):
    def run_fixture(self, xs, dt=0.05, remove_start=False):
        source = self.directory/'prism.bag'
        with rosbag.Bag(str(source), 'w') as bag:
            for i, x in enumerate(xs):
                msg = PointStamped()
                msg.header.stamp = rospy.Time.from_sec(1700000000 + dt*i)
                msg.point.x = x
                bag.write(ap20.TOPIC_PRISM_POSITION, msg, msg.header.stamp)
        with patch.object(ap20.plt, 'savefig'), patch.object(ap20, 'ENABLE_POSTWRITE_DELTA_PLOT', False), patch.object(ap20, 'ENABLE_POSTWRITE_DT_HDR_PLOT', False), patch.object(ap20, 'ENABLE_POSTWRITE_DT_AND_X_STACKED_PLOT', False):
            return ap20.process_prism_position_bag(str(source), str(self.output), False, remove_start, False)[0]

    def test_zero_peaks_keeps_positions(self):
        self.assertEqual(self.run_fixture([0.0]*120), 0)
        with rosbag.Bag(str(self.output)) as bag:
            self.assertEqual(bag.get_message_count(), 120)

    def test_one_peak_keeps_positions(self):
        xs = [0.0 if i < 30 else max(0.0, 4-(i-30)/15) for i in range(120)]
        self.assertEqual(self.run_fixture(xs), 0)
        with rosbag.Bag(str(self.output)) as bag:
            self.assertEqual(bag.get_message_count(), 120)

    def test_two_peaks_retains_bounded_interval(self):
        self.assertEqual(self.run_fixture([0.0]*30+[4.0]*60+[0.0]*30), 0)
        with rosbag.Bag(str(self.output)) as bag:
            self.assertEqual(bag.get_message_count(), 60)

    def test_empty_filters_preserve_previous_output(self):
        self.output.write_bytes(b'previous result')
        self.assertEqual(self.run_fixture([0.0]*120, dt=0.3), 5)
        self.assertEqual(self.output.read_bytes(), b'previous result')

    def test_remove_start_cannot_report_empty_success(self):
        self.output.write_bytes(b'previous result')
        self.assertEqual(self.run_fixture([0.0]*120, remove_start=True), 5)
        self.assertEqual(self.output.read_bytes(), b'previous result')


class PipelineTests(TemporaryCase):
    def test_hdr_headerless_passthrough_and_encoding(self):
        source = self.directory/'input.bag'
        with rosbag.Bag(str(source), 'w') as bag:
            image = CompressedImage()
            image.header.stamp = rospy.Time(100)
            image.format = 'jpeg'
            bag.write('/gt_box/hdr_front/image_raw/compressed', image, rospy.Time(101))
            bag.write('/metadata', String(data='retained'), rospy.Time(102))
        processor = hdr.RosbagValidatorAndProcessor([], '_encoding')
        processor.encode_only_bag(str(source), str(self.output), 'hdr_front')
        with rosbag.Bag(str(self.output)) as bag:
            messages = list(bag.read_messages())
        self.assertEqual(len(messages), 2)
        self.assertEqual(messages[0].message.format, 'jpg')
        self.assertEqual(messages[0].timestamp.to_sec(), 100)
        self.assertEqual(messages[1].message.data, 'retained')
        self.assertEqual(messages[1].timestamp.to_sec(), 102)
        self.assertTrue(source.exists())

    def test_hdr_missing_images_preserves_previous_output(self):
        source = self.directory/'input.bag'
        string_bag(source)
        self.output.write_bytes(b'previous result')
        processor = hdr.RosbagValidatorAndProcessor([], '_encoding')
        with self.assertRaises(ValueError):
            processor.encode_only_bag(str(source), str(self.output), 'hdr_front')
        self.assertEqual(self.output.read_bytes(), b'previous result')

    def test_hdr_validation_failure_propagates(self):
        processor = hdr.RosbagValidatorAndProcessor(['*hdr_front.bag'], '_encoding')
        with patch.object(hdr, 'get_bag', return_value=str(self.directory/'input.bag')), patch.object(processor, 'validate_and_process_bag', return_value=False), patch.object(hdr, 'upload_bag') as upload:
            with self.assertRaises(RuntimeError):
                processor.run()
            upload.assert_not_called()

    def test_explicit_lidar_selection_uses_download_lookup(self):
        with patch.object(lidar, 'get_bag', return_value='/mission/npc_velodyne_processed.bag') as get:
            self.assertEqual(len(lidar.resolve_inputs(['velodyne'])), 1)
            get.assert_called_once_with('*npc_velodyne_processed.bag', auto_download=True, return_upon_no_files=False)

    def test_discovery_uses_local_inputs_and_rejects_empty(self):
        with patch.object(lidar, 'get_bag', side_effect=[None, 'hesai.bag', None]):
            self.assertEqual(lidar.resolve_inputs(), [('hesai', 'hesai.bag')])
        with patch.object(lidar, 'get_bag', return_value=None):
            with self.assertRaises(FileNotFoundError):
                lidar.resolve_inputs()

    def test_calibration_requires_every_input(self):
        (self.directory/'left.bag').touch()
        self.assertIsNone(calibration.check_expected_bags_present(['left.bag', 'right.bag'], self.directory))
        (self.directory/'right.bag').touch()
        self.assertEqual(len(calibration.check_expected_bags_present(['left.bag', 'right.bag'], self.directory)), 2)


class ConverterCliTests(TemporaryCase):
    def cli(self, name, convert_name, pattern):
        tree = ast.parse((SCRIPTS/'special'/name).read_text())
        main = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == 'main')
        convert = Mock()
        ns = dict(argparse=argparse, Path=Path, sys=sys, BATCH_GLOB=pattern, **{convert_name:convert})
        exec(compile(ast.Module(body=[main], type_ignores=[]), name, 'exec'), ns)
        return ns['main'], convert

    def test_explicit_files_and_batch_failure_status_both_directions(self):
        for name, flag, func, extension in [
            ('generizableROS2conversion.py', '--inbag', 'convert_single_bag', '.bag'),
            ('generizableROS2toROS1conversion.py', '--inmcap', 'convert_single_mcap', '.mcap'),
        ]:
            with self.subTest(name=name):
                main, convert = self.cli(name, func, '*'+extension)
                first = self.directory/('first'+extension)
                second = self.directory/('second'+extension)
                first.touch(); second.touch()
                self.assertEqual(main([flag, str(first)]), 0)
                self.assertEqual(convert.call_count, 1)
                self.assertEqual(convert.call_args[0][0], first)
                convert.reset_mock();convert.side_effect=[None, RuntimeError('invalid bag')]
                self.assertEqual(main([flag, str(self.directory)]), 1)
                self.assertEqual(convert.call_count, 2)


def extracted_function(path, name, namespace):
    tree = ast.parse(path.read_text())
    function = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == name)
    future = ast.ImportFrom(module='__future__', names=[ast.alias(name='annotations')], level=0)
    module = ast.fix_missing_locations(ast.Module(body=[future, function], type_ignores=[]))
    exec(compile(module, str(path), 'exec'), namespace)
    return namespace[name]


class AdditionalReliabilityTests(TemporaryCase):
    def test_optional_mcap_marker_and_ambiguity(self):
        parse = extracted_function(SCRIPTS/'general/mcap_to_rosbag.py', 'find_original_txt_and_parse', dict(Path=Path, re=re))
        self.assertIsNone(parse(self.directory))
        (self.directory/'2025-01-01-00-00-00__original__mission.yml').touch()
        self.assertEqual(parse(self.directory)[1:3], ('2025-01-01-00-00-00', 'mission'))
        (self.directory/'2025-01-02-00-00-00__original__other.yml').touch()
        with self.assertRaises(ValueError):
            parse(self.directory)

    def test_anonymizers_reject_missing_or_unprocessed_images(self):
        root = PACKAGE.parent/'box_converter/image-anonymization-pipeline'
        for filename in ('anonimization_standalone.py', 'anonimization_standalone_chat.py'):
            with self.subTest(filename=filename):
                function = extracted_function(root/filename, 'anonymize_image_topics_in_bagfile', dict(Path=Path, rosbag=rosbag, tqdm=tqdm))
                source = self.directory/'source.bag'
                string_bag(source)
                with self.assertRaisesRegex(ValueError, 'None of the selected'):
                    function(source, self.output, image_topics=['/images'], detectors=None, visualization_state=None)
                self.assertFalse(self.output.exists())
                with rosbag.Bag(str(source), 'w') as bag:
                    bag.write('/metadata', String(data='first'), rospy.Time(1))
                    bag.write('/images', CompressedImage(), rospy.Time(2))
                with self.assertRaisesRegex(ValueError, 'No images were processed'):
                    function(source, self.output, image_topics=['/images'], detectors=None, visualization_state=None, head=1)
                self.assertFalse(self.output.exists())
                self.assertFalse(list(self.directory.glob('.anonymizing-*')))

    def test_calibration_updates_preserve_headers_and_unrelated_messages(self):
        source = self.directory/'mission_nuc_alphasense_color.bag'
        info_topic = '/gt_box/alphasense_driver_node/cam1/color/camera_info'
        with rosbag.Bag(str(source), 'w') as bag:
            info = CameraInfo()
            info.header.stamp = rospy.Time(100)
            info.header.frame_id = 'camera'
            bag.write(info_topic, info, rospy.Time(101))
            bag.write('/metadata', String(data='retained'), rospy.Time(102))
        new_info = CameraInfo()
        new_info.K = [100., 0., 50., 0., 100., 50., 0., 0., 1.]
        pattern = '*_nuc_alphasense_color.bag'
        with patch.object(calibration, 'CAMERA_INFO_PATTERNS', [pattern]):
            calibration.update_camera_info_inplace({'/gt_box/alphasense_driver_node/cam1': new_info}, {pattern: str(source)})
        with rosbag.Bag(str(source)) as bag:
            messages = list(bag.read_messages())
        self.assertEqual(len(messages), 2)
        self.assertEqual(messages[0].message.K[0], 100.)
        self.assertEqual(messages[0].message.header.stamp.to_sec(), 100.)
        self.assertEqual(messages[0].timestamp.to_sec(), 101.)
        self.assertEqual(messages[1].message.data, 'retained')
        with rosbag.Bag(str(self.directory/'mission_nuc_alphasense_calib.bag')) as bag:
            self.assertEqual(bag.get_message_count(), 1)

    def test_failed_calibration_preserves_input_and_sidecar(self):
        source = self.directory/'mission_nuc_alphasense_color.bag'
        with rosbag.Bag(str(source), 'w') as bag:
            bag.write('/unknown/camera_info', CameraInfo(), rospy.Time(1))
        original = source.read_bytes()
        sidecar = self.directory/'mission_nuc_alphasense_calib.bag'
        sidecar.write_bytes(b'previous calibration')
        pattern = '*_nuc_alphasense_color.bag'
        with patch.object(calibration, 'CAMERA_INFO_PATTERNS', [pattern]):
            with self.assertRaises(ValueError):
                calibration.update_camera_info_inplace({}, {pattern: str(source)})
        self.assertEqual(source.read_bytes(), original)
        self.assertEqual(sidecar.read_bytes(), b'previous calibration')

    def test_standalone_imu_conversion_does_not_require_livox(self):
        lookup = Mock(side_effect=AssertionError('Livox lookup must be skipped'))
        parser = Mock()
        namespace = dict(argparse=argparse, Path=Path, MISSION_DATA=str(self.directory),
                         configure_logger=lambda **kw: Mock(), find_livox_bag=lookup,
                         find_single_rawimusx_ascii=lambda root: root/'RAWIMUSX.ASCII',
                         RAWIMUDataParser=Mock(return_value=parser), CPT7_TOPIC_DEFAULT='/imu')
        main = extracted_function(SCRIPTS/'cpt7/export_raw_imu_bag.py', 'main', namespace)
        with patch.object(sys, 'argv', ['imu', '--skip_verify', '--output_bag', str(self.output)]):
            with self.assertRaises(SystemExit) as exit_code:
                main()
        self.assertEqual(exit_code.exception.code, 0)
        lookup.assert_not_called()
        parser.write_to_rosbag.assert_called_once_with(str(self.output))

    def test_evaluation_records_short_and_unassociated_runs(self):
        self.addCleanup(ap20.matplotlib.rcParams.update, ap20.matplotlib.rcParams.copy())
        evaluation = load_script('verification/grandtour_point_relation.py')
        from evo.tools import file_interface
        reference = self.directory/'reference.tum'
        estimate = self.directory/'estimate.tum'
        reference.write_text(''.join(f'{i/10} {i/10} 0 0 0 0 0 1\n' for i in range(150)))
        estimate.write_text(''.join(f'{i/10} {i/10} 0 0 0 0 0 1\n' for i in range(50)))
        result_path = self.directory/'ate_short_align-800_results.zip'
        result_path.write_bytes(b'stale result')
        evaluation.run_eval('short', str(reference), str(estimate), {}, str(self.directory), {}, 'ate', True)
        result = file_interface.load_res_file(str(result_path))
        self.assertEqual(result.info['status'], 'failed')
        self.assertIn('50', result.info['reason'])
        self.assertTrue(np.isnan(result.stats['mean']))
        estimate.write_text(''.join(f'{100+i/10} {i/10} 0 0 0 0 0 1\n' for i in range(150)))
        evaluation.run_eval('unassociated', str(reference), str(estimate), {}, str(self.directory), {}, 'ate', True)
        result = file_interface.load_res_file(str(self.directory/'ate_unassociated_align-800_results.zip'))
        self.assertEqual(result.info['status'], 'failed')
        self.assertIn('association', result.info['reason'])

    def test_unit_precedence_and_failed_stage(self):
        source = SCRIPTS/'verification/evo_evaluation.py'
        parse = extracted_function(source, 'parse_unit_to_scale', {})
        self.assertEqual(parse(None, 'cm'), ('cm', 100.0))
        self.assertEqual(parse('m', 'cm'), ('m', 1.0))
        checked = extracted_function(source, 'run_required_ros_command', dict(run_ros_command=lambda *a, **k: 7))
        with self.assertRaises(RuntimeError):
            checked('failed stage')

    def test_lidar_failure_does_not_upload(self):
        with patch.object(lidar, 'get_bag', return_value='/mission/npc_velodyne_processed.bag'), patch.object(lidar, 'run_ros_command', return_value=7), patch.object(lidar, 'upload_bag') as upload:
            with self.assertRaises(RuntimeError):
                lidar.main(['--lidars=velodyne'])
            upload.assert_not_called()

    def test_ros_command_quoting_and_failure_status(self):
        run = extracted_function(PACKAGE/'python/box_auto/utils/utils.py', 'run_ros_command', dict(subprocess=subprocess, PRE=''))
        target = self.directory/'path with spaces.txt'
        value = "literal $(not_a_command) and 'quotes'"
        self.assertEqual(run('printf %s '+shlex.quote(value)+' > '+shlex.quote(str(target))), 0)
        self.assertEqual(target.read_text(), value)
        self.assertEqual(run('exit 7'), 7)

    def test_compose_paths_and_dependencies_resolve(self):
        root = PACKAGE.parents[1]
        services = yaml.safe_load((PACKAGE/'compose/kleinkram.yaml').read_text())['services']
        for name, config in services.items():
            for dependency in config.get('depends_on', {}):
                self.assertIn(dependency, services)
            for command in config.get('command', []):
                for relative in re.findall(r'/home/catkin_ws/src/grand_tour_box/([^ "]+\.py)', command):
                    self.assertTrue((root/relative).is_file(), (name, relative))
        self.assertIn('color_correction', services['calibration']['depends_on'])
        self.assertIn('color_correction', services['pointcloud_undistortion']['depends_on'])
        def visit(name, ancestors):
            self.assertNotIn(name, ancestors)
            for child in services[name].get('depends_on', {}):
                visit(child, ancestors | {name})
        for name in services:
            visit(name, set())


if __name__ == '__main__':
    unittest.main()
