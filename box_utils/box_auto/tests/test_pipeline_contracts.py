"""Clock, fusion-input and concurrent-output contracts; no external services."""

import argparse
import contextlib
import ctypes
import errno
import importlib.util
import io
import logging
import math
import multiprocessing
import os
from pathlib import Path
import shlex
import shutil
import types
from unittest.mock import Mock, patch

import rosbag
import rospy
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage

from test_dataset_reliability import TemporaryCase, PACKAGE, SCRIPTS, extracted_function


spec = importlib.util.spec_from_file_location("publication_contract", PACKAGE / "python/box_auto/utils/atomic_file.py")
atomic = importlib.util.module_from_spec(spec)
spec.loader.exec_module(atomic)


def publish_competing_output(destination, payload, barrier, results):
    try:
        with atomic.atomic_output(destination, overwrite=False) as pending:
            pending.write_bytes(payload)
            barrier.wait(timeout=15)
        results.put("published")
    except FileExistsError:
        results.put("exists")
    except Exception as error:
        results.put(repr(error))


class PublicationContracts(TemporaryCase):
    def test_concurrent_publishers_keep_one_complete_output(self):
        for force_fallback in (False, True):
            with self.subTest(force_fallback=force_fallback), contextlib.ExitStack() as stack:
                if force_fallback:
                    libc = types.SimpleNamespace(renameat2=Mock(return_value=-1))
                    stack.enter_context(patch.object(ctypes, "CDLL", return_value=libc))
                    stack.enter_context(patch.object(ctypes, "get_errno", return_value=errno.EINVAL))
                    stack.enter_context(patch.object(os, "link", side_effect=OSError(errno.ENOSYS, "unsupported")))
                context = multiprocessing.get_context("fork")
                barrier, results = context.Barrier(4), context.Queue()
                payloads = [bytes([index]) * 65536 for index in range(4)]
                workers = [
                    context.Process(target=publish_competing_output, args=(self.output, payload, barrier, results))
                    for payload in payloads
                ]
                try:
                    for worker in workers:
                        worker.start()
                    for worker in workers:
                        worker.join(timeout=20)
                        self.assertEqual(worker.exitcode, 0)
                    statuses = [results.get(timeout=2) for _ in workers]
                    self.assertEqual(sorted(statuses), ["exists"] * 3 + ["published"])
                    self.assertIn(self.output.read_bytes(), payloads)
                    self.assertFalse(list(self.directory.glob(".output.bag.*")))
                finally:
                    for worker in workers:
                        if worker.is_alive():
                            worker.terminate()
                            worker.join(timeout=5)
                    results.close()
                self.output.unlink()

    def test_dangling_destination_symlink_is_preserved(self):
        # The removable filesystem may not support symlinks; exercise this
        # contract on the host filesystem when running the rest of the suite there.
        try:
            self.output.symlink_to(self.directory / "missing")
        except OSError as error:
            if error.errno in (errno.EPERM, errno.ENOSYS, errno.EOPNOTSUPP):
                self.skipTest("Filesystem does not support symlinks")
            raise
        with self.assertRaises(FileExistsError):
            with atomic.atomic_output(self.output, overwrite=False) as pending:
                pending.write_bytes(b"complete")
        self.assertTrue(self.output.is_symlink())


class FusionInputContracts(TemporaryCase):
    def test_model_required_for_both_layouts_and_result_names_are_distinct(self):
        model = self.directory / "mission_tf_model.bag"
        transform = TransformStamped()
        transform.header.frame_id, transform.child_frame_id = "base", "box_base"
        transform.transform.rotation.w = 1.0
        with rosbag.Bag(str(model), "w") as bag:
            bag.write("/tf_static", TFMessage([transform]), rospy.Time(1))
        for patterns in (["*_ap20_robot.bag", "*_cpt7_raw_imu.bag"], ["*_ap20_prism_position.bag", "*_cpt7_imu.bag"]):
            files = {pattern: (str(self.directory / pattern[1:]), True) for pattern in patterns}
            files["*_tf_model.bag"] = (str(model), True)
            namespace = dict(
                Path=Path,
                rosbag=rosbag,
                MISSION_DATA=str(self.directory),
                PATTERNS=patterns + ["*_cpt7_ie_tc.bag", "*_tf_model.bag"],
                TF_MODEL_PATTERN="*_tf_model.bag",
                get_file=lambda pattern, files=files: files.get(pattern, (None, False)),
            )
            resolve = extracted_function(SCRIPTS / "application/graph_msf.py", "resolve_fusion_inputs", namespace)
            with contextlib.redirect_stdout(io.StringIO()):
                inputs, selected = resolve()
                self.assertEqual(selected, model)
                self.assertIn(str(model), inputs)
                del files["*_tf_model.bag"]
                files["*_tf_minimal.bag"] = (str(model), True)
                with self.assertRaises(FileNotFoundError):
                    resolve()
        derive = extracted_function(SCRIPTS / "application/graph_msf.py", "fusion_output_paths", dict(Path=Path))
        self.assertEqual(
            [Path(p).name for p in derive(model)],
            ["mission_tf_static_gt.bag", "mission_gt_tf.bag", "mission_gt_pose.bag"],
        )
        with self.assertRaises(ValueError):
            derive(self.directory / "mission_tf_minimal.bag")

    def test_model_without_static_tf_is_rejected(self):
        with rosbag.Bag(str(self.output), "w") as bag:
            bag.write("/metadata", String("not a model"), rospy.Time(1))
        namespace = dict(
            Path=Path,
            rosbag=rosbag,
            MISSION_DATA=str(self.directory),
            PATTERNS=["*_tf_model.bag"],
            TF_MODEL_PATTERN="*_tf_model.bag",
            get_file=lambda pattern: (str(self.output), True),
        )
        resolve = extracted_function(SCRIPTS / "application/graph_msf.py", "resolve_fusion_inputs", namespace)
        with self.assertRaises(ValueError):
            resolve()
        with rosbag.Bag(str(self.output), "w") as bag:
            bag.write("/tf_static", TFMessage([]), rospy.Time(1))
        with self.assertRaises(ValueError):
            resolve()


class GNSSRateContracts(TemporaryCase):
    def test_default_20_hz_survives_all_tc_retries(self):
        (self.directory / "ie").mkdir()
        source = self.directory / "ie/input.LOG"
        source.write_text("synthetic fixture; external processor is mocked")
        commands = []

        def run(command, **kwargs):
            commands.append(command)
            mode = command[command.index("-procmode") + 1]
            return types.SimpleNamespace(
                returncode=1 if mode == "tc" else 0, stdout="transient test failure", stderr=""
            )

        namespace = dict(
            argparse=argparse,
            math=math,
            Path=Path,
            shlex=shlex,
            shutil=shutil,
            logging=logging,
            MISSION_DATA=str(self.directory),
            os=types.SimpleNamespace(environ={"IE_API_KEY": "synthetic-token"}, chdir=Mock()),
            time=types.SimpleNamespace(sleep=Mock()),
            getpass=types.SimpleNamespace(getuser=lambda: "test-user"),
            subprocess=types.SimpleNamespace(run=run, PIPE=-1),
            get_file=lambda *args: (str(source), True),
        )
        main = extracted_function(SCRIPTS / "cpt7/gnss_process.py", "main", namespace)
        with contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(main([]), 1)
        self.assertEqual(len(commands), 12)  # Four other modes plus eight TC attempts.
        self.assertEqual({command[command.index("-procint") + 1] for command in commands}, {"0.05"})
