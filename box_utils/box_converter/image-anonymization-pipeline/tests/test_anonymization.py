"""Disposable synthetic fixtures. No detector inference, datasets or cloud requests.

Run with: conda run -n christian_grandtour python -m unittest discover -s tests -v
"""

import importlib.util
import importlib
import os
from pathlib import Path
import subprocess
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import cv2
import numpy as np
import rosbag
import rospy
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Header, String

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
import anonymization as cloud
import bag_anonymization as writer
from image_codec import decode_image_message

spec = importlib.util.spec_from_file_location("anonymization_cluster", ROOT / "cluster/create_submit_files.py")
cluster = importlib.util.module_from_spec(spec)
spec.loader.exec_module(cluster)


def identity(image, index):
    return image.copy(), None, 0


class TemporaryCase(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory(prefix="anonymization-unit-")
        self.addCleanup(temporary.cleanup)
        self.directory = Path(temporary.name)
        self.source = self.directory / "source.bag"
        self.output = self.directory / "output.bag"

    def make_bag(self, *, corrupt=False, extra=False):
        rng = np.random.RandomState(42)
        self.rgb = rng.randint(0, 256, (32, 48, 3), dtype=np.uint8)
        self.mono = rng.randint(0, 65536, (32, 48), dtype=np.uint16)
        ok, data = cv2.imencode(".png", self.mono)
        assert ok
        self.raw = Image(
            header=Header(seq=17, stamp=rospy.Time(123, 45), frame_id="camera"),
            height=32,
            width=48,
            encoding="rgb8",
            step=144,
            data=self.rgb.tobytes(),
        )
        self.png = CompressedImage(
            header=Header(seq=18, stamp=rospy.Time(124, 67), frame_id="mono"),
            format="mono16; png compressed",
            data=b"broken" if corrupt else data.tobytes(),
        )
        with rosbag.Bag(str(self.source), "w") as bag:
            bag.write("/metadata", String(data="unchanged"), rospy.Time(1, 9))
            bag.write("/rgb", self.raw, rospy.Time(2, 11))
            bag.write("/mono", self.png, rospy.Time(3, 13))
            if extra:
                bag.write("/other_camera", self.raw, rospy.Time(4, 15))

    def run_writer(self, **kwargs):
        options = dict(image_topics=["/rgb", "/mono"], transform=identity)
        options.update(kwargs)
        return writer.process_bag(self.source, self.output, **options)

    def assert_unpublished(self):
        self.assertFalse(self.output.exists())
        self.assertFalse(list(self.directory.glob(".anonymizing-*")))


class BagTests(TemporaryCase):
    def test_redaction_preserves_encoding_headers_times_and_unselected_pixels(self):
        self.make_bag()

        def redact(image, index):
            result, annotated = cloud.blur_and_annotated_image_given_detections(image, [[5.1, 6.1, 22.9, 24.9]])
            return result, annotated, 1

        self.run_writer(transform=redact, require_all_image_topics=True)
        with rosbag.Bag(str(self.output)) as bag:
            records = {topic: (msg, stamp) for topic, msg, stamp in bag.read_messages()}
        self.assertEqual(len(records), 5)
        self.assertEqual(records["/metadata"][0].data, "unchanged")
        self.assertEqual(records["/metadata"][1].to_nsec(), 1000000009)
        self.assertEqual(records["/rgb"][0].encoding, "rgb8")
        self.assertEqual(records["/mono"][0].format, self.png.format)
        mask = np.zeros((32, 48), dtype=bool)
        mask[6:25, 5:23] = True
        for topic, original, msgtype, stamp in (
            ("/rgb", self.raw, "sensor_msgs/Image", 2000000011),
            ("/mono", self.png, "sensor_msgs/CompressedImage", 3000000013),
        ):
            output, timestamp = records[topic]
            self.assertEqual(
                (output.header.seq, output.header.frame_id, output.header.stamp.to_nsec()),
                (original.header.seq, original.header.frame_id, original.header.stamp.to_nsec()),
            )
            self.assertEqual(timestamp.to_nsec(), stamp)
            before = decode_image_message(original, msgtype)[0]
            after = decode_image_message(output, msgtype)[0]
            self.assertEqual(before.dtype, after.dtype)
            np.testing.assert_array_equal(before[~mask], after[~mask])
            self.assertTrue(np.any(before[mask] != after[mask]))
            self.assertEqual(records[topic + "/n_detections"][0].data, 1)

    def test_topic_selection_is_validated_before_output(self):
        self.make_bag(extra=True)
        for topics in ([], ["/absent"], ["/rgb", "/absent"], ["/metadata"]):
            with self.subTest(topics=topics), self.assertRaises(ValueError):
                self.run_writer(image_topics=topics)
            self.assert_unpublished()
        with self.assertRaisesRegex(ValueError, "Unselected image topics"):
            self.run_writer(require_all_image_topics=True)
        self.assert_unpublished()

    def test_cloud_wrapper_requires_all_camera_topics(self):
        self.make_bag(extra=True)
        with self.assertRaisesRegex(ValueError, "Unselected image topics"):
            cloud.anonymize_image_topics_in_bagfile(
                self.source, self.output, image_topics=["/rgb", "/mono"], detectors=None
            )
        self.assert_unpublished()

    def test_local_subset_preserves_other_camera(self):
        self.make_bag(extra=True)
        self.run_writer()
        with rosbag.Bag(str(self.output)) as bag:
            other = next(bag.read_messages(topics=["/other_camera"])).message
        self.assertEqual(bytes(other.data), bytes(self.raw.data))

    def test_corrupt_second_image_removes_partial_output(self):
        self.make_bag(corrupt=True)
        with self.assertRaisesRegex(ValueError, "Cannot decode"):
            self.run_writer()
        self.assert_unpublished()

    def test_inference_failure_or_interruption_removes_partial_output(self):
        self.make_bag()
        for exception in (RuntimeError("detector failed"), KeyboardInterrupt()):

            def fail_on_second(image, index):
                if index == 1:
                    raise exception
                return identity(image, index)

            with self.subTest(exception=type(exception)), self.assertRaises(type(exception)):
                self.run_writer(transform=fail_on_second)
            self.assert_unpublished()

    def test_changed_numeric_depth_or_shape_is_rejected(self):
        self.make_bag()
        for transform in (lambda image, i: (image.astype(np.float32), None, 0), lambda image, i: (image[:-1], None, 0)):
            with self.assertRaisesRegex(ValueError, "dimensions or numeric depth"):
                self.run_writer(transform=transform)
            self.assert_unpublished()

    def test_existing_output_and_same_path_are_preserved(self):
        self.make_bag()
        self.output.write_bytes(b"existing result")
        with self.assertRaises(FileExistsError):
            self.run_writer()
        self.assertEqual(self.output.read_bytes(), b"existing result")
        original = self.source.read_bytes()
        with self.assertRaises(FileExistsError):
            writer.process_bag(self.source, self.source, image_topics=["/rgb"], transform=identity)
        self.assertEqual(self.source.read_bytes(), original)

    def test_concurrent_publication_keeps_other_writers_output(self):
        self.make_bag()
        link = os.link

        def raced_link(source, destination):
            Path(destination).write_bytes(b"other writer")
            return link(source, destination)

        with patch.object(writer.os, "link", side_effect=raced_link), self.assertRaises(FileExistsError):
            self.run_writer()
        self.assertEqual(self.output.read_bytes(), b"other writer")
        self.assertFalse(list(self.directory.glob(".anonymizing-*")))

    def test_sample_cannot_succeed_without_every_selected_topic(self):
        self.make_bag()
        for head in (0, -1, 1, 2):
            with self.subTest(head=head), self.assertRaises(ValueError):
                self.run_writer(head=head)
            self.assert_unpublished()


class UploadTests(TemporaryCase):
    def setUp(self):
        super().setUp()
        self.output.write_bytes(b"synthetic completed output")
        self.verify = self.enter_patch("verify")
        self.upload = self.enter_patch("upload")
        self.upload.return_value = SimpleNamespace(failed=0)
        self.delete = self.enter_patch("delete_file")
        self.list_files = self.enter_patch("list_files")

    def enter_patch(self, name):
        context = patch.object(cloud.kleinkram, name, autospec=True)
        result = context.start()
        self.addCleanup(context.stop)
        return result

    def statuses(self, *values):
        self.verify.side_effect = [{self.output: value} for value in values]

    def upload_file(self, **options):
        return cloud.upload_simple("exact-mission-id", self.output, attempts=2, poll_interval=0, **options)

    def test_success_requires_verified_hash_and_uses_mission_id(self):
        self.statuses("missing", "computing hash", "uploaded")
        self.assertTrue(self.upload_file())
        self.upload.assert_called_once_with(mission_id="exact-mission-id", files=[self.output], create=False)
        for call in self.verify.call_args_list:
            self.assertEqual(call.kwargs, dict(mission_id="exact-mission-id", files=[self.output]))
        self.delete.assert_not_called()

    def test_already_uploaded_is_idempotent(self):
        self.statuses("uploaded")
        self.assertTrue(self.upload_file())
        self.upload.assert_not_called()
        self.delete.assert_not_called()

    def test_missing_or_mismatched_after_upload_is_failure(self):
        for status in ("missing", "hash mismatch"):
            with self.subTest(status=status):
                self.statuses("missing", status, status)
                with self.assertRaises(RuntimeError):
                    self.upload_file()
        self.delete.assert_not_called()

    def test_existing_conflict_requires_explicit_replacement(self):
        self.statuses("hash mismatch")
        with self.assertRaisesRegex(RuntimeError, "cannot be replaced"):
            self.upload_file()
        self.upload.assert_not_called()
        self.delete.assert_not_called()

    def test_replacement_matches_exact_name(self):
        self.statuses("hash mismatch", "uploaded")
        self.list_files.return_value = [
            SimpleNamespace(name=self.output.name + ".backup", id="unrelated", state="OK"),
            SimpleNamespace(name=self.output.name, id="exact", state="OK"),
        ]
        self.assertTrue(self.upload_file(delete=True))
        self.delete.assert_called_once_with("exact")
        self.list_files.assert_called_once_with(mission_ids=["exact-mission-id"], file_names=[self.output.name])

    def test_in_progress_upload_is_never_deleted(self):
        self.statuses("uploading", "uploading")
        with self.assertRaisesRegex(RuntimeError, "timed out"):
            self.upload_file(delete=True)
        self.delete.assert_not_called()
        self.upload.assert_not_called()

    def test_failed_api_or_transfer_propagates_without_retrying_upload(self):
        self.statuses("missing")
        self.upload.side_effect = RuntimeError("network error")
        with self.assertRaisesRegex(RuntimeError, "network error"):
            self.upload_file()
        self.assertEqual(self.upload.call_count, 1)
        self.upload.side_effect = None
        self.upload.return_value = SimpleNamespace(failed=1)
        self.statuses("missing")
        with self.assertRaisesRegex(RuntimeError, "Upload failed"):
            self.upload_file()


class EntryPointTests(TemporaryCase):
    def test_archive_preserves_existing_files(self):
        self.source.write_bytes(b"completed bag")
        archive = self.directory / "archive"
        cloud.archive_output(self.source, archive)
        self.assertEqual((archive / self.source.name).read_bytes(), b"completed bag")
        self.source.write_bytes(b"new result")
        with self.assertRaises(FileExistsError):
            cloud.archive_output(self.source, archive)
        self.assertEqual((archive / self.source.name).read_bytes(), b"completed bag")
        self.assertEqual(self.source.read_bytes(), b"new result")

    def test_failed_archive_copy_does_not_publish_partial_file(self):
        self.source.write_bytes(b"completed bag")
        archive = self.directory / "archive"

        def fail_copy(source, destination):
            Path(destination).write_bytes(b"partial")
            raise OSError("copy failed")

        with patch.object(cloud.shutil, "copyfile", side_effect=fail_copy), self.assertRaises(OSError):
            cloud.archive_output(self.source, archive)
        self.assertEqual(list(archive.iterdir()), [])
        self.assertEqual(self.source.read_bytes(), b"completed bag")

    def test_local_previews_are_opt_in(self):
        self.source.touch()
        for name in ("anonimization_standalone", "anonimization_standalone_chat"):
            module = importlib.import_module(name)
            detectors = SimpleNamespace(scripted_instances_patch=None)
            detectors._replace = lambda **kwargs: detectors
            for preview in (False, True):
                with self.subTest(script=name, preview=preview), patch.object(
                    module, "load_detectors", return_value=detectors
                ), patch.object(
                    module,
                    "initialize_visualization",
                    side_effect=lambda: module.VisualizationState(module.VISUALIZATION_MODE, None),
                ), patch.object(
                    module, "anonymize_image_topics_in_bagfile"
                ) as process:
                    arguments = [str(self.source)]
                    if preview:
                        arguments += ["--preview-dir", str(self.directory / "previews")]
                    self.assertEqual(module.main(arguments), 0)
                    mode = process.call_args.kwargs["visualization_state"].mode
                    self.assertEqual(mode, module.VisualizationMode.SAVE if preview else module.VisualizationMode.NONE)

    def test_sample_never_uploads_and_has_sample_name(self):
        for kind in ("face", "lp"):
            (self.directory / f"ego_blur_{kind}_gen1.jit").touch()
        config = {"input.bag": (["/image"], False, "mission_anon.bag")}
        with patch.object(
            cloud, "get_mission", return_value=SimpleNamespace(id="mission-id", name="pub_mission")
        ), patch.object(cloud, "get_anonymizer_config", return_value=config), patch.object(
            cloud, "load_detectors"
        ), patch.object(
            cloud, "download_files"
        ), patch.object(
            cloud, "anonymize_image_topics_in_bagfile"
        ) as process, patch.object(
            cloud, "upload_files"
        ) as upload:
            self.assertEqual(
                cloud.main(
                    [
                        "--mission-id",
                        "mission-id",
                        "--cam",
                        "alphasense",
                        "--models",
                        str(self.directory),
                        "--data-dir",
                        str(self.directory),
                        "--head",
                        "1",
                        "--replace-existing",
                    ]
                ),
                0,
            )
            self.assertEqual(process.call_args.args[1].name, "mission_anon_sample.bag")
            self.assertEqual(process.call_args.kwargs["head"], 1)
            upload.assert_not_called()

    def test_models_use_gen1_names_and_fail_without_silent_gen2_fallback(self):
        for kind in ("face", "lp"):
            (self.directory / f"ego_blur_{kind}_gen2.jit").touch()
        with self.assertRaises(FileNotFoundError):
            cloud.gen1_model_paths(self.directory)
        for kind in ("face", "lp"):
            (self.directory / f"ego_blur_{kind}.jit").touch()
        self.assertEqual(
            [p.name for p in cloud.gen1_model_paths(self.directory)], ["ego_blur_face.jit", "ego_blur_lp.jit"]
        )
        for kind in ("face", "lp"):
            (self.directory / f"ego_blur_{kind}_gen1.jit").touch()
        self.assertTrue(all("gen1" in p.name for p in cloud.gen1_model_paths(self.directory)))

    def test_generated_job_propagates_setup_and_processor_failures(self):
        job = self.directory / "job.sh"
        job.write_text(cluster.render_job("mission", "11111111-1111-1111-1111-111111111111", "zed2i"))
        # Stub every external operation, including cat: no credentials or cluster access.
        for name in ("tar", "module", "cat", "apptainer"):
            executable = self.directory / name
            executable.write_text("#!/bin/sh\nexit " + ("23" if name == "apptainer" else "0") + "\n")
            executable.chmod(0o755)
        env = dict(os.environ, PATH=str(self.directory) + os.pathsep + os.environ["PATH"], TMPDIR=str(self.directory))
        self.assertEqual(subprocess.run(["bash", str(job)], env=env, capture_output=True).returncode, 23)
        (self.directory / "tar").write_text("#!/bin/sh\nexit 17\n")
        self.assertEqual(subprocess.run(["bash", str(job)], env=env, capture_output=True).returncode, 17)


if __name__ == "__main__":
    unittest.main()
