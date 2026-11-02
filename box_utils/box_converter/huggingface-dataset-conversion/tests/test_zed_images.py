"""Exercise actual decoder/writer functions without importing unrelated robot message types."""

import ast
import __future__
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest

import cv2
from cv_bridge import CvBridge
from imageio import imwrite
import numpy as np
from sensor_msgs.msg import CompressedImage

SOURCE = Path(__file__).parents[1] / "dataset_builder/build_data.py"
nodes = [
    node
    for node in ast.parse(SOURCE.read_text()).body
    if isinstance(node, ast.FunctionDef)
    and node.name in ("_decode_depth_message", "_extract_and_save_image_from_message")
]
namespace = {"np": np, "cv2": cv2, "imwrite": imwrite}
exec(
    compile(ast.Module(body=nodes, type_ignores=[]), str(SOURCE), "exec", flags=__future__.annotations.compiler_flag),
    namespace,
)
decode = namespace["_decode_depth_message"]
write_image = namespace["_extract_and_save_image_from_message"]


def message(image, format="16UC1_compressed_png", prefix=b""):
    ok, encoded = cv2.imencode(".png", image)
    assert ok
    return CompressedImage(format=format, data=prefix + encoded.tobytes())


class ZedImages(unittest.TestCase):
    def setUp(self):
        self.pixels = np.array([[0, 1, 65, 100], [1000, 32767, 65534, 65535]], dtype=np.uint16)

    def test_raw_png_preserves_all_uint16_values(self):
        actual = decode(message(self.pixels))
        np.testing.assert_array_equal(actual, self.pixels)
        self.assertEqual(actual.dtype, np.uint16)

    def test_legacy_uint16_header(self):
        actual = decode(message(self.pixels, "16UC1; compressedDepth png", b"\0" * 12))
        np.testing.assert_array_equal(actual, self.pixels)

    def test_both_export_modes_round_trip_without_unit_changes(self):
        with tempfile.TemporaryDirectory() as tmp:
            for i, mode in enumerate(("depth_zed2i", "zed2i_confidence")):
                write_image(
                    message(self.pixels),
                    i,
                    SimpleNamespace(parsing=mode, format="png"),
                    image_dir=Path(tmp),
                    cv_bridge=CvBridge(),
                )
                actual = cv2.imread(str(Path(tmp) / f"{i:06d}.png"), cv2.IMREAD_UNCHANGED)
                np.testing.assert_array_equal(actual, self.pixels)
                self.assertEqual(actual.dtype, np.uint16)

    def test_rejects_corruption_inverse_depth_and_wrong_bit_depth(self):
        invalid = [
            CompressedImage(format="16UC1_compressed_png", data=b""),
            CompressedImage(format="16UC1_compressed_png", data=b"\x89PNG\r\n\x1a\n"),
            message(self.pixels, "32FC1; compressedDepth png", b"\0" * 12),
            message(self.pixels, "16UC1_compressed_png", b"\0" * 12),
            message(self.pixels.astype(np.uint8)),
            message(np.zeros((2, 4, 3), dtype=np.uint16)),
            CompressedImage(format="16UC1; compressedDepth rvl", data=b"not png"),
        ]
        for msg in invalid:
            with self.subTest(format=msg.format), self.assertRaises(ValueError):
                decode(msg)

    def test_error_and_lossy_format_create_no_image(self):
        with tempfile.TemporaryDirectory() as tmp:
            for format, msg in [("jpg", message(self.pixels)), ("png", CompressedImage(format="png", data=b"bad"))]:
                with self.assertRaises(ValueError):
                    write_image(
                        msg,
                        0,
                        SimpleNamespace(parsing="depth_zed2i", format=format),
                        image_dir=Path(tmp),
                        cv_bridge=CvBridge(),
                    )
            self.assertEqual(list(Path(tmp).iterdir()), [])


if __name__ == "__main__":
    unittest.main()
