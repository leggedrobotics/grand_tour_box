"""ROS image codecs shared by both standalone anonymizers.

Sensor headers, numeric pixel depth, byte order and alpha are preserved. Model
input scaling is separate from output encoding so inference cannot change units.
"""

import copy
import cv2
import numpy as np
from sensor_msgs.msg import Image


def _decode_raw_image_with_step(data, height, width, step, dtype, channels):
    dtype = np.dtype(dtype)
    if (
        height <= 0
        or width <= 0
        or step < width * channels * dtype.itemsize
        or step % dtype.itemsize
        or len(data) != height * step
    ):
        raise ValueError("Image dimensions, row stride and data length disagree")
    rows = np.frombuffer(data, dtype=dtype).reshape(height, step // dtype.itemsize)
    pixels = rows[:, : width * channels].copy()
    return pixels.reshape((height, width) if channels == 1 else (height, width, channels))


def _compressed_codec_from_format(fmt):
    text = str(fmt).lower()
    if "compresseddepth" in text:
        raise ValueError("Depth images are not supported by the RGB anonymizer")
    if "png" in text:
        return "png"
    if "jpg" in text or "jpeg" in text:
        return "jpeg"
    raise ValueError(f"Unsupported compressed image format: {fmt!r}")


def decode_image_message(msg, msgtype):
    compressed = msgtype == "sensor_msgs/CompressedImage"
    if compressed:
        _compressed_codec_from_format(msg.format)
        image = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_UNCHANGED)
        if image is None:
            raise ValueError("Cannot decode compressed image")
        if image.ndim == 3 and image.shape[2] == 4:
            image = cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)
    elif msgtype == "sensor_msgs/Image":
        encoding = msg.encoding.lower()
        channels = {"mono8": 1, "mono16": 1, "bgr8": 3, "rgb8": 3, "bgra8": 4, "rgba8": 4}.get(encoding)
        if channels is None:
            raise ValueError(f"Unsupported image encoding: {encoding}")
        dtype = np.dtype(">u2" if msg.is_bigendian else "<u2") if encoding == "mono16" else np.uint8
        image = _decode_raw_image_with_step(msg.data, msg.height, msg.width, msg.step, dtype, channels)
        if encoding == "mono16":
            image = image.astype(np.uint16)
        elif encoding == "rgb8":
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        elif encoding in ("rgba8", "bgra8"):
            image = cv2.cvtColor(image, cv2.COLOR_RGBA2BGR if encoding == "rgba8" else cv2.COLOR_BGRA2BGR)
    else:
        raise ValueError(f"Expected an image message, got {msgtype}")
    grayscale = image.ndim == 2
    if grayscale:
        image = np.repeat(image[..., None], 3, axis=2)
    if image.ndim != 3 or image.shape[2] != 3 or image.dtype not in (np.uint8, np.uint16):
        raise ValueError("Expected an 8/16-bit mono or color image")
    return image, grayscale, compressed


def _reencode_compressed_image(blurred, grayscale, fmt, original_msg=None, jpeg_quality=95, png_compression=3):
    codec = _compressed_codec_from_format(fmt)
    pixels = blurred[..., 0] if grayscale else blurred
    if codec == "jpeg" and pixels.dtype != np.uint8:
        raise ValueError("JPEG cannot preserve 16-bit pixel values")
    if original_msg is not None and codec == "png":
        original = cv2.imdecode(np.frombuffer(original_msg.data, np.uint8), cv2.IMREAD_UNCHANGED)
        if original is not None and original.ndim == 3 and original.shape[2] == 4:
            pixels = np.dstack((pixels, original[..., 3]))
    ok, encoded = cv2.imencode(
        ".png" if codec == "png" else ".jpg",
        pixels,
        [cv2.IMWRITE_PNG_COMPRESSION, png_compression] if codec == "png" else [cv2.IMWRITE_JPEG_QUALITY, jpeg_quality],
    )
    if not ok:
        raise ValueError("Cannot encode anonymized image")
    return encoded.tobytes()


def _encode_raw_image_msg(blurred, original_msg):
    encoding = original_msg.encoding.lower()
    pixels = blurred[..., 0] if encoding in ("mono8", "mono16") else blurred
    if encoding == "rgb8":
        pixels = cv2.cvtColor(pixels, cv2.COLOR_BGR2RGB)
    elif encoding in ("bgra8", "rgba8"):
        pixels = cv2.cvtColor(pixels, cv2.COLOR_BGR2RGBA if encoding == "rgba8" else cv2.COLOR_BGR2BGRA)
        original = _decode_raw_image_with_step(
            original_msg.data, original_msg.height, original_msg.width, original_msg.step, np.uint8, 4
        )
        pixels[..., 3] = original[..., 3]
    elif encoding not in ("mono8", "mono16", "bgr8"):
        raise ValueError(f"Unsupported encoding: {encoding}")
    if encoding == "mono16":
        pixels = pixels.astype(">u2" if original_msg.is_bigendian else "<u2")
    pixels = np.ascontiguousarray(pixels)
    if pixels.shape[:2] != (original_msg.height, original_msg.width):
        raise ValueError("Anonymization changed image dimensions")
    return Image(
        header=copy.deepcopy(original_msg.header),
        height=original_msg.height,
        width=original_msg.width,
        encoding=original_msg.encoding,
        is_bigendian=original_msg.is_bigendian,
        step=pixels.nbytes // original_msg.height,
        data=pixels.tobytes(),
    )
