import os
import yaml
import cv2
import numpy as np
import rosbag

from pathlib import Path
from tqdm import tqdm
from typing import Optional
from concurrent.futures import ProcessPoolExecutor, as_completed
from sensor_msgs.msg import CompressedImage

from py_raw_image_pipeline import RawImagePipeline
from box_auto.utils import get_bag, upload_bag, WS

# Library versions: Python 3.8.10, cv2 4.13.0, numpy 1.24.4
# IT MIGHT THROW A GRAY IMAGE IF THE VERSIONING IS NOT PERFECT.

ENABLE_LIVE_VIZ = False

PIPELINES = None


def dumpCameraInfo(camera_info_msg, filename: str):
    camera_info = {
        "image_width": camera_info_msg.width,
        "image_height": camera_info_msg.height,
        "camera_name": "alphasense_cam1",
        "camera_matrix": {"rows": 3, "cols": 3, "data": camera_info_msg.K},
        "distortion_model": camera_info_msg.distortion_model,
        "distortion_coefficients": {"rows": 1, "cols": len(camera_info_msg.D), "data": camera_info_msg.D},
        "rectification_matrix": {"rows": 3, "cols": 3, "data": camera_info_msg.R},
        "projection_matrix": {"rows": 3, "cols": 4, "data": camera_info_msg.P},
    }
    with open(filename, "w") as file:
        yaml.dump(camera_info, file, default_flow_style=False)


def canShowWindows() -> bool:
    if not os.environ.get("DISPLAY") and os.name != "nt":
        return False
    try:
        bi = cv2.getBuildInformation()
        return any(k in bi for k in ("GTK", "Qt", "COCOA", "Win32 UI", "WIN32UI"))
    except Exception:
        return False


def maybeShowImage(win_name: str, img) -> bool:
    if not ENABLE_LIVE_VIZ:
        return False
    if not canShowWindows():
        return False
    try:
        cv2.imshow(win_name, img)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            return True
    except cv2.error:
        return False
    return False


def timeFromHeader(msg):
    header = getattr(msg, "header", None)
    if header is None:
        raise RuntimeError("Message has no header")

    stamp = getattr(header, "stamp", None)
    if stamp is None:
        raise RuntimeError("Message header has no stamp")

    secs = getattr(stamp, "secs", None)
    nsecs = getattr(stamp, "nsecs", None)
    if secs is None or nsecs is None:
        raise RuntimeError("Message header stamp has no secs/nsecs")

    if secs == 0 and nsecs == 0:
        raise RuntimeError("Message header stamp is zero")

    return stamp


def initWorker(config_file: str, calib_by_caminfo_topic: dict, use_gpu: bool):
    try:
        cv2.setNumThreads(1)
    except Exception:
        pass

    global PIPELINES
    PIPELINES = {
        caminfo_topic: RawImagePipeline(use_gpu, config_file, calib_file)
        for caminfo_topic, calib_file in calib_by_caminfo_topic.items()
    }


def processOneCompressed(caminfo_topic: str, jpeg_bytes: bytes, pipeline_encoding: str, jpeg_quality: int) -> bytes:
    global PIPELINES
    if PIPELINES is None:
        raise RuntimeError("Worker pipelines not initialized")

    arr = np.frombuffer(jpeg_bytes, dtype=np.uint8)
    img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
    if img is None:
        raise RuntimeError("cv2.imdecode failed")

    out = PIPELINES[caminfo_topic].process(img, pipeline_encoding)

    ok, enc = cv2.imencode(".jpg", out, [int(cv2.IMWRITE_JPEG_QUALITY), int(jpeg_quality)])
    if not ok:
        raise RuntimeError("cv2.imencode failed")

    return enc.tobytes()


def processRosbagParallel(
    input_bag: str,
    output_bag_path: str,
    image_topics: list,
    camera_info_topics: list,
    config_file: str,
    use_gpu: bool = False,
    jpeg_quality: int = 90,
    max_workers: Optional[int] = None,
    max_inflight: Optional[int] = None,
):
    cv2.setNumThreads(1)

    img_to_caminfo = dict(zip(image_topics, camera_info_topics))
    topics_to_read = list(set(image_topics) | set(camera_info_topics))

    bag = rosbag.Bag(input_bag, "r")

    calib_dir = Path(output_bag_path).parent / "calib_tmp"
    calib_dir.mkdir(parents=True, exist_ok=True)

    calib_by_caminfo_topic = {}
    first_caminfo_msgs = {}
    remaining = set(camera_info_topics)

    for topic, msg, _ in bag.read_messages(topics=camera_info_topics):
        if topic in remaining:
            first_caminfo_msgs[topic] = msg
            remaining.remove(topic)
            if len(remaining) == 0:
                break

    if len(remaining) != 0:
        missing = sorted(list(remaining))
        bag.close()
        raise RuntimeError("Missing camera_info topics in bag: {}".format(missing))

    for caminfo_topic, msg in first_caminfo_msgs.items():
        calib_file = calib_dir / (caminfo_topic.replace("/", "_") + ".yaml")
        dumpCameraInfo(msg, str(calib_file))
        calib_by_caminfo_topic[caminfo_topic] = str(calib_file)

    total_messages = bag.get_message_count(topic_filters=topics_to_read)

    out_bag = rosbag.Bag(output_bag_path, "w", compression="lz4")

    if max_workers is None:
        cpu = os.cpu_count()
        if cpu is None:
            cpu = 1
        max_workers = max(1, cpu - 1)

    if max_inflight is None:
        max_inflight = max_workers * 4

    futures = {}

    if ENABLE_LIVE_VIZ:
        initWorker(config_file, calib_by_caminfo_topic, use_gpu)
        for topic, msg, _ in bag.read_messages(topics=image_topics):
            caminfo_topic = img_to_caminfo[topic]
            out_topic = topic.replace("color", "color_corrected")
            raw_bytes = bytes(msg.data)

            raw_arr = np.frombuffer(raw_bytes, dtype=np.uint8)
            raw_img = cv2.imdecode(raw_arr, cv2.IMREAD_COLOR)

            proc_bytes = processOneCompressed(caminfo_topic, raw_bytes, "bgr8", int(jpeg_quality))
            proc_arr = np.frombuffer(proc_bytes, dtype=np.uint8)
            proc_img = cv2.imdecode(proc_arr, cv2.IMREAD_COLOR)

            side_by_side = np.concatenate([raw_img, proc_img], axis=1)
            cv2.imshow(out_topic, side_by_side)
            cv2.waitKey(0)

            h, w = raw_img.shape[:2]
            n_ch = raw_img.shape[2] if raw_img.ndim == 3 else 1
            print("[{}]  shape={}  size={}x{}  channels={}  dtype={}".format(
                topic, raw_img.shape, w, h, n_ch, raw_img.dtype))
            return

    try:
        with ProcessPoolExecutor(
            max_workers=max_workers,
            initializer=initWorker,
            initargs=(config_file, calib_by_caminfo_topic, use_gpu),
        ) as ex:
            for topic, msg, _ in tqdm(
                bag.read_messages(topics=topics_to_read),
                total=total_messages,
                desc="Processing Messages",
            ):
                if topic in camera_info_topics:
                    write_t = timeFromHeader(msg)
                    out_bag.write(topic, msg, write_t)
                    continue

                if topic in image_topics:
                    msg_type_str = getattr(type(msg), "_type", "")
                    if msg_type_str.find("CompressedImage") == -1:
                        raise RuntimeError("Expected CompressedImage on {}, got {}".format(topic, msg_type_str))

                    write_t = timeFromHeader(msg)

                    caminfo_topic = img_to_caminfo[topic]
                    out_topic = topic.replace("color", "color_corrected")

                    if len(futures) >= max_inflight:
                        done = next(as_completed(futures))
                        out_bytes = done.result()
                        out_topic_d, header_d, stamp_d = futures.pop(done)

                        out_msg = CompressedImage()
                        out_msg.header = header_d
                        out_msg.format = "jpeg"
                        out_msg.data = out_bytes
                        out_bag.write(out_topic_d, out_msg, stamp_d)

                    fut = ex.submit(
                        processOneCompressed,
                        caminfo_topic,
                        bytes(msg.data),
                        "bgr8",
                        int(jpeg_quality),
                    )
                    futures[fut] = (out_topic, msg.header, write_t)
                    continue

            for done in as_completed(list(futures.keys())):
                out_bytes = done.result()
                out_topic_d, header_d, stamp_d = futures.pop(done)

                out_msg = CompressedImage()
                out_msg.header = header_d
                out_msg.format = "jpeg"
                out_msg.data = out_bytes
                out_bag.write(out_topic_d, out_msg, stamp_d)

    finally:
        bag.close()
        out_bag.close()
        if ENABLE_LIVE_VIZ:
            try:
                cv2.destroyAllWindows()
            except cv2.error:
                pass

        import shutil
        shutil.rmtree(calib_dir, ignore_errors=True)

def createRandomCollageFromBag(output_bag_path: str, image_topics: list, output_png_path: str, dpi: int = 800):
    import random

    corrected_topics = [t.replace("color", "color_corrected") for t in image_topics]
    n_cams = len(corrected_topics)
    if n_cams != 5:
        raise RuntimeError("Expected 5 camera topics, got {}".format(n_cams))

    def camIdFromTopic(t: str) -> int:
        try:
            after = t.split("/cam", 1)[1]
            num = after.split("/", 1)[0]
            return int(num)
        except Exception:
            raise RuntimeError("Could not parse cam id from topic {}".format(t))

    cam_pose = {
        1: "front_center",
        2: "front_right",
        3: "front_left",
        4: "left",
        5: "right",
    }

    grayscale_ids = {2, 3}

    cam_topic_by_id = {camIdFromTopic(t): t for t in corrected_topics}
    desired_ids = [3, 1, 2, 4, 5]
    desired_topics = [cam_topic_by_id[i] for i in desired_ids]

    def overlayLabel(img, text: str):
        font = cv2.FONT_HERSHEY_SIMPLEX
        scale = max(0.6, min(img.shape[0], img.shape[1]) / 800.0)
        thickness = max(1, int(round(scale * 2)))
        pad = max(4, int(round(scale * 8)))
        (tw, th), baseline = cv2.getTextSize(text, font, scale, thickness)
        x, y = pad, pad + th
        cv2.rectangle(img, (0, 0), (tw + 2 * pad, th + 2 * pad), (0, 0, 0), -1)
        cv2.putText(img, text, (x, y), font, scale, (255, 255, 255), thickness, cv2.LINE_AA)

    bag = rosbag.Bag(output_bag_path, "r")

    buffer = {}
    selected = None
    num_candidates = 0

    try:
        for topic, msg, _ in bag.read_messages(topics=corrected_topics):
            msg_type_str = getattr(type(msg), "_type", "")
            if msg_type_str.find("CompressedImage") == -1:
                raise RuntimeError("Expected CompressedImage on {}, got {}".format(topic, msg_type_str))

            stamp = timeFromHeader(msg)
            key = (int(stamp.secs), int(stamp.nsecs))

            d = buffer.get(key)
            if d is None:
                d = {}
                buffer[key] = d

            d[topic] = bytes(msg.data)

            if len(d) == n_cams:
                num_candidates += 1
                if random.randint(1, num_candidates) == 1:
                    selected = d.copy()
                buffer.pop(key, None)

    finally:
        bag.close()

    if selected is None:
        raise RuntimeError("No timestamp found with all 5 camera images present")

    imgs_by_topic = {}
    for t in corrected_topics:
        jpeg_bytes = selected.get(t, None)
        if jpeg_bytes is None:
            raise RuntimeError("Missing image for topic {}".format(t))
        arr = np.frombuffer(jpeg_bytes, dtype=np.uint8)
        img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        if img is None:
            raise RuntimeError("cv2.imdecode failed for topic {}".format(t))

        cam_id = camIdFromTopic(t)
        if cam_id in grayscale_ids:
            if img.ndim == 3 and img.shape[2] == 3:
                g = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            else:
                g = img
            img = cv2.cvtColor(g, cv2.COLOR_GRAY2BGR)

        imgs_by_topic[t] = img

    imgs_for_size = [imgs_by_topic[t] for t in desired_topics]
    hs = [im.shape[0] for im in imgs_for_size]
    ws = [im.shape[1] for im in imgs_for_size]
    H = int(min(hs))
    W = int(min(ws))
    if H <= 0 or W <= 0:
        raise RuntimeError("Invalid image sizes for collage")

    resized_by_topic = {}
    for t, im in imgs_by_topic.items():
        if im.shape[0] != H or im.shape[1] != W:
            rim = cv2.resize(im, (W, H), interpolation=cv2.INTER_AREA)
        else:
            rim = im
        cam_id = camIdFromTopic(t)
        label = "cam{}: {}".format(cam_id, cam_pose.get(cam_id, "unknown"))
        overlayLabel(rim, label)
        resized_by_topic[t] = rim

    collage = np.zeros((2 * H, 3 * W, 3), dtype=np.uint8)

    # Layout (mount orientation):
    # top row: front_left (cam3) | front_center (cam1) | front_right (cam2)
    # bottom row: left (cam4) | (empty) | right (cam5)
    collage[0:H, 0:W] = resized_by_topic[cam_topic_by_id[3]]
    collage[0:H, W : 2 * W] = resized_by_topic[cam_topic_by_id[1]]
    collage[0:H, 2 * W : 3 * W] = resized_by_topic[cam_topic_by_id[2]]
    collage[H : 2 * H, 0:W] = resized_by_topic[cam_topic_by_id[4]]
    collage[H : 2 * H, 2 * W : 3 * W] = resized_by_topic[cam_topic_by_id[5]]

    try:
        from PIL import Image
    except Exception as e:
        raise RuntimeError("PIL is required to save PNG with DPI metadata: {}".format(e))

    rgb = cv2.cvtColor(collage, cv2.COLOR_BGR2RGB)
    im = Image.fromarray(rgb)
    im.save(output_png_path, format="PNG", dpi=(int(dpi), int(dpi)))


if __name__ == "__main__":
    INPUT_PATTERN = "_nuc_alphasense.bag"
    config_file = str(Path(WS) / "src/grand_tour_box/box_utils/box_auto/cfg/color_calib_file.yaml")

    camera_info_topics = ["/gt_box/alphasense_driver_node/cam{}/color/camera_info".format(n) for n in [1, 2, 3, 4, 5]]
    image_topics = ["/gt_box/alphasense_driver_node/cam{}/color/image/compressed".format(n) for n in [1, 2, 3, 4, 5]]

    input_bag = get_bag("*{}".format(INPUT_PATTERN))
    output_bag_path = input_bag.replace(INPUT_PATTERN, "_nuc_alphasense_color.bag")

    processRosbagParallel(
        input_bag=input_bag,
        output_bag_path=output_bag_path,
        image_topics=image_topics,
        camera_info_topics=camera_info_topics,
        config_file=config_file,
        use_gpu=False,
        jpeg_quality=95,
        max_workers=None,
        max_inflight=None,
    )

    if ENABLE_LIVE_VIZ:
        raise SystemExit(0)

    collage_path = str(Path(output_bag_path).with_suffix("")) + "_random_collage.png"
    createRandomCollageFromBag(output_bag_path, image_topics, collage_path, dpi=800)

    upload_bag(output_bag_path)
    raise SystemExit(0)
