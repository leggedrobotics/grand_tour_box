#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
from datetime import datetime
import inspect
from typing import List, Optional, Tuple, Dict

import numpy as np
import open3d as o3d

# ---------------------------------------------------------------------
# Hard-coded parameters
# ---------------------------------------------------------------------

# One or more ROS bag paths (ROS1 .bag files).
# You can include bags that do NOT contain PointCloud2 topics (they'll be skipped for those topics).
BAG_PATHS = []
TF_BAG_PATHS = []

TF_TOPIC = "/tf"
TF_STATIC_TOPIC = "/tf_static"
TARGET_FRAME = "dlio_map"
TF_CACHE_TIME_SECONDS = 10_000.0  # large enough for your bag range

# Viewer camera should be posed to match this TF frame (expressed in TARGET_FRAME).
VIEW_CAMERA_FRAME = "hesai_lidar"
# Which axes of that frame correspond to camera forward/up
VIEW_CAMERA_FORWARD_AXIS = np.array([1.0, 0.0, 0.0], dtype=np.float64)  # +X
VIEW_CAMERA_UP_AXIS = np.array([0.0, 0.0, 1.0], dtype=np.float64)  # +Z

# PointCloud2 topics to extract (one PointCloud2 per topic, per bag)
POINTCLOUD_TOPICS = [
    "/boxi/hesai/points_undistorted",
    "/anymal/velodyne/points_undistorted",
    "/boxi/livox/points_undistorted",
]

# Depth image topics to convert to point clouds (one depth->pointcloud per topic, per bag)
DEPTH_IMAGE_TOPICS = [
    "/anymal/depth_camera/front_lower/depth/image_rect_raw",
    "/anymal/depth_camera/front_upper/depth/image_rect_raw",
    "/anymal/depth_camera/left/depth/image_rect_raw",
    "/anymal/depth_camera/rear_lower/depth/image_rect_raw",
    "/anymal/depth_camera/rear_upper/depth/image_rect_raw",
    "/anymal/depth_camera/right/depth/image_rect_raw",
]
DEPTH_CAMERAINFO_SUFFIX = "/camera_info"
DEPTH_IMAGE_SUFFIX = "/image_rect_raw"

# Depth->pointcloud conversion controls
DEPTH_STRIDE = 4  # sample every N pixels in u/v (4 -> ~16x fewer points)
DEPTH_MIN_RANGE_M = 0.6
DEPTH_MAX_RANGE_M = 2.0  # <-- requested: max distance = 2 meters
DEPTH_UINT16_SCALE_M = 0.001  # 16UC1 commonly in millimeters -> meters
DEPTH_VOXEL_DOWNSAMPLE_M: Optional[float] = None  # e.g. 0.05; None disables

# Base color per topic (RGB in 0..1). Used to colorize each loaded point cloud.
TOPIC_BASE_COLORS: Dict[str, Tuple[float, float, float]] = {
    "/boxi/hesai/points_undistorted": (1.0, 0.0, 0.0),
    "/anymal/velodyne/points_undistorted": (0.0, 1.0, 0.0),
    "/boxi/livox/points_undistorted": (0.0, 0.0, 1.0),
    # Depth cameras (distinct colors)
    "/anymal/depth_camera/front_lower/depth/image_rect_raw": (1.0, 0.5, 0.0),
    "/anymal/depth_camera/front_upper/depth/image_rect_raw": (1.0, 0.5, 0.0),
    "/anymal/depth_camera/left/depth/image_rect_raw": (1.0, 0.5, 0.0),
    "/anymal/depth_camera/rear_lower/depth/image_rect_raw": (1.0, 0.5, 0.0),
    "/anymal/depth_camera/rear_upper/depth/image_rect_raw": (1.0, 0.5, 0.0),
    "/anymal/depth_camera/right/depth/image_rect_raw": (1.0, 0.5, 0.0),
}

# Offset N (seconds) from bag start time; for each topic, take first message at/after start+N
OFFSET_SECONDS = 10.0

# Where to save PNG screenshots (created if it does not exist)
SCREENSHOT_DIR = os.path.join(os.getcwd(), "screenshots")
SCREENSHOT_BASENAME = "view"
SCREENSHOT_EXT = ".png"

# Window & background settings
VIEW_WINDOW_NAME = "PointCloud Viewer (ROS bag)"
VIEW_WIDTH = 1024
VIEW_HEIGHT = 768
VIEW_BACKGROUND = (1.0, 1.0, 1.0)

# Material for the mesh in the modern renderer
TERRAIN_SHADER = "defaultLit"
TERRAIN_BASE_COLOR = (1.0, 1.0, 1.0, 1.0)
TERRAIN_ROUGHNESS = 1.0
TERRAIN_METALLIC = 0.0

COLORED_SHADER = "defaultUnlitTransparency"
COLORED_BASE_COLOR = (1.0, 1.0, 1.0, 1.0)

# Lighting controls
USE_IBL = True
USE_SUN = True
IBL_INTENSITY = 10000.0
SUN_INTENSITY = 10000.0

# Vertex color gamma correction (applied to pointcloud colors if enabled)
APPLY_COLOR_GAMMA = False
COLOR_GAMMA = 0.1

# Known shader names from Open3D resources
KNOWN_SHADERS = [
    "defaultLit",
    "defaultLitSSR",
    "defaultLitTransparency",
    "defaultUnlit",
    "defaultUnlitTransparency",
    "unlitLine",
    "unlitSolidColor",
    "unlitGradient",
    "unlitBackground",
    "unlitPolygonOffset",
    "pointcloud",
    "normals",
    "depth",
    "img_blit",
    "infiniteGroundPlane",
    "ui_blit",
]

# Global TF buffer (built in main)
TF_BUFFER = None

# Global time reference for camera pose lookup (set from first loaded geometry)
VIEW_TF_TIME = None


# ---------------------------------------------------------------------
# Original mesh helper kept (not used for pointclouds, but preserved)
# ---------------------------------------------------------------------


def split_mesh_by_vertex_color(mesh: o3d.geometry.TriangleMesh, eps: float = 1e-4):
    """Split mesh into (terrain_mesh, colored_mesh) based on vertex colors."""
    if len(mesh.vertex_colors) != len(mesh.vertices) or len(mesh.triangles) == 0:
        return mesh, None

    V = np.asarray(mesh.vertices)
    C = np.asarray(mesh.vertex_colors)
    F = np.asarray(mesh.triangles, dtype=np.int64)

    if C.size == 0:
        return mesh, None

    has_color_v = (C > eps).any(axis=1)
    tri_has_color = np.any(has_color_v[F], axis=1)
    tri_no_color = ~tri_has_color

    if not tri_has_color.any() or not tri_no_color.any():
        return mesh, None

    def make_submesh(tri_mask):
        F_sub = F[tri_mask]
        if F_sub.size == 0:
            return None
        uniq_vidx, inv = np.unique(F_sub.reshape(-1), return_inverse=True)
        V_sub = V[uniq_vidx]
        C_sub = C[uniq_vidx]
        F_sub_remap = inv.reshape(-1, 3).astype(np.int32)

        m = o3d.geometry.TriangleMesh()
        m.vertices = o3d.utility.Vector3dVector(V_sub)
        m.triangles = o3d.utility.Vector3iVector(F_sub_remap)
        if C_sub.size:
            m.vertex_colors = o3d.utility.Vector3dVector(C_sub)
        m.compute_triangle_normals()
        m.compute_vertex_normals()
        return m

    colored = make_submesh(tri_has_color)
    terrain = make_submesh(tri_no_color)

    if terrain is None or colored is None:
        return mesh, None

    terrain.vertex_colors = o3d.utility.Vector3dVector()
    return terrain, colored


# ---------------------------------------------------------------------
# Lighting / GUI callbacks
# ---------------------------------------------------------------------


def _configure_sun_light(o3dvis) -> None:
    """Configure sun light on the visualizer (intensity + enable flag)."""
    try:
        scene = getattr(o3dvis, "scene", None)
        if scene is None:
            return
        low_scene = getattr(scene, "scene", None)
        if low_scene is None:
            return

        direction = np.array([0.577, -0.577, -0.577], dtype=np.float32)
        color = np.array([1.0, 1.0, 1.0], dtype=np.float32)

        try:
            low_scene.set_sun_light(direction, color, float(SUN_INTENSITY))
        except Exception as e:
            print(f"Warning: set_sun_light failed: {e}")

        try:
            low_scene.enable_sun_light(bool(USE_SUN))
        except Exception as e:
            print(f"Warning: enable_sun_light failed: {e}")
    except Exception as e:
        print(f"Warning: failed to configure sun lighting: {e}")


def _toggle_sun_mouse_mode(o3dvis) -> None:
    """Action callback to toggle between camera rotation and sun rotation."""
    try:
        import open3d.visualization.gui as gui
    except Exception as e:
        print(f"Open3D GUI not available, cannot toggle sun mouse mode: {e}")
        return

    Controls = gui.SceneWidget.Controls

    try:
        current = o3dvis.mouse_mode
    except Exception as e:
        print(f"Cannot read mouse_mode: {e}")
        return

    try:
        if current == Controls.ROTATE_SUN:
            o3dvis.mouse_mode = Controls.ROTATE_CAMERA
            print("Mouse mode: ROTATE_CAMERA (normal view control)")
        else:
            o3dvis.mouse_mode = Controls.ROTATE_SUN
            print("Mouse mode: ROTATE_SUN (drag to change light direction)")
        o3dvis.post_redraw()
    except Exception as e:
        print(f"Failed to toggle sun mouse mode: {e}")


# ---------------------------------------------------------------------
# Screenshot helpers
# ---------------------------------------------------------------------


def _ensure_screenshot_dir(screenshot_dir: str) -> None:
    try:
        os.makedirs(screenshot_dir, exist_ok=True)
    except Exception as e:
        print(f"Warning: failed to create screenshot directory '{screenshot_dir}': {e}")


def _make_screenshot_path(screenshot_dir: str) -> str:
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    fname = f"{SCREENSHOT_BASENAME}_{ts}{SCREENSHOT_EXT}"
    return os.path.join(screenshot_dir, fname)


def _save_screenshot_from_o3dvis(o3dvis, screenshot_dir: str) -> None:
    _ensure_screenshot_dir(screenshot_dir)
    path = _make_screenshot_path(screenshot_dir)

    try:
        if hasattr(o3dvis, "export_current_image"):
            o3dvis.export_current_image(path)
            print(f"[Screenshot] Saved PNG to: {path}")
        else:
            scene = getattr(o3dvis, "scene", None)
            low_scene = getattr(scene, "scene", None) if scene is not None else None
            if low_scene is None:
                print("Screenshot failed: no export_current_image and no low-level Scene")
                return

            def _on_image(image):
                try:
                    o3d.io.write_image(path, image, 9)
                    print(f"[Screenshot] Saved PNG to: {path}")
                except Exception as e2:
                    print(f"Screenshot write failed: {e2}")

            low_scene.render_to_image(_on_image)
    except Exception as e:
        print(f"Screenshot failed: {e}")


# ---------------------------------------------------------------------
# TF utilities (ROS1)  [offline TF buffer; no ROS master needed]
# ---------------------------------------------------------------------


def _normalize_frame(frame_id: str) -> str:
    if frame_id is None:
        return ""
    frame_id = str(frame_id)
    return frame_id.lstrip("/").strip()


def _clone_transform_stamped(tr):
    from geometry_msgs.msg import TransformStamped

    out = TransformStamped()
    out.header.stamp = tr.header.stamp
    out.header.frame_id = _normalize_frame(tr.header.frame_id)
    out.child_frame_id = _normalize_frame(tr.child_frame_id)

    out.transform.translation.x = float(tr.transform.translation.x)
    out.transform.translation.y = float(tr.transform.translation.y)
    out.transform.translation.z = float(tr.transform.translation.z)

    out.transform.rotation.x = float(tr.transform.rotation.x)
    out.transform.rotation.y = float(tr.transform.rotation.y)
    out.transform.rotation.z = float(tr.transform.rotation.z)
    out.transform.rotation.w = float(tr.transform.rotation.w)
    return out


def _build_tf_buffer(tf_bag_paths: List[str]):
    """
    Build an offline TF buffer from /tf and /tf_static in bag files.

    Uses tf2_py.BufferCore instead of tf2_ros.Buffer to avoid any dependency on ROS master.
    """
    import rosbag
    import tf2_py as tf2

    class _Dur:
        def __init__(self, s: float):
            self._s = float(s)

        def to_sec(self) -> float:
            return self._s

    try:
        buf = tf2.BufferCore(float(TF_CACHE_TIME_SECONDS))
    except Exception:
        buf = tf2.BufferCore(_Dur(TF_CACHE_TIME_SECONDS))

    has_set_static = hasattr(buf, "set_transform_static")

    for bag_path in tf_bag_paths:
        bag = rosbag.Bag(bag_path, "r")
        try:
            for topic, msg, _t in bag.read_messages(topics=[TF_STATIC_TOPIC, TF_TOPIC]):
                is_static = topic == TF_STATIC_TOPIC
                for tr in getattr(msg, "transforms", []):
                    tr2 = _clone_transform_stamped(tr)
                    if not (tr2.header.frame_id and tr2.child_frame_id):
                        continue

                    if is_static and has_set_static:
                        buf.set_transform_static(tr2, "bag")
                    else:
                        try:
                            buf.set_transform(tr2, "bag", bool(is_static))
                        except TypeError:
                            buf.set_transform(tr2, "bag")
        finally:
            bag.close()

    return buf


def _transform_to_matrix(transform_stamped) -> np.ndarray:
    import tf.transformations as tft

    t = transform_stamped.transform.translation
    q = transform_stamped.transform.rotation
    M = tft.quaternion_matrix([q.x, q.y, q.z, q.w]).astype(np.float64)
    M[0, 3] = float(t.x)
    M[1, 3] = float(t.y)
    M[2, 3] = float(t.z)
    return M


def _is_zero_time(stamp) -> bool:
    try:
        return float(stamp.to_sec()) == 0.0
    except Exception:
        return False


def _transform_pointcloud_to_target(pc: o3d.geometry.PointCloud, msg_with_header, bag_time) -> None:
    global TF_BUFFER
    if TF_BUFFER is None:
        return

    target = _normalize_frame(TARGET_FRAME)
    source = _normalize_frame(getattr(getattr(msg_with_header, "header", None), "frame_id", ""))

    if not source or source == target:
        return

    stamp = getattr(getattr(msg_with_header, "header", None), "stamp", None)
    if stamp is None or _is_zero_time(stamp):
        stamp = bag_time

    if not hasattr(TF_BUFFER, "lookup_transform_core"):
        raise RuntimeError("TF_BUFFER does not expose lookup_transform_core(); not BufferCore?")

    try:
        tf_st = TF_BUFFER.lookup_transform_core(target, source, stamp)
    except Exception:
        tf_st = TF_BUFFER.lookup_transform_core(target, source, bag_time)

    M = _transform_to_matrix(tf_st)
    pc.transform(M)


def _set_view_camera_to_frame(o3dvis) -> None:
    """
    Best-effort: set the viewer camera pose to match VIEW_CAMERA_FRAME expressed in TARGET_FRAME.
    Uses VIEW_TF_TIME as the lookup time.
    """
    global TF_BUFFER, VIEW_TF_TIME
    if TF_BUFFER is None or VIEW_TF_TIME is None:
        return

    target = _normalize_frame(TARGET_FRAME)
    view_frame = _normalize_frame(VIEW_CAMERA_FRAME)
    if not view_frame:
        return

    try:
        tf_st = TF_BUFFER.lookup_transform_core(target, view_frame, VIEW_TF_TIME)
    except Exception as e:
        print(f"Warning: could not lookup view camera transform {target}<-{view_frame}: {e}")
        return

    M = _transform_to_matrix(tf_st)
    R = M[:3, :3]
    p = M[:3, 3]

    forward = R @ VIEW_CAMERA_FORWARD_AXIS
    up = R @ VIEW_CAMERA_UP_AXIS

    if np.linalg.norm(forward) < 1e-9 or np.linalg.norm(up) < 1e-9:
        return

    forward = forward / np.linalg.norm(forward)
    up = up / np.linalg.norm(up)

    eye = p.astype(np.float32)
    center = (p + forward).astype(np.float32)
    up_f = up.astype(np.float32)

    # Prefer the modern API
    try:
        if hasattr(o3dvis, "scene") and hasattr(o3dvis.scene, "camera"):
            o3dvis.scene.camera.look_at(center, eye, up_f)
            print(f"Camera set to frame '{VIEW_CAMERA_FRAME}' (in '{TARGET_FRAME}').")
            return
    except Exception as e:
        print(f"Warning: scene.camera.look_at failed: {e}")

    # Fallbacks across Open3D builds
    try:
        if hasattr(o3dvis, "setup_camera"):
            # Common signature: setup_camera(fov_deg, center, eye, up)
            o3dvis.setup_camera(60.0, center, eye, up_f)
            print(f"Camera set to frame '{VIEW_CAMERA_FRAME}' (in '{TARGET_FRAME}').")
            return
    except Exception as e:
        print(f"Warning: setup_camera fallback failed: {e}")


# ---------------------------------------------------------------------
# Visualization wrapper
# ---------------------------------------------------------------------


def draw_with_bg(
    geometries,
    *,
    window_name="Mesh and Reference",
    width=1024,
    height=768,
    bg=(1.0, 1.0, 1.0),
    screenshot_dir=SCREENSHOT_DIR,
) -> None:
    draw_fn = o3d.visualization.draw

    geoms = geometries
    try:
        from open3d.visualization import rendering

        terrain_mat = rendering.MaterialRecord()
        terrain_mat.shader = TERRAIN_SHADER
        terrain_mat.base_color = TERRAIN_BASE_COLOR
        terrain_mat.base_roughness = TERRAIN_ROUGHNESS
        terrain_mat.base_metallic = TERRAIN_METALLIC

        wrapped = []
        for idx, g in enumerate(geometries):
            wrapped.append({"name": f"geom{idx}", "geometry": g, "material": terrain_mat})
        geoms = wrapped
    except Exception:
        geoms = geometries

    try:
        sig = inspect.signature(draw_fn)
        params = sig.parameters
    except Exception:
        params = {}

    kwargs = {}

    if "window_name" in params:
        kwargs["window_name"] = window_name
    elif "title" in params:
        kwargs["title"] = window_name

    if "width" in params:
        kwargs["width"] = width
    if "height" in params:
        kwargs["height"] = height

    if "bg_color" in params:
        kwargs["bg_color"] = (bg[0], bg[1], bg[2], 1.0)

    if "mesh_show_back_face" in params:
        kwargs["mesh_show_back_face"] = True

    if "show_skybox" in params:
        kwargs["show_skybox"] = False

    def _save_screenshot_action(o3dvis) -> None:
        _save_screenshot_from_o3dvis(o3dvis, screenshot_dir)

    if "actions" in params:
        kwargs.setdefault("actions", [])
        kwargs["actions"].append(("Toggle Sun Mouse Mode", _toggle_sun_mouse_mode))
        kwargs["actions"].append(("Save PNG Screenshot (P)", _save_screenshot_action))

    if "on_init" in params:

        def _on_init(o3dvis):
            try:
                o3dvis.set_background(np.array([bg[0], bg[1], bg[2], 1.0], dtype=np.float32))
            except Exception:
                pass

            _configure_sun_light(o3dvis)

            # Set camera to VIEW_CAMERA_FRAME
            _set_view_camera_to_frame(o3dvis)

            try:
                import open3d.visualization.gui as gui

                if hasattr(o3dvis, "set_on_key"):

                    def _on_key(event):
                        try:
                            if event.type == gui.KeyEvent.Type.DOWN and event.key == gui.KeyName.P:
                                _save_screenshot_from_o3dvis(o3dvis, screenshot_dir)
                                return True
                        except Exception as e:
                            print(f"Key handler failed: {e}")
                        return False

                    o3dvis.set_on_key(_on_key)
                    print("Keybinding installed: press 'P' to save a PNG screenshot.")
            except Exception as e:
                print(f"Could not install key handler on O3DVisualizer: {e}")

        kwargs["on_init"] = _on_init

    draw_fn(geoms, **kwargs)


def list_available_shaders() -> None:
    try:
        from open3d.visualization import rendering
    except Exception:
        print("open3d.visualization.rendering not available")
        return

    print('Known shaders ("*" = accepted by this build):')
    for name in KNOWN_SHADERS:
        ok = False
        try:
            m = rendering.MaterialRecord()
            m.shader = name
            ok = True
        except Exception:
            ok = False
        mark = "*" if ok else " "
        print(f"  {mark} {name}")


# ---------------------------------------------------------------------
# ROS bag point cloud extraction (ROS1 PointCloud2)
# ---------------------------------------------------------------------


def _pc2_fields(msg) -> List[str]:
    try:
        return [f.name for f in msg.fields]
    except Exception:
        return []


def _decode_rgb_packed_to_float01(rgb_packed: np.ndarray) -> np.ndarray:
    if rgb_packed.dtype == np.float32:
        packed_u32 = rgb_packed.view(np.uint32)
    elif rgb_packed.dtype == np.uint32:
        packed_u32 = rgb_packed
    else:
        packed_u32 = rgb_packed.astype(np.uint32, copy=False)

    r = (packed_u32 >> 16) & 0xFF
    g = (packed_u32 >> 8) & 0xFF
    b = packed_u32 & 0xFF
    return np.stack([r, g, b], axis=1).astype(np.float32) / 255.0


def _pc2_to_numpy_ros1(msg) -> Tuple[np.ndarray, Optional[np.ndarray]]:
    fields = _pc2_fields(msg)
    if not all(k in fields for k in ("x", "y", "z")):
        raise RuntimeError(f"PointCloud2 message missing x/y/z fields. Fields: {fields}")

    color_mode = None
    field_names = ["x", "y", "z"]
    if "rgb" in fields:
        field_names.append("rgb")
        color_mode = "rgb"
    elif "rgba" in fields:
        field_names.append("rgba")
        color_mode = "rgba"
    elif all(k in fields for k in ("r", "g", "b")):
        field_names += ["r", "g", "b"]
        color_mode = "separate_rgb"
    elif "intensity" in fields:
        field_names.append("intensity")
        color_mode = "intensity"

    # Decode with declared types/offsets and row stride. Casting packed UINT32
    # colors to float loses bits; packed float RGBA can be NaN despite valid XYZ.
    types = {1: "i1", 2: "u1", 3: "i2", 4: "u2", 5: "i4", 6: "u4", 7: "f4", 8: "f8"}
    selected = {field.name: field for field in msg.fields if field.name in field_names}
    if any(f.count != 1 or f.datatype not in types for f in selected.values()):
        raise ValueError("Viewer requires scalar point fields with supported types")
    endian = ">" if msg.is_bigendian else "<"
    dtype = np.dtype(
        {
            "names": field_names,
            "formats": [endian + types[selected[name].datatype] for name in field_names],
            "offsets": [selected[name].offset for name in field_names],
            "itemsize": msg.point_step,
        }
    )
    if msg.row_step < msg.width * msg.point_step or len(msg.data) < msg.height * msg.row_step:
        raise ValueError("Truncated PointCloud2 payload or invalid row stride")
    pts = np.ndarray(
        (msg.height, msg.width), dtype=dtype, buffer=msg.data, strides=(msg.row_step, msg.point_step)
    ).reshape(-1)
    pts = pts[np.isfinite(pts["x"]) & np.isfinite(pts["y"]) & np.isfinite(pts["z"])]
    if pts.size == 0:
        return np.zeros((0, 3), dtype=np.float64), None

    xyz = np.stack([pts["x"], pts["y"], pts["z"]], axis=1).astype(np.float64)

    colors = None
    if color_mode == "rgb":
        colors = _decode_rgb_packed_to_float01(pts["rgb"].astype(pts.dtype["rgb"].newbyteorder("="), copy=False))
    elif color_mode == "rgba":
        colors = _decode_rgb_packed_to_float01(pts["rgba"].astype(pts.dtype["rgba"].newbyteorder("="), copy=False))
    elif color_mode == "separate_rgb":
        r = pts["r"].astype(np.float32)
        g = pts["g"].astype(np.float32)
        b = pts["b"].astype(np.float32)
        if np.nanmax([r.max(initial=0), g.max(initial=0), b.max(initial=0)]) > 1.0:
            colors = np.stack([r, g, b], axis=1) / 255.0
        else:
            colors = np.stack([r, g, b], axis=1)
        colors = np.clip(colors, 0.0, 1.0)
    elif color_mode == "intensity":
        inten = pts["intensity"].astype(np.float32)
        if inten.size > 0:
            lo = float(np.nanmin(inten))
            hi = float(np.nanmax(inten))
            denom = (hi - lo) if hi > lo else 1.0
            g = np.clip((inten - lo) / denom, 0.0, 1.0)
            colors = np.stack([g, g, g], axis=1).astype(np.float32)

    return xyz, colors


def _make_o3d_pointcloud(xyz: np.ndarray, colors: Optional[np.ndarray]) -> o3d.geometry.PointCloud:
    pc = o3d.geometry.PointCloud()
    pc.points = o3d.utility.Vector3dVector(xyz)

    if colors is not None and colors.shape[0] == xyz.shape[0]:
        cols = np.clip(colors, 0.0, 1.0).astype(np.float64, copy=False)
        if APPLY_COLOR_GAMMA:
            cols = np.power(cols, COLOR_GAMMA, dtype=np.float64)
        pc.colors = o3d.utility.Vector3dVector(cols)

    return pc


def _try_read_ros1_first_pc_after(
    bag_path: str, topic: str, offset_sec: float
) -> Tuple[o3d.geometry.PointCloud, float, object, object]:
    import rosbag

    bag = rosbag.Bag(bag_path, "r")
    try:
        start_time = float(bag.get_start_time())
        target_time = start_time + float(offset_sec)

        chosen_msg = None
        chosen_t = None
        for _topic, msg, t in bag.read_messages(topics=[topic]):
            ts = float(t.to_sec()) if hasattr(t, "to_sec") else float(t)
            if ts >= target_time:
                chosen_msg = msg
                chosen_t = t
                break

        if chosen_msg is None:
            raise RuntimeError(
                f"No PointCloud2 message found on topic '{topic}' at/after "
                f"{offset_sec:.3f}s from bag start (bag: {bag_path})."
            )

        xyz, colors = _pc2_to_numpy_ros1(chosen_msg)
        pc = _make_o3d_pointcloud(xyz, colors)

        _transform_pointcloud_to_target(pc, chosen_msg, chosen_t)

        return pc, float(chosen_t.to_sec()), chosen_msg, chosen_t
    finally:
        bag.close()


def read_first_pointcloud_after_offset(
    bag_path: str, topic: str, offset_sec: float
) -> Tuple[o3d.geometry.PointCloud, float, str, object, object]:
    pc, ts, msg, t = _try_read_ros1_first_pc_after(bag_path, topic, offset_sec)
    return pc, ts, "ros1", msg, t


# ---------------------------------------------------------------------
# Depth image -> pointcloud (ROS1 sensor_msgs/Image + sensor_msgs/CameraInfo)
# ---------------------------------------------------------------------


def _camera_info_to_intrinsics(cam_info_msg) -> Tuple[float, float, float, float]:
    # The selected topics are rectified depth images. P, rather than the raw
    # camera matrix K, defines their projection (matching image_geometry).
    projection = getattr(cam_info_msg, "P", None)
    if projection is None or len(projection) != 12 or not np.isfinite(projection).all():
        raise RuntimeError("Rectified depth requires a finite CameraInfo.P[12]")
    bx, by = max(1, cam_info_msg.binning_x), max(1, cam_info_msg.binning_y)
    fx, fy = float(projection[0]) / bx, float(projection[5]) / by
    cx = (float(projection[2]) - cam_info_msg.roi.x_offset) / bx
    cy = (float(projection[6]) - cam_info_msg.roi.y_offset) / by
    if fx <= 0.0 or fy <= 0.0:
        raise RuntimeError(f"Invalid rectified intrinsics: fx={fx}, fy={fy}")
    return fx, fy, cx, cy


def _image_msg_to_depth_meters(img_msg) -> np.ndarray:
    """
    Decode sensor_msgs/Image depth message to float32 meters (H,W).
    Supports 16UC1 (mm) and 32FC1 (m). Handles row step padding and endianness.
    """
    enc = str(getattr(img_msg, "encoding", "")).strip()
    h = int(getattr(img_msg, "height", 0))
    w = int(getattr(img_msg, "width", 0))
    step = int(getattr(img_msg, "step", 0))
    data = getattr(img_msg, "data", b"")

    if h <= 0 or w <= 0 or step <= 0 or data is None:
        raise RuntimeError(f"Bad Image message fields: h={h}, w={w}, step={step}")

    if enc == "16UC1":
        dtype = np.uint16
        bpp = 2
        scale = float(DEPTH_UINT16_SCALE_M)
    elif enc in ("32FC1",):
        dtype = np.float32
        bpp = 4
        scale = 1.0
    else:
        raise RuntimeError(f"Unsupported depth encoding '{enc}'. Expected 16UC1 or 32FC1.")

    if step < w * bpp or len(data) < h * step:
        raise RuntimeError("Depth image has a truncated payload or invalid row stride")
    endian = ">" if img_msg.is_bigendian else "<"
    dtype = np.dtype(dtype).newbyteorder(endian)
    arr = np.ndarray((h, w), dtype=dtype, buffer=data, strides=(step, bpp))

    depth_m = arr.astype(np.float32) * float(scale)
    return depth_m


def _read_first_image_after_offset(bag_path: str, image_topic: str, offset_sec: float):
    import rosbag

    bag = rosbag.Bag(bag_path, "r")
    try:
        start_time = float(bag.get_start_time())
        target_time = start_time + float(offset_sec)

        chosen_msg = None
        chosen_t = None
        for _topic, msg, t in bag.read_messages(topics=[image_topic]):
            ts = float(t.to_sec()) if hasattr(t, "to_sec") else float(t)
            if ts >= target_time:
                chosen_msg = msg
                chosen_t = t
                break

        if chosen_msg is None:
            raise RuntimeError(
                f"No Image message found on topic '{image_topic}' at/after "
                f"{offset_sec:.3f}s from bag start (bag: {bag_path})."
            )
        return chosen_msg, chosen_t
    finally:
        bag.close()


def _read_camera_info_at_or_near(bag_path: str, info_topic: str, t_ref):
    """
    Return CameraInfo nearest in time, preferring latest <= t_ref, otherwise first >= t_ref.
    """
    import rosbag

    last_le = None
    first_ge = None

    bag = rosbag.Bag(bag_path, "r")
    try:
        for _topic, msg, t in bag.read_messages(topics=[info_topic]):
            if t <= t_ref:
                last_le = msg
                continue
            first_ge = msg
            break
    finally:
        bag.close()

    if last_le is not None:
        return last_le
    if first_ge is not None:
        return first_ge
    raise RuntimeError(f"No CameraInfo messages found on topic '{info_topic}' (bag: {bag_path}).")


def _depth_to_pointcloud(depth_m: np.ndarray, fx: float, fy: float, cx: float, cy: float, stride: int) -> np.ndarray:
    """
    depth_m: (H,W) float32 meters
    Returns xyz (N,3) float64 in the camera frame.
    """
    if stride < 1:
        stride = 1

    h, w = depth_m.shape
    vv = np.arange(0, h, stride, dtype=np.int32)
    uu = np.arange(0, w, stride, dtype=np.int32)
    v, u = np.meshgrid(vv, uu, indexing="ij")

    z = depth_m[v, u]
    valid = np.isfinite(z) & (z > DEPTH_MIN_RANGE_M) & (z < DEPTH_MAX_RANGE_M)
    if not np.any(valid):
        return np.zeros((0, 3), dtype=np.float64)

    u = u[valid].astype(np.float32)
    v = v[valid].astype(np.float32)
    z = z[valid].astype(np.float32)

    x = (u - float(cx)) * z / float(fx)
    y = (v - float(cy)) * z / float(fy)

    xyz = np.stack([x, y, z], axis=1).astype(np.float64)
    return xyz


def read_first_depth_pointcloud_after_offset(
    bag_path: str,
    depth_image_topic: str,
    camera_info_topic: str,
    offset_sec: float,
    stride: int = DEPTH_STRIDE,
) -> Tuple[o3d.geometry.PointCloud, float, str, object, object]:
    img_msg, t_img = _read_first_image_after_offset(bag_path, depth_image_topic, offset_sec)

    cam_info = _read_camera_info_at_or_near(bag_path, camera_info_topic, t_img)
    fx, fy, cx, cy = _camera_info_to_intrinsics(cam_info)

    depth_m = _image_msg_to_depth_meters(img_msg)
    xyz = _depth_to_pointcloud(depth_m, fx, fy, cx, cy, stride=stride)

    pc = _make_o3d_pointcloud(xyz, colors=None)

    if DEPTH_VOXEL_DOWNSAMPLE_M is not None and DEPTH_VOXEL_DOWNSAMPLE_M > 0.0 and len(pc.points) > 0:
        pc = pc.voxel_down_sample(float(DEPTH_VOXEL_DOWNSAMPLE_M))

    _transform_pointcloud_to_target(pc, img_msg, t_img)

    return pc, float(t_img.to_sec()), "depth_image", img_msg, t_img


# ---------------------------------------------------------------------
# Main helpers
# ---------------------------------------------------------------------


def _bag_topics(bag_path: str) -> set:
    import rosbag

    bag = rosbag.Bag(bag_path, "r")
    try:
        info = bag.get_type_and_topic_info()
        return set(info.topics.keys()) if hasattr(info, "topics") else set()
    finally:
        bag.close()


def _depth_image_to_info_topic(image_topic: str) -> str:
    if image_topic.endswith(DEPTH_IMAGE_SUFFIX):
        return image_topic[: -len(DEPTH_IMAGE_SUFFIX)] + DEPTH_CAMERAINFO_SUFFIX
    return image_topic.rstrip("/") + "/camera_info"


# ---------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------


def main(argv=None) -> int:
    import argparse

    global TF_BUFFER, VIEW_TF_TIME, BAG_PATHS, TF_BAG_PATHS, TARGET_FRAME
    global OFFSET_SECONDS, POINTCLOUD_TOPICS, DEPTH_IMAGE_TOPICS, SCREENSHOT_DIR
    parser = argparse.ArgumentParser(description="Inspect transformed point clouds and depth images")
    parser.add_argument("--bag", action="append", required=True)
    parser.add_argument("--tf-bag", action="append", required=True)
    parser.add_argument("--target-frame", required=True)
    parser.add_argument("--point-topic", action="append")
    parser.add_argument("--depth-topic", action="append")
    parser.add_argument("--offset", type=float, default=0.0)
    parser.add_argument("--screenshot-dir", default="screenshots")
    parser.add_argument("--check-only", action="store_true", help="Load and transform geometry without a window")
    args = parser.parse_args(argv)
    if not np.isfinite(args.offset) or args.offset < 0:
        parser.error("offset must be finite and nonnegative")
    BAG_PATHS, TF_BAG_PATHS, TARGET_FRAME = args.bag, args.tf_bag, args.target_frame
    OFFSET_SECONDS, SCREENSHOT_DIR = args.offset, args.screenshot_dir
    if args.point_topic is not None:
        POINTCLOUD_TOPICS = args.point_topic
    if args.depth_topic is not None:
        DEPTH_IMAGE_TOPICS = args.depth_topic

    if not BAG_PATHS:
        print("BAG_PATHS is empty.")
        return 1
    if not POINTCLOUD_TOPICS and not DEPTH_IMAGE_TOPICS:
        print("No topics configured.")
        return 1
    if not TF_BAG_PATHS:
        print("TF_BAG_PATHS is empty.")
        return 1

    list_available_shaders()

    try:
        TF_BUFFER = _build_tf_buffer(TF_BAG_PATHS)
        print(f"Loaded TF from: {TF_BAG_PATHS} (topics: {TF_TOPIC}, {TF_STATIC_TOPIC}), target frame: '{TARGET_FRAME}'")
    except Exception as e:
        print(f"Error building TF buffer: {e}")
        return 2

    geoms: List[o3d.geometry.Geometry] = []

    try:
        import rosbag  # noqa: F401
    except Exception as e:
        print(f"ROS1 backend not available (import rosbag failed): {e}")
        return 3

    for bag_path in BAG_PATHS:
        try:
            topics_in_bag = _bag_topics(bag_path)
        except Exception as e:
            print(f"Error opening bag '{bag_path}': {e}")
            continue

        # --- PointCloud2 topics ---
        for topic in POINTCLOUD_TOPICS:
            if topic not in topics_in_bag:
                continue
            try:
                pc, ts, backend, msg, t_msg = read_first_pointcloud_after_offset(
                    bag_path=bag_path,
                    topic=topic,
                    offset_sec=OFFSET_SECONDS,
                )

                if VIEW_TF_TIME is None:
                    VIEW_TF_TIME = t_msg

                base_color = TOPIC_BASE_COLORS.get(topic, (0.8, 0.8, 0.8))
                try:
                    pc.paint_uniform_color(base_color)
                except Exception:
                    pass

                n_pts = len(pc.points)
                src_frame = _normalize_frame(getattr(getattr(msg, "header", None), "frame_id", ""))
                print(
                    f"[{backend}] {os.path.basename(bag_path)} | {topic}: t={ts:.6f}s "
                    f"(offset={OFFSET_SECONDS:.3f}s), src_frame='{src_frame}' -> '{TARGET_FRAME}', "
                    f"points={n_pts}, color={base_color}"
                )
                geoms.append(pc)
            except Exception as e:
                print(f"Error loading '{bag_path}' topic '{topic}': {e}")

        # --- Depth image topics -> point clouds ---
        for img_topic in DEPTH_IMAGE_TOPICS:
            if img_topic not in topics_in_bag:
                continue

            info_topic = _depth_image_to_info_topic(img_topic)
            if info_topic not in topics_in_bag:
                print(f"Depth image present but CameraInfo missing: image='{img_topic}', info='{info_topic}'")
                continue

            try:
                pc, ts, backend, msg, t_msg = read_first_depth_pointcloud_after_offset(
                    bag_path=bag_path,
                    depth_image_topic=img_topic,
                    camera_info_topic=info_topic,
                    offset_sec=OFFSET_SECONDS,
                    stride=DEPTH_STRIDE,
                )

                if VIEW_TF_TIME is None:
                    VIEW_TF_TIME = t_msg

                base_color = TOPIC_BASE_COLORS.get(img_topic, (0.2, 0.2, 0.2))
                try:
                    pc.paint_uniform_color(base_color)
                except Exception:
                    pass

                n_pts = len(pc.points)
                src_frame = _normalize_frame(getattr(getattr(msg, "header", None), "frame_id", ""))
                print(
                    f"[{backend}] {os.path.basename(bag_path)} | {img_topic}: t={ts:.6f}s "
                    f"(offset={OFFSET_SECONDS:.3f}s), src_frame='{src_frame}' -> '{TARGET_FRAME}', "
                    f"points={n_pts}, stride={DEPTH_STRIDE}, max_depth={DEPTH_MAX_RANGE_M:.2f}m, color={base_color}"
                )
                geoms.append(pc)
            except Exception as e:
                print(f"Error converting depth '{bag_path}' topic '{img_topic}': {e}")

    if not geoms:
        print("No geometries loaded; nothing to visualize.")
        return 4

    if args.check_only:
        print(f"Validated {len(geoms)} transformed point clouds")
        return 0

    try:
        draw_with_bg(
            geoms,
            window_name=VIEW_WINDOW_NAME,
            width=VIEW_WIDTH,
            height=VIEW_HEIGHT,
            bg=VIEW_BACKGROUND,
            screenshot_dir=SCREENSHOT_DIR,
        )
    except Exception as e:
        print(f"Warning: visualization failed: {e}")
        return 5

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
