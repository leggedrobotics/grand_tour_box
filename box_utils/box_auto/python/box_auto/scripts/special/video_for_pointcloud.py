#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Open3D turntable for POINT CLOUDS (.ply with RGB) with bright, color-faithful (UNLIT) rendering
and robust PDF export that mirrors the GUI camera exactly.

- Press 'P' in the GUI to export a PDF.
- --pdf-page {a4,letter,actual} controls page size (default a4).
- --export-scale N supersamples the PNG used in the PDF (default 2).
"""

import os
import sys
import time
import argparse
import numpy as np
from typing import Optional, Sequence, Tuple

from box_auto.utils.atomic_file import atomic_directory

import open3d as o3d
import open3d.visualization.gui as gui
import open3d.visualization.rendering as rendering


# ------------------------------- IO / Utils -------------------------------- #


def _ensure_dir(d: str) -> None:
    os.makedirs(d, exist_ok=True)


def _timestamp() -> str:
    import time as _t

    return _t.strftime("%Y%m%d_%H%M%S")


def _parse_vec3(s: str) -> np.ndarray:
    vals = [float(v) for v in s.split(",")]
    if len(vals) != 3:
        raise ValueError("Expected --cam as 'x,y,z'")
    return np.asarray(vals, dtype=float)


def _parse_rgba(s: str) -> Tuple[float, float, float, float]:
    s = s.strip().lower()
    if s == "black":
        return (0.0, 0.0, 0.0, 1.0)
    if s == "white":
        return (1.0, 1.0, 1.0, 1.0)
    if s.startswith("#"):
        hx = s[1:]
        if len(hx) not in (6, 8):
            raise ValueError("Hex must be #RRGGBB or #RRGGBBAA")
        r = int(hx[0:2], 16) / 255.0
        g = int(hx[2:4], 16) / 255.0
        b = int(hx[4:6], 16) / 255.0
        a = 1.0 if len(hx) == 6 else int(hx[6:8], 16) / 255.0
        return (r, g, b, a)
    parts = [p.strip() for p in s.split(",")]
    if len(parts) not in (3, 4):
        raise ValueError("Color must be #hex or 'r,g,b[,a]'")
    vals = [float(v) for v in parts]
    if any(v > 1.0 for v in vals):  # auto-normalize 0..255
        vals = [v / 255.0 for v in vals]
    if len(vals) == 3:
        vals.append(1.0)
    return tuple(float(x) for x in vals)  # type: ignore[return-value]


def _downscale_image(img: o3d.geometry.Image, factor: int) -> o3d.geometry.Image:
    if factor <= 1:
        return img
    import numpy as _np

    arr = _np.asarray(img)
    h, w = arr.shape[:2]
    H, W = (h // factor), (w // factor)
    arr = arr[: H * factor, : W * factor]
    if arr.ndim == 3:
        c = arr.shape[2]
        arr = arr.reshape(H, factor, W, factor, c).mean(axis=(1, 3))
    else:
        arr = arr.reshape(H, factor, W, factor).mean(axis=(1, 3))
    out = arr.astype(_np.float32) if arr.dtype in (_np.float32, _np.float64) else _np.rint(arr).astype(_np.uint8)
    return o3d.geometry.Image(out)


# ------------------------------- Material ----------------------------------- #


def _make_unlit_point_material(*, point_size: float, base_color=(1.0, 1.0, 1.0, 1.0)) -> rendering.MaterialRecord:
    mat = rendering.MaterialRecord()
    mat.shader = "defaultUnlit"
    mat.base_color = base_color
    mat.sRGB_color = True
    try:
        mat.point_size = float(point_size)
    except Exception:
        pass
    return mat


# --------------------------- Camera copy utilities -------------------------- #


def _copy_camera_to_offscreen(
    dst_scene: rendering.Open3DScene,
    src_cam: rendering.Camera,
    w: int,
    h: int,
    fallback_center: np.ndarray,
    fallback_aabb: o3d.geometry.AxisAlignedBoundingBox,
    fov_hint_deg: float,
) -> None:
    # Try full matrix copy
    try:
        has_set_view = hasattr(dst_scene.camera, "set_view_matrix")
        has_set_proj = hasattr(dst_scene.camera, "set_projection_matrix") or hasattr(dst_scene.camera, "set_projection")
        get_view = getattr(src_cam, "get_view_matrix", None)
        get_proj = getattr(src_cam, "get_projection_matrix", None)
        if has_set_view and has_set_proj and get_view and get_proj:
            V = get_view()
            P = get_proj()
            try:
                dst_scene.camera.set_projection_matrix(P)
            except Exception:
                fov = getattr(src_cam, "get_field_of_view", lambda: fov_hint_deg)()
                near = getattr(src_cam, "get_near", lambda: 0.1)()
                far = getattr(src_cam, "get_far", lambda: 1000.0)()
                dst_scene.camera.set_projection(
                    rendering.Camera.Projection.PERSPECTIVE, float(fov), float(near), float(far), int(w), int(h)
                )
            dst_scene.camera.set_view_matrix(V)
            return
    except Exception:
        pass

    # Fallbacks
    try:
        fov = getattr(src_cam, "get_field_of_view", lambda: fov_hint_deg)()
        near = getattr(src_cam, "get_near", lambda: 0.1)()
        far = getattr(src_cam, "get_far", lambda: 1000.0)()
        dst_scene.camera.set_projection(
            rendering.Camera.Projection.PERSPECTIVE, float(fov), float(near), float(far), int(w), int(h)
        )
    except Exception:
        try:
            dst_scene.camera.set_projection(
                rendering.Camera.Projection.PERSPECTIVE, float(fov_hint_deg), 0.1, 1000.0, int(w), int(h)
            )
        except Exception:
            pass

    eye = look = up = None
    try:
        ge = getattr(src_cam, "get_position", None) or getattr(src_cam, "get_eye", None)
        gl = getattr(src_cam, "get_look_at", None) or getattr(src_cam, "get_center", None)
        gu = getattr(src_cam, "get_up_vector", None) or getattr(src_cam, "get_up", None)
        if ge:
            eye = np.asarray(ge(), dtype=float)
        if gl:
            look = np.asarray(gl(), dtype=float)
        if gu:
            up = np.asarray(gu(), dtype=float)
    except Exception:
        pass

    if eye is None or look is None or up is None:
        try:
            V = src_cam.get_view_matrix()
            W = np.linalg.inv(np.asarray(V))
            eye = W[:3, 3]
            up = W[:3, 1]
            forward = -W[:3, 2]
            look = eye + forward
        except Exception:
            look = fallback_center
            extent = float(np.linalg.norm(fallback_aabb.get_extent()))
            eye = fallback_center + np.array([0.0, 0.0, 2.6 * extent])
            up = np.array([0.0, 1.0, 0.0])

    dst_scene.camera.look_at(look, eye, up)


def _set_background_and_fxaa(open3d_scene: rendering.Open3DScene, bg_rgba) -> None:
    open3d_scene.set_background(list(bg_rgba))
    try:
        view = open3d_scene.view if hasattr(open3d_scene, "view") else None
        if view is not None and hasattr(view, "set_use_fxaa"):
            view.set_use_fxaa(True)
        elif hasattr(open3d_scene, "set_use_fxaa"):
            open3d_scene.set_use_fxaa(True)
    except Exception:
        pass


# ---------------------- PDF Export (Offscreen re-render) -------------------- #


def _mm_to_pt(mm: float) -> float:
    return mm * 72.0 / 25.4


_PAGE_SIZES = {
    "a4": (_mm_to_pt(210), _mm_to_pt(297)),
    "letter": (72.0 * 8.5, 72.0 * 11.0),
}


def _write_pdf(png_path: str, pdf_path: str, dpi: int, page: str) -> None:
    # Prefer reportlab for precise page size; fall back to Pillow.
    try:
        from reportlab.pdfgen import canvas  # type: ignore
        from reportlab.lib.utils import ImageReader  # type: ignore

        img = ImageReader(png_path)
        iw, ih = img.getSize()
        if page.lower() == "actual":
            w_in, h_in = iw / float(dpi), ih / float(dpi)
            w_pt, h_pt = w_in * 72.0, h_in * 72.0
            c = canvas.Canvas(pdf_path, pagesize=(w_pt, h_pt))
            c.drawImage(img, 0, 0, width=w_pt, height=h_pt, preserveAspectRatio=False, anchor="c")
            c.showPage()
            c.save()
            return
        pw, ph = _PAGE_SIZES.get(page.lower(), _PAGE_SIZES["a4"])
        c = canvas.Canvas(pdf_path, pagesize=(pw, ph))
        margin = 18.0
        avail_w, avail_h = pw - 2 * margin, ph - 2 * margin
        scale = min(avail_w / iw, avail_h / ih)
        w = iw * scale
        h = ih * scale
        x = (pw - w) / 2.0
        y = (ph - h) / 2.0
        c.drawImage(img, x, y, width=w, height=h, preserveAspectRatio=True, anchor="c")
        c.showPage()
        c.save()
    except Exception:
        try:
            from PIL import Image  # type: ignore

            with Image.open(png_path) as im:
                im.save(pdf_path, "PDF", resolution=float(dpi))
        except Exception as e:
            raise RuntimeError(f"PDF export failed; install 'reportlab' or 'Pillow'. Error: {e}") from e


def _export_view_pdf_offscreen(
    widget: gui.SceneWidget,
    pcd: o3d.geometry.PointCloud,
    T_model: np.ndarray,
    center: np.ndarray,
    aabb: o3d.geometry.AxisAlignedBoundingBox,
    fov_hint_deg: float,
    out_dir: str,
    dpi: int,
    page: str,
    scale: int,
    *,
    base_color_rgba: Tuple[float, float, float, float],
    point_size: float,
    bg_rgba: Tuple[float, float, float, float],
) -> str:
    """Re-render the current GUI view offscreen and write an A4/Letter/Actual-size PDF."""
    _ensure_dir(out_dir)
    stamp = _timestamp()
    base = f"screenshot_{stamp}"
    png_path = os.path.join(out_dir, f"{base}.png")
    pdf_path = os.path.join(out_dir, f"{base}.pdf")

    # Match widget aspect & supersampling
    try:
        render_w = int(getattr(widget.frame, "width", 0)) or 1280
        render_h = int(getattr(widget.frame, "height", 0)) or 800
    except Exception:
        render_w, render_h = 1280, 800
    render_w *= max(1, int(scale))
    render_h *= max(1, int(scale))

    ren = rendering.OffscreenRenderer(int(render_w), int(render_h))
    o3s = ren.scene

    _copy_camera_to_offscreen(o3s, widget.scene.camera, render_w, render_h, center, aabb, fov_hint_deg)
    _set_background_and_fxaa(o3s, bg_rgba)

    # Draw cloud
    try:
        pcd_to_draw = pcd.clone()
    except Exception:
        pcd_to_draw = pcd
    mat = _make_unlit_point_material(point_size=point_size, base_color=base_color_rgba)
    o3s.add_geometry("cloud_export", pcd_to_draw, mat)
    o3s.set_geometry_transform("cloud_export", T_model)

    img = ren.render_to_image()
    o3d.io.write_image(png_path, img)
    _write_pdf(png_path, pdf_path, dpi, page)
    return pdf_path


# ------------------------------- Geometry ----------------------------------- #


def load_point_cloud_fixed(ply_path: str) -> o3d.geometry.PointCloud:
    pcd = o3d.io.read_point_cloud(ply_path)
    if pcd.is_empty():
        raise RuntimeError(f"Failed to read point cloud: {ply_path}")
    # Clean NaN/Inf if available
    try:
        pcd, _ = pcd.remove_non_finite_points(remove_nan=True, remove_infinite=True)
    except Exception:
        pass
    # Normalize colors if needed (Open3D expects 0..1)
    if pcd.has_colors():
        cols = np.asarray(pcd.colors)
        if cols.size and np.any(cols > 1.0):
            cols = np.clip(cols / 255.0, 0.0, 1.0)
            pcd.colors = o3d.utility.Vector3dVector(cols)
    return pcd


def rotation_about_center_T(
    center_vec3,
    yaw_rad: Optional[float] = None,
    rx: float = 0.0,
    ry: float = 0.0,
    rz: float = 0.0,
    *,
    degrees: bool = False,
    order: str = "xyz",
) -> np.ndarray:
    c = np.asarray(center_vec3, dtype=float)
    if yaw_rad is not None:
        ry = ry + (np.deg2rad(yaw_rad) if degrees else yaw_rad)
    if degrees:
        rx, ry, rz = np.deg2rad([rx, ry, rz])

    cx, sx = np.cos(rx), np.sin(rx)
    cy, sy = np.cos(ry), np.sin(ry)
    cz, sz = np.cos(rz), np.sin(rz)

    Rx = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]], dtype=float)
    Ry = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]], dtype=float)
    Rz = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]], dtype=float)

    mats = {"x": Rx, "y": Ry, "z": Rz}
    R = np.eye(3)
    for ax in order.lower():
        R = R @ mats[ax]

    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = c - R @ c
    return T


# ------------------------------- GUI Runner --------------------------------- #


def run_gui_turntable(
    ply_path: str,
    rpm: float = 0.0,
    width: int = 1280,
    height: int = 800,
    fov_deg: float = 90.0,
    init_roll_deg: float = 0.0,
    init_pitch_deg: float = 0.0,
    init_yaw_deg: float = 0.0,
    cam_pos: Optional[Sequence[float]] = None,
    *,
    point_size: float = 5.0,
    base_color_rgba=(1.0, 1.0, 1.0, 1.0),
    bg_rgba=(1.0, 1.0, 1.0, 1.0),
    export_dir: str = "exports",
    export_dpi: int = 2200,
    pdf_page: str = "a4",
    export_scale: int = 2,
) -> None:
    app = gui.Application.instance
    app.initialize()
    win = app.create_window("Open3D Turntable (Point Cloud, Unlit Colors)", width, height)
    widget = gui.SceneWidget()
    win.add_child(widget)
    widget.scene = rendering.Open3DScene(win.renderer)
    o3s = widget.scene

    pcd = load_point_cloud_fixed(ply_path)

    aabb = pcd.get_axis_aligned_bounding_box()
    center = np.asarray(aabb.get_center())
    extent = float(np.linalg.norm(aabb.get_extent()))

    eye = (
        np.asarray(cam_pos, dtype=float).reshape(
            3,
        )
        if cam_pos is not None
        else center + np.array([0.0, 0.0, 2.6 * extent])
    )
    up = np.array([0.0, 1.0, 0.0])

    T_init = rotation_about_center_T(center, rx=init_pitch_deg, ry=init_yaw_deg, rz=init_roll_deg, degrees=True)

    mat = _make_unlit_point_material(point_size=point_size, base_color=base_color_rgba)
    name = "cloud"
    o3s.add_geometry(name, pcd, mat)
    o3s.set_geometry_transform(name, T_init)

    # Camera + viewport
    o3s.camera.look_at(center, eye, up)
    try:
        widget.scene.camera.set_projection(
            o3d.visualization.rendering.Camera.Projection.PERSPECTIVE, float(fov_deg), 0.1, 1000.0, width, height
        )
    except Exception:
        pass
    o3s.show_ground_plane(False, rendering.Scene.GroundPlane.XZ)
    _set_background_and_fxaa(o3s, bg_rgba)

    # Animation
    omega = rpm * 2.0 * np.pi / 60.0
    angle = 0.0
    last_t = time.time()
    paused = False

    def on_tick() -> bool:
        nonlocal angle, last_t, paused
        now = time.time()
        dt = now - last_t
        last_t = now
        if not paused:
            angle = (angle + omega * dt) % (2.0 * np.pi)
        T = rotation_about_center_T(
            center, rx=init_pitch_deg, ry=(init_yaw_deg + np.rad2deg(angle)), rz=init_roll_deg, degrees=True
        )
        o3s.set_geometry_transform(name, T)
        return True

    def on_key(e: gui.KeyEvent) -> bool:
        nonlocal paused, angle
        if e.type != gui.KeyEvent.DOWN:
            return False
        if e.key == gui.KeyName.SPACE:
            paused = not paused
            return True
        if e.key == gui.KeyName.R:
            angle = 0.0
            return True
        if e.key == gui.KeyName.P:
            try:
                T_now = rotation_about_center_T(
                    center, rx=init_pitch_deg, ry=(init_yaw_deg + np.rad2deg(angle)), rz=init_roll_deg, degrees=True
                )
                pdf_path = _export_view_pdf_offscreen(
                    widget=widget,
                    pcd=pcd,
                    T_model=T_now,
                    center=center,
                    aabb=aabb,
                    fov_hint_deg=float(fov_deg),
                    out_dir=export_dir,
                    dpi=int(export_dpi),
                    page=str(pdf_page),
                    scale=int(export_scale),
                    base_color_rgba=base_color_rgba,
                    point_size=point_size,
                    bg_rgba=bg_rgba,
                )
                print(f"[export] Saved PDF ({pdf_page}, scale {export_scale}) -> {pdf_path}")
            except Exception as ex:
                print(f"[export] PDF export failed: {ex}", file=sys.stderr)
            return True
        if e.key == gui.KeyName.Q:
            gui.Application.instance.quit()
            return True
        return False

    win.set_on_tick_event(on_tick)
    win.set_on_key(on_key)
    app.run()


# ------------------------------ Headless Render ----------------------------- #


def render_turntable_frames(
    ply_path: str,
    out_dir: str = "frames",
    n_frames: int = 180,
    width: int = 1280,
    height: int = 720,
    fov_deg: float = 90.0,
    init_roll_deg: float = 0.0,
    init_pitch_deg: float = 0.0,
    init_yaw_deg: float = 0.0,
    ssaa: int = 3,
    cam_pos: Optional[Sequence[float]] = None,
    *,
    point_size: float = 5.0,
    base_color_rgba=(1.0, 1.0, 1.0, 1.0),
    bg_rgba=(1.0, 1.0, 1.0, 1.0),
) -> None:
    if min(width, height, n_frames, ssaa) <= 0:
        raise ValueError("Render dimensions, frame count and supersampling must be positive")
    with atomic_directory(out_dir) as pending:
        out_dir = str(pending)

        hi_w, hi_h = width * max(1, ssaa), height * max(1, ssaa)
        ren = rendering.OffscreenRenderer(int(hi_w), int(hi_h))
        o3s = ren.scene

        pcd = load_point_cloud_fixed(ply_path)

        aabb = pcd.get_axis_aligned_bounding_box()
        center = np.asarray(aabb.get_center())
        extent = float(np.linalg.norm(aabb.get_extent()))

        eye = (
            np.asarray(cam_pos, dtype=float).reshape(
                3,
            )
            if cam_pos is not None
            else center + np.array([0.0, 0.0, 2.6 * extent])
        )
        up = np.array([0.0, 1.0, 0.0])

        mat = _make_unlit_point_material(point_size=point_size, base_color=base_color_rgba)
        name = "cloud"
        o3s.add_geometry(name, pcd, mat)

        try:
            ren.setup_camera(float(fov_deg), center, eye, up)
        except Exception:
            o3s.camera.look_at(center, eye, up)

        o3s.set_background(list(bg_rgba))

        for i in range(n_frames):
            theta_deg = init_yaw_deg + (360.0 * i / n_frames)
            T = rotation_about_center_T(center, rx=init_pitch_deg, ry=theta_deg, rz=init_roll_deg, degrees=True)
            o3s.set_geometry_transform(name, T)
            img = ren.render_to_image()
            img = _downscale_image(img, max(1, ssaa))
            if not o3d.io.write_image(os.path.join(out_dir, f"frame_{i:03d}.png"), img):
                raise IOError("Failed to save rendered frame")


# ---------------------------------- CLI ------------------------------------- #


def main() -> None:
    p = argparse.ArgumentParser(
        description="Open3D point cloud turntable (UNLIT colors) + exact-camera PDF export (press P)"
    )
    p.add_argument("--ply", required=True, help="Path to point cloud .ply (RGB recommended)")
    p.add_argument("--mode", choices=["gui", "headless"], default="gui")
    p.add_argument("--width", type=int, default=1280)
    p.add_argument("--height", type=int, default=800)
    p.add_argument("--rpm", type=float, default=0.0)
    p.add_argument("--fov", type=float, default=90.0)
    p.add_argument("--init-yaw", type=float, default=0.0)
    p.add_argument("--init-pitch", type=float, default=0.0)
    p.add_argument("--init-roll", type=float, default=0.0)
    p.add_argument("--frames", type=int, default=180)
    p.add_argument("--out", default="frames")
    p.add_argument("--ssaa", type=int, default=3)
    p.add_argument("--cam", type=str, default="0,100,0")

    p.add_argument("--point-size", type=float, default=1.0)
    p.add_argument("--color", type=str, default="white", help="Base color if points lack RGB")
    p.add_argument("--bg", type=str, default="white")

    # Export controls
    p.add_argument("--export-dir", type=str, default="exports")
    p.add_argument("--export-dpi", type=int, default=2200)
    p.add_argument("--pdf-page", type=str, choices=["a4", "letter", "actual"], default="a4")
    p.add_argument("--export-scale", type=int, default=2)

    args = p.parse_args()
    if min(args.width, args.height, args.frames, args.ssaa) <= 0:
        p.error("width, height, frames and ssaa must be positive")
    if not np.isfinite(args.fov) or not 0 < args.fov < 180:
        p.error("fov must be between 0 and 180 degrees")

    cam_pos = _parse_vec3(args.cam) if args.cam is not None else None
    base_color_rgba = _parse_rgba(args.color)
    bg_rgba = _parse_rgba(args.bg)

    common_kwargs = dict(
        point_size=float(args.point_size),
        base_color_rgba=base_color_rgba,
        bg_rgba=bg_rgba,
    )

    if args.mode == "gui":
        run_gui_turntable(
            ply_path=args.ply,
            rpm=args.rpm,
            width=args.width,
            height=args.height,
            fov_deg=args.fov,
            init_roll_deg=args.init_roll,
            init_pitch_deg=args.init_pitch,
            init_yaw_deg=args.init_yaw,
            cam_pos=cam_pos,
            export_dir=args.export_dir,
            export_dpi=args.export_dpi,
            pdf_page=args.pdf_page,
            export_scale=args.export_scale,
            **common_kwargs,
        )
    else:
        render_turntable_frames(
            ply_path=args.ply,
            out_dir=args.out,
            n_frames=args.frames,
            width=args.width,
            height=args.height,
            fov_deg=args.fov,
            init_roll_deg=args.init_roll,
            init_pitch_deg=args.init_pitch,
            init_yaw_deg=args.init_yaw,
            ssaa=args.ssaa,
            cam_pos=cam_pos,
            **common_kwargs,
        )


if __name__ == "__main__":
    main()
