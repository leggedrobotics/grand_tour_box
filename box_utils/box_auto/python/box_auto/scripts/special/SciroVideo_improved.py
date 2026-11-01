#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Open3D lit turntable with robust PDF export that mirrors the GUI camera exactly.

- Press 'P' in the GUI to export a PDF.
- --pdf-page {a4,letter,actual} controls page size (default a4).
- --export-scale N supersamples the PNG used in the PDF (default 1).
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


def _hsv_to_rgba(h, s, v, a=1.0):
    import colorsys

    r, g, b = colorsys.hsv_to_rgb(h % 1.0, np.clip(s, 0, 1), np.clip(v, 0, 1))
    return (float(r), float(g), float(b), float(a))


def _vibrant_color(seed: int = 0) -> Tuple[float, float, float, float]:
    h = (seed * 0.61803398875) % 1.0
    return _hsv_to_rgba(h, 0.9, 0.98, 1.0)


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


# ------------------------------- Materials ---------------------------------- #


def make_full_pbr_material(
    base_color=(1.0, 1.0, 1.0, 1.0),
    base_metallic=0.0,
    base_roughness=0.25,
    base_reflectance=0.6,
    base_clearcoat=0.25,
    base_clearcoat_roughness=0.12,
    base_anisotropy=0.0,
    emissive_color=(0.0, 0.0, 0.0, 1.0),
    transmission=0.0,
    thickness=1.0,
    has_alpha=False,
    sRGB_color=True,
    albedo_path=None,
    normal_path=None,
    roughness_path=None,
    metallic_path=None,
    ao_path=None,
    aorm_path=None,
    reflectance_path=None,
    clearcoat_path=None,
    clearcoat_roughness_path=None,
    anisotropy_path=None,
    cull_back_faces=False,
    shader: str = "defaultLit",
) -> rendering.MaterialRecord:
    mat = rendering.MaterialRecord()
    mat.shader = shader
    mat.base_color = base_color
    mat.base_metallic = float(base_metallic)
    mat.base_roughness = float(base_roughness)
    mat.base_reflectance = float(base_reflectance)
    mat.base_clearcoat = float(base_clearcoat)
    mat.base_clearcoat_roughness = float(base_clearcoat_roughness)
    mat.base_anisotropy = float(base_anisotropy)
    mat.emissive_color = emissive_color
    mat.transmission = float(transmission)
    mat.thickness = float(thickness)
    mat.has_alpha = bool(has_alpha)
    mat.sRGB_color = bool(sRGB_color)
    if hasattr(mat, "cull_back_faces"):
        mat.cull_back_faces = bool(cull_back_faces)

    def _load(p):
        return o3d.io.read_image(p) if (p and os.path.isfile(p)) else None

    if albedo_path:
        mat.albedo_img = _load(albedo_path)
    if normal_path:
        mat.normal_img = _load(normal_path)
    if roughness_path:
        mat.roughness_img = _load(roughness_path)
    if metallic_path:
        mat.metallic_img = _load(metallic_path)
    if ao_path:
        mat.ao_img = _load(ao_path)
    if aorm_path:
        mat.ao_rough_metal_img = _load(aorm_path)
    if reflectance_path:
        mat.reflectance_img = _load(reflectance_path)
    if clearcoat_path:
        mat.clearcoat_img = _load(clearcoat_path)
    if clearcoat_roughness_path:
        mat.clearcoat_roughness_img = _load(clearcoat_roughness_path)
    if anisotropy_path:
        mat.anisotropy_img = _load(anisotropy_path)
    return mat


def make_material_preset(
    preset: str, base_color=(1.0, 1.0, 1.0, 1.0), emissive=(0.0, 0.0, 0.0, 1.0), two_sided=True
) -> rendering.MaterialRecord:
    p = preset.lower()
    shader = "defaultLit"
    if p == "matte":
        mat = make_full_pbr_material(
            base_color=base_color,
            base_metallic=0.0,
            base_roughness=0.85,
            base_reflectance=0.55,
            base_clearcoat=0.0,
            emissive_color=emissive,
            cull_back_faces=not two_sided,
            shader=shader,
        )
    elif p == "glossy":
        mat = make_full_pbr_material(
            base_color=base_color,
            base_metallic=0.0,
            base_roughness=0.15,
            base_reflectance=0.62,
            base_clearcoat=0.3,
            base_clearcoat_roughness=0.08,
            emissive_color=emissive,
            cull_back_faces=not two_sided,
            shader=shader,
        )
    elif p == "metallic":
        mat = make_full_pbr_material(
            base_color=base_color,
            base_metallic=1.0,
            base_roughness=0.2,
            base_reflectance=0.9,
            base_clearcoat=0.25,
            emissive_color=emissive,
            cull_back_faces=not two_sided,
            shader=shader,
        )
    elif p == "carpaint":
        mat = make_full_pbr_material(
            base_color=base_color,
            base_metallic=0.6,
            base_roughness=0.18,
            base_reflectance=0.75,
            base_clearcoat=0.7,
            base_clearcoat_roughness=0.05,
            base_anisotropy=0.15,
            emissive_color=emissive,
            cull_back_faces=not two_sided,
            shader=shader,
        )
    elif p == "ceramic":
        mat = make_full_pbr_material(
            base_color=base_color,
            base_metallic=0.0,
            base_roughness=0.25,
            base_reflectance=0.68,
            base_clearcoat=0.2,
            base_clearcoat_roughness=0.1,
            emissive_color=emissive,
            cull_back_faces=not two_sided,
            shader=shader,
        )
    elif p == "velvet":
        mat = make_full_pbr_material(
            base_color=base_color,
            base_metallic=0.0,
            base_roughness=0.9,
            base_reflectance=0.4,
            base_clearcoat=0.0,
            base_anisotropy=0.6,
            emissive_color=emissive,
            cull_back_faces=not two_sided,
            shader=shader,
        )
    elif p == "neon":
        shader = "defaultUnlit"
        mat = make_full_pbr_material(
            base_color=base_color,
            base_metallic=0.0,
            base_roughness=1.0,
            base_reflectance=0.0,
            emissive_color=tuple(max(e, bc) for e, bc in zip(emissive, base_color)),
            cull_back_faces=not two_sided,
            shader=shader,
        )
    elif p == "glass":
        shader = "defaultLitTransparency"
        mat = make_full_pbr_material(
            base_color=(base_color[0], base_color[1], base_color[2], 0.1),
            base_metallic=0.0,
            base_roughness=0.04,
            base_reflectance=0.95,
            base_clearcoat=0.0,
            emissive_color=emissive,
            transmission=1.0,
            thickness=1.0,
            has_alpha=True,
            cull_back_faces=not two_sided,
            shader=shader,
        )
    else:
        mat = make_full_pbr_material(
            base_color=base_color,
            base_metallic=0.0,
            base_roughness=0.2,
            base_reflectance=0.6,
            base_clearcoat=0.25,
            emissive_color=emissive,
            cull_back_faces=not two_sided,
            shader=shader,
        )
    return mat


# ---------------------------- Lighting / Render ----------------------------- #


def add_lights(
    open3d_scene: rendering.Open3DScene,
    aabb: o3d.geometry.AxisAlignedBoundingBox,
    *,
    bg_rgba=(0.0, 0.0, 0.0, 1.0),
    ibl_path: Optional[str] = None,
    ibl_intensity: float = 35000.0,
    use_skybox: bool = False,
    enable_shadows: bool = True,
    use_sun: bool = True,
    neutral_lights: bool = False,
) -> None:
    scene = open3d_scene.scene
    extent = float(np.linalg.norm(aabb.get_extent()))
    c = np.asarray(aabb.get_center())

    open3d_scene.set_background(list(bg_rgba))
    try:
        open3d_scene.show_skybox(bool(use_skybox))
    except Exception:
        pass

    try:
        if ibl_path and os.path.isfile(ibl_path):
            scene.set_indirect_light(ibl_path)
            scene.enable_indirect_light(True)
            scene.set_indirect_light_intensity(float(ibl_intensity))
        else:
            scene.enable_indirect_light(False)
    except Exception:
        try:
            scene.enable_indirect_light(False)
        except Exception:
            pass

    if use_sun:
        sun_col = [1.0, 1.0, 1.0] if neutral_lights else [1.0, 0.96, 0.9]
        scene.set_sun_light([0.577, -0.577, -0.577], sun_col, 90000.0)
        scene.enable_sun_light(True)
    else:
        scene.enable_sun_light(False)

    key_col = [1.0, 1.0, 1.0] if neutral_lights else [1.0, 0.92, 0.84]
    fill_col = [1.0, 1.0, 1.0] if neutral_lights else [0.76, 0.85, 1.0]
    rim_col = [1.0, 1.0, 1.0] if neutral_lights else [0.8, 0.9, 1.0]

    scene.add_point_light(
        "key", key_col, c + np.array([1.8 * extent, 0.8 * extent, 1.2 * extent]), 80000.0, 0.6, bool(enable_shadows)
    )
    scene.add_point_light(
        "fill", fill_col, c + np.array([-1.2 * extent, -0.3 * extent, 0.7 * extent]), 26000.0, 0.7, False
    )
    scene.add_point_light(
        "rim", rim_col, c + np.array([-1.6 * extent, 1.2 * extent, -0.6 * extent]), 42000.0, 0.6, False
    )

    try:
        scene.set_shadow_intensity(0.5 if enable_shadows else 0.0)
    except Exception:
        pass


def _apply_post_fx(open3d_scene: rendering.Open3DScene, *, fxaa: bool = True, tone_map: str = "filmic") -> None:
    try:
        view = open3d_scene.view if hasattr(open3d_scene, "view") else None
        if view is not None and hasattr(view, "set_use_fxaa"):
            view.set_use_fxaa(bool(fxaa))
        elif hasattr(open3d_scene, "set_use_fxaa"):
            open3d_scene.set_use_fxaa(bool(fxaa))
    except Exception:
        pass
    try:
        tm = tone_map.strip().lower()
        enum = rendering.ToneMapping.FILMIC
        if tm in ("aces", "acesfilmic", "aces_filmic") and hasattr(rendering.ToneMapping, "ACES"):
            enum = rendering.ToneMapping.ACES
        elif tm == "linear" and hasattr(rendering.ToneMapping, "LINEAR"):
            enum = rendering.ToneMapping.LINEAR
        elif tm == "reinhard" and hasattr(rendering.ToneMapping, "REINHARD"):
            enum = rendering.ToneMapping.REINHARD
        open3d_scene.scene.set_tonemapping(enum)
    except Exception:
        pass


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
    """
    Copy projection + view from GUI camera to offscreen camera, with many fallbacks.
    Guarantees aspect parity by using the same (w,h).
    """
    # 1) Try full matrix copy
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
                # set_projection(projtype, fov, near, far, w, h)
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

    # 2) Try param-based projection + look_at
    try:
        fov = getattr(src_cam, "get_field_of_view", lambda: fov_hint_deg)()
        near = getattr(src_cam, "get_near", lambda: 0.1)()
        far = getattr(src_cam, "get_far", lambda: 1000.0)()
        dst_scene.camera.set_projection(
            rendering.Camera.Projection.PERSPECTIVE, float(fov), float(near), float(far), int(w), int(h)
        )
    except Exception:
        # Best-effort: at least ensure viewport aspect via set_projection if available
        try:
            dst_scene.camera.set_projection(
                rendering.Camera.Projection.PERSPECTIVE, float(fov_hint_deg), 0.1, 1000.0, int(w), int(h)
            )
        except Exception:
            pass

    # 3) Extrinsics
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
        # 4) Derive from view matrix if available
        try:
            V = src_cam.get_view_matrix()
            # Open3D uses row-major numpy arrays; camera space forward is -Z.
            # Invert V to get world transform of the camera.
            W = np.linalg.inv(np.asarray(V))
            eye = W[:3, 3]
            # Camera basis in world (columns of W): [right, up, forward]
            up = W[:3, 1]
            forward = -W[:3, 2]
            look = eye + forward
        except Exception:
            # 5) Final fallback: orbit defaults
            look = fallback_center
            extent = float(np.linalg.norm(fallback_aabb.get_extent()))
            eye = fallback_center + np.array([0.0, 0.0, 2.6 * extent])
            up = np.array([0.0, 1.0, 0.0])

    dst_scene.camera.look_at(look, eye, up)


# ---------------------- PDF Export (Offscreen re-render) -------------------- #


def _mm_to_pt(mm: float) -> float:
    return mm * 72.0 / 25.4


_PAGE_SIZES = {
    "a4": (_mm_to_pt(210), _mm_to_pt(297)),  # 595x842
    "letter": (72.0 * 8.5, 72.0 * 11.0),  # 612x792
}


def _write_pdf(png_path: str, pdf_path: str, dpi: int, page: str) -> None:
    # If we can, prefer reportlab for exact page control.
    try:
        from reportlab.pdfgen import canvas  # type: ignore
        from reportlab.lib.utils import ImageReader  # type: ignore

        img = ImageReader(png_path)
        iw, ih = img.getSize()  # pixels
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
        # Fit while preserving aspect ratio with small margins
        margin = 18.0  # 0.25in
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
        # Pillow fallback (no page control beyond "actual")
        try:
            from PIL import Image  # type: ignore

            with Image.open(png_path) as im:
                im.save(pdf_path, "PDF", resolution=float(dpi))
        except Exception as e:
            raise RuntimeError(f"PDF export failed; install 'reportlab' or 'Pillow'. Error: {e}") from e


def _export_view_pdf_offscreen(
    widget: gui.SceneWidget,
    mesh: o3d.geometry.TriangleMesh,
    T_model: np.ndarray,
    center: np.ndarray,
    aabb: o3d.geometry.AxisAlignedBoundingBox,
    fov_hint_deg: float,
    out_dir: str,
    dpi: int,
    page_mode: str,
    scale: int,
    *,
    material_preset: str,
    base_color_rgba: Tuple[float, float, float, float],
    emissive_rgba: Tuple[float, float, float, float],
    two_sided: bool,
    bg_rgba: Tuple[float, float, float, float],
    ibl_path: Optional[str],
    ibl_intensity: float,
    use_skybox: bool,
    fxaa: bool,
    tone_map: str,
    enable_shadows: bool,
    use_sun: bool,
    neutral_lights: bool,
) -> str:
    """Re-render the current GUI view offscreen and write an A4/Letter/Actual-size PDF."""
    _ensure_dir(out_dir)
    stamp = _timestamp()
    base = f"screenshot_{stamp}"
    png_path = os.path.join(out_dir, f"{base}.png")
    pdf_path = os.path.join(out_dir, f"{base}.pdf")

    # Get current widget drawable area
    try:
        render_w = int(getattr(widget.frame, "width", 0)) or 1280
        render_h = int(getattr(widget.frame, "height", 0)) or 800
    except Exception:
        render_w, render_h = 1280, 800
    render_w *= max(1, int(scale))
    render_h *= max(1, int(scale))

    # Build offscreen renderer
    ren = rendering.OffscreenRenderer(int(render_w), int(render_h))
    o3s = ren.scene

    # Match camera precisely
    _copy_camera_to_offscreen(o3s, widget.scene.camera, render_w, render_h, center, aabb, fov_hint_deg)

    # Lighting and post FX
    add_lights(
        o3s,
        aabb,
        bg_rgba=bg_rgba,
        ibl_path=ibl_path,
        ibl_intensity=ibl_intensity,
        use_skybox=use_skybox,
        enable_shadows=enable_shadows,
        use_sun=use_sun,
        neutral_lights=neutral_lights,
    )
    _apply_post_fx(o3s, fxaa=fxaa, tone_map=tone_map)

    # Geometry + transform + material
    try:
        mesh_to_draw = mesh.clone()
    except Exception:
        mesh_to_draw = mesh
    mat = make_material_preset(material_preset, base_color_rgba, emissive_rgba, two_sided)
    o3s.add_geometry("model_export", mesh_to_draw, mat)
    o3s.set_geometry_transform("model_export", T_model)

    # Render and save PNG
    img = ren.render_to_image()
    o3d.io.write_image(png_path, img)

    # Write PDF (fit to page or 1:1)
    _write_pdf(png_path, pdf_path, dpi, page_mode)

    # Cleanup temp PNG
    # try: os.remove(png_path)
    # except Exception: pass

    return pdf_path


# ------------------------------- Geometry ----------------------------------- #


def load_mesh_fixed(mesh_path: str, smooth_iters: int = 0) -> o3d.geometry.TriangleMesh:
    mesh = o3d.io.read_triangle_mesh(mesh_path)
    if mesh.is_empty() or not mesh.has_triangles() or not np.isfinite(np.asarray(mesh.vertices)).all():
        raise ValueError("Expected a finite, nonempty triangle mesh")
    if smooth_iters < 0:
        raise ValueError("smooth_iters must be nonnegative")
    if smooth_iters:
        mesh = mesh.filter_smooth_simple(number_of_iterations=smooth_iters)
    mesh.compute_triangle_normals()
    mesh.compute_vertex_normals()
    return mesh


def orient_normals_towards_point(
    m: o3d.geometry.TriangleMesh,
    cam_pos: Sequence[float],
    *,
    world_T: Optional[np.ndarray] = None,
) -> None:
    from box_auto.scripts.special.orient_normals_etna import orient_components

    camera = np.asarray(cam_pos, dtype=float)
    if camera.shape != (3,) or not np.isfinite(camera).all():
        raise ValueError("Camera position must be a finite 3-vector")
    # Mesh vertices are local; GUI/headless display applies world_T separately.
    if world_T is not None:
        camera = (np.linalg.inv(world_T) @ np.r_[camera, 1.0])[:3]
    if m.has_triangle_uvs() or m.textures:
        raise ValueError("Camera orientation is only supported for untextured meshes; UV winding must be preserved")
    orient_components(m, camera)
    m.compute_triangle_normals()
    m.compute_vertex_normals()


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
    mesh_path: str,
    rpm: float = 3.0,
    width: int = 1280,
    height: int = 800,
    fov_deg: float = 45.0,
    init_roll_deg: float = 0.0,
    init_pitch_deg: float = 0.0,
    init_yaw_deg: float = 90.0,
    smooth_iters: int = 0,
    orient_to_camera: bool = True,
    cam_pos: Optional[Sequence[float]] = None,
    show_cam_sphere: bool = True,
    cam_sphere_scale: float = 0.05,
    *,
    material_preset: str = "glossy",
    base_color_rgba=(1.0, 1.0, 1.0, 1.0),
    emissive_rgba=(0.0, 0.0, 0.0, 1.0),
    two_sided: bool = True,
    bg_rgba=(0.0, 0.0, 0.0, 1.0),
    ibl_path: Optional[str] = None,
    ibl_intensity: float = 35000.0,
    use_skybox: bool = False,
    fxaa: bool = True,
    tone_map: str = "filmic",
    show_axes: bool = True,
    enable_shadows: bool = True,
    use_sun: bool = True,
    neutral_lights: bool = False,
    export_dir: str = "exports",
    export_dpi: int = 1200,
    pdf_page: str = "a4",
    export_scale: int = 1,
) -> None:
    app = gui.Application.instance
    app.initialize()
    win = app.create_window("Open3D Turntable (Lit)", width, height)
    widget = gui.SceneWidget()
    win.add_child(widget)
    widget.scene = rendering.Open3DScene(win.renderer)
    o3s = widget.scene

    mesh = load_mesh_fixed(mesh_path, smooth_iters=smooth_iters)

    aabb = mesh.get_axis_aligned_bounding_box()
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

    if orient_to_camera:
        orient_normals_towards_point(mesh, eye, world_T=T_init)

    mat = make_material_preset(material_preset, base_color_rgba, emissive_rgba, two_sided)
    name = "model"
    o3s.add_geometry(name, mesh, mat)

    if show_cam_sphere:
        # Visual marker (optional)
        sph = o3d.geometry.TriangleMesh.create_sphere(radius=max(1e-9, cam_sphere_scale * extent), resolution=32)
        sph.compute_vertex_normals()
        sph.translate(eye, relative=False)
        mat_s = make_full_pbr_material(
            base_color=(1.0, 0.0, 0.0, 1.0), emissive_color=(1.0, 0.0, 0.0, 1.0), base_roughness=0.2
        )
        o3s.add_geometry("cam_sphere", sph, mat_s)

    o3s.camera.look_at(center, eye, up)
    try:
        widget.scene.camera.set_projection(
            o3d.visualization.rendering.Camera.Projection.PERSPECTIVE, float(fov_deg), 0.1, 1000.0, width, height
        )
    except Exception:
        pass

    o3s.show_ground_plane(False, rendering.Scene.GroundPlane.XZ)
    o3s.show_axes(bool(show_axes))
    add_lights(
        o3s,
        aabb,
        bg_rgba=bg_rgba,
        ibl_path=ibl_path,
        ibl_intensity=ibl_intensity,
        use_skybox=use_skybox,
        enable_shadows=enable_shadows,
        use_sun=use_sun,
        neutral_lights=neutral_lights,
    )
    _apply_post_fx(o3s, fxaa=fxaa, tone_map=tone_map)

    # Initial transform
    o3s.set_geometry_transform(name, T_init)

    # Animation
    omega = rpm * 2.0 * np.pi / 60.0
    angle = 0.0
    last_t = time.time()
    paused = False
    axes_visible = bool(show_axes)

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
        nonlocal paused, angle, axes_visible
        if e.type != gui.KeyEvent.DOWN:
            return False
        if e.key == gui.KeyName.SPACE:
            paused = not paused
            return True
        if e.key == gui.KeyName.R:
            angle = 0.0
            return True
        if e.key == gui.KeyName.X:
            axes_visible = not axes_visible
            o3s.show_axes(axes_visible)
            return True
        if e.key == gui.KeyName.P:
            try:
                # Current model transform on this frame
                T_now = rotation_about_center_T(
                    center, rx=init_pitch_deg, ry=(init_yaw_deg + np.rad2deg(angle)), rz=init_roll_deg, degrees=True
                )
                pdf_path = _export_view_pdf_offscreen(
                    widget=widget,
                    mesh=mesh,
                    T_model=T_now,
                    center=center,
                    aabb=aabb,
                    fov_hint_deg=float(fov_deg),
                    out_dir=export_dir,
                    dpi=int(export_dpi),
                    page_mode=str(pdf_page),
                    scale=int(export_scale),
                    material_preset=material_preset,
                    base_color_rgba=base_color_rgba,
                    emissive_rgba=emissive_rgba,
                    two_sided=two_sided,
                    bg_rgba=bg_rgba,
                    ibl_path=ibl_path,
                    ibl_intensity=ibl_intensity,
                    use_skybox=use_skybox,
                    fxaa=fxaa,
                    tone_map=tone_map,
                    enable_shadows=enable_shadows,
                    use_sun=use_sun,
                    neutral_lights=neutral_lights,
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
    mesh_path: str,
    out_dir: str = "frames",
    n_frames: int = 120,
    width: int = 1280,
    height: int = 720,
    fov_deg: float = 45.0,
    init_roll_deg: float = 0.0,
    init_pitch_deg: float = 0.0,
    init_yaw_deg: float = 90.0,
    ssaa: int = 1,
    smooth_iters: int = 0,
    orient_to_camera: bool = True,
    cam_pos: Optional[Sequence[float]] = None,
    show_cam_sphere: bool = True,
    cam_sphere_scale: float = 0.05,
    *,
    material_preset: str = "glossy",
    base_color_rgba=(1.0, 1.0, 1.0, 1.0),
    emissive_rgba=(0.0, 0.0, 0.0, 1.0),
    two_sided: bool = True,
    bg_rgba=(0.0, 0.0, 0.0, 1.0),
    ibl_path: Optional[str] = None,
    ibl_intensity: float = 35000.0,
    use_skybox: bool = False,
    fxaa: bool = True,
    tone_map: str = "filmic",
    enable_shadows: bool = True,
    use_sun: bool = True,
    neutral_lights: bool = False,
) -> None:
    if min(width, height, n_frames, ssaa) <= 0:
        raise ValueError("Render dimensions, frame count and supersampling must be positive")
    with atomic_directory(out_dir) as pending:
        out_dir = str(pending)

        hi_w, hi_h = width * max(1, ssaa), height * max(1, ssaa)
        ren = rendering.OffscreenRenderer(int(hi_w), int(hi_h))
        o3s = ren.scene

        mesh = load_mesh_fixed(mesh_path, smooth_iters=smooth_iters)

        aabb = mesh.get_axis_aligned_bounding_box()
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

        if orient_to_camera:
            orient_normals_towards_point(mesh, eye, world_T=T_init)

        mat = make_material_preset(material_preset, base_color_rgba, emissive_rgba, two_sided)
        name = "model"
        o3s.add_geometry(name, mesh, mat)

        if show_cam_sphere:
            sph = o3d.geometry.TriangleMesh.create_sphere(radius=max(1e-9, cam_sphere_scale * extent), resolution=32)
            sph.compute_vertex_normals()
            sph.translate(eye, relative=False)
            mat_s = make_full_pbr_material(
                base_color=(1.0, 0.0, 0.0, 1.0), emissive_color=(1.0, 0.0, 0.0, 1.0), base_roughness=0.2
            )
            o3s.add_geometry("cam_sphere", sph, mat_s)

        try:
            ren.setup_camera(float(fov_deg), center, eye, up)
        except Exception:
            o3s.camera.look_at(center, eye, up)

        add_lights(
            o3s,
            aabb,
            bg_rgba=bg_rgba,
            ibl_path=ibl_path,
            ibl_intensity=ibl_intensity,
            use_skybox=use_skybox,
            enable_shadows=enable_shadows,
            use_sun=use_sun,
            neutral_lights=neutral_lights,
        )
        _apply_post_fx(o3s, fxaa=fxaa, tone_map=tone_map)

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
    p = argparse.ArgumentParser(description="Open3D turntable + exact-camera PDF export (press P)")
    p.add_argument("--mesh", required=True, help="Path to mesh")
    p.add_argument("--mode", choices=["gui", "headless"], default="gui")
    p.add_argument("--width", type=int, default=1280)
    p.add_argument("--height", type=int, default=800)
    p.add_argument("--rpm", type=float, default=0.0)
    p.add_argument("--fov", type=float, default=15.0)
    p.add_argument("--init-yaw", type=float, default=0.0)
    p.add_argument("--init-pitch", type=float, default=90.0)
    p.add_argument("--init-roll", type=float, default=0.0)
    p.add_argument("--smooth-iters", type=int, default=0)
    p.add_argument("--frames", type=int, default=180)
    p.add_argument("--out", default="frames")
    p.add_argument("--ssaa", type=int, default=3)
    p.add_argument("--orient-to-camera", type=int, choices=[0, 1], default=0)
    p.add_argument("--cam", type=str, default="0,100,0")
    p.add_argument("--show-cam-sphere", type=int, choices=[0, 1], default=0)
    p.add_argument("--cam-sphere-scale", type=float, default=0.05)

    p.add_argument(
        "--material-preset",
        choices=["matte", "glossy", "carpaint", "ceramic", "velvet", "neon", "glass", "metallic"],
        default="matte",
    )
    p.add_argument("--color", type=str, default="white")
    p.add_argument("--emissive", type=str, default="0.5,0,0,1")
    p.add_argument("--two-sided", type=int, choices=[0, 1], default=1)
    p.add_argument("--bg", type=str, default="white")
    p.add_argument("--ibl", type=str, default=None)
    p.add_argument("--ibl-intensity", type=float, default=35000.0)
    p.add_argument("--skybox", type=int, choices=[0, 1], default=0)
    p.add_argument("--fxaa", type=int, choices=[0, 1], default=1)
    p.add_argument("--tone-map", type=str, choices=["filmic", "aces", "linear", "reinhard"], default="filmic")

    p.add_argument("--show-axes", type=int, choices=[0, 1], default=0)
    p.add_argument("--shadows", type=int, choices=[0, 1], default=0)
    p.add_argument("--use-sun", type=int, choices=[0, 1], default=1)
    p.add_argument("--neutral-lights", type=int, choices=[0, 1], default=0)

    # Export controls
    p.add_argument("--export-dir", type=str, default="exports")
    p.add_argument("--export-dpi", type=int, default=2200)
    p.add_argument(
        "--pdf-page",
        type=str,
        choices=["a4", "letter", "actual"],
        default="a4",
        help="PDF page size: a4/letter fit the image; actual keeps 1:1 pixels at export-dpi (can be tiny).",
    )
    p.add_argument(
        "--export-scale",
        type=int,
        default=2,
        help="Upscale the offscreen render resolution by this factor before embedding in PDF.",
    )

    args = p.parse_args()
    if min(args.width, args.height, args.frames, args.ssaa) <= 0:
        p.error("width, height, frames and ssaa must be positive")
    if not np.isfinite(args.fov) or not 0 < args.fov < 180:
        p.error("fov must be between 0 and 180 degrees")

    cam_pos = _parse_vec3(args.cam) if args.cam is not None else None
    orient_flag = bool(args.orient_to_camera)
    show_marker = bool(args.show_cam_sphere)

    if args.color.strip().lower() == "auto":
        base_color_rgba = _vibrant_color(seed=7)
    else:
        base_color_rgba = _parse_rgba(args.color)
    emissive_rgba = _parse_rgba(args.emissive)
    bg_rgba = _parse_rgba(args.bg)
    two_sided = bool(args.two_sided)
    fxaa = bool(args.fxaa)
    use_skybox = bool(args.skybox)
    show_axes = bool(args.show_axes)
    enable_shadows = bool(args.shadows)
    use_sun = bool(args.use_sun)
    neutral_lights = bool(args.neutral_lights)

    common_kwargs = dict(
        material_preset=args.material_preset,
        base_color_rgba=base_color_rgba,
        emissive_rgba=emissive_rgba,
        two_sided=two_sided,
        bg_rgba=bg_rgba,
        ibl_path=args.ibl,
        ibl_intensity=args.ibl_intensity,
        use_skybox=use_skybox,
        fxaa=fxaa,
        tone_map=args.tone_map,
    )

    if args.mode == "gui":
        run_gui_turntable(
            mesh_path=args.mesh,
            rpm=args.rpm,
            width=args.width,
            height=args.height,
            fov_deg=args.fov,
            init_roll_deg=args.init_roll,
            init_pitch_deg=args.init_pitch,
            init_yaw_deg=args.init_yaw,
            smooth_iters=args.smooth_iters,
            orient_to_camera=orient_flag,
            cam_pos=cam_pos,
            show_cam_sphere=show_marker,
            cam_sphere_scale=args.cam_sphere_scale,
            show_axes=show_axes,
            enable_shadows=enable_shadows,
            use_sun=use_sun,
            neutral_lights=neutral_lights,
            export_dir=args.export_dir,
            export_dpi=args.export_dpi,
            pdf_page=args.pdf_page,
            export_scale=args.export_scale,
            **common_kwargs,
        )
    else:
        render_turntable_frames(
            mesh_path=args.mesh,
            out_dir=args.out,
            n_frames=args.frames,
            width=args.width,
            height=args.height,
            fov_deg=args.fov,
            init_roll_deg=args.init_roll,
            init_pitch_deg=args.init_pitch,
            init_yaw_deg=args.init_yaw,
            ssaa=args.ssaa,
            smooth_iters=args.smooth_iters,
            orient_to_camera=orient_flag,
            cam_pos=cam_pos,
            show_cam_sphere=show_marker,
            cam_sphere_scale=args.cam_sphere_scale,
            enable_shadows=enable_shadows,
            use_sun=use_sun,
            neutral_lights=neutral_lights,
            **common_kwargs,
        )


if __name__ == "__main__":
    main()
