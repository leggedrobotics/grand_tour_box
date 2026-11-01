#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import numpy as np
import open3d as o3d
import open3d.visualization.gui as gui
import open3d.visualization.rendering as rendering

# ---------------------------------------------------------------------
# Hard-coded parameters
# ---------------------------------------------------------------------

# Path to the mesh to visualize (.ply/.obj/etc.)


# Window & background settings
VIEW_WINDOW_NAME = "Mesh and Reference (RED BG)"
VIEW_WIDTH = 1024
VIEW_HEIGHT = 768
BG_RGBA = (1.0, 1.0, 1.0, 1.0)

# Camera / framing
FOV_DEG = 75.0
CAM_POS = (0.0, 100.0, 100.0)  # same as --cam 0,100,0 in the turntable script
CAM_UP = (1.0, 0.0, 0.0)

# PBR material preset (same model as the turntable script)
MATERIAL_PRESET = "matte"
BASE_COLOR_RGBA = (1.0, 1.0, 1.0, 1.0)
EMISSIVE_RGBA = (0.0, 0.0, 0.0, 1.0)
TWO_SIDED = True

# Lighting / post FX (mirrors defaults from the turntable script)
IBL_PATH = None
IBL_INTENSITY = 35000.0
USE_SKYBOX = False
ENABLE_SHADOWS = False
USE_SUN = True
NEUTRAL_LIGHTS = False
FXAA = True
TONE_MAP = "filmic"

# Vertex color gamma correction
APPLY_COLOR_GAMMA = True
COLOR_GAMMA = 0.5


# ---------------------------------------------------------------------
# Materials (copied / simplified from the turntable script)
# ---------------------------------------------------------------------


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
    return mat


def make_material_preset(
    preset: str, base_color_rgba=(1.0, 1.0, 1.0, 1.0), emissive_rgba=(0.0, 0.0, 0.0, 1.0), two_sided=True
) -> rendering.MaterialRecord:
    p = preset.lower()
    shader = "defaultLit"

    if p == "matte":
        mat = make_full_pbr_material(
            base_color=base_color_rgba,
            base_metallic=0.0,
            base_roughness=0.85,
            base_reflectance=0.55,
            base_clearcoat=0.0,
            emissive_color=emissive_rgba,
            cull_back_faces=not two_sided,
            shader=shader,
        )
    elif p == "glossy":
        mat = make_full_pbr_material(
            base_color=base_color_rgba,
            base_metallic=0.0,
            base_roughness=0.15,
            base_reflectance=0.62,
            base_clearcoat=0.3,
            base_clearcoat_roughness=0.08,
            emissive_color=emissive_rgba,
            cull_back_faces=not two_sided,
            shader=shader,
        )
    elif p == "metallic":
        mat = make_full_pbr_material(
            base_color=base_color_rgba,
            base_metallic=1.0,
            base_roughness=0.2,
            base_reflectance=0.9,
            base_clearcoat=0.25,
            emissive_color=emissive_rgba,
            cull_back_faces=not two_sided,
            shader=shader,
        )
    elif p == "carpaint":
        mat = make_full_pbr_material(
            base_color=base_color_rgba,
            base_metallic=0.6,
            base_roughness=0.18,
            base_reflectance=0.75,
            base_clearcoat=0.7,
            base_clearcoat_roughness=0.05,
            base_anisotropy=0.15,
            emissive_color=emissive_rgba,
            cull_back_faces=not two_sided,
            shader=shader,
        )
    elif p == "ceramic":
        mat = make_full_pbr_material(
            base_color=base_color_rgba,
            base_metallic=0.0,
            base_roughness=0.25,
            base_reflectance=0.68,
            base_clearcoat=0.2,
            base_clearcoat_roughness=0.1,
            emissive_color=emissive_rgba,
            cull_back_faces=not two_sided,
            shader=shader,
        )
    elif p == "velvet":
        mat = make_full_pbr_material(
            base_color=base_color_rgba,
            base_metallic=0.0,
            base_roughness=0.9,
            base_reflectance=0.4,
            base_clearcoat=0.0,
            base_anisotropy=0.6,
            emissive_color=emissive_rgba,
            cull_back_faces=not two_sided,
            shader=shader,
        )
    elif p == "neon":
        shader = "defaultUnlit"
        emissive_boost = tuple(max(e, bc) for e, bc in zip(emissive_rgba, base_color_rgba))
        mat = make_full_pbr_material(
            base_color=base_color_rgba,
            base_metallic=0.0,
            base_roughness=1.0,
            base_reflectance=0.0,
            emissive_color=emissive_boost,
            cull_back_faces=not two_sided,
            shader=shader,
        )
    elif p == "glass":
        shader = "defaultLitTransparency"
        mat = make_full_pbr_material(
            base_color=(base_color_rgba[0], base_color_rgba[1], base_color_rgba[2], 0.1),
            base_metallic=0.0,
            base_roughness=0.04,
            base_reflectance=0.95,
            base_clearcoat=0.0,
            emissive_color=emissive_rgba,
            transmission=1.0,
            thickness=1.0,
            has_alpha=True,
            cull_back_faces=not two_sided,
            shader=shader,
        )
    else:
        mat = make_full_pbr_material(
            base_color=base_color_rgba,
            base_metallic=0.0,
            base_roughness=0.2,
            base_reflectance=0.6,
            base_clearcoat=0.25,
            emissive_color=emissive_rgba,
            cull_back_faces=not two_sided,
            shader=shader,
        )
    return mat


# ---------------------------------------------------------------------
# Lighting / post-processing (same as turntable script)
# ---------------------------------------------------------------------


def add_lights(
    open3d_scene: rendering.Open3DScene,
    aabb: o3d.geometry.AxisAlignedBoundingBox,
    *,
    bg_rgba=(1.0, 1.0, 1.0, 1.0),
    ibl_path=None,
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


def apply_post_fx(open3d_scene: rendering.Open3DScene, *, fxaa: bool = True, tone_map: str = "filmic") -> None:
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


# ---------------------------------------------------------------------
# Main GUI viewer (adapted to hard-coded pipeline)
# ---------------------------------------------------------------------


def main(argv=None) -> None:
    import argparse

    parser = argparse.ArgumentParser(description=__doc__ or "View a triangle mesh")
    parser.add_argument("--mesh", required=True)
    parser.add_argument("--check-only", action="store_true", help="Validate geometry without opening a window")
    args = parser.parse_args(argv)
    mesh = o3d.io.read_triangle_mesh(args.mesh)
    if mesh.is_empty():
        raise ValueError(f"Mesh is empty or failed to load: {args.mesh}")
    if not mesh.has_triangles() or not np.isfinite(np.asarray(mesh.vertices)).all():
        raise ValueError("Expected finite vertices and triangle faces")
    if args.check_only:
        print(f"Validated {len(mesh.vertices)} vertices and {len(mesh.triangles)} triangles")
        return

    try:
        mesh.compute_triangle_normals()
        mesh.compute_vertex_normals()
    except Exception as e:
        print(f"Normal computation failed: {e}")

    if APPLY_COLOR_GAMMA and len(mesh.vertex_colors) == len(mesh.vertices):
        cols = np.asarray(mesh.vertex_colors)
        cols = np.clip(cols, 0.0, 1.0)
        cols = np.power(cols, COLOR_GAMMA, dtype=np.float64)
        mesh.vertex_colors = o3d.utility.Vector3dVector(cols)

    aabb = mesh.get_axis_aligned_bounding_box()
    center = np.asarray(aabb.get_center())
    eye = np.asarray(CAM_POS, dtype=float)
    up = np.asarray(CAM_UP, dtype=float)

    app = gui.Application.instance
    app.initialize()
    win = app.create_window(VIEW_WINDOW_NAME, VIEW_WIDTH, VIEW_HEIGHT)
    widget = gui.SceneWidget()
    win.add_child(widget)

    o3s = rendering.Open3DScene(win.renderer)
    widget.scene = o3s

    mat = make_material_preset(
        MATERIAL_PRESET,
        base_color_rgba=BASE_COLOR_RGBA,
        emissive_rgba=EMISSIVE_RGBA,
        two_sided=TWO_SIDED,
    )
    o3s.add_geometry("mesh", mesh, mat)

    try:
        o3s.camera.set_projection(
            rendering.Camera.Projection.PERSPECTIVE,
            float(FOV_DEG),
            0.1,
            1000.0,
            VIEW_WIDTH,
            VIEW_HEIGHT,
        )
    except Exception:
        pass

    o3s.camera.look_at(center, eye, up)
    o3s.show_ground_plane(False, rendering.Scene.GroundPlane.XZ)
    o3s.show_axes(False)

    add_lights(
        o3s,
        aabb,
        bg_rgba=BG_RGBA,
        ibl_path=IBL_PATH,
        ibl_intensity=IBL_INTENSITY,
        use_skybox=USE_SKYBOX,
        enable_shadows=ENABLE_SHADOWS,
        use_sun=USE_SUN,
        neutral_lights=NEUTRAL_LIGHTS,
    )
    apply_post_fx(o3s, fxaa=FXAA, tone_map=TONE_MAP)

    app.run()


if __name__ == "__main__":
    main()
