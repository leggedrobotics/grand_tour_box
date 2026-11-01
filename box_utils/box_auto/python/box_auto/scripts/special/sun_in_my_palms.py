#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
from datetime import datetime

import open3d as o3d
import numpy as np
import inspect

# ---------------------------------------------------------------------
# Hard-coded parameters
# ---------------------------------------------------------------------

# Path to the mesh to visualize (.ply/.obj/etc.)


# Where to save PNG screenshots (created if it does not exist)
SCREENSHOT_DIR = "screenshots"
SCREENSHOT_BASENAME = "view"
SCREENSHOT_EXT = ".png"

# Window & background settings
VIEW_WINDOW_NAME = "Mesh and Reference (RED BG)"
VIEW_WIDTH = 1024
VIEW_HEIGHT = 768
VIEW_BACKGROUND = (1.0, 1.0, 1.0)

# Material for the mesh in the modern renderer
#   - terrain (no per-vertex colors) -> lit, uses base_color only
#   - colored path                    -> unlit, uses vertex colors as-is
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

# Vertex color gamma correction
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

    # Terrain should not use vertex colors; clear them
    terrain.vertex_colors = o3d.utility.Vector3dVector()

    return terrain, colored


def _configure_sun_light(o3dvis) -> None:
    """Configure sun light on the visualizer (intensity + enable flag)."""
    try:
        scene = getattr(o3dvis, "scene", None)
        if scene is None:
            return
        # scene is an Open3DScene; the low-level Scene is scene.scene
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


def _ensure_screenshot_dir() -> None:
    try:
        os.makedirs(SCREENSHOT_DIR, exist_ok=True)
    except Exception as e:
        print(f"Warning: failed to create screenshot directory '{SCREENSHOT_DIR}': {e}")


def _make_screenshot_path() -> str:
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    fname = f"{SCREENSHOT_BASENAME}_{ts}{SCREENSHOT_EXT}"
    return os.path.join(SCREENSHOT_DIR, fname)


def _save_screenshot_from_o3dvis(o3dvis) -> None:
    """
    Save current view of O3DVisualizer as PNG.

    Uses O3DVisualizer.export_current_image if available.
    Image resolution equals the current window size; increase VIEW_WIDTH/HEIGHT
    before launching if you want higher resolution.
    """
    _ensure_screenshot_dir()
    path = _make_screenshot_path()

    try:
        if hasattr(o3dvis, "export_current_image"):
            o3dvis.export_current_image(path)
            print(f"[Screenshot] Saved PNG to: {path}")
        else:
            # Fallback: use underlying Scene.render_to_image if reachable
            scene = getattr(o3dvis, "scene", None)
            low_scene = getattr(scene, "scene", None) if scene is not None else None
            if low_scene is None:
                print("Screenshot failed: no export_current_image and no low-level Scene")
                return

            def _on_image(image):
                try:
                    # PNG quality argument matches Open3D examples
                    o3d.io.write_image(path, image, 9)
                    print(f"[Screenshot] Saved PNG to: {path}")
                except Exception as e2:
                    print(f"Screenshot write failed: {e2}")

            low_scene.render_to_image(_on_image)
    except Exception as e:
        print(f"Screenshot failed: {e}")


def _save_screenshot_action(o3dvis) -> None:
    """Action callback used by the draw() 'actions' list."""
    _save_screenshot_from_o3dvis(o3dvis)


def draw_with_bg(
    geometries,
    *,
    window_name="Mesh and Reference",
    width=1024,
    height=768,
    bg=(1.0, 1.0, 1.0),
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

        colored_mat = rendering.MaterialRecord()
        colored_mat.shader = COLORED_SHADER
        colored_mat.base_color = COLORED_BASE_COLOR

        wrapped = []
        for idx, g in enumerate(geometries):
            if idx == 0 and len(geometries) == 2:
                mat = terrain_mat
                name = "terrain"
            elif idx == 1 and len(geometries) == 2:
                mat = colored_mat
                name = "path"
            else:
                # Fallback: treat as terrain
                mat = terrain_mat
                name = f"geom{idx}"
            wrapped.append(
                {
                    "name": name,
                    "geometry": g,
                    "material": mat,
                }
            )
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

    # Add actions (buttons) if supported
    if "actions" in params:
        kwargs.setdefault("actions", [])
        kwargs["actions"].append(("Toggle Sun Mouse Mode", _toggle_sun_mouse_mode))
        kwargs["actions"].append(("Save PNG Screenshot (P)", _save_screenshot_action))

    # Configure lighting + key binding via on_init if supported
    if "on_init" in params:

        def _on_init(o3dvis):
            # apply background again in case draw() didn't
            try:
                o3dvis.set_background(np.array([bg[0], bg[1], bg[2], 1.0], dtype=np.float32))
            except Exception:
                pass

            _configure_sun_light(o3dvis)

            # Install 'P' key screenshot handler if the API supports it
            try:
                import open3d.visualization.gui as gui

                # Some builds may not expose set_on_key on O3DVisualizer
                if hasattr(o3dvis, "set_on_key"):

                    def _on_key(event):
                        try:
                            if event.type == gui.KeyEvent.Type.DOWN and event.key == gui.KeyName.P:
                                _save_screenshot_from_o3dvis(o3dvis)
                                # Stop further handling
                                return True
                        except Exception as e:
                            print(f"Key handler failed: {e}")
                        # Let other handlers process the key
                        return False

                    o3dvis.set_on_key(_on_key)
                    print("Keybinding installed: press 'P' to save a PNG screenshot.")
                else:
                    print(
                        "Warning: this Open3D build does not expose 'set_on_key' on "
                        "O3DVisualizer; 'P' keybinding is unavailable. "
                        "Use the 'Save PNG Screenshot (P)' action button instead."
                    )
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


def main(argv=None) -> None:
    global SCREENSHOT_DIR
    import argparse

    parser = argparse.ArgumentParser(description=__doc__ or "View a triangle mesh")
    parser.add_argument("--mesh", required=True)
    parser.add_argument("--check-only", action="store_true", help="Validate geometry without opening a window")
    parser.add_argument("--screenshot-dir", default="screenshots")
    args = parser.parse_args(argv)
    SCREENSHOT_DIR = args.screenshot_dir
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

    terrain_mesh, colored_mesh = split_mesh_by_vertex_color(mesh)

    if colored_mesh is None:
        geoms = [mesh]
    else:
        geoms = [terrain_mesh, colored_mesh]

    try:
        nF = len(np.asarray(mesh.triangles))
        nUV = len(mesh.triangle_uvs) if hasattr(mesh, "triangle_uvs") else 0
        missing_uv_corners = 3 * nF - nUV
        print(f"Triangles: {nF} | UV corners: {nUV} | Missing UV corners: {missing_uv_corners}")

        list_available_shaders()

        draw_with_bg(
            geoms,
            window_name=VIEW_WINDOW_NAME,
            width=VIEW_WIDTH,
            height=VIEW_HEIGHT,
            bg=VIEW_BACKGROUND,
        )
    except Exception as e:
        raise RuntimeError("Visualization failed") from e

    print(f"Mesh has {len(mesh.vertices)} vertices and {len(mesh.triangles)} triangles")


if __name__ == "__main__":
    main()
