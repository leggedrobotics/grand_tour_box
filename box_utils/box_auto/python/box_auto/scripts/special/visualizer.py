#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Minimal Open3D viewer that reproduces the visualization settings
from the fix_normals script: same Visualizer, background, backface
setting, and normals computation state.
"""

import open3d as o3d
import numpy as np

# ---------------------------------------------------------------------
# Hard-coded parameters
# ---------------------------------------------------------------------

# Path to the mesh to visualize (.ply/.obj/etc.)


# Window & background settings (identical to the original call)
VIEW_WINDOW_NAME = "Mesh and Reference (RED BG)"
VIEW_WIDTH = 1024
VIEW_HEIGHT = 768
VIEW_BACKGROUND = (1.0, 1.0, 1.0)


def draw_with_bg(
    geometries,
    *,
    window_name="Mesh and Reference",
    width=1024,
    height=768,
    bg=(1.0, 1.0, 1.0),
) -> None:
    """Show geometries with a solid background color (default: RED)."""
    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name=window_name, width=width, height=height)
    for g in geometries:
        vis.add_geometry(g)
    opt = vis.get_render_option()
    opt.background_color = np.asarray(bg, dtype=np.float32)
    opt.mesh_show_back_face = True
    vis.run()
    vis.destroy_window()


def main(argv=None) -> None:
    import argparse

    parser = argparse.ArgumentParser(description=__doc__ or "View a triangle mesh")
    parser.add_argument("--mesh", required=True)
    parser.add_argument("--check-only", action="store_true", help="Validate geometry without opening a window")
    args = parser.parse_args(argv)
    # Load mesh
    mesh = o3d.io.read_triangle_mesh(args.mesh)
    if mesh.is_empty():
        raise ValueError(f"Mesh is empty or failed to load: {args.mesh}")
    if not mesh.has_triangles() or not np.isfinite(np.asarray(mesh.vertices)).all():
        raise ValueError("Expected finite vertices and triangle faces")
    if args.check_only:
        print(f"Validated {len(mesh.vertices)} vertices and {len(mesh.triangles)} triangles")
        return

    # Match the "state" before visualization in your original script:
    # triangle + vertex normals are computed at that point.
    try:
        mesh.compute_triangle_normals()
        mesh.compute_vertex_normals()
    except Exception as e:
        print(f"Normal computation failed: {e}")

    # UV stats and visualization, identical logic to the original script
    try:
        nF = len(np.asarray(mesh.triangles))
        nUV = len(mesh.triangle_uvs) if hasattr(mesh, "triangle_uvs") else 0
        missing_uv_corners = 3 * nF - nUV
        print(f"Triangles: {nF} | UV corners: {nUV} | Missing UV corners: {missing_uv_corners}")

        draw_with_bg(
            [mesh],
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
