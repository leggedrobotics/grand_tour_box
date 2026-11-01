#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Fix inconsistent or flipped normals in a triangle mesh and optionally
fill small holes and duplicate faces to make the mesh two-sided.

Pipeline:
  1) Clean + orient (component/global/camera; inward/outward)
  2) (Optional) Fill small holes by centroid-fan triangulation (colors/UVs interpolated)
  3) (Optional) Make geometry two-sided (duplicate faces; optional vertex split)
"""

from __future__ import annotations
import argparse
from pathlib import Path
from box_auto.utils.atomic_file import atomic_output
import open3d as o3d
import numpy as np
from collections import defaultdict

###############################################################################
# Configuration

# Path to the input mesh file (PLY/OBJ/etc.). Edit this to point to your mesh.

# Path to save the processed mesh. Set to None to skip saving.

# Orientation strategy: 'component', 'global', or 'camera'.
#   * 'component' – orient each connected component outward from the mesh centroid
#   * 'global'    – orient the entire mesh based on its average normal direction
#   * 'camera'    – orient each component relative to a camera position
METHOD = "global"

# Camera position used when METHOD == 'camera'. Provide a 3-tuple/list of floats.
CAMERA_POSITION = (0.0, -2000.0, 5000.0)

# If True, orient normals toward the reference (centroid or camera) instead of away.
INWARD = True

# --- Simple small-hole filling (centroid fan) ---
FILL_SMALL_HOLES = False
# Only fill loops with at most this many edges (keep it small & robust).
MAX_HOLE_EDGES = 24  # small, robust fans
MAX_HOLE_PERIM_RATIO = 0.03  # perimeter ≤ 3% of bbox diagonal

# Make geometry two-sided by duplicating faces with reversed winding.
MAKE_TWO_SIDED = True
# If True, also duplicate vertices so backfaces have independent vertex normals.
SPLIT_VERTICES = True

###############################################################################


def _swap_winding_inplace(tris: np.ndarray, mask: np.ndarray | None = None) -> None:
    """Reverse the winding of triangles by swapping columns 1 and 2 in-place."""
    if mask is None:
        tmp = tris[:, 1].copy()
        tris[:, 1] = tris[:, 2]
        tris[:, 2] = tmp
        return
    idx = np.nonzero(mask)[0]
    if idx.size == 0:
        return
    tmp = tris[idx, 1].copy()
    tris[idx, 1] = tris[idx, 2]
    tris[idx, 2] = tmp


def load_and_clean(mesh_path: str) -> o3d.geometry.TriangleMesh:
    """Load a mesh and remove degenerate, duplicate, and non-manifold elements."""
    mesh = o3d.io.read_triangle_mesh(mesh_path)
    if mesh.is_empty():
        return mesh
    # mesh.remove_degenerate_triangles()
    # mesh.remove_duplicated_triangles()
    # mesh.remove_duplicated_vertices()
    # mesh.remove_unreferenced_vertices()
    # mesh.remove_non_manifold_edges()
    return mesh


def orient_components(mesh: o3d.geometry.TriangleMesh, reference: np.ndarray, *, inward: bool = False) -> None:
    """Orient connected components of a mesh relative to a reference point."""
    if mesh.is_empty():
        return

    mesh.orient_triangles()

    # Labels for each triangle's connected component (Open3D returns a tuple in newer versions)
    res = mesh.cluster_connected_triangles()
    labels = res[0] if isinstance(res, (list, tuple)) else res
    labels = np.asarray(labels, dtype=int)

    verts = np.asarray(mesh.vertices)
    tris = np.asarray(mesh.triangles)
    ref = np.asarray(reference, dtype=float).reshape(
        3,
    )
    new_tris = tris.copy()

    for cid in np.unique(labels):
        mask = labels == cid
        idx = np.nonzero(mask)[0]
        if idx.size == 0:
            continue

        tri_verts = verts[new_tris[idx]]  # (Nc, 3, 3)
        e1 = tri_verts[:, 1] - tri_verts[:, 0]
        e2 = tri_verts[:, 2] - tri_verts[:, 0]
        fn = np.cross(e1, e2)  # area-weighted normals
        fn_sum = fn.sum(axis=0)

        nrm = np.linalg.norm(fn_sum)
        if nrm < 1e-12:
            continue
        fn_avg = fn_sum / nrm

        tri_centers = tri_verts.mean(axis=1)
        comp_center = tri_centers.mean(axis=0)
        to_ref = ref - comp_center
        dot = float(np.dot(fn_avg, to_ref))

        flip = (dot < 0 and not inward) or (dot > 0 and inward)
        if flip:
            _swap_winding_inplace(new_tris, mask=mask)

    mesh.triangles = o3d.utility.Vector3iVector(new_tris.astype(np.int32, copy=False))
    mesh.compute_triangle_normals()
    mesh.compute_vertex_normals()


def orient_global(mesh: o3d.geometry.TriangleMesh, *, inward: bool = False) -> None:
    """Orient the whole mesh based on the average face-normal direction."""
    if mesh.is_empty():
        return

    mesh.orient_triangles()
    mesh.compute_triangle_normals()

    tris = np.asarray(mesh.triangles)
    verts = np.asarray(mesh.vertices)
    tri_centers = verts[tris].mean(axis=1)
    normals = np.asarray(mesh.triangle_normals)
    centroid = tri_centers.mean(axis=0)

    score = np.sign(((tri_centers - centroid) * normals).sum(axis=1)).mean()
    should_flip = (score < 0 and not inward) or (score > 0 and inward)
    if should_flip:
        new_tris = tris.copy()
        _swap_winding_inplace(new_tris)
        mesh.triangles = o3d.utility.Vector3iVector(new_tris.astype(np.int32, copy=False))
        mesh.compute_triangle_normals()
        mesh.compute_vertex_normals()


def fix_normals(
    mesh_path: str,
    *,
    method: str = "component",
    camera: tuple | list | None = None,
    inward: bool = False,
) -> o3d.geometry.TriangleMesh:
    """Load a mesh, orient normals according to the specified method, and return it."""
    mesh = load_and_clean(mesh_path)
    if mesh.is_empty():
        return mesh

    mesh.compute_triangle_normals()

    if method == "global":
        orient_global(mesh, inward=inward)
    elif method == "camera":
        if camera is None:
            raise ValueError("camera position must be provided when method='camera'")
        orient_components(mesh, np.asarray(camera, dtype=float), inward=inward)
    elif method == "component":
        # A closed component's area-weighted normals sum to zero, so an average
        # normal cannot choose its orientation. Signed volume resolves its winding.
        if not mesh.orient_triangles():
            raise ValueError("Mesh is not orientable")
        labels = np.asarray(mesh.cluster_connected_triangles()[0])
        tris = np.asarray(mesh.triangles).copy()
        verts = np.asarray(mesh.vertices)
        for label in np.unique(labels):
            mask = labels == label
            faces = tris[mask]
            edges = np.sort(np.concatenate((faces[:, [0, 1]], faces[:, [1, 2]], faces[:, [2, 0]])), axis=1)
            if not np.all(np.unique(edges, axis=0, return_counts=True)[1] == 2):
                raise ValueError(
                    "Component orientation requires closed surfaces; use --method camera for an open surface"
                )
            origin = verts[np.unique(faces)].mean(axis=0)
            points = verts[faces] - origin
            volume6 = np.einsum("ij,ij->i", points[:, 0], np.cross(points[:, 1], points[:, 2])).sum()
            if abs(volume6) < np.finfo(float).eps * max(np.ptp(points, axis=(0, 1)).max() ** 3, np.finfo(float).tiny):
                raise ValueError("Component has indeterminate signed volume")
            if (volume6 < 0) != inward:
                _swap_winding_inplace(tris, mask)
        mesh.triangles = o3d.utility.Vector3iVector(tris)
        mesh.compute_triangle_normals()
        mesh.compute_vertex_normals()
    else:
        raise ValueError(f"Unknown method: {method}")

    return mesh


###############################################################################
# Simple small-hole filler (centroid-fan + color/UV/material interpolation)


def _find_boundary_loops(V: np.ndarray, F: np.ndarray) -> list[list[int]]:
    """Return boundary loops as ordered lists of vertex indices (very simple)."""
    edge_faces = defaultdict(list)
    oriented = {}
    for fi, (a, b, c) in enumerate(F):
        for u, v in ((a, b), (b, c), (c, a)):
            k = (u, v)
            ku = (min(u, v), max(u, v))
            edge_faces[ku].append(fi)
            oriented[ku] = k  # last wins; for boundary edges there's only one

    # Undirected boundary edges (those used by exactly one face)
    boundary_edges = [oriented[k] for k, fl in edge_faces.items() if len(fl) == 1]
    if not boundary_edges:
        return []

    # Build undirected adjacency along boundary
    adj = defaultdict(list)
    for u, v in boundary_edges:
        if v not in adj[u]:
            adj[u].append(v)
        if u not in adj[v]:
            adj[v].append(u)

    # Extract loops
    visited_edges = set()
    loops = []
    for u, v in boundary_edges:
        ekey = (min(u, v), max(u, v))
        if ekey in visited_edges:
            continue
        # Start a loop
        start = u
        prev = None
        curr = u
        # Pick a neighbor to start
        if not adj[curr]:
            continue
        nxt = adj[curr][0]
        loop = [curr]
        while True:
            prev, curr = curr, nxt
            loop.append(curr)
            visited_edges.add((min(prev, curr), max(prev, curr)))
            # Choose next neighbor (not going back)
            nbrs = adj[curr]
            if len(nbrs) == 0:
                break  # open boundary (shouldn't happen for a hole)
            if len(nbrs) == 1:
                nxt = nbrs[0]
            else:
                nxt = nbrs[0] if nbrs[0] != prev else nbrs[1]
            if curr == start:
                break
            # Safety: avoid runaway
            if len(loop) > 10000:
                break
        if len(loop) >= 4 and loop[0] == loop[-1]:
            loops.append(loop[:-1])  # closed; drop duplicate last vertex
    return loops


def _newell_normal(P: np.ndarray, loop: list[int]) -> np.ndarray:
    """Newell's method for polygon normal from ordered loop vertices."""
    n = np.zeros(3, dtype=np.float64)
    L = len(loop)
    for i in range(L):
        p = P[loop[i]]
        q = P[loop[(i + 1) % L]]
        n += np.cross(p, q)
    nrm = np.linalg.norm(n)
    return n / nrm if nrm > 1e-20 else n


def _fill_small_holes_centroid_fan(mesh: o3d.geometry.TriangleMesh, max_edges: int, perim_ratio: float | None) -> None:
    """Fill small boundary loops with a centroid-fan; colors/UVs/materials interpolated."""
    V = np.asarray(mesh.vertices)
    F = np.asarray(mesh.triangles, dtype=np.int64)
    if V.size == 0 or F.size == 0:
        return

    # Precompute face normals for neighbor direction
    v0, v1, v2 = V[F[:, 0]], V[F[:, 1]], V[F[:, 2]]
    fn = np.cross(v1 - v0, v2 - v0)
    fn /= np.clip(np.linalg.norm(fn, axis=1, keepdims=True), 1e-20, None)

    loops = _find_boundary_loops(V, F)
    if not loops:
        return

    # Perimeter threshold
    perim_thresh = None
    if perim_ratio is not None:
        aabb = mesh.get_axis_aligned_bounding_box()
        diag = float(np.linalg.norm(np.asarray(aabb.get_extent(), dtype=float)))
        perim_thresh = perim_ratio * diag

    has_vc = len(mesh.vertex_colors) == len(V)
    VC = np.asarray(mesh.vertex_colors) if has_vc else None

    has_uv = hasattr(mesh, "triangle_uvs") and (len(mesh.triangle_uvs) == 3 * len(F))
    UV = np.asarray(mesh.triangle_uvs).reshape(-1, 3, 2) if has_uv else None

    has_mat = hasattr(mesh, "triangle_material_ids") and (len(mesh.triangle_material_ids) == len(F))
    TIDS = np.asarray(mesh.triangle_material_ids, dtype=np.int32) if has_mat else None

    # Map directed edge -> (face id, corner index of first, corner index of second)
    dir_edge2facecorner = {}
    for fi, (a, b, c) in enumerate(F):
        dir_edge2facecorner[(a, b)] = (fi, 0, 1)
        dir_edge2facecorner[(b, c)] = (fi, 1, 2)
        dir_edge2facecorner[(c, a)] = (fi, 2, 0)
        dir_edge2facecorner[(b, a)] = (fi, 1, 0)
        dir_edge2facecorner[(c, b)] = (fi, 2, 1)
        dir_edge2facecorner[(a, c)] = (fi, 0, 2)

    # Edge -> adjacent faces (undirected) to find neighbor normals/materials
    edge_faces = defaultdict(list)
    for fi, (a, b, c) in enumerate(F):
        edge_faces[tuple(sorted((a, b)))].append(fi)
        edge_faces[tuple(sorted((b, c)))].append(fi)
        edge_faces[tuple(sorted((c, a)))].append(fi)

    V_list = V.tolist()
    F_list = F.tolist()
    if has_vc:
        VC_list = VC.tolist()
    if has_uv:
        UV_list = UV.reshape(-1, 2).tolist()  # flatten to [ (u,v), ... ]
    if has_mat:
        TIDS_list = TIDS.tolist()

    new_tris = 0
    for loop in loops:
        L = len(loop)
        if L < 3 or L > max_edges:
            continue

        peri = float(np.sum(np.linalg.norm(V[loop] - V[np.roll(loop, -1)], axis=1)))
        if perim_thresh is not None and peri > perim_thresh:
            continue

        # Orient loop to match neighboring faces
        nb_normals = []
        for i in range(L):
            a, b = loop[i], loop[(i + 1) % L]
            ef = edge_faces.get(tuple(sorted((a, b))), [])
            if len(ef) == 1:
                nb_normals.append(fn[ef[0]])
        avg_nb = np.mean(nb_normals, axis=0) if nb_normals else None

        pn = _newell_normal(V, loop)
        if avg_nb is not None and np.dot(pn, avg_nb) < 0:
            loop = list(reversed(loop))
            pn = -pn

        # New centroid vertex (pos, color)
        cpos = V[loop].mean(axis=0)
        cidx = len(V_list)
        V_list.append(cpos.tolist())
        if has_vc:
            ccol = VC[loop].mean(axis=0)
            VC_list.append(ccol.tolist())

        # Centroid UV (mean of per-vertex UVs sampled from outgoing edges)
        if has_uv:
            loop_uv = []
            for i in range(L):
                a, b = loop[i], loop[(i + 1) % L]
                if (a, b) in dir_edge2facecorner:
                    fi, ia, ib = dir_edge2facecorner[(a, b)]
                else:
                    fi, ib, ia = dir_edge2facecorner[(b, a)]
                uva = UV[fi, ia]
                loop_uv.append(uva)
            cuv = np.mean(np.stack(loop_uv, axis=0), axis=0)

        # Triangulate as a fan (a, b, cidx)
        for i in range(L):
            a, b = loop[i], loop[(i + 1) % L]
            F_list.append([int(a), int(b), int(cidx)])
            new_tris += 1

            if has_uv:
                if (a, b) in dir_edge2facecorner:
                    fi, ia, ib = dir_edge2facecorner[(a, b)]
                else:
                    fi, ib, ia = dir_edge2facecorner[(b, a)]
                uva = UV[fi, ia]
                uvb = UV[fi, ib]
                UV_list.extend([uva.tolist(), uvb.tolist(), cuv.tolist()])

            if has_mat:
                ef = edge_faces.get(tuple(sorted((a, b))), [])
                tid = TIDS[ef[0]] if (ef and len(ef) >= 1) else 0
                TIDS_list.append(int(tid))

    if new_tris == 0:
        return

    # Commit
    mesh.vertices = o3d.utility.Vector3dVector(np.asarray(V_list, dtype=float))
    mesh.triangles = o3d.utility.Vector3iVector(np.asarray(F_list, dtype=np.int32))
    if has_vc:
        mesh.vertex_colors = o3d.utility.Vector3dVector(np.asarray(VC_list, dtype=float))
    if has_uv:
        mesh.triangle_uvs = o3d.utility.Vector2dVector(np.asarray(UV_list, dtype=float))
    if has_mat:
        mesh.triangle_material_ids = o3d.utility.IntVector(np.asarray(TIDS_list, dtype=np.int32))

    # Recompute normals
    mesh.compute_triangle_normals()
    mesh.compute_vertex_normals()


###############################################################################
# Two-sided duplication


def make_two_sided(mesh: o3d.geometry.TriangleMesh, *, split_vertices: bool = True) -> None:
    """Duplicate faces (and optionally vertices) so the mesh becomes two-sided."""
    if mesh.is_empty():
        return

    V = np.asarray(mesh.vertices)
    F = np.asarray(mesh.triangles)

    if V.size == 0 or F.size == 0:
        return

    F_back = F[:, [0, 2, 1]]

    if split_vertices:
        nV = len(V)
        V2 = V.copy()
        mesh.vertices = o3d.utility.Vector3dVector(np.vstack([V, V2]))

        if len(mesh.vertex_colors) == len(V):
            VC = np.asarray(mesh.vertex_colors)
            mesh.vertex_colors = o3d.utility.Vector3dVector(np.vstack([VC, VC.copy()]))

        if len(mesh.vertex_normals) == len(V):
            VN = np.asarray(mesh.vertex_normals)
            mesh.vertex_normals = o3d.utility.Vector3dVector(np.vstack([VN, VN.copy()]))

        F_back = F_back + nV

    mesh.triangles = o3d.utility.Vector3iVector(np.vstack([F, F_back]).astype(np.int32, copy=False))

    # Per-corner triangle UVs (if present)
    try:
        if len(mesh.triangle_uvs) == 3 * len(F):
            UV = np.asarray(mesh.triangle_uvs).reshape(-1, 3, 2)
            UV_back = UV[:, [0, 2, 1], :].reshape(-1, 2)
            mesh.triangle_uvs = o3d.utility.Vector2dVector(np.vstack([UV.reshape(-1, 2), UV_back]))
    except Exception:
        pass

    if hasattr(mesh, "triangle_material_ids") and len(mesh.triangle_material_ids) == len(F):
        tids = np.asarray(mesh.triangle_material_ids)
        mesh.triangle_material_ids = o3d.utility.IntVector(np.hstack([tids, tids]))

    mesh.compute_triangle_normals()
    mesh.compute_vertex_normals()


###############################################################################
# Viewer with red background (to distinguish holes vs untextured faces)


def draw_with_bg(geometries, *, window_name="Mesh and Reference", width=1024, height=768, bg=(1.0, 1.0, 1.0)) -> None:
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


###############################################################################
# Main


def main(argv=None) -> None:
    parser = argparse.ArgumentParser(description="Orient mesh normals with explicit geometry edits")
    parser.add_argument("input", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--method", choices=("component", "global", "camera"), required=True)
    parser.add_argument("--camera", nargs=3, type=float)
    parser.add_argument("--inward", action="store_true")
    parser.add_argument("--fill-small-holes", action="store_true")
    parser.add_argument("--two-sided", action="store_true")
    parser.add_argument("--visualize", action="store_true")
    args = parser.parse_args(argv)
    if args.input.resolve() == args.output.resolve() or args.output.exists():
        parser.error("output must be a new path distinct from input")
    if args.output.suffix.lower() != ".ply":
        parser.error("output must be PLY for atomic single-file publication")
    if args.method == "camera" and (args.camera is None or not np.isfinite(args.camera).all()):
        parser.error("camera method requires a finite --camera x y z")
    # A chosen reference resolves sign ambiguity; it does not establish the
    # physical inside/outside of an open or concave reconstructed surface.
    mesh = fix_normals(str(args.input), method=args.method, camera=args.camera, inward=args.inward)
    if mesh.is_empty() or not np.isfinite(np.asarray(mesh.vertices)).all():
        raise ValueError("Mesh is empty or contains invalid vertices")
    if mesh.has_triangle_uvs() or mesh.textures:
        raise ValueError("PLY cannot preserve mesh textures/UVs; use a texture-aware mesh pipeline")
    if args.fill_small_holes:
        _fill_small_holes_centroid_fan(mesh, MAX_HOLE_EDGES, MAX_HOLE_PERIM_RATIO)
    if args.two_sided:
        make_two_sided(mesh, split_vertices=True)
    mesh.compute_triangle_normals()
    mesh.compute_vertex_normals()
    if not np.isfinite(np.asarray(mesh.vertex_normals)).all():
        raise ValueError("Mesh has nonfinite normals")
    with atomic_output(args.output, overwrite=False) as temporary:
        encoded = temporary.with_suffix(".ply")
        if not o3d.io.write_triangle_mesh(str(encoded), mesh):
            raise OSError("Failed to write mesh")
        encoded.replace(temporary)
    if args.visualize:
        draw_with_bg([mesh])
    print(f"Saved {len(mesh.vertices)} vertices and {len(mesh.triangles)} triangles to {args.output}")


if __name__ == "__main__":
    main()
