#!/usr/bin/env python3
"""
CUPOCH normal estimation with chunking + halo to avoid CUDA OOM.
Includes optional RGB uniformity improvement via bilateral color smoothing
with a neighbor-consistency gate.
"""

import argparse
from pathlib import Path
from box_auto.utils.atomic_file import atomic_output
import sys
import math
import numpy as np
import cupoch as cph

# --------- hard-coded params ----------
INPUT_PLY = "/home/tutuna/fast_livo_ws/src/FAST-LIVO2/Log/PCD/2ndmuseum_dense_normals_small_cut.ply"
OUTPUT_PLY = "/home/tutuna/fast_livo_ws/src/FAST-LIVO2/Log/PCD/2ndmuseum_dense_normals_small_cut_normals.ply"

# Neighborhood params (normals)
RADIUS = 0.2
MAX_NN = 40

# Chunking params
VOXEL_SIZE_FOR_GRID = RADIUS
BLOCK_CELLS = 16
HALO_CELLS = 1
MAX_CHUNK_POINTS = 15_200_000

# Behavior toggles
USE_KNN_SEARCH = False
DO_ORIENT = True
WRITE_ASCII = True
COMPRESS_PLY = True

# Visualization
POINT_SIZE = 1.0  # default point size

# --------- Color smoothing (RGB uniformity) ----------
DO_COLOR_SMOOTH = False  # enable/disable
COLOR_USE_KNN = False  # True: knn; False: radius search
COLOR_MAX_NN = 30  # neighbors per query (if KNN)
COLOR_RADIUS = 0.15  # neighbors radius (if radius search)
COLOR_SIGMA_SPATIAL = 0.075  # spatial sigma (meters) for bilateral
COLOR_SIGMA_COLOR = 0.08  # color sigma in linear RGB [0..1]
COLOR_TAU = 0.18  # gate: replace only if ||ci - μ_neigh||_2 > tau
COLOR_SMOOTH_ITERS = 1  # iterations of smoothing
LINEARIZE_SRGB = True  # sRGB <-> linear for better averaging
# -----------------------------------------------------


def _make_search_param(radius: float, max_nn: int, use_knn: bool):
    geo = cph.geometry
    if use_knn and hasattr(geo, "KDTreeSearchParamKNN"):
        return geo.KDTreeSearchParamKNN(knn=max_nn)
    if hasattr(geo, "KDTreeSearchParamRadius"):
        try:
            return geo.KDTreeSearchParamRadius(radius=radius, max_nn=max_nn)
        except TypeError:
            return geo.KDTreeSearchParamRadius(radius=radius)
    if hasattr(geo, "KDTreeSearchParam"):
        try:
            return geo.KDTreeSearchParam(radius=radius, max_nn=max_nn)
        except TypeError as cause:
            raise RuntimeError("This Cupoch build cannot provide bounded radius neighborhoods") from cause
    raise RuntimeError("No KDTreeSearchParam* found in this CUPOCH build.")


def _estimate_chunk_normals_gpu_adaptive(pts_chunk: np.ndarray, camera_location: np.ndarray):
    sub = cph.geometry.PointCloud()
    sub.points = cph.utility.Vector3fVector(pts_chunk.astype(np.float32, copy=False))

    # A fixed-radius neighborhood is bounded by the halo. Shrinking the
    # neighborhood after OOM would silently change the estimator's geometry.
    sub.estimate_normals(_make_search_param(RADIUS, int(MAX_NN), USE_KNN_SEARCH))

    # normals → CPU
    try:
        normals_np = np.asarray(sub.normals.cpu(), dtype=np.float32)
    except Exception:
        normals_np = np.asarray(sub.normals, dtype=np.float32)

    invalid = ~np.isfinite(normals_np).all(axis=1) | (np.linalg.norm(normals_np, axis=1) < 1e-6)
    if invalid.any():
        # Cupoch can produce NaNs on exactly planar neighborhoods. Recompute
        # those rows with the existing Open3D radius estimator, retaining the
        # same spatial support and avoiding arbitrary replacement directions.
        import open3d as o3d

        cpu = o3d.geometry.PointCloud()
        cpu.points = o3d.utility.Vector3dVector(pts_chunk.astype(np.float64))
        cpu.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=RADIUS, max_nn=int(MAX_NN)))
        normals_np[invalid] = np.asarray(cpu.normals)[invalid]

    # Flip normals toward camera
    if DO_ORIENT and normals_np.size:
        vec = camera_location.astype(np.float32)[None, :] - pts_chunk.astype(np.float32)
        dot = (normals_np * vec).sum(axis=1)
        normals_np[dot < 0.0] *= -1.0

    del sub
    return normals_np


# -------- sRGB <-> linear helpers --------
def _srgb_to_linear(c):
    a = 0.055
    return np.where(c <= 0.04045, c / 12.92, ((c + a) / (1 + a)) ** 2.4)


def _linear_to_srgb(c):
    a = 0.055
    return np.where(c <= 0.0031308, 12.92 * c, (1 + a) * (c ** (1 / 2.4)) - a)


# ----------------------------------------


def _kdt_search(kdt, p32: np.ndarray, use_knn: bool, max_nn: int, radius: float):
    """
    Compatibility layer across Cupoch/Open3D variants:
    prefers *3f methods, falls back to *3d if available.
    Returns (idx_np, dist2_np) truncated to k.
    """
    if use_knn:
        if hasattr(kdt, "search_knn_vector_3f"):
            k, idx, dist2 = kdt.search_knn_vector_3f(p32, int(max_nn))
        elif hasattr(kdt, "search_knn_vector_3d"):
            k, idx, dist2 = kdt.search_knn_vector_3d(p32.astype(np.float64), int(max_nn))
        else:
            raise AttributeError("KDTreeFlann has neither search_knn_vector_3f nor _3d")
    else:
        if hasattr(kdt, "search_radius_vector_3f"):
            k, idx, dist2 = kdt.search_radius_vector_3f(p32, float(radius))
        elif hasattr(kdt, "search_radius_vector_3d"):
            k, idx, dist2 = kdt.search_radius_vector_3d(p32.astype(np.float64), float(radius))
        else:
            raise AttributeError("KDTreeFlann has neither search_radius_vector_3f nor _3d")

    if k <= 0:
        return np.empty((0,), dtype=np.int64), np.empty((0,), dtype=np.float32)
    idx = np.asarray(idx[:k], dtype=np.int64)
    dist2 = np.asarray(dist2[:k], dtype=np.float32)
    return idx, dist2


def _bilateral_color_chunk(
    pts_chunk: np.ndarray,
    colors_chunk: np.ndarray,
    query_local_indices: np.ndarray,
    use_knn: bool,
    max_nn: int,
    radius: float,
    sigma_spatial: float,
    sigma_color: float,
    tau_gate: float,
    iters: int,
) -> np.ndarray:
    """
    Edge-preserving bilateral smoothing of colors within one chunk.
    Only updates colors for points specified by query_local_indices (inner/core).
    """
    assert pts_chunk.shape[0] == colors_chunk.shape[0]
    sub_pcd = cph.geometry.PointCloud()
    sub_pcd.points = cph.utility.Vector3fVector(pts_chunk.astype(np.float32, copy=False))
    kdt = cph.geometry.KDTreeFlann(sub_pcd)

    out = colors_chunk.copy()
    eps = 1e-12

    for _ in range(max(1, iters)):
        new_colors = out.copy()
        for li in query_local_indices:  # local index in this sub-cloud
            p32 = pts_chunk[li].astype(np.float32)
            c0 = out[li]

            idx, dist2 = _kdt_search(kdt, p32, use_knn, max_nn, radius)
            if idx.size == 0:
                continue

            cj = out[idx]
            # spatial weights
            ws = np.exp(-dist2 / (2.0 * (sigma_spatial**2) + eps))
            # color weights (use current colors)
            dc2 = np.sum((cj - c0) ** 2, axis=1)
            wc = np.exp(-dc2 / (2.0 * (sigma_color**2) + eps))

            w = ws * wc + eps
            c_mean = (w[:, None] * cj).sum(axis=0) / np.sum(w)

            # neighbor-consistency gate
            if np.linalg.norm(c0 - c_mean) > tau_gate:
                new_colors[li] = c_mean  # replace only if it doesn't "fit"
            else:
                # small nudging to reduce noise while preserving edges
                new_colors[li] = 0.5 * c0 + 0.5 * c_mean

        out = new_colors

    return out


def _visualize(out_pcd, point_size: float):
    vis_mod = cph.visualization

    # Preferred if available: key callbacks (']' increase, '[' decrease, '0' apply default)
    if hasattr(vis_mod, "draw_geometries_with_key_callbacks"):

        def _inc(vis):
            ro = vis.get_render_option()
            ro.point_size = min(ro.point_size + 1.0, 50.0)
            vis.update_renderer()
            return False

        def _dec(vis):
            ro = vis.get_render_option()
            ro.point_size = max(ro.point_size - 1.0, 1.0)
            vis.update_renderer()
            return False

        def _set_default(vis):
            ro = vis.get_render_option()
            ro.point_size = float(point_size)
            vis.update_renderer()
            return False

        callbacks = {ord("]"): _inc, ord("["): _dec, ord("0"): _set_default}
        try:
            vis_mod.draw_geometries_with_key_callbacks([out_pcd], callbacks, window_name="Normals+Colors (Cupoch)")
            return
        except Exception:
            pass  # fall through to plain Visualizer

    # Fallback: plain Visualizer (no hotkeys), but sets initial size.
    vis = vis_mod.Visualizer()
    vis.create_window(window_name="Normals+Colors (Cupoch)")
    vis.add_geometry(out_pcd)
    ro = vis.get_render_option()
    ro.point_size = float(point_size)
    vis.run()
    vis.destroy_window()


def main(argv=None):
    global INPUT_PLY, OUTPUT_PLY, RADIUS, VOXEL_SIZE_FOR_GRID, DO_COLOR_SMOOTH, DO_ORIENT
    parser = argparse.ArgumentParser(description="Estimate normals with bounded spatial neighborhoods")
    parser.add_argument("input", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--radius", type=float, default=RADIUS)
    parser.add_argument("--smooth-colors", action="store_true")
    parser.add_argument("--camera", nargs=3, type=float, help="Orient normals toward this explicit viewpoint")
    parser.add_argument("--visualize", action="store_true")
    args = parser.parse_args(argv)
    if args.input.resolve() == args.output.resolve() or args.output.exists():
        parser.error("output must be a new path distinct from input")
    if args.output.suffix.lower() != ".ply":
        parser.error("output must be PLY")
    if not math.isfinite(args.radius) or args.radius <= 0:
        parser.error("radius must be positive and finite")
    INPUT_PLY, OUTPUT_PLY = str(args.input), str(args.output)
    RADIUS = VOXEL_SIZE_FOR_GRID = args.radius
    DO_COLOR_SMOOTH, DO_ORIENT = args.smooth_colors, args.camera is not None
    if args.camera and not np.isfinite(args.camera).all():
        parser.error("camera must be finite")
    # The halo spans neighboring blocks, not individual grid cells.
    if DO_COLOR_SMOOTH and COLOR_RADIUS > HALO_CELLS * BLOCK_CELLS * RADIUS:
        parser.error("color radius exceeds the halo support")
    pcd = cph.io.read_point_cloud(
        INPUT_PLY,
        remove_infinite_points=False,
        remove_nan_points=False,
        print_progress=True,
    )
    if pcd.is_empty():
        print(f"Error: input has no points: {INPUT_PLY}", file=sys.stderr)
        sys.exit(1)

    # Points → numpy
    try:
        pts = np.asarray(pcd.points.cpu(), dtype=np.float32)
    except Exception:
        pts = np.asarray(pcd.points, dtype=np.float32)

    if not np.isfinite(pts).all():
        raise ValueError("Input contains nonfinite points; clean explicitly before estimating normals")
    N = pts.shape[0]
    if N == 0:
        print("No points after cleaning.", file=sys.stderr)
        sys.exit(1)

    # Colors → numpy (if present)
    colors_np = None
    try:
        if len(pcd.colors) == N:
            try:
                colors_np = np.asarray(pcd.colors.cpu(), dtype=np.float32)
            except Exception:
                colors_np = np.asarray(pcd.colors, dtype=np.float32)
            if colors_np.max() > 1.5:
                colors_np = colors_np / 255.0
            colors_np = np.clip(colors_np, 0.0, 1.0)
            if DO_COLOR_SMOOTH and LINEARIZE_SRGB:
                colors_np = _srgb_to_linear(colors_np)
    except Exception:
        colors_np = None  # no colors or mismatch

    # Camera for sign disambiguation
    mins = pts.min(axis=0)
    camera_location = np.array(args.camera if args.camera else [0, 0, 0], dtype=np.float32)

    # Chunking indices
    cell = max(1e-6, VOXEL_SIZE_FOR_GRID)
    idxf = np.floor((pts - mins) / cell).astype(np.int64)
    cx, cy, cz = idxf[:, 0], idxf[:, 1], idxf[:, 2]
    bx = cx // BLOCK_CELLS
    by = cy // BLOCK_CELLS
    bz = cz // BLOCK_CELLS

    from collections import defaultdict

    block_to_indices = defaultdict(list)
    for i in range(N):
        block_to_indices[(bx[i], by[i], bz[i])].append(i)

    normals_out = np.zeros((N, 3), dtype=np.float32)
    colors_out = colors_np.copy() if colors_np is not None else None

    total_blocks = len(block_to_indices)
    processed = 0
    for key, core_idx_list in block_to_indices.items():
        processed += 1
        bx0, by0, bz0 = key

        core_idx = np.asarray(core_idx_list, dtype=np.int64)
        if core_idx.size == 0:
            continue

        # Halo selection
        mask_halo = (
            (bx >= bx0 - HALO_CELLS)
            & (bx <= bx0 + HALO_CELLS)
            & (by >= by0 - HALO_CELLS)
            & (by <= by0 + HALO_CELLS)
            & (bz >= bz0 - HALO_CELLS)
            & (bz <= bz0 + HALO_CELLS)
        )
        cand_idx = np.nonzero(mask_halo)[0]

        if cand_idx.size > MAX_CHUNK_POINTS:
            raise MemoryError("Chunk exceeds MAX_CHUNK_POINTS; reduce block size while retaining its radius halo")
        else:
            inner_idx_global = core_idx
            loc = {int(ii): k for k, ii in enumerate(cand_idx.tolist())}
            inner_local = np.fromiter((loc[int(ii)] for ii in inner_idx_global.tolist()), dtype=np.int64)

            # Normals
            normals_chunk = _estimate_chunk_normals_gpu_adaptive(pts[cand_idx], camera_location)
            normals_out[inner_idx_global] = normals_chunk[inner_local]

            # Colors (optional)
            if DO_COLOR_SMOOTH and colors_out is not None:
                smoothed = _bilateral_color_chunk(
                    pts_chunk=pts[cand_idx],
                    colors_chunk=colors_out[cand_idx],
                    query_local_indices=inner_local,
                    use_knn=COLOR_USE_KNN,
                    max_nn=COLOR_MAX_NN,
                    radius=COLOR_RADIUS,
                    sigma_spatial=COLOR_SIGMA_SPATIAL,
                    sigma_color=COLOR_SIGMA_COLOR,
                    tau_gate=COLOR_TAU,
                    iters=COLOR_SMOOTH_ITERS,
                )
                colors_out[inner_idx_global] = smoothed[inner_local]

        if processed % 10 == 0 or processed == total_blocks:
            print(f"Processed blocks: {processed}/{total_blocks} ({processed / total_blocks * 100:.1f}%)")

    lengths = np.linalg.norm(normals_out, axis=1)
    if not np.isfinite(normals_out).all() or np.any(lengths < 1e-6):
        raise ValueError("Normal estimation produced invalid or zero normals")
    normals_out /= lengths[:, None]

    # Assemble output point cloud (keep colors if present)
    out_pcd = cph.geometry.PointCloud()
    out_pcd.points = cph.utility.Vector3fVector(pts.astype(np.float32, copy=False))
    out_pcd.normals = cph.utility.Vector3fVector(normals_out)

    if colors_out is not None:
        if DO_COLOR_SMOOTH and LINEARIZE_SRGB:
            colors_write = _linear_to_srgb(np.clip(colors_out, 0.0, 1.0))
        else:
            colors_write = np.clip(colors_out, 0.0, 1.0)
        out_pcd.colors = cph.utility.Vector3fVector(colors_write.astype(np.float32, copy=False))

    with atomic_output(OUTPUT_PLY, overwrite=False) as temporary:
        encoded = temporary.with_suffix(".ply")
        ok = cph.io.write_point_cloud(str(encoded), out_pcd, write_ascii=False, compressed=False, print_progress=True)
        if not ok:
            raise OSError("Failed to write normal-estimated point cloud")
        encoded.replace(temporary)

    print(
        f"Saved: {OUTPUT_PLY} with {N} points, normals, and "
        f"{'colors' if colors_out is not None else 'no colors'} ("
        f"{'smoothed' if DO_COLOR_SMOOTH and colors_out is not None else 'raw'})."
    )

    # ---- Visualization (Cupoch) ----
    if args.visualize:
        _visualize(out_pcd, POINT_SIZE)


if __name__ == "__main__":
    main()
