#!/usr/bin/env python3
"""Visualize a BIEVR-LIO map export.

Works on either the exported .pcd or the native .bumpmap (via load_bumpmap.py).

The interesting view is ``--mode quads`` (the default for a .bumpmap): every
observed bump-image pixel is drawn as a real ``px_size``-wide square lying in its
voxel's oriented plane, raised by the pixel's bump height — i.e. the
representation shown in ``doc/bumpimage.png``, not a point cloud sampled from it.

The stored heights are **not comparable between voxels**: each is signed along
its own plane normal, whose direction the fit picks arbitrarily (48 % of adjacent
near-parallel voxels disagree), and measured from a plane at an arbitrary offset.
Colouring by them paints one flat colour per 0.5 m voxel — the blue/green/red
patchwork across a continuous floor. ``--color bump`` therefore demeans each
voxel and flips signs to agree with neighbours; ``--color bump_raw`` shows the
unnormalized file contents.

Usage (the 3-D modes need Open3D — use ``../viz_venv/bin/python``):
    P=~/repos/slam_rnd/viz_venv/bin/python
    $P scripts/view_map.py <map.bumpmap>                       # oriented pixel squares
    $P scripts/view_map.py <map.bumpmap> --crop 30,-45,6,10    # a readable 10 m ball
    $P scripts/view_map.py <map.bumpmap> --color height        # colour by world z
    $P scripts/view_map.py <map.bumpmap> --mode points         # old sampled cloud
    $P scripts/view_map.py <map.pcd>                           # cloud (pcd has no planes)
    # raw per-voxel bump images as 2-D heatmaps:
    $P scripts/view_map.py <map.bumpmap> --mode patches --patches 25
    # dump the quad mesh for MeshLab/CloudCompare instead of opening a window:
    $P scripts/view_map.py <map.bumpmap> --save /tmp/map_quads.ply

The pixels are ``px_size`` (5 cm) across, so a whole km-scale drift is a hairline
when zoomed out — ``--crop X,Y,Z,R`` is the mode that shows the representation.
Big maps are also capped at ``--max-quads`` (default 2 M) by dropping whole
voxels; ``--stride 1 --max-quads 0`` draws everything.

Controls in the Open3D window: drag to rotate, scroll to zoom, [ ] to size points.
"""
import argparse
import itertools
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from load_bumpmap import (  # noqa: E402
    _invert_3x4,
    iter_bumpmap,
    load_bumpmap,
    read_header,
    reconstruct_points,
)

# Corner offsets of one pixel square in the voxel's local plane, CCW about +z.
_CORNERS = np.array([[-0.5, -0.5], [0.5, -0.5], [0.5, 0.5], [-0.5, 0.5]])

# Arbitrary generic direction used to seed each connected component's normal
# orientation. Deliberately not an axis, so no common surface sits at exactly 90
# deg to it and flips on floating-point noise.
_ORIENT_REF = np.array([0.31, 0.43, 0.85]) / np.linalg.norm([0.31, 0.43, 0.85])


def read_pcd_xyzi(path):
    """Minimal reader for the binary PCD written by exportMap (x y z intensity, float32)."""
    with open(path, "rb") as f:
        header, n = [], 0
        while True:
            line = f.readline().decode("ascii", "replace")
            header.append(line)
            if line.startswith("POINTS"):
                n = int(line.split()[1])
            if line.startswith("DATA"):
                if "binary" not in line:
                    raise SystemExit("Only DATA binary PCDs are supported by this quick reader.")
                break
        data = np.frombuffer(f.read(n * 16), dtype=np.float32).reshape(n, 4)
    return data[:, :3].astype(np.float64), data[:, 3].astype(np.float64)


def _voxel_quads(voxel, px_size, image_key, shrink):
    """One voxel -> (vertices (4M,3) world, bump height (M,), weight (M,)).

    Each observed pixel becomes a square of side ``px_size * shrink`` centred on
    the same 3-D point ``reconstruct_points`` would emit, lying in the voxel's
    plane (local xy) at the pixel's bump height (local z).
    """
    w = voxel["bump_weights"]
    img = voxel[image_key]
    ii, jj = np.nonzero(w > 0)
    if ii.size == 0:
        return None

    centre = np.empty((ii.size, 3), dtype=np.float64)
    centre[:, 0] = jj * px_size
    centre[:, 1] = ii * px_size
    centre[:, 2] = img[ii, jj]

    v = np.repeat(centre[:, None, :], 4, axis=1)  # (M, 4, 3)
    v[:, :, 0] += _CORNERS[:, 0] * px_size * shrink
    v[:, :, 1] += _CORNERS[:, 1] * px_size * shrink

    T_W_C = _invert_3x4(voxel["T_C_W"])
    verts = v.reshape(-1, 3) @ T_W_C[:, :3].T + T_W_C[:, 3]
    return verts, centre[:, 2], w[ii, jj]


def _candidates(path, header, crop):
    """Positions of the voxels eligible for drawing, in file order.

    Without a crop that's every voxel; with one it's those whose centroid is
    within ``r`` of ``(x, y, z)``, found by a geometry-only pass (bump images
    are seeked over, not decoded).
    """
    if crop is None:
        return np.arange(header["n_voxels"])
    centre, radius = np.asarray(crop[:3], dtype=np.float64), float(crop[3])
    _, it = iter_bumpmap(path, include_images=False)
    keep = [v["position"] for v in it
            if np.linalg.norm(v["centroid"] - centre) <= radius]
    if not keep:
        raise SystemExit(f"{path}: no voxel centroid within {radius} m of {centre.tolist()}.")
    return np.asarray(keep)


def _pick_stride(path, max_quads, stride, candidates, n_sample=256):
    """Stride over ``candidates`` that keeps the mesh under ``max_quads`` squares.

    Bump images are mostly empty, so rows*cols is far too loose a bound. Sample
    a spread of candidate voxels for the mean observed pixel count and
    extrapolate.
    """
    if stride is not None:
        return stride
    if max_quads <= 0 or candidates.size == 0:
        return 1
    sample = candidates[:: max(1, candidates.size // n_sample)]
    _, it = iter_bumpmap(path, include_images=True, wanted_positions=set(sample.tolist()))
    counts = [int(np.count_nonzero(v["bump_weights"] > 0)) for v in it]
    if not counts:
        return 1
    total = float(np.mean(counts)) * candidates.size
    return max(1, math.ceil(total / max_quads))


def _orient_voxels(indices, normals, min_dot=0.7):
    """Per-voxel sign (+1/-1) making neighbouring plane normals agree.

    A voxel's bump height is signed along *its own* plane normal, and the fit
    picks that normal's direction arbitrarily — 48 % of adjacent near-parallel
    voxel pairs disagree — so the same physical bump reads +2 cm in one voxel
    and -2 cm in the next. Flood-fill the voxel lattice, flipping a neighbour
    whose normal opposes an already-oriented one (only when the two are within
    ~45 deg, so a genuinely different surface is left alone). Each component's
    seed is oriented against a fixed generic reference, so voxels the flood fill
    never connects (isolated, or a strided draw) still agree when they face the
    same way.
    """
    where = {tuple(idx): i for i, idx in enumerate(indices)}
    offsets = [o for o in itertools.product((-1, 0, 1), repeat=3) if o != (0, 0, 0)]
    sign = np.zeros(len(indices))
    for start in range(len(indices)):
        if sign[start]:
            continue
        sign[start] = 1.0 if normals[start] @ _ORIENT_REF >= 0 else -1.0
        stack = [start]
        while stack:
            i = stack.pop()
            ix, iy, iz = indices[i]
            for dx, dy, dz in offsets:
                j = where.get((ix + dx, iy + dy, iz + dz))
                if j is None or sign[j]:
                    continue
                dot = float(normals[i] @ normals[j])
                if abs(dot) < min_dot:
                    continue
                sign[j] = sign[i] if dot > 0 else -sign[i]
                stack.append(j)
    return sign


def build_quad_mesh(path, image_key="bump_img", max_quads=2_000_000, stride=None,
                    shrink=1.0, crop=None):
    """Build the oriented-pixel mesh for a .bumpmap.

    ``crop`` is an optional ``(x, y, z, radius)`` sphere on voxel centroids.
    Returns ``(vertices (4M,3), triangles (2M,3), scalars, info)``; ``scalars``
    holds one value per square: ``bump`` (the normalized roughness — see
    ``--color`` in the CLI), ``bump_raw`` (what the file literally stores) and
    ``weight``.
    """
    header = read_header(path)
    px_size = header["px_size"]
    candidates = _candidates(path, header, crop)
    stride = _pick_stride(path, max_quads, stride, candidates)
    drawn = candidates[::stride]
    wanted = None if (crop is None and stride == 1) else set(drawn.tolist())

    _, it = iter_bumpmap(path, include_images=True, wanted_positions=wanted)
    verts, bumps, weights = [], [], []
    residuals, counts, indices, normals = [], [], [], []
    for voxel in it:
        out = _voxel_quads(voxel, px_size, image_key, shrink)
        if out is None:
            continue
        verts.append(out[0])
        bumps.append(out[1])
        weights.append(out[2])
        # Each voxel's plane sits at an arbitrary offset along its own normal, so
        # the raw height carries a per-voxel bias that paints the whole 0.5 m
        # voxel one flat colour. Only the deviation within the voxel is terrain.
        residuals.append(out[1] - out[1].mean())
        counts.append(out[1].size)
        indices.append(voxel["voxel_index"])
        normals.append(voxel["normal"])
    if not verts:
        raise SystemExit(f"{path}: no observed pixels to draw.")

    vertices = np.concatenate(verts, axis=0)
    n_voxels = len(counts)
    sign = _orient_voxels(indices, np.asarray(normals))
    scalars = {
        "bump": np.concatenate(residuals) * np.repeat(sign, counts),
        "bump_raw": np.concatenate(bumps),
        "weight": np.concatenate(weights),
    }

    n_quads = scalars["bump"].size
    base = np.arange(n_quads, dtype=np.int32) * 4
    triangles = np.empty((n_quads, 2, 3), dtype=np.int32)
    triangles[:, 0, :] = base[:, None] + np.array([0, 1, 2], dtype=np.int32)
    triangles[:, 1, :] = base[:, None] + np.array([0, 2, 3], dtype=np.int32)

    info = {"stride": stride, "voxels": n_voxels, "quads": n_quads,
            "px_size": px_size, "n_voxels_total": header["n_voxels"],
            "candidates": int(candidates.size), "flipped": int((sign < 0).sum())}
    return vertices, triangles.reshape(-1, 3), scalars, info


def scalar_to_rgb(values, symmetric=False):
    """Turbo-map a scalar field to (N,3) RGB, clipped to robust limits.

    ``symmetric`` centres the colormap on 0 (+-the 98th percentile of |v|), so a
    zero-mean field like the bump residual reads flat green with the bumps as
    red/blue — and equal deviations up and down get equal colours.
    """
    v = np.asarray(values, dtype=np.float64)
    if v.size < 2:
        return turbo(np.zeros(max(v.size, 0)) + 0.5)
    if symmetric:
        lim = float(np.percentile(np.abs(v), 98.0))
        lo, hi = -lim, lim
    else:
        lo, hi = np.percentile(v, [2.0, 98.0])
    if hi <= lo:
        lo, hi = float(v.min()), float(v.max())
    v = np.clip((v - lo) / (hi - lo + 1e-12), 0.0, 1.0)
    return turbo(v)


def turbo(v):
    """Map [0,1] -> RGB, falling back to blue->red if matplotlib is missing."""
    try:
        import matplotlib
        return np.asarray(matplotlib.colormaps["turbo"](v))[:, :3]
    except Exception:
        return np.stack([v, np.zeros_like(v), 1 - v], axis=1)


def write_ply(path, vertices, triangles, colors):
    """Binary-little-endian PLY, so the mesh is viewable without Open3D."""
    rgb = (np.clip(colors, 0, 1) * 255).astype(np.uint8)
    vert = np.empty(len(vertices), dtype=[("x", "<f4"), ("y", "<f4"), ("z", "<f4"),
                                          ("red", "u1"), ("green", "u1"), ("blue", "u1")])
    vert["x"], vert["y"], vert["z"] = vertices[:, 0], vertices[:, 1], vertices[:, 2]
    vert["red"], vert["green"], vert["blue"] = rgb[:, 0], rgb[:, 1], rgb[:, 2]
    face = np.empty(len(triangles), dtype=[("n", "u1"), ("v", "<i4", 3)])
    face["n"] = 3
    face["v"] = triangles
    with open(path, "wb") as f:
        f.write(b"ply\nformat binary_little_endian 1.0\n")
        f.write(f"element vertex {len(vertices)}\n".encode())
        f.write(b"property float x\nproperty float y\nproperty float z\n"
                b"property uchar red\nproperty uchar green\nproperty uchar blue\n")
        f.write(f"element face {len(triangles)}\n".encode())
        f.write(b"property list uchar int vertex_indices\nend_header\n")
        f.write(vert.tobytes())
        f.write(face.tobytes())


def show_patches(path, n_wanted):
    """The raw per-voxel bump images as a 2-D heatmap montage."""
    import matplotlib.pyplot as plt
    _, voxels = load_bumpmap(path)
    voxels = sorted(voxels, key=lambda v: int((v["bump_weights"] > 0).sum()), reverse=True)
    n = min(n_wanted, len(voxels))
    cols = int(np.ceil(np.sqrt(n)))
    rows = int(np.ceil(n / cols))
    fig, axes = plt.subplots(rows, cols, figsize=(2 * cols, 2 * rows), squeeze=False)
    for ax in np.ravel(axes):
        ax.axis("off")
    for k in range(n):
        v = voxels[k]
        img = np.where(v["bump_weights"] > 0, v["bump_img"], np.nan)
        ax = np.ravel(axes)[k]
        ax.imshow(img, cmap="viridis", origin="lower")
        ax.set_title(f"{int((v['bump_weights'] > 0).sum())} px", fontsize=7)
    fig.suptitle(f"{Path(path).name}: {n} voxel bump images (height, m)")
    plt.tight_layout()
    plt.show()


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("path", help=".pcd or .bumpmap")
    ap.add_argument("--mode", choices=["quads", "points", "patches"], default=None,
                    help="quads: oriented per-pixel squares (default for .bumpmap); "
                         "points: reconstructed cloud; patches: 2-D bump-image montage")
    ap.add_argument("--color", choices=["bump", "bump_raw", "height", "weight"], default=None,
                    help="bump = per-voxel-demeaned, neighbour-oriented roughness on a "
                         "symmetric scale (default for quads); bump_raw = the height the "
                         "file stores, which carries a per-voxel offset and an arbitrary "
                         "sign; height = world z (default for points); weight = obs count")
    ap.add_argument("--smoothed", action="store_true",
                    help="draw bump_smoothed (what the optimizer samples) instead of bump_img")
    ap.add_argument("--max-quads", type=int, default=2_000_000,
                    help="quad budget; whole voxels are dropped to stay under it (0 = no cap)")
    ap.add_argument("--stride", type=int, default=None,
                    help="keep every N-th voxel explicitly, overriding --max-quads")
    ap.add_argument("--shrink", type=float, default=1.0,
                    help="scale each square (e.g. 0.9 to leave visible gaps between pixels)")
    ap.add_argument("--crop", metavar="X,Y,Z,R",
                    help="draw only voxels whose centroid is within R m of (X,Y,Z) — "
                         "the pixels are 5 cm, so a whole 1 km drift is unreadable zoomed out")
    ap.add_argument("--patches", type=int, default=25, help="patches mode: how many voxels")
    ap.add_argument("--save", help="write the geometry to this .ply and exit without a window")
    args = ap.parse_args()

    p = Path(args.path)
    if args.mode is None:
        args.mode = "quads" if p.suffix == ".bumpmap" else "points"
    if args.mode != "points" and p.suffix != ".bumpmap":
        raise SystemExit(f"--mode {args.mode} needs the native .bumpmap file.")

    if args.mode == "patches":
        show_patches(p, args.patches)
        return

    if args.mode == "quads":
        crop = None
        if args.crop:
            crop = [float(x) for x in args.crop.split(",")]
            if len(crop) != 4:
                raise SystemExit("--crop wants four numbers: X,Y,Z,R")
        image_key = "bump_smoothed" if args.smoothed else "bump_img"
        vertices, triangles, scalars, info = build_quad_mesh(
            p, image_key=image_key, max_quads=args.max_quads, stride=args.stride,
            shrink=args.shrink, crop=crop)
        if args.color is None:
            args.color = "bump"
        if args.color == "height":
            scal, symmetric = vertices.reshape(-1, 4, 3)[:, :, 2].mean(axis=1), False
        elif args.color == "weight":
            scal, symmetric = np.log1p(np.clip(scalars["weight"], 0, None)), False
        else:
            scal, symmetric = scalars[args.color], args.color == "bump"
        colors = np.repeat(scalar_to_rgb(scal, symmetric=symmetric), 4, axis=0)

        print(f"{p.name}: {info['quads']:,} pixel squares from {info['voxels']:,}/"
              f"{info['candidates']:,} candidate voxels (of {info['n_voxels_total']:,}, "
              f"stride {info['stride']}), px_size {info['px_size']} m")
        print(f"  normals flipped to agree with neighbours: {info['flipped']:,}/"
              f"{info['voxels']:,} voxels")
        print(f"  bbox {vertices.min(0).round(1)} .. {vertices.max(0).round(1)}")
        if args.save:
            write_ply(args.save, vertices, triangles, colors)
            print(f"  wrote {args.save}")
            return

        import open3d as o3d
        mesh = o3d.geometry.TriangleMesh(
            o3d.utility.Vector3dVector(vertices), o3d.utility.Vector3iVector(triangles))
        mesh.vertex_colors = o3d.utility.Vector3dVector(colors)
        o3d.visualization.draw_geometries(
            [mesh], mesh_show_back_face=True,
            window_name=f"BIEVR bump map: {p.name} (oriented pixels, color={args.color})")
        return

    # --- reconstructed point cloud ---
    if p.suffix == ".pcd":
        xyz, inten = read_pcd_xyzi(p)
    elif p.suffix == ".bumpmap":
        hdr, voxels = load_bumpmap(p)
        pts = reconstruct_points(voxels, hdr["px_size"])  # (N,4): x y z weight
        xyz, inten = pts[:, :3], pts[:, 3]
    else:
        raise SystemExit("Give a .pcd or .bumpmap file.")

    if args.color in (None, "bump", "bump_raw"):
        # a reconstructed point carries no plane of its own to be a bump above
        args.color = "height"
    print(f"{p.name}: {len(xyz):,} points  bbox {xyz.min(0).round(1)} .. {xyz.max(0).round(1)}")
    colors = scalar_to_rgb(xyz[:, 2] if args.color == "height"
                           else np.log1p(np.clip(inten, 0, None)))
    if args.save:
        write_ply(args.save, xyz, np.empty((0, 3), dtype=np.int32), colors)
        print(f"  wrote {args.save}")
        return

    import open3d as o3d
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(xyz)
    pcd.colors = o3d.utility.Vector3dVector(colors)
    o3d.visualization.draw_geometries([pcd], window_name=f"BIEVR map: {p.name} (color={args.color})")


if __name__ == "__main__":
    main()
