#!/usr/bin/env python3
"""Loader for BIEVR-LIO native bump-map dumps (``.bumpmap``).

The ``.bumpmap`` file is the raw voxel representation written by
``BIEVRMap::exportMap`` (see BIEVR/src/bievr_map.cpp). It complements the binary
PCD export: the PCD holds the *reconstructed* world points, while this file holds
the underlying oriented voxels (pose + bump image + weight matrix), so the map can
be re-projected or re-optimized losslessly.

Binary layout (little-endian):

    char     magic[8]     = "BIEVRMP\\0"
    uint32   version      = 1
    float64  voxel_size
    float64  px_size
    uint64   n_voxels
    repeat n_voxels:
        float64 T_C_W[12]   # rows 0..2 of the 3x4 matrix, row-major (R 3x3 | t 3x1)
        float64 T_O_W[12]   # same layout
        float64 centroid[3]
        float64 normal[3]
        uint64  num_points
        int32   rows, cols
        float32 bump_img[rows*cols]      # row-major
        float32 bump_weights[rows*cols]  # row-major

Reconstruction (identical to the C++ / PCD path): for every pixel (i, j) with
``bump_weights[i, j] > 0``,

    p_C = (j * px_size, i * px_size, bump_img[i, j])
    p_W = T_W_C @ p_C          # T_W_C = inverse of the 3x4 T_C_W transform

``reconstruct_points`` returns an (N, 4) array of (x, y, z, weight) so the result
can be cross-checked against the PCD point count and bounding box.
"""

import struct
import sys

import numpy as np

MAGIC = b"BIEVRMP\x00"


def _invert_3x4(T):
    """Invert a 3x4 [R | t] rigid transform, returned as another 3x4 [R^T | -R^T t]."""
    R = T[:, :3]
    t = T[:, 3]
    Rt = R.T
    inv = np.empty((3, 4), dtype=np.float64)
    inv[:, :3] = Rt
    inv[:, 3] = -Rt @ t
    return inv


def load_bumpmap(path):
    """Parse a ``.bumpmap`` file into a header dict and a list of voxel dicts.

    Returns ``(header, voxels)`` where ``header`` has ``voxel_size`` and
    ``px_size``, and each voxel dict has keys ``T_C_W`` (3x4), ``T_O_W`` (3x4),
    ``centroid`` (3,), ``normal`` (3,), ``num_points`` (int), ``bump_img``
    (rows x cols) and ``bump_weights`` (rows x cols).
    """
    with open(path, "rb") as f:
        data = f.read()

    off = 0
    magic = data[off:off + 8]
    off += 8
    if magic != MAGIC:
        raise ValueError(f"Bad magic {magic!r}, expected {MAGIC!r}")

    (version,) = struct.unpack_from("<I", data, off)
    off += 4
    voxel_size, px_size = struct.unpack_from("<dd", data, off)
    off += 16
    (n_voxels,) = struct.unpack_from("<Q", data, off)
    off += 8

    header = {"version": version, "voxel_size": voxel_size, "px_size": px_size,
              "n_voxels": n_voxels}

    voxels = []
    for _ in range(n_voxels):
        T_C_W = np.array(struct.unpack_from("<12d", data, off), dtype=np.float64).reshape(3, 4)
        off += 12 * 8
        T_O_W = np.array(struct.unpack_from("<12d", data, off), dtype=np.float64).reshape(3, 4)
        off += 12 * 8
        centroid = np.array(struct.unpack_from("<3d", data, off), dtype=np.float64)
        off += 3 * 8
        normal = np.array(struct.unpack_from("<3d", data, off), dtype=np.float64)
        off += 3 * 8
        (num_points,) = struct.unpack_from("<Q", data, off)
        off += 8
        rows, cols = struct.unpack_from("<ii", data, off)
        off += 8
        n = rows * cols
        bump_img = np.frombuffer(data, dtype="<f4", count=n, offset=off).reshape(rows, cols).copy()
        off += n * 4
        bump_weights = np.frombuffer(data, dtype="<f4", count=n, offset=off).reshape(rows, cols).copy()
        off += n * 4

        voxels.append({
            "T_C_W": T_C_W,
            "T_O_W": T_O_W,
            "centroid": centroid,
            "normal": normal,
            "num_points": num_points,
            "bump_img": bump_img,
            "bump_weights": bump_weights,
        })

    return header, voxels


def reconstruct_points(voxels, px_size):
    """Reconstruct world points (N, 4) = (x, y, z, weight) from the voxels.

    Uses the same math as the C++ PCD export, so the point count and bounding box
    must match the exported PCD exactly.
    """
    chunks = []
    for v in voxels:
        w = v["bump_weights"]
        img = v["bump_img"]
        ii, jj = np.nonzero(w > 0)
        if ii.size == 0:
            continue
        # p_C = (j * px_size, i * px_size, height)
        p_C = np.empty((ii.size, 3), dtype=np.float64)
        p_C[:, 0] = jj * px_size
        p_C[:, 1] = ii * px_size
        p_C[:, 2] = img[ii, jj]
        T_W_C = _invert_3x4(v["T_C_W"])
        p_W = p_C @ T_W_C[:, :3].T + T_W_C[:, 3]
        out = np.empty((ii.size, 4), dtype=np.float64)
        out[:, :3] = p_W
        out[:, 3] = w[ii, jj]
        chunks.append(out)

    if not chunks:
        return np.empty((0, 4), dtype=np.float64)
    return np.concatenate(chunks, axis=0)


def main(argv):
    if len(argv) < 2:
        print(f"usage: {argv[0]} <path.bumpmap>")
        return 1
    path = argv[1]
    header, voxels = load_bumpmap(path)
    total_valid = sum(int(np.count_nonzero(v["bump_weights"] > 0)) for v in voxels)

    pts = reconstruct_points(voxels, header["px_size"])

    print(f"file:            {path}")
    print(f"version:         {header['version']}")
    print(f"voxel_size:      {header['voxel_size']}")
    print(f"px_size:         {header['px_size']}")
    print(f"n_voxels:        {header['n_voxels']}")
    print(f"total valid px:  {total_valid}")
    print(f"reconstructed:   {pts.shape[0]} points")
    if pts.shape[0] > 0:
        mn = pts[:, :3].min(axis=0)
        mx = pts[:, :3].max(axis=0)
        print(f"bbox min:        [{mn[0]:.3f}, {mn[1]:.3f}, {mn[2]:.3f}]")
        print(f"bbox max:        [{mx[0]:.3f}, {mx[1]:.3f}, {mx[2]:.3f}]")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
