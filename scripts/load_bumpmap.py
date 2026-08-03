#!/usr/bin/env python3
"""Loader for BIEVR-LIO native bump-map dumps (``.bumpmap``).

The ``.bumpmap`` file is the raw voxel representation written by
``BIEVRMap::exportMap`` (see BIEVR/src/bievr_map.cpp). It complements the binary
PCD export: the PCD holds the *reconstructed* world points, while this file holds
the underlying oriented voxels (pose + bump image + weight matrix), so the map can
be re-projected or re-optimized losslessly.

Binary layout (little-endian), format v2 (``BIEVRMap::kNativeFormatVersion``):

    char     magic[8]     = "BIEVRMP\\0"
    uint32   version      = 2
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
        int32   voxel_index[3]           # this voxel's integer hash-key index
        float32 bump_smoothed[rows*cols] # row-major; the image the optimizer
                                         # actually samples

Reconstruction (identical to the C++ / PCD path): for every pixel (i, j) with
``bump_weights[i, j] > 0``,

    p_C = (j * px_size, i * px_size, bump_img[i, j])
    p_W = T_W_C @ p_C          # T_W_C = inverse of the 3x4 T_C_W transform

``reconstruct_points`` returns an (N, 4) array of (x, y, z, weight) so the result
can be cross-checked against the PCD point count and bounding box.

Two ways to read a file:

* ``load_bumpmap(path)`` — parses the whole file and returns
  ``(header, voxels)`` with every voxel's images resident in memory. Fine for
  modestly sized maps; on multi-hundred-MB dumps it holds the whole decoded map
  in RAM at once.
* ``iter_bumpmap(path, include_images=True, wanted_positions=None)`` — streams
  the file voxel by voxel instead of buffering it all. Pass
  ``include_images=False`` to skip decoding ``bump_img``/``bump_weights``/
  ``bump_smoothed`` entirely (their bytes are seeked over, not read) when only
  the geometry fields are needed. Pass ``wanted_positions`` (a set of 0-based
  voxel positions, i.e. on-disk order) to decode only those records — everything
  else is skipped via seek — for sampling a handful of voxels out of a huge
  file without paying to decode the rest. ``load_bumpmap`` is implemented on top
  of this generator.
"""

import struct
import sys

import numpy as np

MAGIC = b"BIEVRMP\x00"

# The only format version this tooling (and BIEVRMap::importMap) accepts.
FORMAT_VERSION = 2

# Fixed-size header: magic, version, voxel_size, px_size, n_voxels.
HEADER_STRUCT = struct.Struct("<8sIddQ")

# Fixed-size portion of every voxel record:
# T_C_W[12], T_O_W[12], centroid[3], normal[3], num_points, rows, cols.
VOXEL_FIXED_STRUCT = struct.Struct("<12d12d3d3dQii")

# voxel_index[3], immediately preceding the smoothed image.
INDEX_STRUCT = struct.Struct("<3i")


def _invert_3x4(T):
    """Invert a 3x4 [R | t] rigid transform, returned as another 3x4 [R^T | -R^T t]."""
    R = T[:, :3]
    t = T[:, 3]
    Rt = R.T
    inv = np.empty((3, 4), dtype=np.float64)
    inv[:, :3] = Rt
    inv[:, 3] = -Rt @ t
    return inv


def _read_header(f):
    """Read and validate the header from an open file positioned at offset 0."""
    raw = f.read(HEADER_STRUCT.size)
    if len(raw) < HEADER_STRUCT.size:
        raise ValueError("file too short to contain a bumpmap header")
    magic, version, voxel_size, px_size, n_voxels = HEADER_STRUCT.unpack(raw)
    if magic != MAGIC:
        raise ValueError(f"Bad magic {magic!r}, expected {MAGIC!r}")
    if version != FORMAT_VERSION:
        raise ValueError(f"Unsupported format version {version}, expected {FORMAT_VERSION}")
    return {"version": version, "voxel_size": voxel_size, "px_size": px_size, "n_voxels": n_voxels}


def read_header(path):
    """Read just the 36-byte header of a ``.bumpmap`` file, without touching any
    voxel data. Cheap even on a huge file."""
    with open(path, "rb") as f:
        return _read_header(f)


def iter_bumpmap(path, include_images=True, wanted_positions=None):
    """Stream a ``.bumpmap`` file voxel by voxel instead of buffering it whole.

    Returns ``(header, voxel_iterator)``. The header is read (and the magic
    validated) before this function returns, so it's available even if the
    caller never touches the iterator. Each item the iterator yields is a dict
    with the same keys as ``load_bumpmap``'s per-voxel dicts, plus ``position``
    (its 0-based order within the file).

    ``include_images=False`` skips decoding ``bump_img``/``bump_weights``/
    ``bump_smoothed`` (seeks past their bytes instead of reading them), for
    callers that only need pose/geometry fields.

    ``wanted_positions``, if given, is a set (or any container supporting
    ``in``) of 0-based voxel positions — only those records are decoded and
    yielded; every other record is skipped via seek without being unpacked.
    This bounds both time and memory when sampling a handful of voxels out of
    a very large file.

    The underlying file handle is closed automatically once the iterator is
    exhausted. If the caller doesn't consume it fully, call
    ``voxel_iterator.close()`` to release the file promptly.
    """
    f = open(path, "rb")
    try:
        header = _read_header(f)
    except Exception:
        f.close()
        raise

    def _gen():
        try:
            n_voxels = header["n_voxels"]
            for p in range(n_voxels):
                raw = f.read(VOXEL_FIXED_STRUCT.size)
                if len(raw) < VOXEL_FIXED_STRUCT.size:
                    raise ValueError(f"{path}: truncated voxel record {p} (fixed fields)")
                vals = VOXEL_FIXED_STRUCT.unpack(raw)
                rows, cols = vals[31], vals[32]
                n_px = rows * cols
                img_bytes = n_px * 4

                if wanted_positions is not None and p not in wanted_positions:
                    f.seek(img_bytes * 3 + INDEX_STRUCT.size, 1)
                    continue

                T_C_W = np.array(vals[0:12], dtype=np.float64).reshape(3, 4)
                T_O_W = np.array(vals[12:24], dtype=np.float64).reshape(3, 4)
                centroid = np.array(vals[24:27], dtype=np.float64)
                normal = np.array(vals[27:30], dtype=np.float64)
                num_points = int(vals[30])

                if include_images:
                    img_raw = f.read(img_bytes)
                    w_raw = f.read(img_bytes)
                    if len(img_raw) < img_bytes or len(w_raw) < img_bytes:
                        raise ValueError(f"{path}: truncated voxel record {p} (bump image)")
                    bump_img = np.frombuffer(img_raw, dtype="<f4", count=n_px).reshape(rows, cols).copy()
                    bump_weights = np.frombuffer(w_raw, dtype="<f4", count=n_px).reshape(rows, cols).copy()
                else:
                    f.seek(img_bytes * 2, 1)
                    bump_img = None
                    bump_weights = None

                idx_raw = f.read(INDEX_STRUCT.size)
                if len(idx_raw) < INDEX_STRUCT.size:
                    raise ValueError(f"{path}: truncated voxel record {p} (voxel_index)")
                voxel_index = INDEX_STRUCT.unpack(idx_raw)
                if include_images:
                    s_raw = f.read(img_bytes)
                    if len(s_raw) < img_bytes:
                        raise ValueError(f"{path}: truncated voxel record {p} (bump_smoothed)")
                    bump_smoothed = np.frombuffer(s_raw, dtype="<f4", count=n_px).reshape(rows, cols).copy()
                else:
                    f.seek(img_bytes, 1)
                    bump_smoothed = None

                yield {
                    "position": p,
                    "T_C_W": T_C_W,
                    "T_O_W": T_O_W,
                    "centroid": centroid,
                    "normal": normal,
                    "num_points": num_points,
                    "rows": int(rows),
                    "cols": int(cols),
                    "bump_img": bump_img,
                    "bump_weights": bump_weights,
                    "voxel_index": voxel_index,
                    "bump_smoothed": bump_smoothed,
                }
        finally:
            f.close()

    return header, _gen()


def load_bumpmap(path):
    """Parse a ``.bumpmap`` file into a header dict and a list of voxel dicts.

    Returns ``(header, voxels)`` where ``header`` has ``version``, ``voxel_size``,
    ``px_size`` and ``n_voxels``, and each voxel dict has keys ``T_C_W`` (3x4),
    ``T_O_W`` (3x4), ``centroid`` (3,), ``normal`` (3,), ``num_points`` (int),
    ``rows``, ``cols``, ``bump_img`` (rows x cols), ``bump_weights`` (rows x cols),
    ``position`` (0-based order in the file), ``voxel_index`` (3-tuple) and
    ``bump_smoothed`` (rows x cols).

    Loads and keeps every voxel's images in memory at once. For large files,
    prefer ``iter_bumpmap`` to stream instead.
    """
    header, gen = iter_bumpmap(path, include_images=True)
    voxels = list(gen)
    return header, voxels


def reconstruct_voxel_points(voxel, px_size):
    """Reconstruct one voxel's world points as an (N, 4) array of (x, y, z, weight).
    Same math as ``reconstruct_points``, factored out for streaming callers that
    process one voxel at a time."""
    w = voxel["bump_weights"]
    img = voxel["bump_img"]
    ii, jj = np.nonzero(w > 0)
    if ii.size == 0:
        return np.empty((0, 4), dtype=np.float64)
    p_C = np.empty((ii.size, 3), dtype=np.float64)
    p_C[:, 0] = jj * px_size
    p_C[:, 1] = ii * px_size
    p_C[:, 2] = img[ii, jj]
    T_W_C = _invert_3x4(voxel["T_C_W"])
    p_W = p_C @ T_W_C[:, :3].T + T_W_C[:, 3]
    out = np.empty((ii.size, 4), dtype=np.float64)
    out[:, :3] = p_W
    out[:, 3] = w[ii, jj]
    return out


def reconstruct_points(voxels, px_size):
    """Reconstruct world points (N, 4) = (x, y, z, weight) from the voxels.

    Uses the same math as the C++ PCD export, so the point count and bounding box
    must match the exported PCD exactly.
    """
    chunks = [reconstruct_voxel_points(v, px_size) for v in voxels]
    chunks = [c for c in chunks if c.shape[0] > 0]
    if not chunks:
        return np.empty((0, 4), dtype=np.float64)
    return np.concatenate(chunks, axis=0)


def main(argv):
    if len(argv) < 2:
        print(f"usage: {argv[0]} <path.bumpmap>")
        return 1
    path = argv[1]

    header, voxel_iter = iter_bumpmap(path, include_images=True)

    n_voxels_seen = 0
    total_valid = 0
    total_points = 0
    mn = None
    mx = None
    for v in voxel_iter:
        n_voxels_seen += 1
        w = v["bump_weights"]
        total_valid += int(np.count_nonzero(w > 0))
        pts = reconstruct_voxel_points(v, header["px_size"])
        if pts.shape[0] == 0:
            continue
        total_points += pts.shape[0]
        vmn = pts[:, :3].min(axis=0)
        vmx = pts[:, :3].max(axis=0)
        mn = vmn if mn is None else np.minimum(mn, vmn)
        mx = vmx if mx is None else np.maximum(mx, vmx)

    print(f"file:            {path}")
    print(f"version:         {header['version']}")
    print(f"voxel_size:      {header['voxel_size']}")
    print(f"px_size:         {header['px_size']}")
    print(f"n_voxels:        {header['n_voxels']}")
    print(f"total valid px:  {total_valid}")
    print(f"reconstructed:   {total_points} points")
    if total_points > 0:
        print(f"bbox min:        [{mn[0]:.3f}, {mn[1]:.3f}, {mn[2]:.3f}]")
        print(f"bbox max:        [{mx[0]:.3f}, {mx[1]:.3f}, {mx[2]:.3f}]")
    if n_voxels_seen != header["n_voxels"]:
        print(f"warning: header says n_voxels={header['n_voxels']} but {n_voxels_seen} records were read")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
