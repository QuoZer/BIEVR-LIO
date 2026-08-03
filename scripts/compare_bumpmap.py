#!/usr/bin/env python3
"""Order-insensitive comparison of two BIEVR-LIO ``.bumpmap`` dumps.

``BIEVRMap::exportMap`` iterates an ``unordered_dense`` hash map, so two dumps of
the "same" map (e.g. the same run written twice) generally do not list their
voxels in the same order. This script keys each voxel by its stored integer voxel
index instead of its position in the file, then compares the two maps key by key:

    python3 scripts/compare_bumpmap.py A.bumpmap B.bumpmap [--tol 1e-6]

For the keys common to both files, this reports the max absolute difference of
T_C_W, T_O_W, centroid, num_points, bump_img, bump_weights and bump_smoothed,
plus the count of voxels whose image shape (rows, cols) disagrees
(shape-mismatched voxels are excluded from the image-difference stats, since
elementwise diffing them is undefined, and always count as a failure).

Exits 0 (PASS) only if every common-key difference is within ``--tol`` and there
are no keys present in just one of the two files.

Memory: one file (the smaller by size, to bound peak RAM) is loaded fully into a
dict keyed by voxel index; the other is streamed voxel by voxel and compared on
the fly, so peak memory is roughly one file's worth of decoded voxels rather
than both.
"""

import argparse
import os
import sys

from load_bumpmap import iter_bumpmap, read_header

FIELDS = ("T_C_W", "T_O_W", "centroid", "num_points", "bump_img", "bump_weights", "bump_smoothed")


def _voxel_key(voxel):
    return tuple(int(x) for x in voxel["voxel_index"])


def _build_index(path):
    """Fully decode `path` into a dict {voxel_key: voxel_dict}. Returns
    (header, index, n_duplicate_keys)."""
    header, gen = iter_bumpmap(path, include_images=True)
    index = {}
    n_dup = 0
    for v in gen:
        key = _voxel_key(v)
        if key in index:
            n_dup += 1
        index[key] = v
    return header, index, n_dup


class Stat:
    """Running max-abs-difference tracker, remembering the worst key."""

    def __init__(self):
        self.max_diff = 0.0
        self.worst_key = None
        self.n_compared = 0

    def update(self, key, diff):
        self.n_compared += 1
        if diff > self.max_diff:
            self.max_diff = diff
            self.worst_key = key


def main(argv):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("file_a")
    ap.add_argument("file_b")
    ap.add_argument("--tol", type=float, default=1e-6, help="max allowed abs difference (default 1e-6)")
    args = ap.parse_args(argv[1:])

    header_a_probe = read_header(args.file_a)
    header_b_probe = read_header(args.file_b)

    # Fully index the physically smaller file to bound peak memory; stream the
    # other one voxel by voxel.
    size_a = os.path.getsize(args.file_a)
    size_b = os.path.getsize(args.file_b)
    if size_a <= size_b:
        base_path, stream_path = args.file_a, args.file_b
        base_label, stream_label = "A", "B"
    else:
        base_path, stream_path = args.file_b, args.file_a
        base_label, stream_label = "B", "A"

    print(f"File A: {args.file_a}")
    print(f"  version={header_a_probe['version']} voxel_size={header_a_probe['voxel_size']} "
          f"px_size={header_a_probe['px_size']} n_voxels={header_a_probe['n_voxels']}")
    print(f"File B: {args.file_b}")
    print(f"  version={header_b_probe['version']} voxel_size={header_b_probe['voxel_size']} "
          f"px_size={header_b_probe['px_size']} n_voxels={header_b_probe['n_voxels']}")
    print()

    if abs(header_a_probe["voxel_size"] - header_b_probe["voxel_size"]) > args.tol or \
       abs(header_a_probe["px_size"] - header_b_probe["px_size"]) > args.tol:
        print("WARNING: voxel_size/px_size differ between the two files -- voxel-index keys and "
              "geometry are not directly comparable.")
    print()

    _, base_index, base_dup = _build_index(base_path)
    if base_dup:
        print(f"WARNING: {base_dup} duplicate voxel-index keys within {base_label} "
              f"({base_path}) -- later one wins in this comparison.")

    _, stream_gen = iter_bumpmap(stream_path, include_images=True)

    matched_keys = set()
    only_in_stream = 0
    stats = {f: Stat() for f in FIELDS}
    n_shape_mismatch = 0
    shape_mismatch_examples = []
    stream_dup = 0
    seen_stream_keys = set()

    for v in stream_gen:
        key = _voxel_key(v)
        if key in seen_stream_keys:
            stream_dup += 1
        seen_stream_keys.add(key)

        base_v = base_index.get(key)
        if base_v is None:
            only_in_stream += 1
            continue
        matched_keys.add(key)

        # T_C_W / T_O_W: max abs diff over all 12 entries.
        stats["T_C_W"].update(key, float(abs(v["T_C_W"] - base_v["T_C_W"]).max()))
        stats["T_O_W"].update(key, float(abs(v["T_O_W"] - base_v["T_O_W"]).max()))
        stats["centroid"].update(key, float(abs(v["centroid"] - base_v["centroid"]).max()))
        stats["num_points"].update(key, float(abs(v["num_points"] - base_v["num_points"])))

        shape_v = (v["rows"], v["cols"])
        shape_b = (base_v["rows"], base_v["cols"])
        if shape_v != shape_b:
            n_shape_mismatch += 1
            if len(shape_mismatch_examples) < 5:
                shape_mismatch_examples.append((key, shape_b, shape_v))
            continue  # can't diff differently-shaped images elementwise

        stats["bump_img"].update(key, float(abs(v["bump_img"] - base_v["bump_img"]).max()))
        stats["bump_weights"].update(key, float(abs(v["bump_weights"] - base_v["bump_weights"]).max()))
        stats["bump_smoothed"].update(key, float(abs(v["bump_smoothed"] - base_v["bump_smoothed"]).max()))

    only_in_base = set(base_index.keys()) - matched_keys
    n_only_a = len(only_in_base) if base_label == "A" else only_in_stream
    n_only_b = only_in_stream if base_label == "A" else len(only_in_base)
    n_common = len(matched_keys)

    if stream_dup:
        print(f"WARNING: {stream_dup} duplicate voxel-index keys within {stream_label} "
              f"({stream_path}) -- only the last occurrence participated in the comparison.")

    print(f"Voxel counts:   A={header_a_probe['n_voxels']}  B={header_b_probe['n_voxels']}")
    print(f"Keys only in A: {n_only_a}")
    print(f"Keys only in B: {n_only_b}")
    print(f"Keys in both:   {n_common}")
    print()

    print(f"{'field':<16} {'n_compared':>10} {'max abs diff':>16} {'within tol':>11}")
    ok = True
    for f in FIELDS:
        s = stats[f]
        within = s.max_diff <= args.tol
        ok &= within
        worst = f"  (key={s.worst_key})" if s.worst_key is not None else ""
        print(f"{f:<16} {s.n_compared:>10} {s.max_diff:>16.9g} {str(within):>11}{worst}")

    print()
    print(f"Voxels with mismatched image shape (excluded from image diffs): {n_shape_mismatch}")
    for key, shape_b, shape_v in shape_mismatch_examples:
        print(f"  key={key}: base(rows,cols)={shape_b} vs stream(rows,cols)={shape_v}")

    no_unmatched = (n_only_a == 0 and n_only_b == 0)
    no_shape_mismatch = n_shape_mismatch == 0
    result_ok = ok and no_unmatched and no_shape_mismatch

    print()
    if result_ok:
        print(f"RESULT: PASS  (all common-key differences <= {args.tol:g}, no unmatched keys)")
    else:
        reasons = []
        if not ok:
            reasons.append("some field exceeded --tol")
        if not no_unmatched:
            reasons.append("unmatched keys present")
        if not no_shape_mismatch:
            reasons.append("shape-mismatched voxels present")
        print(f"RESULT: FAIL  ({', '.join(reasons)})")

    return 0 if result_ok else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv))
