#!/usr/bin/env python3
"""Convert a DLIO 3D map (PCD) into a Nav2 occupancy map (pgm + yaml).

The DLIO map from the G1 replay/live stack has its z datum on the floor
(frames/baselink = base_footprint). Occupied cells come from points inside
the obstacle band; free cells come from observed floor returns (points near
z = 0), which the Mid-360 sees densely; everything else stays unknown.

The band defaults match scan_from_cloud_g1.yaml (0.29 .. 1.44 m above the
floor) so the map judges the same obstacles the live /scan sees.

Usage:
  dlio_pcd_to_nav2_map.py <dlio_map.pcd> -o <out_prefix> [--resolution 0.05]
      [--z-min 0.29] [--z-max 1.44] [--floor-band 0.15] [--min-points 1]
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np


def load_pcd(path: Path) -> np.ndarray:
    raw = path.read_bytes()
    header_end = raw.find(b"DATA")
    line_end = raw.find(b"\n", header_end)
    fields = {}
    for line in raw[:line_end].decode("ascii", errors="replace").splitlines():
        parts = line.strip().split()
        if parts:
            fields[parts[0]] = parts[1:]
    names = fields["FIELDS"]
    sizes = [int(s) for s in fields["SIZE"]]
    types = list(fields["TYPE"])
    counts = [int(c) for c in fields.get("COUNT", ["1"] * len(names))]
    npoints = int(fields["POINTS"][0])
    kind = fields["DATA"][0]
    if kind == "ascii":
        arr = np.loadtxt(raw[line_end + 1:].decode("ascii").splitlines())
        col = {}
        c = 0
        for name, cnt in zip(names, counts):
            col[name] = c
            c += cnt
        return arr[:, [col["x"], col["y"], col["z"]]]
    np_types = {("F", 4): "f4", ("F", 8): "f8", ("U", 1): "u1", ("U", 2): "u2",
                ("U", 4): "u4", ("I", 1): "i1", ("I", 2): "i2", ("I", 4): "i4"}
    dtype = []
    for name, size, typ, cnt in zip(names, sizes, types, counts):
        base = np_types[(typ, size)]
        dtype.append((name, base, (cnt,)) if cnt > 1 else (name, base))
    arr = np.frombuffer(raw, dtype=np.dtype(dtype), count=npoints,
                        offset=line_end + 1)
    return np.stack([arr["x"], arr["y"], arr["z"]], axis=1).astype(np.float64)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("pcd", type=Path)
    ap.add_argument("-o", "--out-prefix", type=Path, required=True,
                    help="output prefix; writes <prefix>.pgm and <prefix>.yaml")
    ap.add_argument("--resolution", type=float, default=0.05)
    ap.add_argument("--z-min", type=float, default=0.29,
                    help="obstacle band lower bound above floor [m]")
    ap.add_argument("--z-max", type=float, default=1.44,
                    help="obstacle band upper bound above floor [m]")
    ap.add_argument("--floor-band", type=float, default=0.15,
                    help="|z| below this is a floor return -> free cell [m]")
    ap.add_argument("--min-points", type=int, default=1,
                    help="points per cell to mark occupied")
    ap.add_argument("--pad", type=float, default=1.0,
                    help="map border padding around the point extent [m]")
    args = ap.parse_args()

    P = load_pcd(args.pcd)
    print(f"{args.pcd}: {len(P)} points")
    occ_pts = P[(P[:, 2] >= args.z_min) & (P[:, 2] <= args.z_max)]
    floor_pts = P[np.abs(P[:, 2]) <= args.floor_band]
    if not len(occ_pts):
        print("no points in obstacle band", file=sys.stderr)
        return 1

    allxy = np.vstack([occ_pts[:, :2], floor_pts[:, :2]])
    res = args.resolution
    x0 = float(allxy[:, 0].min() - args.pad)
    y0 = float(allxy[:, 1].min() - args.pad)
    w = int(np.ceil((allxy[:, 0].max() + args.pad - x0) / res))
    h = int(np.ceil((allxy[:, 1].max() + args.pad - y0) / res))

    def rasterize(pts):
        g = np.zeros((h, w), dtype=np.int32)
        ix = ((pts[:, 0] - x0) / res).astype(int)
        iy = ((pts[:, 1] - y0) / res).astype(int)
        ok = (ix >= 0) & (ix < w) & (iy >= 0) & (iy < h)
        np.add.at(g, (iy[ok], ix[ok]), 1)
        return g

    occ = rasterize(occ_pts) >= args.min_points
    free = (rasterize(floor_pts) >= 1) & ~occ

    img = np.full((h, w), 205, dtype=np.uint8)  # unknown
    img[free] = 254
    img[occ] = 0
    img = np.flipud(img)  # pgm row 0 is max y

    pgm_path = args.out_prefix.with_suffix(".pgm")
    yaml_path = args.out_prefix.with_suffix(".yaml")
    pgm_path.parent.mkdir(parents=True, exist_ok=True)
    with open(pgm_path, "wb") as f:
        f.write(f"P5\n{w} {h}\n255\n".encode("ascii"))
        f.write(img.tobytes())
    yaml_path.write_text(
        f"image: {pgm_path.name}\n"
        f"mode: trinary\n"
        f"resolution: {res}\n"
        f"origin: [{x0:.6f}, {y0:.6f}, 0.0]\n"
        "negate: 0\n"
        "occupied_thresh: 0.65\n"
        "free_thresh: 0.25\n"
    )
    print(f"wrote {pgm_path} ({w}x{h} @ {res} m) and {yaml_path}")
    print(f"occupied cells: {int(occ.sum())}, free cells: {int(free.sum())}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
