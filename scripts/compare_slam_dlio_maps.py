#!/usr/bin/env python3
"""Compare a slam_toolbox occupancy map against a DLIO 3D map (PCD).

Rasterizes an obstacle-height slice of the DLIO point cloud onto the exact
grid of the saved slam_toolbox/Nav2 occupancy map, aligns the two through the
final map->odom transform (the slam map lives in `map`, the DLIO cloud in
`odom`), and reports coherence metrics plus a visual overlay:

  - chamfer distances between the two occupied-cell sets (both directions)
  - fraction of occupied cells matched within a tolerance
  - overlay.png  : slam-only (red), DLIO-only (blue), agreement (black)
  - slam_map.png / dlio_slice.png : the individual rasters

The DLIO map's z datum is the floor (the replay runs DLIO with
frames/baselink = base_footprint), so --z-min/--z-max are heights above the
floor. The default band matches the /scan band of scan_from_cloud_g1.yaml
(sensor at ~1.24 m: world 0.29 .. 1.44 m) so the comparison judges the same
obstacles slam_toolbox actually saw.

Usage:
  compare_slam_dlio_maps.py --map-dir <dir>            # save_current_maps.sh layout
  compare_slam_dlio_maps.py --slam-yaml m.yaml --pcd dlio_map.pcd [--tf-bag replay_outputs]
"""

from __future__ import annotations

import argparse
import math
import re
import sqlite3
import sys
from pathlib import Path

import numpy as np
import yaml

try:
    from PIL import Image
except ImportError:
    Image = None


def load_slam_map(yaml_path: Path):
    meta = yaml.safe_load(yaml_path.read_text())
    img_path = yaml_path.parent / meta["image"]
    if Image is not None:
        img = np.asarray(Image.open(img_path))
    else:
        img = read_pgm(img_path)
    res = float(meta["resolution"])
    origin = meta["origin"]  # [x, y, yaw] of the lower-left pixel corner
    negate = int(meta.get("negate", 0))
    occ_thresh = float(meta.get("occupied_thresh", 0.65))
    free_thresh = float(meta.get("free_thresh", 0.25))
    # map_saver trinary convention: 0 = occupied, 254 = free, 205 = unknown.
    p = img.astype(np.float64) / 255.0
    occ_prob = p if negate else 1.0 - p
    occupied = occ_prob > occ_thresh
    free = occ_prob < free_thresh
    # PGM row 0 is the TOP of the map (max y); flip so row 0 is min y.
    occupied = np.flipud(occupied)
    free = np.flipud(free)
    return occupied, free, res, (float(origin[0]), float(origin[1])), meta


def read_pgm(path: Path) -> np.ndarray:
    data = path.read_bytes()
    if not data.startswith(b"P5"):
        raise ValueError(f"{path}: only binary P5 PGM supported")
    tokens = []
    idx = 2
    while len(tokens) < 3:
        m = re.match(rb"\s*(#[^\n]*\n|\S+)", data[idx:])
        if m is None:
            raise ValueError(f"{path}: bad PGM header")
        tok = m.group(1)
        idx += m.end()
        if not tok.startswith(b"#"):
            tokens.append(int(tok))
    width, height, _maxval = tokens
    idx += 1  # single whitespace after maxval
    return np.frombuffer(data, dtype=np.uint8, count=width * height,
                         offset=idx).reshape(height, width)


def load_pcd(path: Path) -> np.ndarray:
    """Minimal PCD reader returning Nx3 xyz (binary and ascii DATA)."""
    raw = path.read_bytes()
    header_end = raw.find(b"DATA")
    line_end = raw.find(b"\n", header_end)
    header = raw[:line_end].decode("ascii", errors="replace")
    fields = {}
    for line in header.splitlines():
        parts = line.strip().split()
        if parts:
            fields[parts[0]] = parts[1:]
    names = fields["FIELDS"]
    sizes = [int(s) for s in fields["SIZE"]]
    types = [t for t in fields["TYPE"]]
    counts = [int(c) for c in fields.get("COUNT", ["1"] * len(names))]
    npoints = int(fields["POINTS"][0])
    data_kind = fields["DATA"][0]
    if data_kind == "ascii":
        body = raw[line_end + 1:].decode("ascii")
        arr = np.loadtxt(body.splitlines(), dtype=np.float64)
        cols = {}
        col = 0
        for name, cnt in zip(names, counts):
            cols[name] = col
            col += cnt
        return arr[:, [cols["x"], cols["y"], cols["z"]]]
    if data_kind != "binary":
        raise ValueError(f"{path}: unsupported DATA {data_kind}")
    np_types = {("F", 4): "f4", ("F", 8): "f8", ("U", 1): "u1", ("U", 2): "u2",
                ("U", 4): "u4", ("I", 1): "i1", ("I", 2): "i2", ("I", 4): "i4"}
    dtype = []
    for name, size, typ, cnt in zip(names, sizes, types, counts):
        base = np_types[(typ, size)]
        dtype.append((name, base, (cnt,)) if cnt > 1 else (name, base))
    arr = np.frombuffer(raw, dtype=np.dtype(dtype), count=npoints,
                        offset=line_end + 1)
    return np.stack([arr["x"], arr["y"], arr["z"]], axis=1).astype(np.float64)


def last_map_to_odom(tf_bag_dir: Path):
    """Final map->odom 2D transform (x, y, yaw) from a recorded replay bag."""
    dbs = sorted(tf_bag_dir.glob("*.db3"))
    if not dbs:
        return None
    from rclpy.serialization import deserialize_message
    from tf2_msgs.msg import TFMessage
    con = sqlite3.connect(str(dbs[0]))
    cur = con.execute(
        "SELECT id FROM topics WHERE name = '/tf'")
    row = cur.fetchone()
    if row is None:
        con.close()
        return None
    topic_id = row[0]
    result = None
    for (blob,) in con.execute(
            "SELECT data FROM messages WHERE topic_id = ? ORDER BY timestamp",
            (topic_id,)):
        msg = deserialize_message(blob, TFMessage)
        for t in msg.transforms:
            if t.header.frame_id == "map" and t.child_frame_id == "odom":
                q = t.transform.rotation
                yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
                result = (t.transform.translation.x,
                          t.transform.translation.y, yaw)
    con.close()
    return result


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--map-dir", type=Path,
                    help="save_current_maps.sh output dir (nav2_map.yaml + "
                         "dlio_map.pcd + replay_outputs/)")
    ap.add_argument("--slam-yaml", type=Path)
    ap.add_argument("--pcd", type=Path)
    ap.add_argument("--tf-bag", type=Path,
                    help="rosbag2 dir with recorded /tf for map->odom "
                         "(default: <map-dir>/replay_outputs)")
    ap.add_argument("--z-min", type=float, default=0.29,
                    help="slice band lower bound above floor [m]")
    ap.add_argument("--z-max", type=float, default=1.44,
                    help="slice band upper bound above floor [m]")
    ap.add_argument("--min-points", type=int, default=1,
                    help="points per cell to mark DLIO occupancy")
    ap.add_argument("--match-tol", type=float, default=0.15,
                    help="distance tolerance counting a cell as matched [m]")
    ap.add_argument("--out-dir", type=Path)
    args = ap.parse_args()

    if args.map_dir:
        slam_yaml = args.slam_yaml or args.map_dir / "nav2_map.yaml"
        pcd_path = args.pcd or args.map_dir / "dlio_map.pcd"
        tf_bag = args.tf_bag or args.map_dir / "replay_outputs"
        out_dir = args.out_dir or args.map_dir / "compare"
    else:
        if not (args.slam_yaml and args.pcd):
            ap.error("need --map-dir or both --slam-yaml and --pcd")
        slam_yaml, pcd_path, tf_bag = args.slam_yaml, args.pcd, args.tf_bag
        out_dir = args.out_dir or slam_yaml.parent / "compare"

    occupied, free, res, (ox, oy), _meta = load_slam_map(slam_yaml)
    h, w = occupied.shape
    points = load_pcd(pcd_path)

    tf = None
    if tf_bag and Path(tf_bag).is_dir():
        tf = last_map_to_odom(Path(tf_bag))
    if tf is None:
        print("WARN: no map->odom found; assuming identity", file=sys.stderr)
        tf = (0.0, 0.0, 0.0)
    tx, ty, yaw = tf
    print(f"map->odom: x={tx:.3f} y={ty:.3f} yaw={math.degrees(yaw):.2f} deg")

    band = points[(points[:, 2] >= args.z_min) & (points[:, 2] <= args.z_max)]
    print(f"PCD points: {len(points)} total, {len(band)} in "
          f"z=[{args.z_min}, {args.z_max}]")
    c, s = math.cos(yaw), math.sin(yaw)
    mx = c * band[:, 0] - s * band[:, 1] + tx
    my = s * band[:, 0] + c * band[:, 1] + ty

    ix = np.floor((mx - ox) / res).astype(int)
    iy = np.floor((my - oy) / res).astype(int)
    inb = (ix >= 0) & (ix < w) & (iy >= 0) & (iy < h)
    n_out = int((~inb).sum())
    if n_out:
        print(f"WARN: {n_out} slice points fall outside the slam map extent")
    counts = np.zeros((h, w), dtype=np.int32)
    np.add.at(counts, (iy[inb], ix[inb]), 1)
    dlio_occ = counts >= args.min_points

    from scipy.ndimage import distance_transform_edt
    metrics = {}
    for name, src, ref in (("slam_vs_dlio", occupied, dlio_occ),
                           ("dlio_vs_slam", dlio_occ, occupied)):
        if not src.any() or not ref.any():
            print(f"{name}: EMPTY set (src {int(src.sum())}, "
                  f"ref {int(ref.sum())})")
            continue
        dist = distance_transform_edt(~ref) * res
        d = dist[src]
        matched = float((d <= args.match_tol).mean())
        metrics[name] = (d, matched)
        print(f"{name}: cells={int(src.sum())} mean={d.mean():.3f}m "
              f"median={np.median(d):.3f}m p95={np.percentile(d, 95):.3f}m "
              f"max={d.max():.3f}m matched(<= {args.match_tol}m)="
              f"{100 * matched:.1f}%")

    out_dir.mkdir(parents=True, exist_ok=True)
    if Image is None:
        print("WARN: PIL missing, skipping PNG output", file=sys.stderr)
        return 0

    def save(mask_img: np.ndarray, name: str):
        # Flip back so the PNG is oriented like RViz (y up).
        Image.fromarray(np.flipud(mask_img)).save(out_dir / name)
        print(f"wrote {out_dir / name}")

    canvas = np.full((h, w, 3), 255, dtype=np.uint8)
    canvas[free] = (232, 232, 232)
    both = occupied & dlio_occ
    slam_only = occupied & ~dlio_occ
    dlio_only = dlio_occ & ~occupied
    canvas[slam_only] = (220, 50, 47)
    canvas[dlio_only] = (38, 139, 210)
    canvas[both] = (0, 0, 0)
    save(canvas, "overlay.png")

    gray = np.full((h, w), 255, dtype=np.uint8)
    gray[free] = 232
    gray[occupied] = 0
    save(gray, "slam_map.png")

    gray2 = np.full((h, w), 255, dtype=np.uint8)
    gray2[dlio_occ] = 0
    save(gray2, "dlio_slice.png")
    print("overlay legend: black=agree, red=slam-only, blue=dlio-only, "
          "light gray=slam free space")
    return 0


if __name__ == "__main__":
    sys.exit(main())
