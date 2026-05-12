#!/usr/bin/env python3
"""Summarize perception-stack health metrics from a recorded ROS 2 bag.

Reads a rosbag2 SQLite directory and reports the numbers we use to validate
non-repetitive-LiDAR tuning before/after changes:

  - /odom_raw, /odom, /lidar/points, /lidar/points_undistorted,
    /lidar/points_scan_accumulated and /scan publication rates.
  - /scan angular coverage (% of bins with finite range, per scan).
  - odom -> base_footprint TF rate (the EKF's owned TF).
  - /map cell-count growth.

Usage:
  scripts/perception_metrics.py <bag_dir>

Example:
  scripts/perception_metrics.py /root/rosbags/go2_real/go2_real_mapping_20260512_120809
"""

from __future__ import annotations

import argparse
import math
import sqlite3
import statistics
import sys
from collections import Counter
from pathlib import Path


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("bag", type=Path, help="Path to a rosbag2 directory (contains metadata.yaml).")
    args = ap.parse_args()

    bag_dir: Path = args.bag
    if not bag_dir.is_dir():
        print(f"Not a directory: {bag_dir}", file=sys.stderr)
        return 2
    db_candidates = sorted(bag_dir.glob("*.db3"))
    if not db_candidates:
        print(f"No .db3 in {bag_dir}", file=sys.stderr)
        return 2
    db_path = db_candidates[0]

    # Lazy ROS imports so --help works without sourcing ROS.
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import LaserScan, PointCloud2
    from nav_msgs.msg import OccupancyGrid
    from tf2_msgs.msg import TFMessage

    con = sqlite3.connect(db_path)
    cur = con.cursor()

    cur.execute("SELECT id, name FROM topics")
    topic_ids = {name: tid for tid, name in cur.fetchall()}

    rate_topics = [
        "/odom_raw",
        "/odom",
        "/lidar/points",
        "/lidar/points_undistorted",
        "/lidar/points_scan_accumulated",
        "/scan",
        "/map",
    ]

    print(f"Bag : {bag_dir}\n")
    print(f"{'topic':<40}  {'count':>7}  {'dur':>6}  {'rate Hz':>10}")
    print("-" * 70)
    for name in rate_topics:
        tid = topic_ids.get(name)
        if tid is None:
            print(f"{name:<40}  {'-':>7}  {'-':>6}  {'-':>10}")
            continue
        cur.execute(f"SELECT timestamp FROM messages WHERE topic_id={tid} ORDER BY timestamp")
        stamps = [r[0] for r in cur.fetchall()]
        if not stamps:
            print(f"{name:<40}  {0:>7}  {'-':>6}  {'-':>10}")
            continue
        dur_ns = stamps[-1] - stamps[0]
        dur = dur_ns / 1e9 if dur_ns > 0 else 1
        rate = len(stamps) / dur if dur > 0 else 0
        print(f"{name:<40}  {len(stamps):>7}  {dur:>6.1f}  {rate:>10.2f}")

    # odom -> base_footprint TF rate (EKF-owned)
    tf_id = topic_ids.get("/tf")
    if tf_id is not None:
        cur.execute(f"SELECT data, timestamp FROM messages WHERE topic_id={tf_id} ORDER BY timestamp")
        pair_stamps: dict[tuple[str, str], list[int]] = {}
        for data, ts in cur.fetchall():
            msg = deserialize_message(bytes(data), TFMessage)
            for t in msg.transforms:
                key = (t.header.frame_id, t.child_frame_id)
                pair_stamps.setdefault(key, []).append(ts)
        print()
        print("TF rates (key pairs):")
        for key in (("odom", "base_footprint"), ("map", "odom"),
                    ("base_footprint", "base_link"),
                    ("base_footprint", "lidar_stabilized")):
            stamps = pair_stamps.get(key, [])
            if not stamps:
                print(f"  {key[0]:>20} -> {key[1]:<25}  not seen")
                continue
            dur_ns = stamps[-1] - stamps[0] if len(stamps) > 1 else 1
            dur = dur_ns / 1e9 if dur_ns > 0 else 1
            rate = len(stamps) / dur if dur > 0 else 0
            print(f"  {key[0]:>20} -> {key[1]:<25}  count={len(stamps):>6}  rate={rate:>7.2f} Hz")

    # /scan angular coverage (% bins with finite range)
    scan_id = topic_ids.get("/scan")
    if scan_id is not None:
        cur.execute(
            f"SELECT data FROM messages WHERE topic_id={scan_id} ORDER BY timestamp LIMIT 200"
        )
        rows = cur.fetchall()
        if rows:
            cov_pcts: list[float] = []
            n_bins_total = None
            for (data,) in rows:
                msg = deserialize_message(bytes(data), LaserScan)
                if n_bins_total is None:
                    n_bins_total = len(msg.ranges)
                hits = sum(
                    1 for r in msg.ranges if math.isfinite(r) and msg.range_min <= r <= msg.range_max
                )
                cov_pcts.append(100.0 * hits / max(1, len(msg.ranges)))
            print()
            print("/scan angular coverage (first 200 scans):")
            print(f"  bins per scan       : {n_bins_total}")
            print(f"  finite-range mean   : {statistics.mean(cov_pcts):.1f} %")
            print(f"  finite-range median : {statistics.median(cov_pcts):.1f} %")
            print(f"  finite-range min/max: {min(cov_pcts):.1f} / {max(cov_pcts):.1f} %")

    # /map growth (occupied + free cells)
    map_id = topic_ids.get("/map")
    if map_id is not None:
        cur.execute(f"SELECT data FROM messages WHERE topic_id={map_id} ORDER BY timestamp")
        rows = cur.fetchall()
        if rows:
            first = deserialize_message(bytes(rows[0][0]), OccupancyGrid)
            last = deserialize_message(bytes(rows[-1][0]), OccupancyGrid)

            def cell_counts(grid):
                occ = unk = free = 0
                for c in grid.data:
                    if c == -1:
                        unk += 1
                    elif c == 0:
                        free += 1
                    else:
                        occ += 1
                return occ, free, unk

            o1, f1, u1 = cell_counts(first)
            o2, f2, u2 = cell_counts(last)
            print()
            print("/map cell counts (first vs last):")
            print(f"  size first  : {first.info.width} x {first.info.height}")
            print(f"  size last   : {last.info.width} x {last.info.height}")
            print(f"  occupied    : {o1} -> {o2}")
            print(f"  free        : {f1} -> {f2}")
            print(f"  unknown     : {u1} -> {u2}")

    con.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
