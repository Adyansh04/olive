#!/usr/bin/env python3
"""Offline bag analysis: odometry frame_ids + fused-vs-ground-truth error.

Reads a recorded rosbag (mcap or sqlite3; file-level zstd compression is
handled automatically via the `zstd` CLI) and reports:
  - the frame_id / child_frame_id and first position of each odometry stream
    (a quick check that ground truth, wheel odom and the fused output are in the
    frames you expect);
  - fused vs ground-truth position error over time, split at the marker anchor.
    The pre-anchor error is the spawn->world frame offset (NOT drift): the fused
    trajectory starts spawn-relative and snaps into the world frame when the
    first marker anchors. Post-anchor, an early-vs-late comparison flags whether
    error is bounded or accumulating.

Complements ate_eval.py (which samples the live streams during a run); this
works entirely offline on a recorded bag.

Usage:
    ros2 run olive analyze_bag.py ~/olive_ws/bags/maze_square_3loops
    ros2 run olive analyze_bag.py <bag> --fused-topic /olive/odometry --gt-topic /ground_truth
"""
import argparse
import math
import os
import shutil
import subprocess
import tempfile

import rosbag2_py
import yaml
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def stamp_s(h):
    return h.stamp.sec + h.stamp.nanosec * 1e-9


def open_reader(bag):
    """Open a SequentialReader, transparently decompressing a file-zstd bag.

    Returns (reader, tempdir_or_None). Caller removes the tempdir when done.
    """
    meta_path = os.path.join(bag, "metadata.yaml")
    with open(meta_path) as f:
        meta = yaml.safe_load(f)["rosbag2_bagfile_information"]
    storage_id = meta["storage_identifier"]

    tmp = None
    uri = bag
    if str(meta.get("compression_mode", "")).upper() == "FILE":
        # rosbag2_py's plain reader can't decompress file-level zstd; expand the
        # splits to a temp dir and hand the reader an uncompressed copy.
        if not shutil.which("zstd"):
            raise SystemExit("bag is file-compressed but the `zstd` CLI was not found")
        tmp = tempfile.mkdtemp(prefix="olive_bag_")
        for rel in meta["relative_file_paths"]:
            src = os.path.join(bag, rel)
            base = os.path.basename(rel)
            dst = os.path.join(tmp, base[:-5] if base.endswith(".zstd") else base)
            subprocess.run(["zstd", "-d", "-q", src, "-o", dst], check=True)
        text = open(meta_path).read()
        text = text.replace("compression_format: zstd", "compression_format: ''")
        text = text.replace("compression_mode: FILE", "compression_mode: ''")
        text = text.replace(".mcap.zstd", ".mcap").replace(".db3.zstd", ".db3")
        with open(os.path.join(tmp, "metadata.yaml"), "w") as f:
            f.write(text)
        uri = tmp

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=uri, storage_id=storage_id),
                rosbag2_py.ConverterOptions("", ""))
    return reader, tmp


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("bag", help="path to the rosbag directory")
    ap.add_argument("--fused-topic", default="/olive/odometry")
    ap.add_argument("--gt-topic", default="/ground_truth")
    ap.add_argument("--wheel-topic", default="/odom")
    ap.add_argument("--anchor-thresh", type=float, default=0.5,
                    help="error (m) below which the trajectory is considered world-anchored")
    args = ap.parse_args()

    want = {args.fused_topic, args.gt_topic, args.wheel_topic}
    reader, tmp = open_reader(args.bag)
    try:
        types = {t.name: t.type for t in reader.get_all_topics_and_types()}
        gt, fused, wheel, first = [], [], [], {}
        while reader.has_next():
            topic, data, _ = reader.read_next()
            if topic not in want:
                continue
            msg = deserialize_message(data, get_message(types[topic]))
            p = msg.pose.pose.position
            rec = (stamp_s(msg.header), p.x, p.y, yaw_of(msg.pose.pose.orientation))
            if topic not in first:
                first[topic] = (msg.header.frame_id, msg.child_frame_id, p.x, p.y)
            (gt if topic == args.gt_topic else fused if topic == args.fused_topic
             else wheel).append(rec)
    finally:
        if tmp:
            shutil.rmtree(tmp, ignore_errors=True)

    print("=== FRAME IDS + first position ===")
    for t in (args.gt_topic, args.wheel_topic, args.fused_topic):
        f = first.get(t)
        if f:
            print(f"  {t:18s} frame_id={f[0]!r:10s} child={f[1]!r:16s} "
                  f"first_xy=({f[2]:+.2f},{f[3]:+.2f})")

    if not fused or not gt:
        print("\n(no fused/ground-truth samples found - check --fused-topic/--gt-topic)")
        return

    gt.sort()
    fused.sort()
    gi = 0
    errs = []
    t0 = fused[0][0]
    for (tf, xf, yf, _) in fused:
        while gi + 1 < len(gt) and abs(gt[gi + 1][0] - tf) <= abs(gt[gi][0] - tf):
            gi += 1
        _, xg, yg, _ = gt[gi]
        errs.append((tf - t0, math.hypot(xf - xg, yf - yg)))

    anchor_t = next((t for t, e in errs if e < args.anchor_thresh), None)
    post = [e for t, e in errs if anchor_t is not None and t >= anchor_t]

    print("\n=== FUSED vs GROUND TRUTH (position error) ===")
    print(f"  fused samples: {len(fused)}   gt samples: {len(gt)}   duration: {errs[-1][0]:.0f}s")
    print(f"  initial error (pre-anchor spawn-frame offset): {errs[0][1]:.2f} m")
    print(f"  first drop <{args.anchor_thresh} m (marker anchor) at: {anchor_t:.1f}s"
          if anchor_t is not None else "  never anchored (error stayed above threshold)")
    if post:
        rmse = math.sqrt(sum(e * e for e in post) / len(post))
        print(f"  POST-ANCHOR:  RMSE={rmse:.3f} m   mean={sum(post)/len(post):.3f} m   "
              f"max={max(post):.3f} m   final={post[-1]:.3f} m   (n={len(post)})")
        third = len(post) // 3
        if third:
            early = sum(post[:third]) / third
            late = sum(post[-third:]) / third
            print(f"  drift check:  early-third mean={early:.3f} m  ->  late-third mean={late:.3f} m  "
                  f"({'GROWING' if late > early + 0.02 else 'stable/bounded'})")

    if wheel:
        print("\n=== WHEEL odom (spawn-relative; expected NOT to match world GT) ===")
        print(f"  first_xy=({wheel[0][1]:+.2f},{wheel[0][2]:+.2f})  "
              f"last_xy=({wheel[-1][1]:+.2f},{wheel[-1][2]:+.2f})")


if __name__ == "__main__":
    main()
