#!/usr/bin/env python3
"""
plot_poses.py -- Visualize AprilTag pose distributions from cycle JSON logs.

Quantifies PnP pose ambiguity by plotting raw camera-frame translations
recorded during stationary sessions. Compare two sessions (e.g. 5cm vs 20cm
tag) to see how tag size affects estimation stability.

Usage:
  python plot_poses.py <log_dir>
  python plot_poses.py <log_dir_5cm> <log_dir_20cm> --label1 "5cm" --label2 "20cm"
  python plot_poses.py <log_dir> --tag-id 2 --out my_plot.png
"""

import argparse
import glob
import json
import os
import sys

import numpy as np
import matplotlib.pyplot as plt


def load_poses(log_dir, tag_id):
    """Load all cycle JSON files and return (N, 4) array of [ts, tx, ty, tz]."""
    pattern = os.path.join(log_dir, "cycle_*_raw_*.json")
    files = sorted(glob.glob(pattern))
    if not files:
        print(f"  warning: no cycle_*_raw_*.json files in {log_dir}", file=sys.stderr)
        return np.empty((0, 4))

    records = []
    for fpath in files:
        with open(fpath) as f:
            frames = json.load(f)
        for frame in frames:
            for tag in frame.get("tags", []):
                if tag["id"] == tag_id and tag.get("pose_t"):
                    t = tag["pose_t"]
                    records.append([frame["ts"], t[0], t[1], t[2]])

    if not records:
        return np.empty((0, 4))

    arr = np.array(records, dtype=np.float64)
    arr[:, 0] -= arr[0, 0]  # relative timestamps
    return arr


def print_stats(label, data):
    if len(data) == 0:
        print(f"  {label}: no data")
        return
    print(f"  {label} ({len(data)} frames, {data[-1, 0]:.1f}s):")
    for i, axis in enumerate("XYZ"):
        v = data[:, i + 1]
        print(f"    {axis}: mean={v.mean():.4f}  std={v.std():.4f}  "
              f"range=[{v.min():.4f}, {v.max():.4f}]  m")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                  formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("dirs", nargs="+", metavar="log_dir",
                    help="1 or 2 session log directories")
    ap.add_argument("--tag-id", type=int, default=2,
                    help="AprilTag ID to plot (default: 2)")
    ap.add_argument("--label1", default="session 1")
    ap.add_argument("--label2", default="session 2")
    ap.add_argument("--out", default="pose_distributions.png",
                    help="output PNG path (default: pose_distributions.png)")
    args = ap.parse_args()

    dirs = args.dirs[:2]
    labels = [args.label1, args.label2][: len(dirs)]
    colors = ["steelblue", "tomato"]

    print(f"Tag ID: {args.tag_id}")
    datasets = []
    for label, d in zip(labels, dirs):
        print(f"\nLoading {label}: {d}")
        data = load_poses(d, args.tag_id)
        print_stats(label, data)
        datasets.append(data)

    fig, axes = plt.subplots(2, 3, figsize=(14, 7))
    ax_ts   = axes[0]
    ax_hist = axes[1]

    for data, label, color in zip(datasets, labels, colors):
        if len(data) == 0:
            continue
        ts = data[:, 0]
        for i, axis in enumerate("XYZ"):
            v = data[:, i + 1]
            ax_ts[i].plot(ts, v, linewidth=0.7, alpha=0.7, color=color, label=label)
            ax_hist[i].hist(v, bins=80, density=True, alpha=0.5, color=color, label=label)

    axis_labels = ["X (m)", "Y (m)", "Z (m)"]
    for i, lbl in enumerate(axis_labels):
        ax_ts[i].set_title(f"{lbl} over time")
        ax_ts[i].set_xlabel("Time (s)")
        ax_ts[i].set_ylabel(lbl)
        ax_ts[i].legend(fontsize=8)
        ax_ts[i].grid(True, alpha=0.3)

        ax_hist[i].set_title(f"{lbl} distribution")
        ax_hist[i].set_xlabel(lbl)
        ax_hist[i].set_ylabel("Density")
        ax_hist[i].legend(fontsize=8)
        ax_hist[i].grid(True, alpha=0.3)

    fig.suptitle(
        f"Tag {args.tag_id} PnP pose ambiguity — camera-frame translations",
        fontsize=13,
    )
    plt.tight_layout()
    plt.savefig(args.out, dpi=150)
    print(f"\nSaved: {args.out}")
    plt.show()


if __name__ == "__main__":
    main()
