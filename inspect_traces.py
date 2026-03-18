#!/usr/bin/env python3
"""Inspect trace segment pickle files — shows per-segment details and summary stats."""

import sys
import pickle
from collections import Counter

DEFAULT_PATH = "save_data_test/20260318/trace_segments_0.pkl"

path = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_PATH
print(f"Loading: {path}\n")

with open(path, "rb") as f:
    data = pickle.load(f)

segments = data["segments"]
total = len(segments)
print(f"Total segments: {total}\n")

side_counts = Counter()
side_waypoints = Counter()
total_waypoints = 0

print(f"{'#':>3}  {'Color':>5}  {'Side':<6}  {'WPs':>5}  {'First WP':<30}  {'Last WP':<30}")
print("-" * 90)

for i, seg in enumerate(segments):
    wps = seg.waypoints or []
    n_wp = len(wps)
    side_name = seg.side.name if seg.side else "?"

    first = f"({', '.join(f'{v:.3f}' for v in wps[0][:3])})" if wps else "-"
    last = f"({', '.join(f'{v:.3f}' for v in wps[-1][:3])})" if wps else "-"

    print(f"{i:>3}  {seg.color:>5}  {side_name:<6}  {n_wp:>5}  {first:<30}  {last:<30}")

    total_waypoints += n_wp
    side_counts[side_name] += 1
    side_waypoints[side_name] += n_wp

print(f"\n--- Summary ---")
print(f"Total waypoints: {total_waypoints}")
print(f"Segments by side: {dict(side_counts)}")
print(f"Waypoints by side: {dict(side_waypoints)}")
