import sys
import pickle
import numpy as np
import argparse

parser = argparse.ArgumentParser(description="View pickle file contents")
parser.add_argument("path", help="Path to pickle file")
parser.add_argument("--short", "-s", action="store_true",
                    help="Truncate waypoints to first 10 elements")
args = parser.parse_args()

path = args.path
with open(path, "rb") as f:
    data = pickle.load(f)

def preview(val, limit=10):
    if isinstance(val, np.ndarray):
        print(f"  ndarray shape={val.shape} dtype={val.dtype}")
        flat = val.flatten()
        print(f"  first {min(limit, len(flat))}: {flat[:limit]}")
    elif isinstance(val, (list, tuple)):
        print(f"  {type(val).__name__} len={len(val)}")
        for item in val[:limit]:
            if args.short and hasattr(item, 'waypoints'):
                fields = {k: v for k, v in item.__dict__.items() if k != 'waypoints'}
                wp = item.waypoints
                print(f"    {type(item).__name__}({', '.join(f'{k}={v}' for k, v in fields.items())}, "
                      f"waypoints=[{len(wp)} total, first {limit} shown])")
                for w in wp[:limit]:
                    print(f"      {w}")
            else:
                print(f"    {item}")
    else:
        s = str(val)
        print(f"  {type(val).__name__}: {s[:200]}")

if isinstance(data, dict):
    print(f"dict with {len(data)} keys:\n")
    for key in data:
        print(f"[{key}] ({type(data[key]).__name__})")
        preview(data[key])
        print()
elif isinstance(data, (list, tuple)):
    print(f"{type(data).__name__} with {len(data)} entries:\n")
    for item in data[:10]:
        preview(item)
        print()
else:
    preview(data)
