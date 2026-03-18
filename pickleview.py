import sys
import pickle
import numpy as np

path = sys.argv[1]
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
