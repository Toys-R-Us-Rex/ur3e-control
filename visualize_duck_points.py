import base64
import json

from matplotlib import cm, pyplot as plt
import matplotlib
import trimesh

import numpy as np


RESULTS_PATH = "/tmp/results3_.json"


def main():
    with open(RESULTS_PATH, "r") as f:
        results = json.load(f)
    
    with open("duckify_simulation/paths/random.json", "r") as f:
        trace_data = json.load(f)
    
    path = np.array(trace_data["traces"][0]["path"])
    pts = path[:, 0]
    
    counts = np.zeros(len(pts))
    valid_counts = np.zeros(len(pts))
    
    for result in results:
        indices = np.array(result["indices"]) - 1
        counts[indices] += 1
        mask = result["mask"].encode("utf-8")
        mask = base64.b64decode(mask)
        mask = np.array(list(mask), dtype=np.uint8)
        mask = np.unpackbits(mask)
        mask = mask[:len(indices)]
        valid_counts[indices] += mask
    
    print(counts)
    print(valid_counts)
    
    ratios = valid_counts.astype(np.float64) / counts
    plt.hist(ratios)
    plt.show()
    
    cmap = cm.get_cmap("viridis")
    norm = matplotlib.colors.Normalize(vmin=0, vmax=1)
    scalar_map = cm.ScalarMappable(norm=norm, cmap=cmap)
    colors = scalar_map.to_rgba(ratios)
    colors[valid_counts == 0] = (255, 0, 0, 255)
    colors[counts == 0] = (0, 0, 0, 0)
    #color = np.clip(ratios * 255, 0, 255).astype(np.uint8)
    #print(np.unique(color, return_counts=True))
    #colors = np.vstack([
    #    255 - color, color, np.zeros(len(color)), np.full(len(color), 255)
    #]).T
    #print(colors.shape)
    #print(colors[:10])
    
    mesh = trimesh.load_mesh("duckify_simulation/3d_objects/duck_uv.stl")
    cloud = trimesh.PointCloud(pts, colors=colors)
    scene = trimesh.Scene([mesh, cloud])
    scene.show()

if __name__ == "__main__":
    main()
