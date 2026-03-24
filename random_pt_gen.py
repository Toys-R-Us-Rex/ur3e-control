import json

import numpy as np

import trimesh
import trimesh.sample

def discard_bottom(pts: np.ndarray, face_idx: np.ndarray):
    where = pts[:, 2] > 5
    return pts[where], face_idx[where]

def main():
    mesh = trimesh.load_mesh("duckify_simulation/3d_objects/duck_uv.stl")
    pts, face_idx = trimesh.sample.sample_surface(
        mesh,
        count=2000,
        seed=42
    )
    pts, face_idx = discard_bottom(pts, face_idx)
    normals = mesh.face_normals[face_idx]
    pairs = zip(pts.tolist(), normals.tolist())
    cloud = trimesh.PointCloud(pts, colors=(255, 0, 0))
    scene = trimesh.Scene([mesh, cloud])
    scene.show()

    data = {
        "traces": [
            {
                "color": 0,
                "path": list(pairs)
            }
        ]
    }

    with open("duckify_simulation/paths/random.json", "w") as f:
        json.dump(data, f)

if __name__ == "__main__":
    main()
