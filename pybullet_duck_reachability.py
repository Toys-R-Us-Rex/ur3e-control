#!/usr/bin/env python3

"""general form of pybullet_collision_test.py aimed at testing any point on the Duck surface """

import math
import sys
import time
from pathlib import Path

import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from matplotlib.widgets import Slider
import numpy as np
import pybullet as pb
import trimesh

sys.path.insert(0, str(Path(__file__).resolve().parent))

from URBasic import TCP6D
from scipy.spatial.transform import Rotation as Rot

from duckify_simulation.duckify_sim.robot_control import SimRobotControl
from src.calibration import get_tcp_offset
from src.safety import CollisionChecker
from src.transformation import build_manual_transform
from src.utils import normal_to_rotvec

# ===========================  Setup  ============================

robot = SimRobotControl()

tcps = [
    [0.09923226380963619, -0.3936237333456494, 0.2241707102377116, 0.576534502783353, -2.801623474567781, -0.4841337776020336],
    [-0.05415645689244093, -0.3953984087553033, 0.21928851997659993, 0.23402931706237418, 2.778033572687844, 0.45840199634357587],
    [-0.054524138620445695, -0.39546163267180845, 0.2194560899784096, 0.2335390262119786, 2.783785824873644, 0.4510348107203675],
    [0.0752223292033933, -0.40520983496228175, 0.2260599738507684, 0.4572457692623414, -2.9109062545914886, -0.5364679269112991],
    [0.0829983763105315, -0.3145440306229818, 0.23651078628355368, 0.3951666302237936, -2.8778640875293946, 0.020826760853426392],
    [0.17537792453011916, -0.27966304416915055, 0.1871802429879547, 0.4399208472865239, -2.4166507384311493, 0.11158336986030104],
    [-0.11994350407194002, -0.2833976738613014, 0.18952203929656486, -0.11587882239515394, 2.488783491737581, -0.2874871978684851],
    [0.042587583213620814, -0.42502159418902, 0.2219296443030877, -1.819892078036102, 2.2285954928177354, 0.5297978049692356],
    [0.04244370933942005, -0.42547028481239557, 0.22238439939526014, -1.8232450157901965, 2.2322450410810823, 0.5277815763416596],
    [0.020874413390515264, -0.34192399335641094, 0.2416659479631329, -1.6019421625827266, 2.6368378551232445, 0.06821140243218453],
    [0.020803283214023204, -0.3418580959196876, 0.24363220771625155, -1.6034030755162376, 2.6412352323485133, 0.06264503078869965],
    [-0.0790726920318296, -0.26904313811943686, 0.21128926726360914, -1.5548308665720385, 2.3598614347530305, -0.6531326432466081],
    [0.0427270175708247, -0.2473905824164213, 0.2299557666201055, 2.1997240823524864, -1.7930464879794845, 0.21897511307224746],
    [0.13236126549367827, -0.37709210594126424, 0.2134888319694367, 2.113428037038218, -2.0042534442643, -0.709899151721018],
    [0.053381366331223756, -0.3159752956877128, 0.24229245067174737, 1.8209996559857016, -2.4222938552259685, -0.050512659771693856],
    [-0.049955483412195634, -0.41217334754184276, 0.21506155931685994, -1.6191731898646298, 2.109641966964828, 0.12393449118187727],
    [0.11965373370731985, -0.426189467377058, 0.20111114739956754, -2.3836120944741555, 1.6242241381287574, 0.8568802013152502],
    [0.1072294696915219, -0.2674062588065899, 0.22084409245177472, 2.144447557404073, -1.698541221842688, -0.16060169509460684],
    [-0.10731169114667957, -0.19413670798855612, 0.15654354818487162, 2.3277646358217945, -1.2822755883172325, 1.191980512607587],
    [0.020926595559954138, -0.3139355562333332, 0.24095599082082952, 3.0691506381776072, -0.3992164073191759, 0.04457895483726155],
]

tcp_offset = get_tcp_offset(tcps)
robot.set_tcp(tcp_offset)

# Duck placement: (0, -0.30, 0.155) in meters
obj2robot = build_manual_transform(rz_deg=0.0, translation=(0.0, -0.30, 0.155))
T = obj2robot.T
pos = T[:3, 3].tolist()
R = T[:3, :3]
scale = np.linalg.norm(R[:, 0])
R_pure = R / scale
quat = Rot.from_matrix(R_pure).as_quat().tolist()

checker = CollisionChecker(
    obstacle_stls=[
        {
            'path': 'duckify_simulation/3d_objects/duck_uv.stl',
            'scale': [0.001, 0.001, 0.001],
            'position': pos,
            'orientation': quat,
        },
        {
            'path': 'duckify_simulation/3d_objects/workspace.stl',
            'scale': [1, 1, 1],
            'position': [0, 0, 0],
            'orientation': [0, 0, 0, 1],
            'exclude_links': [1, 2, 3, 4],
        },
        {
            'path': 'duckify_simulation/3d_objects/support_duck_simulation.stl',
            'scale': [0.001, 0.001, 0.001],
            'position': pos,
            'orientation': quat,
        },
    ],
    gui=True,
)

print(f"PyBullet GUI running (cid={checker.cid})")

# Center camera rotation on the duck base
pb.resetDebugVisualizerCamera(
    cameraDistance=0.6,
    cameraYaw=45,
    cameraPitch=-30,
    cameraTargetPosition=pos,
    physicsClientId=checker.cid,
)

# ========================  Load duck mesh  ========================

mesh = trimesh.load('duckify_simulation/3d_objects/duck_uv.stl')

# Sample ~1 point per 10 mm² of surface area
n_samples = int(mesh.area / 10)
print(f"Duck surface area: {mesh.area:.0f} mm² -> sampling {n_samples} points")

samples, face_indices = trimesh.sample.sample_surface(mesh, n_samples)
normals_raw = mesh.face_normals[face_indices]

# Filter: back half (Y >= 0) and exclude flat base (Z >= 1)
mask = (samples[:, 1] >= 0) & (samples[:, 2] >= 1.0)
samples = samples[mask]
normals_raw = normals_raw[mask]
face_indices = face_indices[mask]
n_filtered = len(samples)
print(f"After filtering (Y>=0, Z>=1): {n_filtered} points (removed {n_samples - n_filtered})")

# Save local coords for matplotlib plot later
samples_local = samples.copy()

# Transform points from STL mm coords to robot frame (T includes 0.001 scale)
ones = np.ones((n_filtered, 1))
pts_h = np.hstack([samples, ones])
pts_robot = (T @ pts_h.T).T[:, :3]

# Transform normals by pure rotation (no scale, no translation)
normals_robot = (R_pure @ normals_raw.T).T
normals_robot = normals_robot / np.linalg.norm(normals_robot, axis=1, keepdims=True)

# ========================  Probe surface points  ========================

total = n_filtered
print(f"\nProbing {total} surface points with cone search (orientation_search=True)...")

reachable_count = 0
reachable_mask = np.zeros(total, dtype=bool)
t_start = time.time()

for i in range(total):
    x, y, z = float(pts_robot[i, 0]), float(pts_robot[i, 1]), float(pts_robot[i, 2])
    normal = normals_robot[i]

    rv = normal_to_rotvec(normal)
    rx, ry, rz = float(rv[0]), float(rv[1]), float(rv[2])
    tcp = TCP6D.createFromMetersRadians(x, y, z, rx, ry, rz)

    ok, q, reason, tcp_used = checker.validate_tcp(
        robot, tcp, check_obstacle=False, orientation_search=True,
        max_cone_angle=math.radians(25),
    )

    if ok:
        reachable_count += 1
        reachable_mask[i] = True

    if (i + 1) % 500 == 0 or i == total - 1:
        elapsed = time.time() - t_start
        rate = (i + 1) / elapsed
        eta = (total - i - 1) / rate if rate > 0 else 0
        print(f"  [{i+1}/{total}] {reachable_count} reachable so far "
              f"({elapsed:.0f}s elapsed, ~{eta:.0f}s remaining)")

# ========================  Summary  ========================

elapsed = time.time() - t_start
print(f"\n{'=' * 50}")
print(f"Duck surface reachability ({elapsed:.1f}s)")
print(f"{'=' * 50}")
print(f"  Duck placement:        (0.0, -0.30, 0.155)")
print(f"  Total surface points:  {total}")
print(f"  Reachable:             {reachable_count} ({reachable_count / total:.1%})")
print(f"  Unreachable:           {total - reachable_count} ({(total - reachable_count) / total:.1%})")

# ========================  Matplotlib 3D plot  ========================

fig = plt.figure(figsize=(10, 10))
ax = fig.add_subplot(111, projection='3d')

# Draw mesh faces as light gray
triangles = mesh.vertices[mesh.faces]
poly3d = Poly3DCollection(triangles, facecolors='lightgray', edgecolors='silver',
                          linewidths=0.1, alpha=1.0)
ax.add_collection3d(poly3d)

# Offset dots slightly outward along normals so they sit above the opaque mesh
offset_pts = samples_local + normals_raw * 0.5  # 0.5 mm outward

# Scatter reachability points in local STL coords
unreachable = ~reachable_mask
ax.scatter(offset_pts[unreachable, 0], offset_pts[unreachable, 1],
           offset_pts[unreachable, 2],
           c='red', s=8, alpha=0.9, label=f'Unreachable ({unreachable.sum()})', depthshade=False)
ax.scatter(offset_pts[reachable_mask, 0], offset_pts[reachable_mask, 1],
           offset_pts[reachable_mask, 2],
           c='green', s=8, alpha=0.9, label=f'Reachable ({reachable_mask.sum()})', depthshade=False)

ax.set_xlabel('X (mm)')
ax.set_ylabel('Y (mm)')
ax.set_zlabel('Z (mm)')
ax.set_title(f'Duck Surface Reachability\n'
             f'{reachable_count}/{total} reachable ({reachable_count / total:.1%})')
ax.legend(loc='upper right')

# Equal aspect ratio
all_pts = mesh.vertices
max_range = (all_pts.max(axis=0) - all_pts.min(axis=0)).max() / 2
mid = (all_pts.max(axis=0) + all_pts.min(axis=0)) / 2
ax.set_xlim(mid[0] - max_range, mid[0] + max_range)
ax.set_ylim(mid[1] - max_range, mid[1] + max_range)
ax.set_zlim(mid[2] - max_range, mid[2] + max_range)

# Default view: looking from -Y toward +Y (seeing the Y>0 face)
DEFAULT_ELEV = 15
DEFAULT_AZIM = -90
ax.view_init(elev=DEFAULT_ELEV, azim=DEFAULT_AZIM)

# Azimuth slider
fig.subplots_adjust(bottom=0.12)
ax_slider = fig.add_axes([0.2, 0.03, 0.6, 0.03])
slider = Slider(ax_slider, 'Azimuth', -180, 180, valinit=DEFAULT_AZIM, valstep=1)

def update_azim(val):
    ax.view_init(elev=DEFAULT_ELEV, azim=slider.val)
    fig.canvas.draw_idle()

slider.on_changed(update_azim)

plt.show(block=False)

input("\nPress ENTER to close PyBullet and matplotlib...")
plt.close(fig)
if pb.isConnected(checker.cid):
    pb.disconnect(checker.cid)
    print("PyBullet disconnected")
