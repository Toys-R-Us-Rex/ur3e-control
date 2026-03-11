#!/usr/bin/env python3
"""Standalone PyBullet drawing-pipeline visualizer — no Gazebo or real robot needed.

Orchestrates the drawing pipeline manually (bypassing build_drawing_plan)
so that each stage can be inspected visually in the PyBullet GUI.
"""

import logging
import sys
from pathlib import Path

import numpy as np
import json
from dataclasses import asdict


sys.path.insert(0, str(Path(__file__).resolve().parent))

# Silence verbose IK diagnostics from robot_control and safety
logging.basicConfig(level=logging.WARNING)

import pybullet as pb
from URBasic import Joint6D, TCP6D, UrScript
from duckify_simulation.duckify_sim.robot_control import SimRobotControl
from scipy.spatial.transform import Rotation as Rot
import src.calibration as ur_calibration
import src.transformation as ur_transformation
from src.safety import CollisionChecker
from src.computation import (
    compute_draw_motion, load_traces,
    _validate_surface_points, _split_into_runs, _hover_tcp,
    compute_positioning_motion,
    Segment, MotionType,
)
from src.config import DRAW_V, DRAW_A, APPROACH_V, APPROACH_A, TRAVEL_V, TRAVEL_A

CALIBRE_TRANSFORMATION = False

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def draw_sphere(position, color, radius=0.0008):
    vis = pb.createVisualShape(pb.GEOM_SPHERE, radius=radius, rgbaColor=[*color, 1],
                               physicsClientId=checker.cid)
    body_id = pb.createMultiBody(baseMass=0, baseVisualShapeIndex=vis, basePosition=position,
                                 physicsClientId=checker.cid)
    return body_id


RUN_COLORS = [
    [1, 1, 0],       # yellow
    [0, 1, 0],       # green
    [0, 0.5, 1],     # blue
    [1, 0.5, 0],     # orange
    [1, 0, 1],       # magenta
    [0, 1, 1],       # cyan
    [0.5, 1, 0],     # lime
    [1, 0.7, 0.8],   # pink
    [0.6, 0.3, 1],   # purple
    [1, 1, 0.5],     # light yellow
]


def draw_line_strip(waypoints, color, width=2):
    for i in range(len(waypoints) - 1):
        a = [waypoints[i].x, waypoints[i].y, waypoints[i].z]
        b = [waypoints[i + 1].x, waypoints[i + 1].y, waypoints[i + 1].z]
        pb.addUserDebugLine(a, b, color, lineWidth=width, physicsClientId=checker.cid)


# Helper
def move_simple(robot: SimRobotControl|UrScript, motion, v=0.25, a=1.2):
    for m in motion:
        if isinstance(m, TCP6D):
            robot.movel(m, v, a, wait=True)
        
        elif isinstance(m, Joint6D):
            robot.movej(m, v, a, wait=True)

# ===========================  STAGE 0 — Setup  ============================
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
    [0.020926595559954138, -0.3139355562333332, 0.24095599082082952, 3.0691506381776072, -0.3992164073191759, 0.04457895483726155]
]

tcp_offset = ur_calibration.get_tcp_offset(tcps)
robot.set_tcp(tcp_offset)

home = Joint6D.createFromRadians(1.8859, -1.4452, 1.2389, -1.3639, -1.5693, -0.3849)

# ========================  STAGE 1 — Load traces  =========================
file_path = "duckify_simulation/paths/duck_uv-fancy_test_duck-trace.json"
traces, data = load_traces(file_path)
print(f"Trace data: {len(traces)} traces")

if CALIBRE_TRANSFORMATION:
    p_world = np.array(data["calibration"])
    p_tcp = ur_transformation.collect_data(robot, p_world)
    p_world_stl = ur_transformation.obj_to_stl(p_world)
    obj2robot = ur_transformation.create_transformation(p_world_stl, p_tcp)
else:
    obj2robot = ur_transformation.build_manual_transform(rz_deg=0.0, translation=(0.3, -0.4, 0.3))

selected_trace_indices_eyes = [0, 9]
selected_trace_indices_sideInf = [76, 77, 78]
selected_trace_indices_sideIsc = [19, 41, 59]
selected_trace_indices_medium = [0, 9, 19, 41, 59, 76, 77, 78, 138, 139, 140, 141, 142, 143, 144, 145]
traces = [traces[i] for i in selected_trace_indices_sideIsc]

# ==================  STAGE 2 — Transform raw -> TCP6D  ====================
# compute_draw_motion runs ONCE per trace; strip hover to get surface-only TCPs
surface_tcps_per_trace = []
for t in traces:
    draw_motion = compute_draw_motion(t, obj2robot, max_pts=1000)
    surface_tcps_per_trace.append(draw_motion[1:-1])  # strip hover entry/exit

print(f"Transformed {len(surface_tcps_per_trace)} traces to TCP6D")

# ============  STAGE 3 — Launch PyBullet + preview surface points  =========
T = obj2robot.T
pos = T[:3, 3].tolist()
R = T[:3, :3]
scale = np.linalg.norm(R[:, 0])
R_pure = R / scale
quat = Rot.from_matrix(R_pure).as_quat().tolist()

checker = CollisionChecker(
    obstacle_stls=[{
        'path': 'duckify_simulation/3d_objects/duck_model.stl',
        'scale': [0.001, 0.001, 0.001],
        'position': pos,
        'orientation': quat,
    }],
    gui=True,
)
print(f"PyBullet GUI running (cid={checker.cid})")
print(f"Robot body id: {checker.robot_id}")
print(f"Obstacle ids:  {checker.obstacle_ids}")

for oid in checker.obstacle_ids:
    pos_ob, orn = pb.getBasePositionAndOrientation(oid, physicsClientId=checker.cid)
    aabb_min, aabb_max = pb.getAABB(oid, physicsClientId=checker.cid)
    print(f"\nObstacle {oid}:")
    print(f"  Position:    {pos_ob}")
    print(f"  Orientation: {orn}")
    print(f"  AABB min:    {[f'{v:.4f}' for v in aabb_min]}")
    print(f"  AABB max:    {[f'{v:.4f}' for v in aabb_max]}")
    print(f"  Size (m):    ({aabb_max[0]-aabb_min[0]:.4f}, {aabb_max[1]-aabb_min[1]:.4f}, {aabb_max[2]-aabb_min[2]:.4f})")

# Set home pose
home_list = home.toList()
checker.set_joint_angles(home_list)
print(f"\nSet PyBullet robot to home: {home_list}")

# Preview surface TCPs as orange lines (no hover points)
preview_line_ids = []
for surface_pts in surface_tcps_per_trace:
    for idx in range(1, len(surface_pts)):
        prev_wp = surface_pts[idx - 1].toList()[:3]
        cur_wp = surface_pts[idx].toList()[:3]
        lid = pb.addUserDebugLine(prev_wp, cur_wp, [1, 0.5, 0], lineWidth=1,
                                  physicsClientId=checker.cid)
        preview_line_ids.append(lid)

print(f"\nShowing {len(surface_tcps_per_trace)} traces as orange lines (surface only, no hover).")
input("Press ENTER to continue to validation...")

for lid in preview_line_ids:
    pb.removeUserDebugItem(lid, physicsClientId=checker.cid)

# ===============  STAGE 4 — Validate surface points + visualize  ===========
print("\nValidating all trace waypoints...")

valid_masks_per_trace = []
validation_sphere_ids = []
for trace_i, surface_pts in enumerate(surface_tcps_per_trace):
    n_pts = len(surface_pts)
    print(f"  Trace {trace_i} ({n_pts} pts): ", end="", flush=True)

    valid_mask, reasons = _validate_surface_points(checker, robot, surface_pts)
    valid_masks_per_trace.append(valid_mask)

    ok_count = sum(valid_mask)
    fail_count = n_pts - ok_count

    for i, (wp, ok, reason) in enumerate(zip(surface_pts, valid_mask, reasons)):
        pos_wp = wp.toList()[:3]
        if ok:
            bid = draw_sphere(pos_wp, [1, 1, 0])   # yellow = valid
        else:
            bid = draw_sphere(pos_wp, [1, 0, 0])    # red = failed
            print(f"\n    pt {i} SKIP ({reason})", end="", flush=True)
        validation_sphere_ids.append(bid)

    print(f"\n    => {ok_count} OK, {fail_count} skipped")

input("\nPress ENTER to continue to run splitting...")

# Clear validation spheres (disable rendering for speed)
pb.configureDebugVisualizer(pb.COV_ENABLE_RENDERING, 0, physicsClientId=checker.cid)
for bid in validation_sphere_ids:
    pb.removeBody(bid, physicsClientId=checker.cid)
pb.configureDebugVisualizer(pb.COV_ENABLE_RENDERING, 1, physicsClientId=checker.cid)

# ===================  STAGE 5 — Split into runs  ==========================
print("\nSplitting traces into consecutive valid runs...")

runs_per_trace = []
run_sphere_ids = []
global_run_idx = 0
for trace_i, valid_mask in enumerate(valid_masks_per_trace):
    runs = _split_into_runs(valid_mask)
    runs_per_trace.append(runs)

    if not runs:
        print(f"  Trace {trace_i}: NO valid runs — trace will be skipped")
    else:
        gaps = []
        for j in range(len(runs) - 1):
            gaps.append(f"{runs[j][1]+1}-{runs[j+1][0]-1}")
        gap_str = f", gaps at [{', '.join(gaps)}]" if gaps else ""
        print(f"  Trace {trace_i}: {len(runs)} run(s) — {[f'{s}-{e}' for s, e in runs]}{gap_str}")

        surface_pts = surface_tcps_per_trace[trace_i]
        for run_start, run_end in runs:
            color = RUN_COLORS[global_run_idx % len(RUN_COLORS)]
            for wp in surface_pts[run_start:run_end + 1]:
                bid = draw_sphere(wp.toList()[:3], color)
                run_sphere_ids.append(bid)
            print(f"    Run {global_run_idx} ({run_start}-{run_end}): color {color}")
            global_run_idx += 1

input("\nPress ENTER to continue to hover generation...")

# ==========  STAGE 6 — Hover generation + critical validation  ============
print("\nGenerating and validating hover points...")

validated_runs = []  # list of (trace_i, run_start, run_end, h_entry, h_exit, run_surface)
abort = False
hover_run_idx = 0

for trace_i, (surface_pts, runs) in enumerate(zip(surface_tcps_per_trace, runs_per_trace)):
    for run_start, run_end in runs:
        run_color = RUN_COLORS[hover_run_idx % len(RUN_COLORS)]
        run_surface = surface_pts[run_start:run_end + 1]

        h_entry = _hover_tcp(run_surface[0])
        h_exit = _hover_tcp(run_surface[-1])

        # Validate hover entry
        print(f"  Trace {trace_i} run ({run_start}-{run_end}) entry: ", end="", flush=True)
        ok, _, reason, h_entry_used = checker.validate_tcp(
            robot, h_entry, check_obstacle=True, orientation_search=True,
        )
        if not ok:
            print(f"FAILED ({reason})")
            draw_sphere(h_entry.toList()[:3], [1, 0, 0], radius=0.001)  # red = failed
            abort = True
            hover_run_idx += 1
            continue
        h_entry = h_entry_used  # use adjusted orientation if cone search changed it
        print("OK", end="")

        # Validate hover exit
        print(f" | exit: ", end="", flush=True)
        ok, _, reason, h_exit_used = checker.validate_tcp(
            robot, h_exit, check_obstacle=True, orientation_search=True,
        )
        if not ok:
            print(f"FAILED ({reason})")
            draw_sphere(h_exit.toList()[:3], [1, 0, 0], radius=0.001)  # red = failed
            abort = True
            hover_run_idx += 1
            continue
        h_exit = h_exit_used  # use adjusted orientation if cone search changed it
        print(f"OK — {len(run_surface)} draw pts")

        # Visualize valid hovers in the run's color
        draw_sphere(h_entry.toList()[:3], run_color, radius=0.001)
        draw_sphere(h_exit.toList()[:3], run_color, radius=0.001)

        validated_runs.append((trace_i, run_start, run_end, h_entry, h_exit, run_surface))
        hover_run_idx += 1

if abort:
    print("\nABORT: one or more hover validations failed (see CRITICAL messages above).")
    input("Press ENTER to close PyBullet...")
    if pb.isConnected(checker.cid):
        pb.disconnect(checker.cid)
    sys.exit(1)

print(f"\nAll hover points valid. {len(validated_runs)} run(s) ready for planning.")
input("Press ENTER to continue to pathfinding...")

# ===========  STAGE 7 — Assemble segments + pathfinding  ==================
# Enable pathfinding logging so we see why things fail
logging.getLogger("src.pathfinding").setLevel(logging.INFO)

print("\nAssembling segments and computing travel paths...")

def _fmt_tcp(tcp):
    return f"({tcp.x:.4f}, {tcp.y:.4f}, {tcp.z:.4f})"

segments = []
current_tcp = robot.get_fk(home)
current_label = "HOME"

for run_i, (trace_i, run_start, run_end, h_entry, h_exit, run_surface) in enumerate(validated_runs):
    entry_label = f"Run{run_i} hover-entry"
    exit_label = f"Run{run_i} hover-exit"

    # TRAVEL: current position → hover entry of this run
    print(f"\n  [{current_label}] → [{entry_label}]")
    print(f"    from {_fmt_tcp(current_tcp)}  to {_fmt_tcp(h_entry)}")
    try:
        travel_wps = compute_positioning_motion(robot, checker, current_tcp, h_entry)
        segments.append(Segment(
            motion_type=MotionType.TRAVEL,
            waypoints=travel_wps,
            v=TRAVEL_V, a=TRAVEL_A,
        ))
        print(f"    TRAVEL OK ({len(travel_wps)} wps)")
    except RuntimeError as e:
        print(f"    TRAVEL FAILED — {e}")
        print(f"    Skipping run {run_i}.")
        continue

    # APPROACH: hover entry → first surface point (pen-down)
    print(f"  [{entry_label}] → [Run{run_i} surface[0]]")
    print(f"    from {_fmt_tcp(h_entry)}  to {_fmt_tcp(run_surface[0])}")
    segments.append(Segment(
        motion_type=MotionType.APPROACH,
        waypoints=[h_entry, run_surface[0]],
        v=APPROACH_V, a=APPROACH_A,
    ))
    print(f"    APPROACH down OK")

    # DRAW: on surface
    print(f"  [Run{run_i} surface[0]] → [Run{run_i} surface[{len(run_surface)-1}]]")
    print(f"    from {_fmt_tcp(run_surface[0])}  to {_fmt_tcp(run_surface[-1])}")
    segments.append(Segment(
        motion_type=MotionType.DRAW,
        waypoints=run_surface,
        v=DRAW_V, a=DRAW_A,
    ))
    print(f"    DRAW OK ({len(run_surface)} pts)")

    # APPROACH: last surface point → hover exit (pen-up)
    print(f"  [Run{run_i} surface[{len(run_surface)-1}]] → [{exit_label}]")
    print(f"    from {_fmt_tcp(run_surface[-1])}  to {_fmt_tcp(h_exit)}")
    segments.append(Segment(
        motion_type=MotionType.APPROACH,
        waypoints=[run_surface[-1], h_exit],
        v=APPROACH_V, a=APPROACH_A,
    ))
    print(f"    APPROACH up OK")

    current_tcp = h_exit
    current_label = exit_label

# Final TRAVEL back home
home_tcp = robot.get_fk(home)
print(f"\n  [{current_label}] → [HOME]")
print(f"    from {_fmt_tcp(current_tcp)}  to {_fmt_tcp(home_tcp)}")
try:
    travel_wps = compute_positioning_motion(robot, checker, current_tcp, home_tcp)
    segments.append(Segment(
        motion_type=MotionType.TRAVEL,
        waypoints=travel_wps,
        v=TRAVEL_V, a=TRAVEL_A,
    ))
    print(f"    TRAVEL OK ({len(travel_wps)} wps)")
except RuntimeError as e:
    print(f"    TRAVEL FAILED — {e}")

# Print segment summary
travel_count = sum(1 for s in segments if s.motion_type == MotionType.TRAVEL)
approach_count = sum(1 for s in segments if s.motion_type == MotionType.APPROACH)
draw_count = sum(1 for s in segments if s.motion_type == MotionType.DRAW)
total_wps = sum(len(s.waypoints) for s in segments)
print(f"\nPlan: {len(segments)} segments ({travel_count} TRAVEL, {approach_count} APPROACH, "
      f"{draw_count} DRAW), {total_wps} total waypoints")

input("\nPress ENTER to visualize the final plan...")

# =================  STAGE 8 — Visualize final plan  =======================
# Clear run + hover spheres
pb.configureDebugVisualizer(pb.COV_ENABLE_RENDERING, 0, physicsClientId=checker.cid)
for bid in run_sphere_ids:
    pb.removeBody(bid, physicsClientId=checker.cid)
pb.configureDebugVisualizer(pb.COV_ENABLE_RENDERING, 1, physicsClientId=checker.cid)

print("\nVisualizing final plan step by step...")
print("  TRAVEL = blue, APPROACH = orange, DRAW = green\n")

SEGMENT_COLORS = {
    MotionType.TRAVEL:   [0, 0.5, 1],     # blue
    MotionType.APPROACH: [1, 0.5, 0],     # orange
    MotionType.DRAW:     [0, 1, 0],       # green
}

for i, seg in enumerate(segments):
    color = SEGMENT_COLORS[seg.motion_type]
    width = 3 if seg.motion_type == MotionType.DRAW else 2
    label = seg.motion_type.name
    wp_start = seg.waypoints[0]
    wp_end = seg.waypoints[-1]
    print(f"  Segment {i}: {label} ({len(seg.waypoints)} wps)  "
          f"{_fmt_tcp(wp_start)} → {_fmt_tcp(wp_end)}")
    input(f"    Press ENTER to draw {label}...")
    draw_line_strip(seg.waypoints, color, width)

print(f"\n  {len(segments)} segments visualized")

# ===========================  Cleanup  ====================================
input("\nPress ENTER to close PyBullet...")
if pb.isConnected(checker.cid):
    pb.disconnect(checker.cid)
    print("PyBullet disconnected")

# =================  STAGE 9 — Run in simulation =======================
input("Initialise and launch the robot simulation (with Gazebo)")
from duckify_simulation.duckify_sim import DuckifySim

duckify_sim = DuckifySim()
robot_sim = duckify_sim.robot_control
robot_sim.set_tcp(tcp_offset)
robot_sim.movej(home)

input("Launch the motion")
for m in segments:
    v = m.v
    a = m.a
    print(m.motion_type.name)
    if not m.motion_type == MotionType.DRAW:
        move_simple(robot_sim, m.waypoints, v, a)

# =================  STAGE 10 — Run on robot =======================
from src.logger import LoggingForce
answer = input("Launch on robot? y/n: ").strip().lower()

if answer != "y":
    def segment_to_dict(seg: Segment):
        return {
            "motion_type": seg.motion_type.name,   # or str(seg.motion_type)
            "waypoints": [wp.toList() for wp in seg.waypoints],  # depends on your TCP6D class
            "v": seg.v,
            "a": seg.a,
            "r": seg.r,
        }

    data = [segment_to_dict(s) for s in segments]

    with open("segments.json", "w") as f:
        json.dump(data, f, indent=4)

    print("Segments saved to segments.json")
    print("End program")
    exit()

# If user answered "y", continue to robot execution
print("Launching on robot...")

ip_robot = "0"
while ip_robot not in ["8", "9"]:
    ip_robot = input("fill last number of ip for the robot:")

from URBasic import ISCoin
iscoin = ISCoin(host=f"10.30.5.15{ip_robot}", opened_gripper_size_mm=40)
robot_true = iscoin.robot_control
robot_true.set_tcp(tcp_offset)
robot_true.movej(home)
logger_force = LoggingForce(robot_true)

input("READY TO RUN ROBOT !!!")
logger_force.start_logging("free_force.csv")
for m in segments:
    v = m.v
    a = m.a
    print(m.motion_type.name)
    if not m.motion_type == MotionType.DRAW:
        move_simple(robot_sim, m.waypoints, v, a)
logger_force.stop_logging()

input("READY TO DRAW WITH ROBOT !!!")
logger_force.start_logging()
for m in segments:
    v = m.v
    a = m.a
    print(m.motion_type.name)
    move_simple(robot_sim, m.waypoints, v, a)
logger_force.stop_logging()
