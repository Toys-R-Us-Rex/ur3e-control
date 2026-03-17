#!/usr/bin/env python3

import logging
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
logging.basicConfig(level=logging.WARNING)

from duckify_simulation.duckify_sim.robot_control import SimRobotControl
from src.config import TCPS_20, OBSTACLE_STLS

import pybullet as pb
from URBasic import Joint6D
from src.logger import DataStore
from src.transformation import load_obj2robot, extract_pybullet_pose
from src.safety import setup_checker
from src.computation import (
    load_and_convert_to_tcp,
    assemble_segments,
    collect_joint_waypoints,
)
from src.pybullet_helpers import (
    preview_traces,
    validate_and_visualize,
    split_and_visualize,
    find_hovers,
    visualize_plan,
    animate_plan,
    clear_bodies,
)

DEBUG = True






### main part

record = DataStore()
home = Joint6D.createFromRadians(1.8859, -1.4452, 1.2389, -1.3639, -1.5693, -0.3849)
file_path = "duckify_simulation/paths/duck_uv-fancy_test_duck-trace_v2.json"

# Stage 0: setup robot
robot = SimRobotControl()
_, tcp_offset = record.load_calibration("save_data/calibration_default.pkl")
robot.set_tcp(tcp_offset)

# Stage 1+2: load traces and convert to TCP
obj2robot = load_obj2robot(record)

surface_tcps_per_trace, traces, data = load_and_convert_to_tcp(file_path, obj2robot, max_pts=1000)
print(surface_tcps_per_trace)
print(f"Loaded {len(traces)} traces, transformed to TCP6D")

# Stage 3: launch PyBullet with obstacles positioned using obj2robot
pos, quat, scale = extract_pybullet_pose(obj2robot)
for obs in OBSTACLE_STLS:
    if 'position' not in obs:
        obs['position'] = pos
        obs['orientation'] = quat

checker = setup_checker(OBSTACLE_STLS, gui=True)

pb.resetDebugVisualizerCamera(
    cameraDistance=0.6, cameraYaw=45, cameraPitch=-30,
    cameraTargetPosition=pos, physicsClientId=checker.cid,
)

checker.set_joint_angles(home.toList())
print(f"\nSet PyBullet robot to home: {home.toList()}")

# Preview traces
preview_traces(checker, surface_tcps_per_trace)

# Stage 4: validate surface points
print("\nValidating all trace waypoints...")
valid_masks, surface_joints, validation_spheres = validate_and_visualize(
    checker, robot, surface_tcps_per_trace, home,
)
if DEBUG:
    input("\nPress ENTER to continue to run splitting...")
clear_bodies(checker.cid, validation_spheres)

# Stage 5: split into runs
print("\nSplitting traces into consecutive valid runs...")
runs_per_trace, run_spheres = split_and_visualize(checker, surface_tcps_per_trace, valid_masks)
if DEBUG:
    input("\nPress ENTER to continue to hover generation...")

# Stage 6: find hover points
print("\nGenerating and validating hover points...")
validated_runs = find_hovers(checker, robot, surface_tcps_per_trace, runs_per_trace, surface_joints)
if DEBUG:
    input("Press ENTER to continue to pathfinding...")

# Stage 7: assemble segments
segments = assemble_segments(robot, checker, validated_runs, surface_joints, home)
input("\nPress ENTER to visualize the final plan...")

# Stage 8: visualize
clear_bodies(checker.cid, run_spheres)
visualize_plan(checker, segments, debug=DEBUG)

# Stage 8.5: animate
animate_plan(checker, segments)

# Stage 9: remove waypoints
print("\n" + "==============")
print("Waypoint removal")
print("=" * 60)
all_joint_waypoints = collect_joint_waypoints(segments)

# Stage 10: save
record.save_waypoints(all_joint_waypoints)

# Cleanup
input("\nPress ENTER to close PyBullet...")
if pb.isConnected(checker.cid):
    pb.disconnect(checker.cid)
    print("PyBullet disconnected")
