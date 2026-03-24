#!/usr/bin/env python3

import time
import pybullet as pb
from duckify_simulation.duckify_sim.robot_control import SimRobotControl
from src.logger import DataStore
from src.config import OBSTACLE_STLS, HOMEJ
from src.segment import SideType
from src.transformation import extract_pybullet_pose
from src.safety import setup_checker
from src.pybullet_helpers import (
    preview_traces,
    validate_and_visualize,
    split_and_visualize,
    find_hovers,
    clear_bodies,
)
from src.computation import assemble_segments, smoothing

SIDE = SideType.RIGHT

day = time.strftime("%Y%m%d")
ds = DataStore(f"save_data_test/{day}/")

tcp_segments = ds.load_tcp_segments()
traces = [t for t in tcp_segments if t.side == SIDE]
print(f"Loaded {len(tcp_segments)} tcp_segments, {len(traces)} on {SIDE.name} side")

robot = SimRobotControl()
_, tcp_offset = ds.load_calibration()
robot.set_tcp(tcp_offset)

obj2robot = ds.load_transformation()
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
checker.set_joint_angles(HOMEJ.toList())

surface_tcps_per_trace = [t.waypoints for t in traces]

preview_traces(checker, surface_tcps_per_trace)
input("Press ENTER to validate...")

valid_masks, surface_joints, validation_spheres = validate_and_visualize(
    checker, robot, surface_tcps_per_trace, HOMEJ,
)
input("Press ENTER to continue...")
clear_bodies(checker.cid, validation_spheres)

runs_per_trace, run_spheres = split_and_visualize(checker, surface_tcps_per_trace, valid_masks)
input("Press ENTER to continue...")

validated_runs = find_hovers(checker, robot, surface_tcps_per_trace, runs_per_trace, surface_joints)
input("Press ENTER to continue...")

segments = assemble_segments(robot, checker, validated_runs, surface_joints, HOMEJ,
                             surface_tcps_per_trace=surface_tcps_per_trace)
smoothing(robot, checker, segments, HOMEJ)

ds.save_joint_segments(segments)
print(f"Saved {len(segments)} joint segments")

input("Press ENTER to close...")
if pb.isConnected(checker.cid):
    pb.disconnect(checker.cid)
    print("PyBullet disconnected")
