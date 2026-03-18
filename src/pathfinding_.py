from duckify_simulation.duckify_sim.robot_control import SimRobotControl

from src.logger import DataStore
from src.utils import ask_yes_no
from src.config import *

import pybullet as pb

from src.safety import setup_checker
from src.transformation import extract_pybullet_pose
from src.pybullet_helpers import clear_bodies, find_hovers, preview_traces, split_and_visualize, validate_and_visualize
from src.computation import assemble_segments
from src.segment import SideType

class Pathfinding:
    def __init__(self, datastore: DataStore, obstacles=OBSTACLE_STLS, side=SideType.LEFT, verbose=True):
        self.ds = datastore
        self.obstacles = obstacles
        self.side = side
        self.verbose = verbose
    
    def run(self):
        if not ask_yes_no("Do you want to launch a pathfinding? y/n \n"):
            self.ds.load_joint_segments()
            return
        
        obj2robot = self.ds.load_transformation()
        traces = self.ds.load_tcp_segments()
        traces = [t for t in traces if t.side == self.side ]
        
        robot = SimRobotControl()
        _, tcp_offset = self.ds.load_calibration()
        robot.set_tcp(tcp_offset)

        pos, quat, scale = extract_pybullet_pose(obj2robot)
        for obs in self.obstacles:
            if 'position' not in obs:
                obs['position'] = pos
                obs['orientation'] = quat

        checker = setup_checker(self.obstacles, gui=self.verbose)
        pb.resetDebugVisualizerCamera(
            cameraDistance=0.6, cameraYaw=45, cameraPitch=-30,
            cameraTargetPosition=pos, physicsClientId=checker.cid,
        )

        checker.set_joint_angles(HOMEJ.toList())
        
        surface_tcps_per_trace = [t.waypoints for t in traces]
        
        preview_traces(checker, surface_tcps_per_trace)
        if not ask_yes_no("Do the trace are correct? y/n \n"):
            if pb.isConnected(checker.cid):
                pb.disconnect(checker.cid)
            raise RuntimeError("The trace are not correct.")


        valid_masks, surface_joints, validation_spheres = validate_and_visualize(
            checker, robot, surface_tcps_per_trace, HOMEJ,
        )

        if not ask_yes_no("Do the trace are correct? y/n \n"):
            if pb.isConnected(checker.cid):
                pb.disconnect(checker.cid)
            raise RuntimeError("The trace are not correct.")
        
        clear_bodies(checker.cid, validation_spheres)

        
        runs_per_trace, _ = split_and_visualize(checker, surface_tcps_per_trace, valid_masks)
        validated_runs = find_hovers(checker, robot, surface_tcps_per_trace, runs_per_trace, surface_joints)
        segments = assemble_segments(robot, checker, validated_runs, surface_joints, HOMEJ)

        self.ds.save_joint_segments(segments)
        
        if pb.isConnected(checker.cid):
            pb.disconnect(checker.cid)
            print("PyBullet disconnected")
        
                             
    def fallback():
        raise NotImplemented("Pathfinding is not implemented yet.")