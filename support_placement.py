from matplotlib import pyplot as plt
import numpy as np

import pybullet as pb

from duckify_simulation.duckify_sim.robot_control import SimRobotControl
from src.computation import assemble_segments, collect_joint_waypoints, load_and_convert_to_tcp, smoothing
from src.config import OBSTACLE_STLS
from src.logger import DataStore
from src.pybullet_helpers import animate_plan, clear_bodies, find_hovers, preview_traces, split_and_visualize, validate_and_visualize, visualize_plan
from src.safety import setup_checker
from src.transformation import extract_pybullet_pose, load_obj2robot
from urbasic.URBasic.waypoint6d import Joint6D


JUMP_TO_VIS = False
VIS_DELAY = 0.1

class SupportPlacer:
    def __init__(self):
        self.record = DataStore()
        self.home = Joint6D.createFromRadians(1.8859, -1.4452, 1.2389, -1.3639, -1.5693, -0.3849)
        self.file_path = "duckify_simulation/paths/duck_uv-test_1_triangle-trace.json"
        # file_path = "duckify_simulation/paths/duck_uv-test_2_circle-trace.json"
        # file_path = "duckify_simulation/paths/duck_uv-dot-trace.json"
        # file_path = "duckify_simulation/paths/square-left-trace.json"
        # file_path = "duckify_simulation/paths/duck_uv-test_8_on_head-trace.json"
        # file_path = "duckify_simulation/paths/duck_uv-test_4_triangle_on_bill-trace.json"
        #file_path = "duckify_simulation/paths/duck_uv-test_10_full_body_line-trace.json"
        self.robot: SimRobotControl = SimRobotControl()
    
    def run(self):
        self.setup_robot()
        self.setup_workspace()
        self.setup_pybullet()
        self.load_trace()
        self.validate()
        self.clean()
    
    def setup_robot(self):
        _, tcp_offset = self.record.load_calibration("save_data/calibration_default.pkl")
        self.robot.set_tcp(tcp_offset)
    
    def setup_workspace(self):
        self.obj2robot = load_obj2robot(
            self.record,
            rz_deg=270.0,
            translation=(0.40, -0.35, 0.155)
        )
        self.pb_pos, self.pb_quat, self.pb_scale = extract_pybullet_pose(self.obj2robot)
        for obs in OBSTACLE_STLS:
            if 'position' not in obs:
                obs['position'] = self.pb_pos
                obs['orientation'] = self.pb_quat
    
    def setup_pybullet(self):
        self.checker = setup_checker(OBSTACLE_STLS, gui=True)

        # to modify the rotation central point on the GUI
        pb.resetDebugVisualizerCamera(
            cameraDistance=0.6,
            cameraYaw=45,
            cameraPitch=-30,
            cameraTargetPosition=self.pb_pos,
            physicsClientId=self.checker.cid,
        )

        self.checker.set_joint_angles(self.home.toList())
        print(f"\nSet PyBullet robot to home: {self.home.toList()}")

    def load_trace(self):
        self.surface_tcps_per_trace, self.traces, self.data = load_and_convert_to_tcp(self.file_path, self.obj2robot, max_pts=1000)
        print(f"Loaded {len(self.traces)} traces, transformed to TCP6D")
        #preview_traces(self.checker, self.surface_tcps_per_trace)

    def validate(self):
        print("\nValidating all trace waypoints...")
        valid_masks, surface_joints, validation_spheres = validate_and_visualize(
            self.checker, self.robot, self.surface_tcps_per_trace, self.home,
        )
        clear_bodies(self.checker.cid, validation_spheres)
        
        flat_mask: list[bool] = sum(valid_masks, [])
        valid_ratio: np.floating = np.mean(np.array(flat_mask, dtype=np.uint8))
        print(f"Valid ratio: {valid_ratio:%}")
    
    def clean(self):
        if pb.isConnected(self.checker.cid):
            pb.disconnect(self.checker.cid)
            print("PyBullet disconnected")


def main():
    placer = SupportPlacer()
    placer.run()

if __name__ == "__main__":
    main()
