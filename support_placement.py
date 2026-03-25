import base64
import json
import os
import random
from multiprocessing import Lock, Pool

import numpy as np
import pybullet as pb
import trimesh

from duckify_simulation.duckify_sim.robot_control import SimRobotControl
from src.computation import load_and_convert_to_tcp
from src.config import OBSTACLE_STLS
from src.logger import DataStore
from src.pybullet_helpers import clear_bodies, validate_and_visualize
from src.safety import setup_checker
from src.transformation import extract_pybullet_pose, load_obj2robot
from urbasic.URBasic.waypoint6d import TCP6D, Joint6D

JUMP_TO_VIS = False
VIS_DELAY = 0.1
DEBUG = False
N_WORKERS = 4
RESULTS_PATH = "results.json"


class SupportPlacer:
    def __init__(self):
        self.record = DataStore()
        self.home = Joint6D.createFromRadians(
            1.8859, -1.4452, 1.2389, -1.3639, -1.5693, -0.3849
        )
        self.model_path = "duckify_simulation/3d_objects/duck_uv.stl"
        self.file_path = "duckify_simulation/paths/random.json"
        # file_path = "duckify_simulation/paths/duck_uv-test_1_triangle-trace.json"
        # file_path = "duckify_simulation/paths/duck_uv-test_2_circle-trace.json"
        # file_path = "duckify_simulation/paths/duck_uv-dot-trace.json"
        # file_path = "duckify_simulation/paths/square-left-trace.json"
        # file_path = "duckify_simulation/paths/duck_uv-test_8_on_head-trace.json"
        # file_path = "duckify_simulation/paths/duck_uv-test_4_triangle_on_bill-trace.json"
        # file_path = "duckify_simulation/paths/duck_uv-test_10_full_body_line-trace.json"
        self.robot: SimRobotControl = SimRobotControl()

    def run(
        self,
        support_angle: float,
        translation: tuple[float, float, float],
        x_filter: int = 0,
        y_filter: int = 0,
    ) -> tuple[float, bytes, list[int]]:
        self.setup_robot()
        self.setup_workspace(support_angle, translation)
        self.setup_pybullet()
        self.load_model()
        self.load_trace(x_filter, y_filter)
        ratio: float
        mask: bytes
        ratio, mask = self.validate()
        self.clean()
        return ratio, mask, self.indices

    def load_model(self):
        self.model = trimesh.load_mesh(self.model_path)
        x, y, z, _, _, _ = self.obj2robot((0, 0, 0, 1, 0, 0))
        self.model_center = (x, y, z)
        print("model center:", self.model_center)

    def setup_robot(self):
        _, tcp_offset = self.record.load_calibration(
            "save_data/calibration_default.pkl"
        )
        self.robot.set_tcp(tcp_offset)

    def setup_workspace(self, angle: float, translation: tuple[float, float, float]):
        self.obj2robot = load_obj2robot(
            self.record, rz_deg=angle, translation=translation
        )
        self.pb_pos, self.pb_quat, self.pb_scale = extract_pybullet_pose(self.obj2robot)
        for obs in OBSTACLE_STLS:
            if "position" not in obs:
                obs["position"] = self.pb_pos
                obs["orientation"] = self.pb_quat

    def setup_pybullet(self):
        self.checker = setup_checker(OBSTACLE_STLS, gui=False)

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

    def load_trace(self, x_filter: int, y_filter: int):
        surface_tcps_per_trace: list[list[TCP6D]]
        surface_tcps_per_trace, self.traces, self.data = load_and_convert_to_tcp(
            self.file_path, self.obj2robot, max_pts=1000
        )
        print(f"Loaded {len(self.traces)} traces, transformed to TCP6D")
        # preview_traces(self.checker, self.surface_tcps_per_trace)
        traces: list[list[TCP6D]] = []
        i = 0
        self.indices = []
        for trace in surface_tcps_per_trace:
            kept: list[TCP6D] = []
            for pt in trace:
                i += 1
                dx = pt.x - self.model_center[0]
                dy = pt.y - self.model_center[1]
                if x_filter * dx < 0:
                    continue
                if y_filter * dy < 0:
                    continue
                kept.append(pt)
                self.indices.append(i)
            print(f"kept: {len(kept)}/{len(trace)}")
            traces.append(kept)
        self.surface_tcps_per_trace = traces

    def validate(self) -> tuple[float, bytes]:
        print("\nValidating all trace waypoints...")
        valid_masks, surface_joints, validation_spheres = validate_and_visualize(
            self.checker,
            self.robot,
            self.surface_tcps_per_trace,
            self.home,
        )
        clear_bodies(self.checker.cid, validation_spheres)

        flat_mask: list[bool] = sum(valid_masks, [])
        valid_ratio: np.floating = np.mean(np.array(flat_mask, dtype=np.uint8))
        print(f"Valid ratio: {valid_ratio:%}")
        return float(valid_ratio), self.pack_mask(flat_mask)

    def clean(self):
        if pb.isConnected(self.checker.cid):
            pb.disconnect(self.checker.cid)
            print("PyBullet disconnected")

    def pack_mask(self, mask: list[bool]) -> bytes:
        a: np.ndarray = np.packbits(mask)  # type: ignore
        return a.tobytes()


Placement = tuple[float, tuple[float, float, float], int, int]


placer: SupportPlacer = None
io_lock = None


def init_process(io_lock_):
    global placer, io_lock
    placer = SupportPlacer()
    io_lock = io_lock_


def generate_placements(
    min_radius: float,
    max_radius: float,
    min_angle: float,
    max_angle: float,
    min_z: float,
    max_z: float,
    count: int,
) -> list[Placement]:
    placements: list[Placement] = []
    radius_range: float = max_radius - min_radius
    angle_range: float = max_angle - min_angle
    z_range: float = max_z - min_z
    for _ in range(count):
        radius: float = random.random() * radius_range + min_radius
        angle: float = random.random() * angle_range + min_angle
        x: float = np.cos(angle) * radius
        y: float = np.sin(angle) * radius
        z: float = random.random() * z_range + min_z

        for r in range(4):
            rot_angle: float = r * 90.0
            x_filter: int = 0
            y_filter: int = 0
            if r % 2 == 0:
                y_filter = -1 if y > 0 else 1
            else:
                x_filter = -1 if x > 0 else 1

            placements.append((rot_angle, (x, y, z), x_filter, y_filter))

    return placements


def try_placement(placement: Placement):
    angle, pos, x_filter, y_filter = placement
    if DEBUG:
        print("#" * 28)
        print(f"# Filters: x={x_filter:2d} y={y_filter:2d}       #")
        print(f"# Angle: {angle:3.0f}               #")
        print(f"# Pos: ({pos[0]:5.2f},{pos[1]:5.2f},{pos[2]:5.2f}) #")
        print("#" * 28)
    ratio: float
    mask: bytes
    indices: list[int]
    ratio, mask, indices = placer.run(angle, pos, x_filter, y_filter)

    io_lock.acquire()
    try:
        results = []
        if os.path.exists(RESULTS_PATH):
            with open(RESULTS_PATH, "r") as f:
                results = json.load(f)

        results.append(
            {
                "angle": angle,
                "pos": pos,
                "filter": {"x": x_filter, "y": y_filter},
                "ratio": ratio,
                "mask": base64.b64encode(mask).decode("utf-8"),
                "indices": indices,
            }
        )
        with open(RESULTS_PATH, "w") as f:
            json.dump(results, f)
    finally:
        io_lock.release()


def main():
    placements: list[Placement] = generate_placements(
        min_radius=0.20,
        max_radius=0.60,
        min_angle=-np.pi,
        max_angle=0,
        min_z=0.155,
        max_z=0.255,
        count=4,
    )

    io_lock = Lock()

    with Pool(N_WORKERS, initializer=init_process, initargs=(io_lock,)) as pool:
        pool.map(try_placement, placements)


if __name__ == "__main__":
    main()
