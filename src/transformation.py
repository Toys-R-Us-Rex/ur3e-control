'''
TCP transformation tool for UR-series robots.

This module provides utilities to:
- collect TCP transformation data (`collect_data`)
- compute the transformation function (`create_transformation`)
- transform TCP points (`tcp_trans`)
- convert between .obj and .stl coordinate systems (`obj_to_stl`, `stl_to_obj`)

Usage
-----
This module is designed to be used with the URBasic library, from which this
project is derived:
    https://github.com/ISC-HEI/ur3e-control

Typical workflow:
    1. Collect transformation samples using `collect_data`.
    2. Compute the transformation using `create_transformation`.
    3. Convert coordinate systems using resulting function `AtoB`.

Additional tool:
    - `tcp_trans`: composes two TCP poses (position + rotation vector) and returns the resulting pose.
    - `obj_to_stl`, `stl_to_obj`: convert between .obj and .stl coordinate systems.

MIT License

Copyright (c) 2026 Mariéthoz Cédric

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

Author:     Mariéthoz Cédric, with assistance from Copilote AI (Microsoft)
Author:     Savioz Pierre-Yves, with assistance from Claude AI (Anthropic)
Course:     HES-SO Valais-Wallis, Engineering Track 304
'''

import json
import numpy as np

from URBasic.waypoint6d import TCP6D
from src.config import *
from src.utils import *
from src.logger import DataStore

from URBasic.iscoin import ISCoin
from duckify_simulation.duckify_sim import DuckifySim
      
def collect_data(robot_arm, world_measure):
    if len(world_measure) < 3:
        print("Minimum 3 measure points.")
        raise ValueError("You don't provide enough world measure")

    num_measure = len(world_measure)
    
    tcps = []

    k = 0
    while k < num_measure:
        robot_arm.freedrive_mode()
        i = input(f"Move robot to {world_measure[k]}, then press ENTER to capture. (or q to quit)")
        if i == "q":
            if k < 3:
                print("You should collect min 3 world points.")
                continue
            else:
                break

        if i == "":
            robot_arm.end_freedrive_mode()

            # Return robot positions
            tcp = robot_arm.get_actual_tcp_pose().toList()

            tcps.append(tcp)
            k += 1
            print("Captured measure ", k)

        if i == "exit":
            robot_arm.end_freedrive_mode()
            raise Exception("Wrong calibration; exit")

    robot_arm.end_freedrive_mode()

    return np.array(tcps)

def create_transformation(A, B) -> AtoB:
    """
    Computes the rigid transform
    that maps A to B using the Kabsch algorithm.
    A and B are Nx3 arrays of corresponding points.
    """

    A = np.asarray(A)[:, :3]
    B = np.asarray(B)[:, :3]
    assert A.shape == B.shape
    N = A.shape[0]

    # 1. Compute centroids
    centroid_A = A.mean(axis=0)
    centroid_B = B.mean(axis=0)

    # 2. Center the points
    AA = A - centroid_A
    BB = B - centroid_B

    # 3. Compute covariance matrix
    H = AA.T @ BB

    # 4. SVD
    U, S, Vt = np.linalg.svd(H)

    # 5. Compute rotation
    R = Vt.T @ U.T

    # Fix improper rotation (reflection)
    if np.linalg.det(R) < 0:
        Vt[2, :] *= -1
        R = Vt.T @ U.T

    # 6. Compute translation
    t = centroid_B - R @ centroid_A

    # 7. Build 4×4 matrix
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t

    # Normal transform (4×4 homogeneous)
    R_normal = np.linalg.inv(R).T
    T_normal = np.eye(4)
    T_normal[:3, :3] = R_normal

    return AtoB(T, T_normal)


# creates a 4D homogenous matrice based on translation vector and angles given manually
def build_manual_transform(rz_deg=OBJ2ROBOT_RZ_DEG, translation=OBJ2ROBOT_TRANSLATION, scale=OBJ2ROBOT_SCALE):
    from scipy.spatial.transform import Rotation as Rot

    rz_rad = np.radians(rz_deg)
    R = Rot.from_euler('z', rz_rad).as_matrix() * scale  # rotation + scale

    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = translation

    # Normal transform: inverse-transpose of R
    R_pure = R / scale
    R_normal = np.linalg.inv(R_pure).T
    T_normal = np.eye(4)
    T_normal[:3, :3] = R_normal

    return AtoB(T, T_normal)

# gets the position, quaternion , scale from obj2robot so that pybullet can place the STL mesh in the space
def extract_pybullet_pose(obj2robot):
    from scipy.spatial.transform import Rotation as Rot
    T = obj2robot.T_position
    pos = T[:3, 3].tolist()
    R = T[:3, :3]
    scale = np.linalg.norm(R[:, 0])
    U, _, Vt = np.linalg.svd(R)
    R_pure = U @ Vt
    if np.linalg.det(R_pure) < 0:
        U[:, -1] *= -1
        R_pure = U @ Vt
    quat = Rot.from_matrix(R_pure).as_quat().tolist()
    return pos, quat, scale

# gets the transform data from the pickle files.
def load_obj2robot(record, rz_deg=OBJ2ROBOT_RZ_DEG):
    a = record.load_transformation("save_data_test/20260317/transformation_0.pkl")
    T_loaded = a.T_position
    if T_loaded is not None:
        translation = tuple(T_loaded[:3, 3])
        print("using the loaded transfomration from pickl")
    else:
        translation = OBJ2ROBOT_TRANSLATION
        print("using default translation")

    return build_manual_transform(rz_deg=rz_deg, translation=translation)


def launch_transformation(robot_ip, file_path, ds: DataStore) -> AtoB:
    try:
        with open(file_path, "r", encoding="utf-8") as f:
            data = json.load(f)
        
        iscoin = ISCoin(host=robot_ip, opened_gripper_size_mm=40)
        _, tcp_offset = ds.load_calibration()
        iscoin.robot_control.set_tcp(tcp_offset)

        p_world = np.array(data["calibration"])
        p_tcps = collect_data(iscoin.robot_control, p_world)

        ds.save_worldtcp(p_world, p_tcps)
        ds.log_worldtcp(p_world,p_tcps)

        obj2robot = create_transformation(p_world, p_tcps)

        ds.save_transformation(obj2robot)
        ds.log_transformation(obj2robot)
        return obj2robot

    except Exception as e:
        ds.log(f"Transforamtion skipped: {e}")
        raise

def test_transforamtion(ds: DataStore, obj2robot: AtoB, robot_ip):
    test = np.array([0,0,2,0,0,-1]) # x,y,z, n1,n2,n3
    tcp = TCP6D.createFromMetersRadians( *obj2robot(test))
    ds.log(f"Conversion test; {test} give {tcp}")
    print(f"Conversion test; {test} give {tcp}")
    if not ask_yes_no("Do you want to test on the Gazebo? y/n \n"):
        return
    
    duckify_sim = DuckifySim()
    robot_sim = duckify_sim.robot_control
    _, tcp_offset = ds.load_calibration()
    robot_sim.set_tcp(tcp_offset)
    robot_sim.movej(HOMEJ)
    robot_sim.movel(tcp)
    robot.movej(HOMEJ)
    
    if not ask_yes_no("Do you want to test on the robot? y/n \n"):
        return

    iscoin = ISCoin(host=robot_ip, opened_gripper_size_mm=40)
    robot = iscoin.robot_control

    robot.set_tcp(tcp_offset)
    robot.movej(HOMEJ)
    robot.movel(tcp)
    robot.movej(HOMEJ)


class Transformation:
    def __init__(self, datastore: DataStore, robot_ip: str, json_calibration: str):
        self.ds = datastore
        self.robot_ip = robot_ip
        self.json_calibration = json_calibration

    def run(self):
        while True:
            if ask_yes_no("Do you have the transformation already saved? y/n \n"):
                obj2robot = self.ds.load_transformation()
                if ask_yes_no("Do you want to test transformation? y/n \n"):
                    test_transforamtion(self.ds, obj2robot, self.robot_ip)
                return

            if ask_yes_no("Do you want to run a robot transformation? y/n \n"):
                obj2robot = launch_transformation(self.robot_ip, self.json_calibration, self.ds)
                if ask_yes_no("Do you want to test transformation? y/n \n"):
                    test_transforamtion(self.ds, obj2robot, self.robot_ip)
                return

            if ask_yes_no("You don't have a transformation or can't run one? y/n \n"):
                self.ds.log("No transformation available.")
                raise RuntimeError("No transformation available.")

            print("Invalid input, please try again.")

    def fallback(self):
        self.ds.log("Fall back transformation.")

        if ask_yes_no("Use default transformation (test only)? y/n \n"):
            self.ds.log("WARNING: Loading default transformation (test only).")
            obj_to_robot = self.ds.load_transformation("save_data/transformation_default.pkl")
            self.ds.save_transformation(obj_to_robot)
            self.ds.log_transformation(obj_to_robot)
            return

        if ask_yes_no("Use identity (no transformation, test only)? y/n \n"):
            self.ds.log("WARNING: Using identity transformation (test only).")
            T_eye = np.eye(4)
            transform = AtoB(T_eye, T_eye)
            self.ds.save_transformation(transform)
            self.ds.log_transformation(transform)
            return

        raise RuntimeError("Fallback transformation aborted by user.")
