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

from src.utils import *
from src.logger import LoggingLog

from URBasic.iscoin import ISCoin

def collect_data(robot_arm, world_measure):
    if len(world_measure) < 3:
        print("Minimum 3 measure points.")
        raise ValueError("You don't provide eanough world measure")

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

def create_transformation(A, B):
    A = np.asarray(A)[:, :3]
    B = np.asarray(B)[:, :3]
    assert A.shape == B.shape
    N = A.shape[0]

    # Build matrix for affine solve: [A | 1]
    P = np.hstack([A, np.ones((N, 1))])  # Nx4

    # Solve P @ X = B, where X is 4×3 (R^T and t)
    X, _, _, _ = np.linalg.lstsq(P, B, rcond=None)

    # Extract R and t
    R = X[:3, :].T   # 3×3
    t = X[3, :]      # 3

    # Build full 4×4 affine matrix
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t

    # Normal transform (4×4 homogeneous)
    R_normal = np.linalg.inv(R).T
    T_normal = np.eye(4)
    T_normal[:3, :3] = R_normal

    def AtoB(p):
        p = np.asarray(p)
        point = p[:3]
        normal = p[3:]
        # Transform point
        p_new = T @ [*point, 1]

        # Transform normal (4×4 homogeneous, extract xyz before normalizing)
        nx,ny,nz = normal
        n_new = (T_normal @ [nx,ny,nz, 1])[:3]
        n_new /= np.linalg.norm(n_new)
        r_new = normal_to_rotvec(n_new)

        return [*p_new[:3], *r_new]

    # Expose 4×4 matrices as attributes for inspection/composition
    AtoB.T = T
    AtoB.T_normal = T_normal

    return AtoB


def build_manual_transform(rz_deg=45.0, translation=(0, -0.2, 0.1), scale=0.001):
    """Build obj2robot callable from manual rz rotation + translation.

    Parameters
    ----------
    rz_deg : float — rotation around Z axis in degrees.
    translation : 3-tuple — (tx, ty, tz) in meters (robot frame).
    scale : float — uniform scale factor (default 0.001 for mm→m).
    """
    from scipy.spatial.transform import Rotation as Rot

    rz_rad = np.radians(rz_deg)
    R = Rot.from_euler('z', rz_rad).as_matrix() * scale  # rotation + scale

    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = translation

    # Normal transform: inverse-transpose of R (scale cancels after normalization)
    R_pure = R / scale
    R_normal = np.linalg.inv(R_pure).T
    T_normal = np.eye(4)
    T_normal[:3, :3] = R_normal

    def obj2robot(p):
        p = np.asarray(p)
        point, normal = p[:3], p[3:]
        p_new = T @ [*point, 1]
        n_new = (T_normal @ [*normal, 1])[:3]
        n_new /= np.linalg.norm(n_new)
        r_new = normal_to_rotvec(n_new)
        return [*p_new[:3], *r_new]

    obj2robot.T = T
    obj2robot.T_normal = T_normal
    return obj2robot

def tcp_trans(tcp1, tcp2):
    # Décomposition
    p1 = np.array(tcp1[:3])
    r1 = np.array(tcp1[3:])
    p2 = np.array(tcp2[:3])
    r2 = np.array(tcp2[3:])

    # Matrices de rotation
    R1 = rotvec_to_rotmat(r1)
    R2 = rotvec_to_rotmat(r2)

    # Composition
    p_new = p1 + R1 @ p2
    R_new = R1 @ R2
    r_new = rotmat_to_rotvec(R_new)

    return np.concatenate([p_new, r_new])

def obj_to_stl(pts):
    """
    Convert from OBJ coords (Y-up) to STL coords (Z-up).
    Mapping: OBJ(x, y, z) → STL(x, -z, y).

    Accepts:
        - a single point: (3,)
        - a list of points: (N, 3)
    """
    pts = np.asarray(pts)

    # Single point
    if pts.ndim == 1:
        x, y, z = pts
        return np.array([x, -z, y])

    # Multiple points (N, 3)
    if pts.ndim == 2 and pts.shape[1] == 3:
        x = pts[:, 0]
        y = pts[:, 1]
        z = pts[:, 2]
        return np.column_stack([x, -z, y])

    raise ValueError("Input must be shape (3,) or (N,3)")

def stl_to_obj(pts):
    """
    Convert from STL coordinates (Z-up) to OBJ coordinates (Y-up).
    Mapping: STL(x, y, z) → OBJ(x, -z, y).

    Accepts:
        - a single point: (3,)
        - a list of points: (N, 3)
    """
    pts = np.asarray(pts)

    # Single point
    if pts.ndim == 1:
        x, y, z = pts
        return np.array([x, -z, y])

    # Multiple points (N, 3)
    if pts.ndim == 2 and pts.shape[1] == 3:
        x = pts[:, 0]
        y = pts[:, 1]
        z = pts[:, 2]
        return np.column_stack([x, -z, y])

    raise ValueError("Input must be shape (3,) or (N,3)")


def launch_transformation(robot_ip, file_path, log: LoggingLog):
    try:
        with open(file_path, "r", encoding="utf-8") as f:
            data = json.load(f)
        
        iscoin = ISCoin(host=robot_ip, opened_gripper_size_mm=40)

        p_world = np.array(data["calibration"])
        p_tcps = collect_data(iscoin.robot_control, p_world)

        log.save_worldtcp(p_world, p_tcps)
        log.log_worldtcp(p_world,p_tcps)

        obj2robot = create_transformation(p_world, p_tcps)

        log.save_transformation(obj2robot)
        log.log_transformation(obj2robot)

        iscoin.close()
        return True
    except Exception as e:
        log.log(f"Transforamtion skipped: {e}")
        return False