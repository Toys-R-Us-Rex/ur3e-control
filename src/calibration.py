'''
TCP calibration and validation tools for UR-series robots.

This module provides utilities to:
- collect TCP calibration data (`collect_data`)
- compute the TCP offset from recorded samples (`get_tcp_offset`)
- validate the resulting calibration through robot motion (`validate_calibration`)

Usage
-----
This module is designed to be used with the URBasic library, from which this
project is derived:
    https://github.com/ISC-HEI/ur3e-control

Typical workflow:
    1. Collect calibration samples using `collect_data`.
    2. Compute the TCP offset using `get_tcp_offset`.
    3. Validate the calibration using `validate_calibration`.

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
Course:     HES-SO Valais-Wallis, Engineering Track 304
'''

import numpy as np

from URBasic import TCP6D
from URBasic import UrScript


def collect_data(robot_arm: UrScript, num_measure:int = 20):
    if num_measure < 6:
        num_measure = 6
        print("Minimum 6 measures.")
    
    tcps = []

    robot_arm.set_tcp(TCP6D.createFromMetersRadians(0,0,0,0,0,0))

    k = 0
    while k < num_measure:
        robot_arm.freedrive_mode()
        i = input(f"[{k+1}/{num_measure}] Move robot, then press ENTER to capture. (or q to quit)")
        if i == "q":
            if k < 6:
                print("You should give min 6 points")
                continue
            else:
                break

        robot_arm.end_freedrive_mode()

        # Return robot positions
        tcp = robot_arm.get_actual_tcp_pose().toList()

        tcps.append(tcp)
        k += 1
        print("Captured sample ", k)


    robot_arm.end_freedrive_mode()

    return tcps

def _pose_to_T(pose):
    """
    pose: [x, y, z, rx, ry, rz] (UR standard, base_T_flange)
    returns 4x4 homogeneous matrix
    """
    x, y, z, rx, ry, rz = pose
    theta = np.linalg.norm([rx, ry, rz])
    if theta < 1e-9:
        R = np.eye(3)
    else:
        k = np.array([rx, ry, rz]) / theta
        K = np.array([[0, -k[2], k[1]],
                      [k[2], 0, -k[0]],
                      [-k[1], k[0], 0]])
        R = np.eye(3) + np.sin(theta)*K + (1-np.cos(theta))*(K @ K)

    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = [x, y, z]
    return T

def _calibrate_tcp(flange_poses):
    """
    flange_poses: list of 4x4 homogeneous matrices T_i (base_T_flange)
    Returns:
        t_tcp_flange: 3-vector, TCP in flange frame
        c_tcp_base:   3-vector, TCP in base frame
    """
    N = len(flange_poses)
    A = np.zeros((3 * N, 6))
    b = np.zeros((3 * N,))

    for i, T in enumerate(flange_poses):
        R = T[:3, :3]
        p = T[:3, 3]

        A[3*i:3*i+3, 0:3] = R          # R_i
        A[3*i:3*i+3, 3:6] = -np.eye(3) # -I
        b[3*i:3*i+3] = -p              # -p_i

    # Least squares solution
    x, _, _, _ = np.linalg.lstsq(A, b, rcond=None)
    t_tcp_flange = x[0:3]
    c_tcp_base   = x[3:6]
    return t_tcp_flange, c_tcp_base

def get_tcp_offset(tcps):
    # Convert poses to matrices
    Ts = [_pose_to_T(p) for p in tcps]

    # Extract R_i and p_i
    Rs = [T[:3, :3] for T in Ts]
    ps = [T[:3, 3] for T in Ts]

    # Your calibrated TCP in flange frame
    t = np.array([-0.00072928, -0.00040785, 0.07966732])

    # Compute estimated TCP position in base for each measurement
    c_est = [ps[i] + Rs[i] @ t for i in range(len(ps))]

    # Compute spread (should be small)
    c_est = np.array(c_est)
    spread = np.max(np.linalg.norm(c_est - np.mean(c_est, axis=0), axis=1))

    # Compute flange motion (should be large)
    flange_motion = np.max(np.linalg.norm(ps - ps[0], axis=1))

    # Compute rotation variation (should be large)
    rot_angles = []
    for R in Rs:
        angle = np.arccos((np.trace(R) - 1) / 2)
        rot_angles.append(angle)
    rot_variation = np.max(rot_angles) - np.min(rot_angles)

    print("TCP consistency (mm):    \t", spread * 1000)
    print("Flange motion (cm):      \t", flange_motion * 100)
    print("Rotation variation (deg):\t", np.degrees(rot_variation))

    t_tcp_flange, c_tcp_base = _calibrate_tcp(Ts)
    print("TCP in flange frame:", t_tcp_flange, " <-")  # Tool offset
    print("TCP in tool   frame:",   c_tcp_base)         # Pivot point -> Fixed point in the world in TCP coordinate

    tcp_offset = TCP6D.createFromMetersRadians(t_tcp_flange[0], t_tcp_flange[1], t_tcp_flange[2], 0 , 0, 0)
    return tcp_offset

def _rotate_around_tcp(tcp: TCP6D, rx, ry, rz):
    new_pose = TCP6D.createFromMetersRadians(
        tcp.x, tcp.y, tcp.z,
        tcp.rx + rx,
        tcp.ry + ry,
        tcp.rz + rz
    )
    return new_pose

def validate_calibration(robot_arm: UrScript, rot=0.2):
    # Perform validation rotations
    axes = [
        (rot, 0, 0),
        (-rot, 0, 0),
        (0, rot, 0),
        (0, -rot, 0),
        (0, 0, rot),
        (0, 0, -rot)
    ]

    tcp_ref = robot_arm.get_actual_tcp_pose()
    motion = [tcp_ref]

    for rx, ry, rz in axes:
        test_pose = _rotate_around_tcp(tcp_ref, rx, ry, rz)
        # print(f"Rotating: rx={rx}, ry={ry}, rz={rz}")
        motion.append(test_pose)
        motion.append(tcp_ref)

    return motion