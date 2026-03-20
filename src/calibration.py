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

from src.utils import *
from src.config import *
from src.logger import DataStore

from URBasic import TCP6D
from URBasic import UrScript
from URBasic import ISCoin


def collect_data(robot_arm: UrScript, num_measure:int = 20):
    """
    Interactively collect multiple TCP poses while the robot is in freedrive mode.

    The user manually moves the robot to different orientations and confirms
    each capture. A minimum of 6 measurements is required for calibration.

    Parameters
    ----------
    robot_arm : UrScript
        Interface to the robot controller, providing freedrive and pose access.
    num_measure : int, optional
        Number of samples to collect. Values below 6 are automatically raised.

    Returns
    -------
    tcps : list of list[float]
        Captured TCP poses in UR format [x, y, z, rx, ry, rz].
    """
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

        if i == "":
            robot_arm.end_freedrive_mode()

            # Return robot positions
            tcp = robot_arm.get_actual_tcp_pose().toList()

            tcps.append(tcp)
            k += 1
            print("Captured sample ", k)
        
        if i == "exit":
            robot_arm.end_freedrive_mode()
            raise Exception("Wrong calibration; exit")

    robot_arm.end_freedrive_mode()

    return tcps

def _calibrate_tcp(flange_poses):
    """
    Solve for the TCP position using a pivot calibration method.

    Given multiple flange poses (base_T_flange), this function computes:
    - the TCP position expressed in the flange frame
    - the fixed TCP position expressed in the base frame

    Parameters
    ----------
    flange_poses : list of ndarray, shape (4, 4)
        Homogeneous transforms of the flange for each measurement.

    Returns
    -------
    t_tcp_flange : ndarray, shape (3,)
        TCP coordinates expressed in the flange frame.
    c_tcp_base : ndarray, shape (3,)
        TCP coordinates expressed in the base frame (pivot point).
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

def get_tcp_offset(tcps, verbose=VERBOSE):
    """
    Estimate the TCP offset from a set of measured poses and validate data quality.

    The function:
    - converts poses to transforms
    - evaluates consistency of the TCP position across measurements
    - checks that flange motion and rotation variation are sufficient
    - performs a least-squares TCP calibration

    Parameters
    ----------
    tcps : list of list[float]
        TCP poses in UR format [x, y, z, rx, ry, rz].

    Returns
    -------
    tcp_offset : TCP6D
        Estimated TCP offset expressed in the flange frame.
    """
    # Convert poses to matrices
    Ts = [pose_to_T(p) for p in tcps]

    if verbose:
        # Extract R_i and p_i
        Rs = [T[:3, :3] for T in Ts]
        ps = [T[:3, 3] for T in Ts]

        # Old calibrated TCP for data validation purpose
        t = np.array(TCP_REF_VAL)

        # Compute estimated TCP position in base for each measurement
        c_est = [ps[i] + Rs[i] @ t for i in range(len(ps))]

        # Compute spread
        c_est = np.array(c_est)
        spread = np.max(np.linalg.norm(c_est - np.mean(c_est, axis=0), axis=1))

        # Compute flange motion
        flange_motion = np.max(np.linalg.norm(ps - ps[0], axis=1))

        # Compute rotation variation
        rot_angles = []
        for R in Rs:
            angle = np.arccos((np.trace(R) - 1) / 2)
            rot_angles.append(angle)
        rot_variation = np.max(rot_angles) - np.min(rot_angles)

        # These value are only indicatif
        print("TCP consistency (mm):    \t", spread * 1000)             #(wanted to be small)
        print("Flange motion (cm):      \t", flange_motion * 100)       #(wanted to be large)
        print("Rotation variation (deg):\t", np.degrees(rot_variation)) #(wanted to be large)

    t_tcp_flange, c_tcp_base = _calibrate_tcp(Ts)
    print("TCP in flange frame:", t_tcp_flange, " <-")  # Tool offset
    print("TCP in tool   frame:",   c_tcp_base)         # Pivot point -> Fixed point in the world in TCP coordinate

    tcp_offset = TCP6D.createFromMetersRadians(t_tcp_flange[0], t_tcp_flange[1], t_tcp_flange[2], 0 , 0, 0)
    return tcp_offset

def _rotate_around_tcp(tcp: TCP6D, rx, ry, rz):
    """
    Create a new TCP pose by applying additional rotations around the TCP.

    Parameters
    ----------
    tcp : TCP6D
        Reference TCP pose.
    rx, ry, rz : float
        Additional rotations (radians) to apply around each axis.

    Returns
    -------
    new_pose : TCP6D
        Updated pose with modified orientation.
    """
    new_pose = TCP6D.createFromMetersRadians(
        tcp.x, tcp.y, tcp.z,
        tcp.rx + rx,
        tcp.ry + ry,
        tcp.rz + rz
    )
    return new_pose

def validate_calibration(robot_arm: UrScript, rot=0.2):
    """
    Generate a sequence of poses to visually validate TCP calibration.

    The robot is rotated around the TCP by ±rot radians about each axis.
    The resulting poses can be plotted or inspected to verify that the
    TCP remains stationary in space.

    Parameters
    ----------
    robot_arm : UrScript
        Robot interface used to read the current TCP pose.
    rot : float, optional
        Rotation magnitude (radians) applied around each axis.

    Returns
    -------
    motion : list of TCP6D
        Sequence of poses including the reference pose and rotated poses.
    """
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


def launch_calibration(robot_ip, ds: DataStore):
    try:
        iscoin = ISCoin(host=robot_ip, opened_gripper_size_mm=40)

        tcps = collect_data(iscoin.robot_control)
        tcp_offset = get_tcp_offset(tcps)

        ds.save_calibration(tcps, tcp_offset)
        ds.log_calibration(tcps, tcp_offset)

        return True
    except Exception as e:
        ds.log(f"Calibration skipped: {e}")
        return False


class Calibration:
    def __init__(self, datastore: DataStore, robot_ip: str):
        self.ds = datastore
        self.robot_ip = robot_ip

    def load_and_log(self, path: str | None = None):
        tcps, tcp_offset = self.ds.load_calibration(path)
        self.ds.save_calibration(tcps, tcp_offset)
        self.ds.log_calibration(tcps, tcp_offset)
        return tcps, tcp_offset

    def run(self):
        while True:

            if ask_yes_no("Do you have a calibration already saved? y/n\n"):
                tcps, tcp_offset = self.ds.load_calibration()
                self.ds.log_calibration(tcps, tcp_offset)
                if ask_yes_no("Do you want to offset the TCP offset? y/n \n"):
                    z = input("How many do you which to slide on Z: \n")
                    tcp_offset.z += np.float(z)
                    self.ds.save_calibration(tcps, tcp_offset)
                    self.ds.log_calibration(tcps, tcp_offset)
                return

            if ask_yes_no("Do you want to use the default? y/n\n"):
                self.ds.log("Load default calibration.")
                tcps, tcp_offset = self.ds.load_calibration("save_data/calibration_default.pkl")
                self.ds.save_calibration(tcps, tcp_offset)
                self.ds.log_calibration(tcps, tcp_offset)
                return

            if ask_yes_no("Do you want to run a robot calibration? y/n\n"):
                success = launch_calibration(self.robot_ip, self.ds)
                if success:
                    return
                else:
                    self.ds.log("Calibration failed. Try again.")
                    continue

            print("Invalid input. Please answer with 'y' or 'n'.")

    def fallback(self):
        self.ds.log("Fall back: load default calibration.")
        tcps, tcp_offset = self.ds.load_calibration("save_data/calibration_default.pkl")
        self.ds.save_calibration(tcps, tcp_offset)
        self.ds.log_calibration(tcps, tcp_offset)
