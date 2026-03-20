"""UR3e forward and inverse kinematics using DH parameters.

Pure-Python analytical kinematics — no robot connection or simulator needed.

MIT License

Copyright (c) 2026 Pierre-Yves Savioz

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

Author:     Pierre-Yves Savioz, with assistance from Claude AI (Anthropic)
Course:     HES-SO Valais-Wallis, Engineering Track 304
URBasic:    Uses data types from URBasic by Martin Huus Bjerge,
            Rope Robotics ApS, Denmark (MIT License, 2017),
            modified by A. Amand, M. Richard, L. Azzalini (HES-SO)
DH params:  Universal Robots UR3e technical specifications
IK method:  Hawkins, K.P. "Analytic Inverse Kinematics for the Universal Robots"
            The analytical IK follows Hawkins (2013). The solving structure was referenced
            from existing open-source implementations (mc-capolei/python-Universal-robot-kinematics),
            adapted for the UR3e DH parameters with added singularity handling and FK validation.
"""

import math
import numpy as np
from numpy import sin, cos, arctan2, arccos, sqrt, pi

from URBasic.waypoint6d import Joint6D, TCP6D

# UR3e DH parameters (standard values from Universal Robots)
# [a, d, alpha] for each joint
UR3E_DH = [
    {"a": 0,        "d": 0.15185,  "alpha": math.pi / 2},   # joint 1
    {"a": -0.24355, "d": 0,        "alpha": 0},              # joint 2
    {"a": -0.2132,  "d": 0,        "alpha": 0},              # joint 3
    {"a": 0,        "d": 0.13105,  "alpha": math.pi / 2},    # joint 4
    {"a": 0,        "d": 0.08535,  "alpha": -math.pi / 2},   # joint 5
    {"a": 0,        "d": 0.0921,   "alpha": 0},              # joint 6
]


def dh_matrix(theta, a, d, alpha):
    """Build a single standard DH transformation matrix."""
    ct, st = cos(theta), sin(theta)
    ca, sa = cos(alpha), sin(alpha)
    return np.array([
        [ct, -st * ca,  st * sa, a * ct],
        [st,  ct * ca, -ct * sa, a * st],
        [0,   sa,       ca,      d     ],
        [0,   0,        0,       1     ],
    ])


def forward_kinematics_matrix(joint_angles):
    """Compute the flange 4x4 homogeneous matrix from joint angles.

    Args:
        joint_angles: list of 6 joint angles in radians.

    Returns:
        4x4 numpy array (homogeneous transformation matrix).
    """
    T = np.eye(4)

    for i in range(6):
        theta = joint_angles[i]
        a = UR3E_DH[i]["a"]
        d = UR3E_DH[i]["d"]
        alpha = UR3E_DH[i]["alpha"]

        ct = math.cos(theta)
        st = math.sin(theta)
        ca = math.cos(alpha)
        sa = math.sin(alpha)

        Ti = np.array([
            [ct, -st * ca,  st * sa, a * ct],
            [st,  ct * ca, -ct * sa, a * st],
            [0,   sa,       ca,      d],
            [0,   0,        0,       1],
        ])

        T = T @ Ti

    return T


def matrix_to_tcp6d(T):
    """Convert a 4x4 homogeneous matrix to TCP6D (position + axis-angle).

    Args:
        T: 4x4 numpy array (homogeneous transformation matrix).

    Returns:
        TCP6D with [x, y, z, rx, ry, rz].
    """
    x, y, z = T[0, 3], T[1, 3], T[2, 3]

    R = T[:3, :3]
    angle = math.acos(max(-1, min(1, (np.trace(R) - 1) / 2)))

    if abs(angle) < 1e-6:
        rx, ry, rz = 0.0, 0.0, 0.0
    elif abs(angle - math.pi) < 1e-6:
        rx = math.pi * math.sqrt(max(0.0, (R[0, 0] + 1) / 2))
        ry = math.pi * math.sqrt(max(0.0, (R[1, 1] + 1) / 2))
        rz = math.pi * math.sqrt(max(0.0, (R[2, 2] + 1) / 2))
    else:
        k = angle / (2 * math.sin(angle))
        rx = k * (R[2, 1] - R[1, 2])
        ry = k * (R[0, 2] - R[2, 0])
        rz = k * (R[1, 0] - R[0, 1])

    return TCP6D.createFromMetersRadians(x, y, z, rx, ry, rz)


def forward_kinematics(joint_angles):
    """Compute the TCP pose from joint angles using UR3e DH parameters.

    Args:
        joint_angles: list of 6 joint angles in radians.

    Returns:
        TCP6D with [x, y, z, rx, ry, rz] (position in meters, rotation as axis-angle).
    """
    return matrix_to_tcp6d(forward_kinematics_matrix(joint_angles))


def pose_to_matrix(pose):
    """Convert a TCP6D (position + axis-angle) to a 4x4 homogeneous matrix.

    The axis-angle vector [rx, ry, rz] encodes both the axis (direction)
    and the angle (magnitude) of rotation.
    """
    x, y, z = pose.x, pose.y, pose.z
    rx, ry, rz = pose.rx, pose.ry, pose.rz

    angle = sqrt(rx*rx + ry*ry + rz*rz)
    T = np.eye(4)
    T[0, 3], T[1, 3], T[2, 3] = x, y, z

    if angle < 1e-10:
        return T  # identity rotation

    # Rodrigues' rotation formula
    kx, ky, kz = rx / angle, ry / angle, rz / angle
    c, s = cos(angle), sin(angle)
    v = 1 - c

    T[0, 0] = kx*kx*v + c
    T[0, 1] = kx*ky*v - kz*s
    T[0, 2] = kx*kz*v + ky*s
    T[1, 0] = kx*ky*v + kz*s
    T[1, 1] = ky*ky*v + c
    T[1, 2] = ky*kz*v - kx*s
    T[2, 0] = kx*kz*v - ky*s
    T[2, 1] = ky*kz*v + kx*s
    T[2, 2] = kz*kz*v + c

    return T


def wrap_angle(a):
    """Wrap angle to [-pi, pi]."""
    return (a + pi) % (2 * pi) - pi


def analytical_ik(T_desired):
    """Analytical (closed-form) inverse kinematics for the UR3e.

    Exploits the spherical wrist (joints 4, 5, 6 axes intersect)
    to decouple position from orientation, yielding up to 8 exact solutions.

    Based on the method from:
        Hawkins, K.P. "Analytic Inverse Kinematics for the Universal Robots"

    Args:
        T_desired: 4x4 homogeneous transformation matrix of the desired TCP pose.

    Returns:
        List of 6-element numpy arrays, each a valid joint solution.
        Empty list if no solution exists.
    """
    # DH constants
    d1 = UR3E_DH[0]["d"]   # 0.15185
    a2 = UR3E_DH[1]["a"]   # -0.24355
    a3 = UR3E_DH[2]["a"]   # -0.21325
    d4 = UR3E_DH[3]["d"]   # 0.13105
    d5 = UR3E_DH[4]["d"]   # 0.08535
    d6 = UR3E_DH[5]["d"]   # 0.0921

    T06 = T_desired

    # Step 1: Find the wrist center (origin of frame 5)
    P_06 = T06[:3, 3]
    z_6 = T06[:3, 2]
    P_05 = P_06 - d6 * z_6

    # Step 2: Solve theta1 (2 solutions)
    p05x, p05y = P_05[0], P_05[1]
    R = sqrt(p05x**2 + p05y**2)
    if R < 1e-10:
        return []  # singularity

    if abs(d4 / R) > 1.0:
        return []

    phi1 = arctan2(p05y, p05x)
    phi2 = arccos(np.clip(d4 / R, -1.0, 1.0))
    theta1_options = [phi1 + phi2 + pi / 2, phi1 - phi2 + pi / 2]

    solutions = []

    for theta1 in theta1_options:
        # Step 3: Solve theta5 (2 solutions per theta1)
        numer5 = P_06[0] * sin(theta1) - P_06[1] * cos(theta1) - d4
        cos5 = np.clip(numer5 / d6, -1.0, 1.0)

        theta5_options = [arccos(cos5), -arccos(cos5)]

        for theta5 in theta5_options:
            sin5 = sin(theta5)

            # Step 4: Solve theta6
            if abs(sin5) < 1e-10:
                theta6 = 0.0
            else:
                m = (-T06[0, 1]*sin(theta1) + T06[1, 1]*cos(theta1)) / sin5
                n = ( T06[0, 0]*sin(theta1) - T06[1, 0]*cos(theta1)) / sin5
                theta6 = arctan2(m, n)

            # Step 5: Solve theta2, theta3, theta4
            T01 = dh_matrix(theta1, UR3E_DH[0]["a"], UR3E_DH[0]["d"], UR3E_DH[0]["alpha"])
            T45 = dh_matrix(theta5, UR3E_DH[4]["a"], UR3E_DH[4]["d"], UR3E_DH[4]["alpha"])
            T56 = dh_matrix(theta6, UR3E_DH[5]["a"], UR3E_DH[5]["d"], UR3E_DH[5]["alpha"])

            T01_inv = np.linalg.inv(T01)
            T4556_inv = np.linalg.inv(T45 @ T56)
            T14 = T01_inv @ T06 @ T4556_inv

            P14 = T14[:3, 3]

            # Solve theta3 using law of cosines
            D_sq = P14[0]**2 + P14[1]**2
            cos3_numer = D_sq - a2**2 - a3**2
            cos3_denom = 2 * a2 * a3

            if abs(cos3_denom) < 1e-10:
                continue

            cos3 = cos3_numer / cos3_denom
            if abs(cos3) > 1.0 + 1e-6:
                continue  # unreachable
            cos3 = np.clip(cos3, -1.0, 1.0)

            theta3_options = [arccos(cos3), -arccos(cos3)]

            for theta3 in theta3_options:
                # Solve theta2
                s3 = sin(theta3)
                A = a2 + a3 * cos3
                B = a3 * s3

                theta2 = arctan2(
                    A * P14[1] - B * P14[0],
                    A * P14[0] + B * P14[1]
                )

                # Solve theta4
                theta234 = arctan2(T14[1, 0], T14[0, 0])
                theta4 = theta234 - theta2 - theta3

                sol = np.array([
                    wrap_angle(theta1),
                    wrap_angle(theta2),
                    wrap_angle(theta3),
                    wrap_angle(theta4),
                    wrap_angle(theta5),
                    wrap_angle(theta6),
                ])
                solutions.append(sol)

    return solutions


def select_closest_ik(solutions, qnear, joint_limits=None):
    """Pick the IK solution closest to qnear (weighted joint-space distance).

    Args:
        solutions: list of 6-element np arrays.
        qnear: 6-element np array (current/preferred joint config).
        joint_limits: optional list of (min, max) per joint.

    Returns:
        Best solution as np array, or None if no valid solution.
    """
    if not solutions:
        return None

    best = None
    best_dist = float("inf")

    for sol in solutions:
        if joint_limits is not None:
            valid = True
            for i, (lo, hi) in enumerate(joint_limits):
                if sol[i] < lo or sol[i] > hi:
                    valid = False
                    break
            if not valid:
                continue

        dist = np.sum((sol - qnear) ** 2)
        if dist < best_dist:
            best_dist = dist
            best = sol

    return best


def get_inverse_kin(pose, qnear, tcp_offset=None, model_correction=None):
    """Standalone inverse kinematics — no robot connection needed.

    Computes up to 8 analytical IK solutions for the UR3e and returns the
    one closest to *qnear*, validated by FK round-trip.

    Parameters
    ----------
    pose : TCP6D
        Target end-effector pose.
    qnear : Joint6D or array-like
        Seed joint configuration for selecting among multiple solutions.
    tcp_offset : 4x4 array, optional
        Tool-center-point offset matrix (default: identity).
    model_correction : 4x4 array, optional
        Model correction matrix (default: identity).

    Returns
    -------
    Joint6D or None
        Best joint solution, or None if no valid solution found.
    """
    if tcp_offset is None:
        tcp_offset = np.eye(4)
    if model_correction is None:
        model_correction = np.eye(4)

    q0 = np.array(qnear.toList() if hasattr(qnear, 'toList') else list(qnear))

    T_desired = pose_to_matrix(pose)
    T_flange = T_desired @ np.linalg.inv(model_correction @ tcp_offset)
    solutions = analytical_ik(T_flange)

    if not solutions:
        return None

    # Validate each solution with FK round-trip and keep only accurate ones
    valid = []
    target = np.array(pose.toList())
    for sol in solutions:
        T_check = forward_kinematics_matrix(sol.tolist()) @ model_correction @ tcp_offset
        tcp_check = matrix_to_tcp6d(T_check)
        err_pos = np.sum((np.array(tcp_check.toList()[:3]) - target[:3]) ** 2)
        if err_pos < 0.001:
            valid.append(sol)

    best = select_closest_ik(valid if valid else solutions, q0)
    if best is None:
        return None

    # Final FK validation
    T_check = forward_kinematics_matrix(best.tolist()) @ model_correction @ tcp_offset
    tcp_check = matrix_to_tcp6d(T_check)
    err = np.sum((np.array(tcp_check.toList()[:3]) - np.array(pose.toList()[:3])) ** 2)
    if err < 0.001:
        return Joint6D.createFromRadians(*best.tolist())

    return None


def get_all_ik_solutions(pose, tcp_offset=None, model_correction=None):
    """Return ALL valid IK solutions for a TCP6D pose.

    Same FK-validation as get_inverse_kin but returns every
    passing solution instead of just the closest to qnear.

    Returns list[Joint6D] (may be empty).
    """
    if tcp_offset is None:
        tcp_offset = np.eye(4)
    if model_correction is None:
        model_correction = np.eye(4)

    T_desired = pose_to_matrix(pose)
    T_flange = T_desired @ np.linalg.inv(model_correction @ tcp_offset)
    solutions = analytical_ik(T_flange)

    if not solutions:
        return []

    target_pos = np.array([pose.x, pose.y, pose.z])
    valid = []
    for sol in solutions:
        T_check = forward_kinematics_matrix(sol.tolist()) @ model_correction @ tcp_offset
        check_pos = np.array([T_check[0, 3], T_check[1, 3], T_check[2, 3]])
        err_pos = np.sum((check_pos - target_pos) ** 2)
        if err_pos < 0.001:
            valid.append(Joint6D.createFromRadians(*sol.tolist()))

    return valid
