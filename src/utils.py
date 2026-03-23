"""
Utility functions for working with 3D rotations and poses.

Usage
-----
This module is designed to be used with our Duckify simulation environment,
and the URBasic library from which it is derived:
    https://github.com/ISC-HEI/ur3e-control

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

Author:     Mariéthoz Cédric, with assistance from Copilot AI (Microsoft)
Co-Author:  Savioz Pierre-Yves, with assistance from Claude AI (Anthropic)
Course:     HES-SO Valais-Wallis, Engineering Track 304
"""

import numpy as np

def normal_to_rotvec(n: np.ndarray|list):
    """
    Compute the rotation vector that aligns the +Z axis with a given normal.

    Parameters
    ----------
    n : array_like, shape (3,)
        Target surface normal. The function computes the rotation that
        points the +Z axis toward -n (useful for tool orientations that
        must face into a surface).

    Returns
    -------
    rotvec : ndarray, shape (3,)
        Axis-angle rotation vector representing the required orientation.
    """

    n = np.asarray(n, dtype=float)
    target = -n  # pen faces into surface
    z_axis = np.array([0.0, 0.0, 1.0])
    cross = np.cross(z_axis, target)
    sin_angle = np.linalg.norm(cross)
    cos_angle = np.dot(z_axis, target)
    if sin_angle < 1e-12:
        if cos_angle > 0:
            return [0.0, 0.0, 0.0]       # target already [0,0,1]
        else:
            return [0.0, np.pi, 0.0]     # 180° flip around Y
    axis = cross / sin_angle
    angle = np.arctan2(sin_angle, cos_angle)
    rotvec = axis * angle
    return rotvec

def rotvec_to_rotmat(r):
    """
    Convert a rotation vector (axis-angle) into a 3x3 rotation matrix.

    Parameters
    ----------
    r : array_like, shape (3,)
        Rotation vector whose direction is the rotation axis and whose
        magnitude is the rotation angle in radians.

    Returns
    -------
    R : ndarray, shape (3, 3)
        Corresponding rotation matrix computed via Rodrigues' formula.
    """

    theta = np.linalg.norm(r)
    if theta < 1e-12:
        return np.eye(3)
    k = r / theta
    K = np.array([[0, -k[2], k[1]],
                  [k[2], 0, -k[0]],
                  [-k[1], k[0], 0]])
    return np.eye(3) + np.sin(theta)*K + (1-np.cos(theta))*(K @ K)

def rotmat_to_rotvec(R):
    """
    Convert a 3x3 rotation matrix into a rotation vector.

    Parameters
    ----------
    R : array_like, shape (3, 3)
        Proper rotation matrix.

    Returns
    -------
    rotvec : ndarray, shape (3,)
        Axis-angle rotation vector whose magnitude is the rotation angle.
    """

    theta = np.arccos((np.trace(R) - 1) / 2)
    if theta < 1e-12:
        return np.zeros(3)
    rx = (R[2,1] - R[1,2]) / (2*np.sin(theta))
    ry = (R[0,2] - R[2,0]) / (2*np.sin(theta))
    rz = (R[1,0] - R[0,1]) / (2*np.sin(theta))
    return theta * np.array([rx, ry, rz])


def pose_to_T(pose):
    """
    Convert a 6-element UR-style pose into a 4x4 homogeneous transform.

    Parameters
    ----------
    pose : array_like, shape (6,)
        Pose in the format [x, y, z, rx, ry, rz], where the last three
        elements form an axis-angle rotation vector (UR convention).

    Returns
    -------
    T : ndarray, shape (4, 4)
        Homogeneous transformation matrix with rotation in the upper-left
        3x3 block and translation in the last column.
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


def fmt_tcp(tcp):
    """
    Format a TCP pose as a string.

    Parameters
    ----------
    tcp : TCP6D
        The TCP pose to format.

    Returns
    -------
    str
        The formatted string.
    """
    return f"({tcp.x:.4f}, {tcp.y:.4f}, {tcp.z:.4f})"


def tcp_trans(tcp1, tcp2):
    """
    Compose two TCP poses (position + rotation vector) and return the resulting pose.

    Parameters
    ----------
    tcp1 : UrScript
        First TCP pose to compose.
    tcp2 : UrScript
        Second TCP pose to compose.

    Returns
    -------
    motion : list of TCP6D
        Sequence of poses including the reference pose and rotated poses.
    """
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
    Mapping: STL(x, y, z) → OBJ(x, z, -y).

    Accepts:
        - a single point: (3,)
        - a list of points: (N, 3)
    """
    pts = np.asarray(pts)

    # Single point
    if pts.ndim == 1:
        x, y, z = pts
        return np.array([x, z, -y])

    # Multiple points (N, 3)
    if pts.ndim == 2 and pts.shape[1] == 3:
        x = pts[:, 0]
        y = pts[:, 1]
        z = pts[:, 2]
        return np.column_stack([x, z, -y])

    raise ValueError("Input must be shape (3,) or (N,3)")


def ask_yes_no(prompt: str) -> bool:
    """
    Ask the user a yes/no question.

    Parameters
    ----------
    prompt : str
        The question to ask.

    Returns
    -------
    bool
        True if the user answers "y", False otherwise.
    """
    return input(prompt).strip().lower() == "y"
    
class AtoB:
    """
    A similarity transform that maps points from one coordinate system to another.
    """
    def __init__(self, T_position, T_orientation):
        self.T_position = T_position
        self.T_orientation = T_orientation
    
    def __call__(self, p):
        p = np.asarray(p)
        point = p[:3]
        normal = p[3:]

        # Transform point
        p_h = np.array([*point, 1.0])
        p_new = self.T_position @ p_h

        # Transform normal
        n_h = np.array([*normal, 1.0])
        n_new = (self.T_orientation @ n_h)[:3]
        n_new /= np.linalg.norm(n_new)

        r_new = normal_to_rotvec(n_new)

        return [*p_new[:3], *r_new]