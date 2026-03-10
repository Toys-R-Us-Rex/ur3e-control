'''
Utility functions for working with 3D rotations and poses.

This module provides lightweight helpers for converting between several
common rotation representations used in robotics and computer vision:

- normal_to_rotvec(n):
    Computes the rotation vector that aligns the +Z axis with a given
    surface normal. Useful for orienting tools or end-effectors so they
    face a surface.

- rotvec_to_rotmat(r):
    Converts a rotation vector (axis-angle representation) into a 3x3
    rotation matrix using Rodrigues' formula.

- rotmat_to_rotvec(R):
    Converts a 3x3 rotation matrix into a rotation vector.

- pose_to_T(pose):
    Converts a 6-element pose [x, y, z, rx, ry, rz] (UR-style axis-angle)
    into a 4x4 homogeneous transformation matrix.

All functions assume right-handed coordinate frames and use NumPy for
vector and matrix operations.

Usage
-----
This module is designed to be used with the URBasic library, from which this
project is derived:
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

Author:     Mariéthoz Cédric, with assistance from Copilote AI (Microsoft)
Course:     HES-SO Valais-Wallis, Engineering Track 304
'''
import numpy as np

def normal_to_rotvec(n):
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