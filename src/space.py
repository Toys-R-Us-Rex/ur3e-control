import numpy as np
from time import sleep

from URBasic import TCP6D
from URBasic import UrScript

def similarity_transform_3D(A, B):
    """
    Limited to similar transformation
    Compute s, R, t such that:  B ≈ s * R @ A + t
    A: Nx3 points in source frame
    B: Nx3 points in target frame
    """
    A = np.asarray(A)[:, :3]
    B = np.asarray(B)[:, :3]
    assert A.shape == B.shape
    N = A.shape[0]

    centroid_A = A.mean(axis=0)
    centroid_B = B.mean(axis=0)

    AA = A - centroid_A
    BB = B - centroid_B

    H = AA.T @ BB

    U, S, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T

    if np.linalg.det(R) < 0:
        Vt[2, :] *= -1
        R = Vt.T @ U.T

    # scale factor
    var_A = (AA ** 2).sum()
    s = (S.sum()) / var_A

    t = centroid_B - s * R @ centroid_A

    def world_to_tcp(p):
        p = np.asarray(p)
        return s * ( R @ p ) + t
    
    return world_to_tcp


def affine_transform_3D(A, B):
    """
    Solve B ≈ A*M + t  for M (3x3) and t (3,)
    A: Nx3 source points
    B: Nx3 target points
    """
    A = np.asarray(A)[:, :3]
    B = np.asarray(B)[:, :3]
    assert A.shape == B.shape
    N = A.shape[0]

    # Build augmented matrix [A | 1]
    A_aug = np.hstack([A, np.ones((N, 1))])   # Nx4

    # Solve least squares: A_aug @ X = B
    # X is 4x3 → last row is translation
    X, _, _, _ = np.linalg.lstsq(A_aug, B, rcond=None)

    M = X[:3, :].T   # 3x3
    t = X[3, :]      # 3,
    t = t.tolist()

    def world_to_tcp(p):
        p = np.asarray(p)
        return M @ p + t
    
    return world_to_tcp

def normal_to_rxyz(n):
    x,y,z = n
    z = -z
    rx = np.arctan2(-y, np.sqrt(x**2 + z**2))
    ry = np.arctan2(x, z)
    rz = 0
    return [rx,ry,rz]

def affine_transform_rot_3D(A, B):
    """
    Compute rigid transform (R, t) such that:
        p_tcp ≈ R * p_world + t
    using the Kabsch algorithm.
    Returns a function that transforms a point and a normal vector.
    """
    A = np.asarray(A)[:, :3]
    B = np.asarray(B)[:, :3]
    assert A.shape == B.shape
    N = A.shape[0]

    # Build augmented matrix [A | 1]
    A_aug = np.hstack([A, np.ones((N, 1))])   # Nx4

    # Solve least squares: A_aug @ X = B
    # X is 4x3 → last row is translation
    X, _, _, _ = np.linalg.lstsq(A_aug, B, rcond=None)

    M = X[:3, :].T   # 3x3
    t = X[3, :]      # 3,
    t = t.tolist()

    # Extract only XYZ from p_tcp (ignore rx, ry, rz)
    p_tcp_xyz = A[:, :3]

    # Compute centroids
    c_world = B.mean(axis=0)
    c_tcp   = p_tcp_xyz.mean(axis=0)

    # Center the points
    W = B - c_world
    T = p_tcp_xyz - c_tcp

    # Compute covariance matrix
    H = W.T @ T

    # SVD for rotation
    U, S, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T

    # Fix improper rotation (reflection)
    if np.linalg.det(R) < 0:
        Vt[-1, :] *= -1
        R = Vt.T @ U.T

    # Return a function that applies the transform
    def transform(point, normal):
        point = np.array(point)
        normal = np.array(normal)

        # Transform cartesian
        p_new = M @ point + t

        # Transform rotation
        r_new = normal_to_rxyz(normal)

        return (*p_new, *r_new)

    return transform


def collect_data(robot_arm: UrScript, world_measure):
    if len(world_measure) < 3:
        print("Minimum 3 measure points.")
        raise ValueError("You don't provide eanough world measure")

    num_measure = len(world_measure)
    
    tcps = []

    robot_arm.set_tcp(TCP6D.createFromMetersRadians(0,0,0,0,0,0))

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

        robot_arm.end_freedrive_mode()

        # Return robot positions
        tcp = robot_arm.get_actual_tcp_pose().toList()

        tcps.append(tcp)
        k += 1
        print("Captured measure ", k)


    robot_arm.end_freedrive_mode()

    return np.array(tcps)

def rotvec_to_rotmat(r):
    """Convert rotation vector to rotation matrix using Rodrigues' formula."""
    theta = np.linalg.norm(r)
    if theta < 1e-12:
        return np.eye(3)
    k = r / theta
    K = np.array([[0, -k[2], k[1]],
                  [k[2], 0, -k[0]],
                  [-k[1], k[0], 0]])
    R = np.eye(3) + np.sin(theta) * K + (1 - np.cos(theta)) * (K @ K)
    return R

def rotmat_to_rotvec(R):
    """Convert rotation matrix to rotation vector."""
    theta = np.arccos((np.trace(R) - 1) / 2)
    if theta < 1e-12:
        return np.zeros(3)
    rx = (R[2,1] - R[1,2]) / (2*np.sin(theta))
    ry = (R[0,2] - R[2,0]) / (2*np.sin(theta))
    rz = (R[1,0] - R[0,1]) / (2*np.sin(theta))
    return theta * np.array([rx, ry, rz])

def pose_to_matrix(p):
    """Convert UR pose to 4x4 homogeneous matrix."""
    x, y, z, rx, ry, rz = p
    R = rotvec_to_rotmat(np.array([rx, ry, rz]))
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = [x, y, z]
    return T

def matrix_to_pose(T):
    """Convert 4x4 homogeneous matrix back to UR pose."""
    x, y, z = T[:3, 3]
    R = T[:3, :3]
    rx, ry, rz = rotmat_to_rotvec(R)
    return [x, y, z, rx, ry, rz]

def pose_trans(p1, p2):
    """Equivalent of UR's pose_trans(p1, p2)."""
    T1 = pose_to_matrix(p1)
    T2 = pose_to_matrix(p2)
    T = T1 @ T2
    return matrix_to_pose(T)
