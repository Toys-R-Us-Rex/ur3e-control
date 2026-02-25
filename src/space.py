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

# Example usage:
# s, R_w2tcp, t_w2tcp = similarity_transform_3D(p_world, p_tcp)

# def world_to_tcp(p):
#     p = np.asarray(p)
#     return s * ( R_w2tcp @ p ) + t_w2tcp

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

# Example usage:
# M, t = affine_transform_3D(p_world, p_tcp[:, :3])

# def world_to_tcp(p):
#     p = np.asarray(p)
#     return M @ p + t


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