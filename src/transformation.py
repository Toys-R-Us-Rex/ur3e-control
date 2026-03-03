import numpy as np

from URBasic import TCP6D
from URBasic import UrScript


def collect_data(robot_arm: UrScript, world_measure):
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

        robot_arm.end_freedrive_mode()

        # Return robot positions
        tcp = robot_arm.get_actual_tcp_pose().toList()

        tcps.append(tcp)
        k += 1
        print("Captured measure ", k)


    robot_arm.end_freedrive_mode()

    return np.array(tcps)

def _normal_to_rxyz(n):
    x,y,z = n
    z = -z
    rx = np.arctan2(-y, np.sqrt(x**2 + z**2))
    ry = np.arctan2(x, z)
    rz = 0
    return [rx,ry,rz]

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

    # Normal transform
    R_normal = np.linalg.inv(R).T
    T_normal = np.eye(4)
    T_normal[:3, :3] = R_normal

    def AtoB(p):
        p = np.asarray(p)
        point = p[:3]
        normal = p[3:]

        # Transform point
        p_new = T @ [*point, 1]

        # Transform normal
        nx,ny,nz = normal
        n_new = T_normal @ [nx,ny,nz, 1]
        n_new /= np.linalg.norm(n_new)
        r_new = _normal_to_rxyz(n_new[:3])

        return [*p_new[:3], *r_new]

    return AtoB


def _rotvec_to_rotmat(r):
    theta = np.linalg.norm(r)
    if theta < 1e-12:
        return np.eye(3)
    k = r / theta
    K = np.array([[0, -k[2], k[1]],
                  [k[2], 0, -k[0]],
                  [-k[1], k[0], 0]])
    return np.eye(3) + np.sin(theta)*K + (1-np.cos(theta))*(K @ K)

def _rotmat_to_rotvec(R):
    theta = np.arccos((np.trace(R) - 1) / 2)
    if theta < 1e-12:
        return np.zeros(3)
    rx = (R[2,1] - R[1,2]) / (2*np.sin(theta))
    ry = (R[0,2] - R[2,0]) / (2*np.sin(theta))
    rz = (R[1,0] - R[0,1]) / (2*np.sin(theta))
    return theta * np.array([rx, ry, rz])

def tcp_trans(tcp1, tcp2):
    # Décomposition
    p1 = np.array(tcp1[:3])
    r1 = np.array(tcp1[3:])
    p2 = np.array(tcp2[:3])
    r2 = np.array(tcp2[3:])

    # Matrices de rotation
    R1 = _rotvec_to_rotmat(r1)
    R2 = _rotvec_to_rotmat(r2)

    # Composition
    p_new = p1 + R1 @ p2
    R_new = R1 @ R2
    r_new = _rotmat_to_rotvec(R_new)

    return np.concatenate([p_new, r_new])