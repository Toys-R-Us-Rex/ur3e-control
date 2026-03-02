import numpy as np

from URBasic import TCP6D
from URBasic import UrScript


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

    # Build homogeneous source matrix
    P = np.hstack([A, np.ones((N, 1))])  # Nx4

    # Solve P @ T = B  → T is 4×3
    T, _, _, _ = np.linalg.lstsq(P, B, rcond=None)

    # Extract affine components
    R = T[:3, :].T      # 3×3 linear part
    t = T[3, :]         # 3-vector translation

    # Precompute normal transform
    R_normal = np.linalg.inv(R).T

    def AtoB(p):
        p = np.asarray(p)
        point = p[:3]
        normal = p[3:]

        # Transform point
        p_new = R @ point + t

        # Transform normal
        n_new = R_normal @ normal
        n_new /= np.linalg.norm(n_new)

        # Convert to your rxyz representation
        r_new = _normal_to_rxyz(n_new)

        return (*p_new, *r_new)

    return AtoB

