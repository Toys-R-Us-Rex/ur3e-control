# uv run python -m tests.test_transformation
import numpy as np

from src.transformation import create_transformation

# ---------------------------------------------------------
# Utilities
# ---------------------------------------------------------

def random_rotation():
    """Generate a random 3×3 rotation matrix."""
    M = np.random.randn(3, 3)
    U, _, Vt = np.linalg.svd(M)
    R = U @ Vt
    if np.linalg.det(R) < 0:
        U[:, -1] *= -1
        R = U @ Vt
    return R

def normal_to_rxyz(n):
    x, y, z = n
    z = -z
    rx = np.arctan2(-y, np.sqrt(x**2 + z**2))
    ry = np.arctan2(x, z)
    rz = 0
    return [rx, ry, rz]

# ---------------------------------------------------------
# Synthetic test
# ---------------------------------------------------------

def transformation_test():
    # 1. True transform
    R_true = random_rotation()
    t_true = np.array([0.5, -1.2, 2.0])

    # 2. Generate synthetic points and normals
    N = 30
    A_points = np.random.randn(N, 3)
    A_normals = np.random.randn(N, 3)
    A_normals /= np.linalg.norm(A_normals, axis=1, keepdims=True)

    A = np.hstack([A_points, A_normals])

    # 3. Apply true transform
    B_points = (R_true @ A_points.T).T + t_true
    B_normals = (R_true @ A_normals.T).T
    B_normals /= np.linalg.norm(B_normals, axis=1, keepdims=True)

    B = np.hstack([B_points, B_normals])

    # 4. Recover transform
    f = create_transformation(A, B)

    # 5. Evaluate accuracy
    point_errors = []
    normal_errors = []
    r_errors = []

    for i in range(N):
        pA = A[i]
        pB_pred = np.array(f(pA))
        pB_true = B[i]

        point_errors.append(np.linalg.norm(pB_pred[:3] - pB_true[:3]))
        # Compare only the vector normals, not the rxyz angles
        n_pred = (R_true @ A_normals[i])
        n_pred /= np.linalg.norm(n_pred)

        n_true = B_normals[i]

        r_pred = normal_to_rxyz(n_pred)
        r_true = normal_to_rxyz(n_true)

        normal_errors.append(np.linalg.norm(n_pred - n_true))
        r_errors.append(np.linalg.norm(np.array(r_pred) - np.array(r_true)))

    print("Mean point error:", np.mean(point_errors))
    print("Max point error:", np.max(point_errors))
    print("Mean normal error:", np.mean(normal_errors))
    print("Max normal error:", np.max(normal_errors))
    print("Mean normal error:", np.mean(r_errors))
    print("Max normal error:", np.max(r_errors))

if __name__ == '__main__':
    transformation_test()