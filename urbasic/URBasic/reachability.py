"""Reachability helper for UR3e using URBasic interfaces.

This module exposes a single public function: is_reachable().
"""

from __future__ import annotations

from typing import Sequence

import math
import numpy as np
from scipy.spatial.transform import Rotation

from .waypoint6d import Joint6D, TCP6D


UR3E_A = np.array([0.0, -0.24355, -0.2132, 0.0, 0.0, 0.0], dtype=float)
UR3E_D = np.array([0.15185, 0.0, 0.0, 0.13105, 0.08535, 0.0921], dtype=float)
UR3E_ALPHA = np.array([math.pi / 2, 0.0, 0.0, math.pi / 2, -math.pi / 2, 0.0], dtype=float)

DEFAULT_LINK_RADII = (0.06, 0.05, 0.05, 0.04, 0.035, 0.03)


def _as_vec3(v: Sequence[float], name: str) -> np.ndarray:
    if len(v) != 3:
        raise ValueError(f"{name} must contain exactly 3 values")
    return np.array([float(v[0]), float(v[1]), float(v[2])], dtype=float)


def _unit(v: np.ndarray) -> np.ndarray:
    n = float(np.linalg.norm(v))
    if n <= 1e-12:
        raise ValueError("normal vector magnitude must be > 0")
    return v / n


def _dh(a: float, alpha: float, d: float, theta: float) -> np.ndarray:
    ct, st = math.cos(theta), math.sin(theta)
    ca, sa = math.cos(alpha), math.sin(alpha)
    return np.array(
        [
            [ct, -st * ca, st * sa, a * ct],
            [st, ct * ca, -ct * sa, a * st],
            [0.0, sa, ca, d],
            [0.0, 0.0, 0.0, 1.0],
        ],
        dtype=float,
    )


def _joint_positions(joints: Sequence[float]) -> list[np.ndarray]:
    T = np.eye(4, dtype=float)
    points = [np.array([0.0, 0.0, 0.0], dtype=float)]
    for i in range(6):
        T = T @ _dh(UR3E_A[i], UR3E_ALPHA[i], UR3E_D[i], float(joints[i]))
        points.append(T[:3, 3].copy())
    return points


def _segment_point_distance(a: np.ndarray, b: np.ndarray, p: np.ndarray) -> float:
    ab = b - a
    denom = float(np.dot(ab, ab))
    if denom <= 1e-12:
        return float(np.linalg.norm(p - a))
    t = float(np.dot(p - a, ab) / denom)
    t = max(0.0, min(1.0, t))
    proj = a + t * ab
    return float(np.linalg.norm(p - proj))


def _segment_collides_box(
    a: np.ndarray,
    b: np.ndarray,
    box_min: np.ndarray,
    box_max: np.ndarray,
    margin: float,
    samples: int = 28,
) -> bool:
    lo = box_min - margin
    hi = box_max + margin
    for i in range(samples + 1):
        t = i / float(samples)
        p = a + t * (b - a)
        if np.all(p >= lo) and np.all(p <= hi):
            return True
    return False


def _box_min_max(obs: dict) -> tuple[np.ndarray, np.ndarray]:
    if "min" in obs and "max" in obs:
        return _as_vec3(obs["min"], "obstacle.min"), _as_vec3(obs["max"], "obstacle.max")
    if "center" in obs and "size" in obs:
        center = _as_vec3(obs["center"], "obstacle.center")
        half = 0.5 * _as_vec3(obs["size"], "obstacle.size")
        return center - half, center + half
    raise ValueError("box obstacle requires either min/max or center/size")


def _configuration_in_collision(
    joints: Sequence[float],
    obstacles: Sequence[dict],
    link_radii: Sequence[float],
    clearance: float,
) -> bool:
    pts = _joint_positions(joints)

    for i in range(6):
        a = pts[i]
        b = pts[i + 1]
        margin = float(link_radii[i]) + float(clearance)

        for obs in obstacles:
            kind = str(obs.get("type", "sphere")).lower()
            if kind == "sphere":
                center = _as_vec3(obs["center"], "obstacle.center")
                radius = float(obs["radius"])
                if _segment_point_distance(a, b, center) <= radius + margin:
                    return True
            elif kind == "box":
                box_min, box_max = _box_min_max(obs)
                if _segment_collides_box(a, b, box_min, box_max, margin):
                    return True
            else:
                raise ValueError(f"Unsupported obstacle type: {kind}")

    return False


def _orientation_from_normal(normal: np.ndarray, roll_hint_rotvec: np.ndarray | None) -> np.ndarray:
    z_axis = _unit(normal)

    x_hint = None
    if roll_hint_rotvec is not None:
        try:
            x_hint = Rotation.from_rotvec(roll_hint_rotvec).as_matrix()[:, 0]
        except Exception:
            x_hint = None

    if x_hint is not None:
        x_proj = x_hint - np.dot(x_hint, z_axis) * z_axis
        if np.linalg.norm(x_proj) > 1e-9:
            x_axis = _unit(x_proj)
        else:
            x_axis = np.array([1.0, 0.0, 0.0], dtype=float)
    else:
        ref = np.array([0.0, 0.0, 1.0], dtype=float)
        if abs(float(np.dot(ref, z_axis))) > 0.95:
            ref = np.array([1.0, 0.0, 0.0], dtype=float)
        x_axis = _unit(np.cross(ref, z_axis))

    y_axis = _unit(np.cross(z_axis, x_axis))
    x_axis = _unit(np.cross(y_axis, z_axis))
    return np.column_stack((x_axis, y_axis, z_axis))


def _target_pose(point: np.ndarray, normal: np.ndarray, roll_hint_rotvec: np.ndarray | None) -> TCP6D:
    R = _orientation_from_normal(normal, roll_hint_rotvec)
    rx, ry, rz = Rotation.from_matrix(R).as_rotvec()
    return TCP6D.createFromMetersRadians(point[0], point[1], point[2], float(rx), float(ry), float(rz))


def is_reachable(
    robot_control,
    point: Sequence[float],
    normal: Sequence[float],
    obstacles: Sequence[dict] | None = None,
    *,
    clearance: float = 0.005,
    link_radii: Sequence[float] = DEFAULT_LINK_RADII,
    interpolation_steps: int = 14,
) -> bool:
    """Return True if a point is reachable with the provided surface normal.

    This function is TCP-aware because it relies on URBasic's
    get_inverse_kin() with the robot's currently configured TCP (set_tcp).
    If your TCP is configured for a pen tip, reachability is evaluated for
    the pen tip; if TCP is at flange/other tool, that is used instead.

    Args:
        robot_control: URBasic control object with get_actual_joint_positions,
            get_actual_tcp_pose, get_inverse_kin.
        point: Target point [x, y, z] in meters (base frame).
        normal: Surface normal [nx, ny, nz] in base frame.
        obstacles: Optional list of obstacle dicts.
            Sphere: {"type": "sphere", "center": [x, y, z], "radius": r}
            Box: {"type": "box", "min": [x, y, z], "max": [x, y, z]}
                 or {"type": "box", "center": [x, y, z], "size": [sx, sy, sz]}
        clearance: Extra collision margin in meters.
        link_radii: Approximate collision radii for the 6 arm links.
        interpolation_steps: Number of checks between current and target joints.

    Returns:
        True if at least one orientation convention along the normal is
        reachable and collision-free, otherwise False.
    """
    if len(link_radii) != 6:
        raise ValueError("link_radii must contain 6 values")
    if interpolation_steps < 1:
        raise ValueError("interpolation_steps must be >= 1")

    p = _as_vec3(point, "point")
    n = _unit(_as_vec3(normal, "normal"))
    obstacles = list(obstacles) if obstacles is not None else []

    current_joints = robot_control.get_actual_joint_positions(wait=True)
    current_tcp = robot_control.get_actual_tcp_pose(wait=True)
    roll_hint = np.array([current_tcp.rx, current_tcp.ry, current_tcp.rz], dtype=float)

    normal_candidates = (n, -n)
    q_start = np.array(current_joints.toList(), dtype=float)

    for candidate_normal in normal_candidates:
        target_tcp = _target_pose(p, candidate_normal, roll_hint)

        try:
            q_target_obj = robot_control.get_inverse_kin(target_tcp, qnear=current_joints)
        except TypeError:
            q_target_obj = robot_control.get_inverse_kin(target_tcp)
        except Exception:
            continue

        if q_target_obj is None:
            continue

        if isinstance(q_target_obj, Joint6D):
            q_goal = np.array(q_target_obj.toList(), dtype=float)
        else:
            q_goal = np.array(q_target_obj, dtype=float)

        if q_goal.shape != (6,):
            continue

        collision_found = False
        if obstacles:
            for step in range(interpolation_steps + 1):
                t = step / float(interpolation_steps)
                q_interp = q_start + t * (q_goal - q_start)
                if _configuration_in_collision(q_interp, obstacles, link_radii, clearance):
                    collision_found = True
                    break

        if not collision_found:
            return True

    return False
