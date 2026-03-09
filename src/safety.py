"""Safety helpers for reachability and collision checks.

Usage:
    from src.safety import is_reachable, get_reachable

    reachable = is_reachable(robot_control, [x, y, z, nx, ny, nz], obstacles=[])
    poses = get_reachable([x, y, z, nx, ny, nz], obstacles=[])

MIT License

Copyright (c) 2026 HES-SO Valais-Wallis, Engineering Track 304

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

Author:     Nathan Antonietti, with assistance from Copilot (Microsoft)
Course:     HES-SO Valais-Wallis, Engineering Track 304
"""

from __future__ import annotations

from typing import Sequence

import math
import numpy as np
from scipy.spatial.transform import Rotation

from URBasic.waypoint6d import TCP6D

UR3E_A = np.array([0.0, -0.24355, -0.2132, 0.0, 0.0, 0.0], dtype=float)
UR3E_D = np.array([0.15185, 0.0, 0.0, 0.13105, 0.08535, 0.0921], dtype=float)
UR3E_ALPHA = np.array([math.pi / 2, 0.0, 0.0, math.pi / 2, -math.pi / 2, 0.0], dtype=float)

DEFAULT_LINK_RADII = (0.06, 0.05, 0.05, 0.04, 0.035, 0.03)




def is_colliding(
    tcp6d: "Sequence[float] | TCP6D",
    obstacles: "Sequence[dict] | None" = None,
    *,
    clearance: float = 0.005,
    link_radii: "Sequence[float]" = DEFAULT_LINK_RADII,
) -> bool:
    """Return True if the given TCP pose is in collision with any obstacle.

    .. note::
        Not yet implemented — always returns ``False``.
        Replace this stub with a real implementation when ready.

    Args:
        tcp6d: TCP pose to test [x, y, z, rx, ry, rz].
        obstacles: Optional list of obstacle dicts.
        clearance: Extra collision margin in metres.
        link_radii: Approximate collision radii for the 6 arm links.

    Returns:
        ``True`` if a collision is detected, ``False`` otherwise.
    """
    return False  # TODO: implement collision check


def get_reachable(
    tcp6d: "Sequence[float] | TCP6D",
    obstacles: "Sequence[dict] | None" = None,
    *,
    step: float = math.pi / 6,
    cone_angle: float = 0.0,
    tilt_step: float | None = None,
    roll_hint: "Sequence[float] | None" = None,
    clearance: float = 0.005,
    link_radii: "Sequence[float]" = DEFAULT_LINK_RADII,
) -> "list[TCP6D]":
    """Return all collision-free TCP6D poses sampled in a cone around the surface normal.

    Samples approach directions inside a cone of half-angle `cone_angle`
    around the given normal.  The cone is discretised into concentric rings
    at tilt angles [0, tilt_step, 2·tilt_step, …, cone_angle]; at each ring
    the azimuth is swept from 0 to 2π in `step`-radian increments (the
    on-axis direction tilt=0 is always included exactly once).  For each
    sampled direction the canonical orientation (roll = 0 relative to that
    direction) is evaluated via :func:`is_colliding`.

    When `cone_angle` is 0 (default) only the exact normal direction is
    tested, preserving the original behaviour.

    Args:
        tcp6d: Target TCP pose [x, y, z, nx, ny, nz].  The first three values
            are the position; the last three are the surface normal (need not
            be a unit vector — it will be normalised internally).
        obstacles: Optional list of obstacle dicts.
            Sphere: {"type": "sphere", "center": [x, y, z], "radius": r}
            Box:    {"type": "box", "min": [x, y, z], "max": [x, y, z]}
                    or {"type": "box", "center": [x, y, z], "size": [sx, sy, sz]}
        step: Azimuth step in radians for sweeping around each tilt ring
            (0 → 2π).  Default is π/6 (30°).  Must be > 0.
        cone_angle: Half-angle of the cone in radians.  0 tests only the
            exact normal.  Must be ≥ 0 and < π/2 (90°).
        tilt_step: Angular spacing between tilt rings in radians.  Defaults
            to ``step`` when omitted.  Must be > 0 when ``cone_angle`` > 0.
        roll_hint: Optional rotation-vector [rx, ry, rz] used as the
            reference orientation at roll = 0.  When omitted, a canonical
            frame is derived from the normal.
        clearance: Extra collision margin in metres passed to
            :func:`is_colliding`.
        link_radii: Approximate collision radii for the 6 arm links.

    Returns:
        List of :class:`~URBasic.waypoint6d.TCP6D` objects — one per sampled
        direction — where no collision was detected.  Returns an empty list
        when every sampled direction collides.
    """
    if step <= 0.0:
        raise ValueError("step must be > 0")
    if cone_angle < 0.0 or cone_angle >= math.pi / 2.0:
        raise ValueError("cone_angle must be in [0, π/2)")

    _tilt_step = tilt_step if tilt_step is not None else step
    if cone_angle > 0.0 and _tilt_step <= 0.0:
        raise ValueError("tilt_step must be > 0")

    p = _as_vec3(tcp6d[:3], "tcp6d position")
    n = _unit(_as_vec3(tcp6d[3:], "tcp6d normal"))
    obstacles = list(obstacles) if obstacles is not None else []
    hint_rotvec = np.array(roll_hint, dtype=float) if roll_hint is not None else None

    # Build two vectors perpendicular to n for cone parametrisation.
    ref = np.array([1.0, 0.0, 0.0], dtype=float)
    if abs(float(np.dot(ref, n))) > 0.95:
        ref = np.array([0.0, 1.0, 0.0], dtype=float)
    x_perp = _unit(np.cross(n, ref))
    y_perp = _unit(np.cross(n, x_perp))

    # Collect tilt ring angles: always start with 0 (the exact normal).
    tilt_angles = [0.0]
    if cone_angle > 1e-12:
        t = _tilt_step
        while t <= cone_angle + 1e-9:
            tilt_angles.append(float(t))
            t += _tilt_step

    result: list[TCP6D] = []
    for tilt in tilt_angles:
        if tilt < 1e-12:
            # On-axis: a single direction, no azimuth sweep needed.
            directions = [n]
        else:
            azimuths = np.arange(0.0, 2.0 * math.pi, float(step))
            directions = [
                math.cos(tilt) * n
                + math.sin(tilt) * (math.cos(float(az)) * x_perp + math.sin(float(az)) * y_perp)
                for az in azimuths
            ]

        for direction in directions:
            candidate_tcp = _target_pose_with_roll(p, _unit(direction), hint_rotvec, 0.0)
            if not is_colliding(candidate_tcp, obstacles, clearance=clearance, link_radii=link_radii):
                result.append(candidate_tcp)

    return result


def have_collision(tcp, robot):
    raise NotImplementedError


def divide_path(tcp1, tcp2):
    raise NotImplementedError


# ==================== UTILS ====================

def _as_vec3(v: Sequence[float], name: str) -> np.ndarray:
    if len(v) != 3:
        raise ValueError(f"{name} must contain exactly 3 values")
    return np.array([float(v[0]), float(v[1]), float(v[2])], dtype=float)


def _unit(v: np.ndarray) -> np.ndarray:
    n = float(np.linalg.norm(v))
    if n <= 1e-12:
        raise ValueError("normal vector magnitude must be > 0")
    return v / n

def _target_pose(point: np.ndarray, normal: np.ndarray, roll_hint_rotvec: np.ndarray | None) -> TCP6D:
    R = _orientation_from_normal(normal, roll_hint_rotvec)
    rx, ry, rz = Rotation.from_matrix(R).as_rotvec()
    return TCP6D.createFromMetersRadians(point[0], point[1], point[2], float(rx), float(ry), float(rz))

def _target_pose_with_roll(
    point: np.ndarray,
    normal: np.ndarray,
    roll_hint_rotvec: np.ndarray | None,
    roll_angle: float,
) -> TCP6D:
    R_base = _orientation_from_normal(normal, roll_hint_rotvec)
    if abs(float(roll_angle)) > 1e-12:
        R_roll = Rotation.from_rotvec(np.array([0.0, 0.0, float(roll_angle)], dtype=float)).as_matrix()
        R = R_base @ R_roll
    else:
        R = R_base

    rx, ry, rz = Rotation.from_matrix(R).as_rotvec()
    return TCP6D.createFromMetersRadians(point[0], point[1], point[2], float(rx), float(ry), float(rz))

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
            ref = np.array([1.0, 0.0, 0.0], dtype=float)
            if abs(float(np.dot(ref, z_axis))) > 0.95:
                ref = np.array([0.0, 1.0, 0.0], dtype=float)
            x_axis = _unit(np.cross(ref, z_axis))
    else:
        ref = np.array([0.0, 0.0, 1.0], dtype=float)
        if abs(float(np.dot(ref, z_axis))) > 0.95:
            ref = np.array([1.0, 0.0, 0.0], dtype=float)
        x_axis = _unit(np.cross(ref, z_axis))

    if np.linalg.norm(np.cross(z_axis, x_axis)) <= 1e-12:
        ref = np.array([0.0, 1.0, 0.0], dtype=float)
        if abs(float(np.dot(ref, z_axis))) > 0.95:
            ref = np.array([1.0, 0.0, 0.0], dtype=float)
        x_axis = _unit(np.cross(ref, z_axis))

    y_axis = _unit(np.cross(z_axis, x_axis))
    x_axis = _unit(np.cross(y_axis, z_axis))
    return np.column_stack((x_axis, y_axis, z_axis))