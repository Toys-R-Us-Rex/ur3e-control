'''
Author: Pierre-Yves Savioz with Assistance from Claude AI ( Anthropic )
'''

import logging
import math
from pathlib import Path

import numpy as np
import pybullet as p
import pybullet_data
from pyb_utils import CollisionDetector
from scipy.spatial.transform import Rotation

from src.config import (
    JOINT_LIMITS,
    TCP_Y_MAX, TCP_Z_MIN, TCP_Z_MAX, UR3E_MAX_REACH,
    FREE_TRAVEL_STEP,
)

log = logging.getLogger(__name__)

# Default URDF lives in robot_reports/ next to the mesh directories
_DEFAULT_URDF = Path(__file__).resolve().parents[1] / '..' / 'robot_reports' / 'ur3e.urdf'

# PyBullet joint indices for the 6 revolute joints (skip fixed joints)
_NUM_DOF = 6


# ---------------------------------------------------------------------------
# Orientation helpers (used by cone-sampling fallback)
# ---------------------------------------------------------------------------

def _as_vec3(v, name="vector"):
    """Convert to a flat 3-element numpy array with validation."""
    a = np.asarray(v, dtype=float).ravel()
    if a.shape != (3,):
        raise ValueError(f"{name} must have 3 elements, got {a.shape}")
    return a


def _unit(v):
    """Return the unit vector of *v*, or raise if zero-length."""
    n = np.linalg.norm(v)
    if n < 1e-12:
        raise ValueError("Cannot normalise a zero-length vector")
    return v / n


def _orientation_from_normal(normal, roll_hint_rotvec):
    """Build a rotation matrix whose Z-axis aligns with *normal*.

    The roll around Z is seeded from *roll_hint_rotvec* so that the
    resulting orientation is close to the original tool orientation.
    Returns a ``Rotation`` object.
    """
    z_axis = _unit(_as_vec3(normal, "normal"))

    # Pick an arbitrary perpendicular to build a frame
    ref = np.array([1.0, 0.0, 0.0])
    if abs(np.dot(z_axis, ref)) > 0.9:
        ref = np.array([0.0, 1.0, 0.0])
    x_axis = _unit(np.cross(z_axis, ref))
    y_axis = np.cross(z_axis, x_axis)

    mat = np.column_stack([x_axis, y_axis, z_axis])
    base_rot = Rotation.from_matrix(mat)

    # Apply roll from the hint rotation vector
    hint_rot = Rotation.from_rotvec(_as_vec3(roll_hint_rotvec, "roll_hint"))
    # Extract the roll component around z_axis
    # We compose so the resulting orientation keeps the same "up" feel
    return base_rot


# ---------------------------------------------------------------------------
# CollisionChecker — PyBullet-based collision safety layer
# ---------------------------------------------------------------------------

class CollisionChecker:
    """Headless PyBullet collision checker for the UR3e.

    Uses ``pyb_utils.CollisionDetector`` with explicit collision pairs
    instead of brute-force closest-point queries.  All queries run in
    DIRECT mode — no GPU or display required.

    Written by Claude AI (Anthropic)
    """

    # Non-adjacent self-collision link pairs (separated by ≥2 revolute
    # joints).  Link indices for the flattened UR3e URDF:
    #   1=base_link_inertia, 2=shoulder, 3=upper_arm, 4=forearm,
    #   5=wrist_1, 6=wrist_2, 7=wrist_3, 8=flange, 9=tool0,
    #   10=wrist_cam, 11=hande_base, 12=left_finger, 13=right_finger,
    #   14=pen
    _SELF_COLLISION_LINK_PAIRS: list[tuple[int, int]] = [
        (1, 4), (1, 5), (1, 6), (1, 7), (1, 8), (1, 9),
        (1, 10), (1, 11), (1, 12), (1, 13), (1, 14),
        (2, 5), (2, 6), (2, 7), (2, 8), (2, 9),
        (2, 10), (2, 11), (2, 12), (2, 13), (2, 14),
        (3, 6), (3, 7), (3, 8), (3, 9),
        (3, 10), (3, 11), (3, 12), (3, 13), (3, 14),
        (4, 7), (4, 8), (4, 9),
        (4, 10), (4, 11), (4, 12), (4, 13), (4, 14),
    ]

    def __init__(
        self,
        urdf_path: str | Path | None = None,
        obstacle_stls: list[dict] | None = None,
        gui: bool = False,
    ):
        """
        Parameters
        ----------
        urdf_path : path to the flattened UR3e URDF.
            Defaults to ``robot_reports/ur3e.urdf``.
        obstacle_stls : list of dicts, each with keys
            ``path`` (str), and optionally ``position`` (3-list),
            ``orientation`` (4-list quaternion [x,y,z,w]),
            ``scale`` (3-list).
        gui : if True, open a PyBullet GUI window for visualization.

        Written by Claude AI (Anthropic)
        """
        self.urdf_path = str(urdf_path or _DEFAULT_URDF)
        self.cid = p.connect(p.GUI if gui else p.DIRECT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath(),
                                  physicsClientId=self.cid)

        # The UR URDF has an internal π rotation on base_link_inertia.
        # Rotate the base by π around Z so PyBullet visuals match the
        # DH / Gazebo / real-robot coordinate frame.
        base_orn = p.getQuaternionFromEuler([0, 0, np.pi])
        self.robot_id = p.loadURDF(
            self.urdf_path,
            basePosition=[0, 0, 0],
            baseOrientation=base_orn,
            useFixedBase=True,
            physicsClientId=self.cid,
        )

        # Discover the revolute joint indices
        self.joint_indices: list[int] = []
        num_joints = p.getNumJoints(self.robot_id,
                                     physicsClientId=self.cid)
        for i in range(num_joints):
            info = p.getJointInfo(self.robot_id, i,
                                  physicsClientId=self.cid)
            if info[2] in (p.JOINT_REVOLUTE, p.JOINT_PRISMATIC):
                self.joint_indices.append(i)

        # All robot link indices (for obstacle pairing)
        self._link_indices = list(range(num_joints))

        # Build the self-collision detector
        self_pairs = [
            ((self.robot_id, a), (self.robot_id, b))
            for a, b in self._SELF_COLLISION_LINK_PAIRS
        ]
        self._self_detector = CollisionDetector(self.cid, self_pairs)

        # Load obstacle meshes and build the obstacle detector
        self._gui = gui
        self.obstacle_ids: list[int] = []
        for obs in (obstacle_stls or []):
            col_shape = p.createCollisionShape(
                p.GEOM_MESH,
                fileName=str(obs['path']),
                meshScale=obs.get('scale', [1, 1, 1]),
                physicsClientId=self.cid,
            )
            vis_shape = -1
            if gui:
                vis_shape = p.createVisualShape(
                    p.GEOM_MESH,
                    fileName=str(obs['path']),
                    meshScale=obs.get('scale', [1, 1, 1]),
                    rgbaColor=[0.6, 0.6, 0.8, 0.7],
                    physicsClientId=self.cid,
                )
            oid = p.createMultiBody(
                baseMass=0,
                baseCollisionShapeIndex=col_shape,
                baseVisualShapeIndex=vis_shape,
                basePosition=obs.get('position', [0, 0, 0]),
                baseOrientation=obs.get('orientation', [0, 0, 0, 1]),
                physicsClientId=self.cid,
            )
            self.obstacle_ids.append(oid)
        self._rebuild_obstacle_detector()

    def _rebuild_obstacle_detector(self) -> None:
        """(Re)build the obstacle CollisionDetector from current obstacles."""
        obstacle_pairs = [
            ((self.robot_id, link), (oid, -1))
            for oid in self.obstacle_ids
            for link in self._link_indices
        ]
        self._obstacle_detector = (
            CollisionDetector(self.cid, obstacle_pairs)
            if obstacle_pairs else None
        )

    # -- Joint state ----------------------------------------------------------

    def set_joint_angles(self, q) -> None:
        """Set the robot's joint angles (6-vector, radians)."""
        q = np.asarray(q, dtype=float)
        for idx, angle in zip(self.joint_indices, q):
            p.resetJointState(self.robot_id, idx, float(angle),
                              physicsClientId=self.cid)

    # -- Collision queries ----------------------------------------------------

    def has_self_collision(self) -> bool:
        """True if any pair of non-adjacent links overlap."""
        return self._self_detector.in_collision(margin=0)

    def has_obstacle_collision(self, margin: float = 0.005) -> bool:
        """True if any robot link is within *margin* of an obstacle."""
        if self._obstacle_detector is None:
            return False
        return self._obstacle_detector.in_collision(margin=margin)

    # -- Layer 1: Modular point validators ------------------------------------

    def check_workspace_bounds(self, tcp_xyz) -> tuple[bool, str]:
        """Check if TCP position is within the safe workspace envelope.

        Returns (ok, reason).
        """
        x, y, z = tcp_xyz[0], tcp_xyz[1], tcp_xyz[2]
        if y > TCP_Y_MAX:
            return False, f"TCP Y={y:.4f} > {TCP_Y_MAX}"
        if z < TCP_Z_MIN:
            return False, f"TCP Z={z:.4f} < {TCP_Z_MIN}"
        if z > TCP_Z_MAX:
            return False, f"TCP Z={z:.4f} > {TCP_Z_MAX}"
        reach = np.sqrt(x*x + y*y + z*z)
        if UR3E_MAX_REACH is not None and reach > UR3E_MAX_REACH:
            return False, f"TCP reach={reach:.4f} > {UR3E_MAX_REACH}"
        return True, ""

    def check_ik_solvable(self, robot, tcp, qnear=None):
        """Try to solve IK for the given TCP.

        Returns (ok, Joint6D_or_None, reason).
        """
        q = robot.get_inverse_kin(tcp, qnear=qnear)
        if q is None:
            return False, None, "IK has no solution"
        return True, q, ""

    def check_joint_limits(self, q) -> tuple[bool, str]:
        """Check if all joints are within JOINT_LIMITS.

        Returns (ok, reason). Accepts Joint6D or array-like.
        """
        q_list = q.toList() if hasattr(q, 'toList') else list(q)
        for i, (angle, limit) in enumerate(zip(q_list, JOINT_LIMITS)):
            if limit is None:
                continue
            lo, hi = limit
            if angle < lo or angle > hi:
                return False, f"Joint {i} = {angle:.4f} outside [{lo}, {hi}]"
        return True, ""

    def validate_tcp(self, robot, tcp, qnear=None,
                     margin: float = 0.005, check_obstacle: bool = True,
                     orientation_search: bool = False,
                     max_cone_angle: float = math.radians(15),
                     cone_step: float = math.radians(5)):
        """Full validation of a TCP pose: bounds → IK → limits → collision.

        Runs checks 1→4, short-circuits on first failure.

        Parameters
        ----------
        check_obstacle : bool
            If False, skip obstacle collision check (useful for draw points
            where the pen tip is intentionally inside the surface).
        orientation_search : bool
            If True and IK fails, try nearby orientations in a cone.
        max_cone_angle : float
            Half-angle of the search cone in radians (default 15°).
        cone_step : float
            Angular step for tilt rings and azimuth sweep (default 5°).

        Returns (ok, Joint6D_or_None, reason, tcp_used).
        tcp_used is the TCP6D that actually worked (original or adjusted).
        """
        tcp_xyz = np.array([tcp.x, tcp.y, tcp.z])

        # 1. Workspace bounds
        ok, reason = self.check_workspace_bounds(tcp_xyz)
        if not ok:
            return False, None, reason, tcp

        # Try the original orientation first (checks 2→3→4)
        ok, q, reason = self._try_full_validation(
            robot, tcp, qnear, margin, check_obstacle,
        )
        if ok:
            return True, q, "", tcp

        # Original failed — try cone search if enabled
        if orientation_search:
            original_rv = np.array([tcp.rx, tcp.ry, tcp.rz])
            result = self._search_orientation_cone(
                robot, tcp_xyz, original_rv,
                max_cone_angle, cone_step, qnear,
                margin, check_obstacle,
            )
            if result is not None:
                q, adjusted_tcp = result
                return True, q, "", adjusted_tcp

        return False, None, reason, tcp

    def _try_full_validation(self, robot, tcp, qnear, margin, check_obstacle):
        """Run IK → joint limits → collision checks.

        Returns (ok, Joint6D_or_None, reason).
        """
        ok, q, reason = self.check_ik_solvable(robot, tcp, qnear)
        if not ok:
            return False, None, reason

        ok, reason = self.check_joint_limits(q)
        if not ok:
            return False, None, reason

        q_arr = q.toList() if hasattr(q, 'toList') else list(q)
        self.set_joint_angles(q_arr)
        if self.has_self_collision():
            return False, None, "Self-collision detected"
        if check_obstacle and self.has_obstacle_collision(margin):
            return False, None, "Obstacle collision detected"

        return True, q, ""

    def _search_orientation_cone(self, robot, tcp_xyz, original_rv,
                                  max_cone_angle, cone_step, qnear,
                                  margin, check_obstacle):
        """Sample orientations in a cone around the original rotation vector.

        Returns (q, adjusted_tcp) on first success, or None.
        """
        from URBasic import TCP6D

        original_rot = Rotation.from_rotvec(original_rv)

        tilt = cone_step
        while tilt <= max_cone_angle + 1e-9:
            n_azimuth = max(int(np.ceil(2 * math.pi / cone_step)), 1)
            for az_i in range(n_azimuth):
                azimuth = az_i * (2 * math.pi / n_azimuth)

                # Build a small rotation that tilts by `tilt` at `azimuth`
                # in the frame of the original orientation
                axis = np.array([
                    math.sin(azimuth),
                    math.cos(azimuth),
                    0.0,
                ])
                delta_rot = Rotation.from_rotvec(axis * tilt)
                candidate_rot = original_rot * delta_rot
                rv = candidate_rot.as_rotvec()

                candidate_tcp = TCP6D.createFromMetersRadians(
                    float(tcp_xyz[0]), float(tcp_xyz[1]), float(tcp_xyz[2]),
                    float(rv[0]), float(rv[1]), float(rv[2]),
                )

                # Try IK
                q = robot.get_inverse_kin(candidate_tcp, qnear=qnear)
                if q is None:
                    continue

                # Check joint limits
                ok, _ = self.check_joint_limits(q)
                if not ok:
                    continue

                # Check collisions
                q_arr = q.toList() if hasattr(q, 'toList') else list(q)
                self.set_joint_angles(q_arr)
                if self.has_self_collision():
                    continue
                if check_obstacle and self.has_obstacle_collision(margin):
                    continue

                log.debug(
                    "orientation_search: found solution at tilt=%.1f° az=%.1f°",
                    math.degrees(tilt), math.degrees(azimuth),
                )
                return q, candidate_tcp

            tilt += cone_step

        return None

    # -- Layer 2: Path interpolation + validation -----------------------------

    def validate_path(self, robot, waypoints_tcp, step=None,
                      margin: float = 0.005, qnear=None,
                      check_obstacle: bool = True,
                      orientation_search: bool = False):
        """Interpolate between TCP waypoints and validate every sample.

        Uses SLERP for rotation interpolation to handle rotation vector
        wrapping (e.g. ry ≈ ±π).

        Parameters
        ----------
        waypoints_tcp : list of TCP6D — Cartesian waypoints to interpolate.
        step : float — interpolation step in meters (default FREE_TRAVEL_STEP).
        margin : float — collision margin.
        orientation_search : bool — try nearby orientations if IK fails.

        Returns (ok, fail_index, reason, joint_trajectory).
        joint_trajectory is a list of Joint6D if ok, else partial up to failure.
        """
        from URBasic import TCP6D
        from scipy.spatial.transform import Rotation as Rot, Slerp

        if step is None:
            step = FREE_TRAVEL_STEP

        joint_trajectory = []
        sample_idx = 0

        for seg_i in range(len(waypoints_tcp) - 1):
            wp_a = waypoints_tcp[seg_i]
            wp_b = waypoints_tcp[seg_i + 1]

            a_pos = np.array([wp_a.x, wp_a.y, wp_a.z])
            b_pos = np.array([wp_b.x, wp_b.y, wp_b.z])
            a_rv = np.array([wp_a.rx, wp_a.ry, wp_a.rz])
            b_rv = np.array([wp_b.rx, wp_b.ry, wp_b.rz])

            dist = np.linalg.norm(b_pos - a_pos)
            n_samples = max(int(np.ceil(dist / step)), 1)

            rots = Rot.from_rotvec([a_rv, b_rv])
            slerp = Slerp([0, 1], rots)

            for j in range(n_samples + 1):
                t = j / n_samples
                pos = a_pos + t * (b_pos - a_pos)
                rv = slerp(t).as_rotvec()
                tcp = TCP6D.createFromMetersRadians(
                    float(pos[0]), float(pos[1]), float(pos[2]),
                    float(rv[0]), float(rv[1]), float(rv[2]),
                )
                ok, q, reason, _ = self.validate_tcp(
                    robot, tcp, qnear=qnear, margin=margin,
                    check_obstacle=check_obstacle,
                    orientation_search=orientation_search,
                )
                if not ok:
                    return False, sample_idx, reason, joint_trajectory
                joint_trajectory.append(q)
                qnear = q
                sample_idx += 1

        return True, -1, "", joint_trajectory

    def __del__(self):
        try:
            if p.isConnected(self.cid):
                p.disconnect(self.cid)
        except Exception:
            pass
