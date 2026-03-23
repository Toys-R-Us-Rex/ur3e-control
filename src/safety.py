"""
Collision checking utilities for robot motion planning.

Usage
-----
This module provides utilities for checking collisions in the robot's environment.

MIT License

Copyright (c) 2026 Savioz Pierre-Yves

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

Author:     Savioz Pierre-Yves, with assistance from Claude AI (Anthropic)
Comments:   Mariethoz Cédric, with assistance from Copilot AI (Microsoft)
Course:     HES-SO Valais-Wallis, Engineering Track 304
"""


import math
import numpy as np
import pybullet as p
import pybullet_data
from pyb_utils import CollisionDetector
from scipy.spatial.transform import Rotation
from pathlib import Path
from URBasic import TCP6D

from duckify_simulation.duckify_sim.robot_control import SimRobotControl
from src.config import *
from src.kinematics import get_all_ik_solutions


class CollisionChecker:

    def __init__(self, obstacle_stls: list = None, gui: bool = False):
        """
        Initialize the collision checker.

        Parameters
        ----------
        obstacle_stls : list, optional
            List of obstacle STL files to load.
        gui : bool, optional
            Whether to display the simulation GUI.
        """
        self.cid = p.connect(p.GUI if gui else p.DIRECT)

        # Load robot — π rotation around Z to match real-robot frame
        self.robot_id = p.loadURDF(
            str(URDF_PATH),
            basePosition=[0, 0, 0],
            baseOrientation=p.getQuaternionFromEuler([0, 0, np.pi]),
            useFixedBase=True,
            physicsClientId=self.cid,
        )

        # read all joints from URDF
        num_joints = p.getNumJoints(self.robot_id, physicsClientId=self.cid)
        self.joint_indices = []
        for i in range(num_joints):
            info = p.getJointInfo(self.robot_id, i, physicsClientId=self.cid)
            if info[2] in (p.JOINT_REVOLUTE, p.JOINT_PRISMATIC):
                self.joint_indices.append(i)
        self.link_indices = list(range(num_joints))

        # Self-collision detector
        self.self_detector = CollisionDetector(
            self.cid,
            [((self.robot_id, a), (self.robot_id, b)) for a, b in SELF_COLLISION_PAIRS]
        )

        # Load obstacles
        self.obstacle_ids = []
        self.obstacle_exclude_links = {}
        for obs in (obstacle_stls or []):
            col_shape = p.createCollisionShape(
                p.GEOM_MESH,
                fileName=str(obs['path']),
                meshScale=obs.get('scale', [1, 1, 1]),
                physicsClientId=self.cid,
                flags=p.GEOM_FORCE_CONCAVE_TRIMESH
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
            self.obstacle_exclude_links[oid] = set(obs.get('exclude_links', []))

        # Obstacle collision detector
        obs_pairs = []
        for oid in self.obstacle_ids:
            excluded = self.obstacle_exclude_links.get(oid, set())
            for link in self.link_indices:
                if link not in excluded:
                    obs_pairs.append(((self.robot_id, link), (oid, -1)))

        if obs_pairs:
            self.obstacle_detector = CollisionDetector(self.cid, obs_pairs)
        else:
            self.obstacle_detector = None

    # -------------------------------------------------------------------------

    # moves instantly the robot to target joint angles position
    def set_joint_angles(self, q: list):
        """
        Set the joint angles of the robot.

        Parameters
        ----------
        q : list
            The joint angles to set.
        """
        for idx, angle in zip(self.joint_indices, q):
            p.resetJointState(self.robot_id, idx, float(angle), physicsClientId=self.cid)


    def in_self_collision(self, margin: float = SELF_COLLISION_MARGIN) -> bool:
        """
        Check if the robot is in self-collision.

        Parameters
        ----------
        margin : float, optional
            The collision margin.

        Returns
        -------
        bool
            True if the robot is in self-collision, False otherwise.
        """
        return self.self_detector.in_collision(margin=margin)

    def in_obstacle_collision(self, margin: float = COLLISION_MARGIN) -> bool:
        """
        Check if the robot is in obstacle collision.

        Parameters
        ----------
        margin : float, optional
            The collision margin.

        Returns
        -------
        bool
            True if the robot is in obstacle collision, False otherwise.
        """
        if self.obstacle_detector is None:
            return False
        return self.obstacle_detector.in_collision(margin=margin)

    def check_workspace_bounds(self, tcp: TCP6D) -> tuple[bool, str]:
        """
        Check if the TCP is within workspace bounds.

        Parameters
        ----------
        tcp : TCP6D
            The TCP pose to check.

        Returns
        -------
        tuple[bool, str]
            A tuple indicating if the TCP is within bounds and a reason string.
        """
        x, y, z = tcp.x, tcp.y, tcp.z
        if y > TCP_Y_MAX:
            return False, f"TCP Y={y:.4f} > {TCP_Y_MAX}"
        if z < TCP_Z_MIN:
            return False, f"TCP Z={z:.4f} < {TCP_Z_MIN}"
        if z > TCP_Z_MAX:
            return False, f"TCP Z={z:.4f} > {TCP_Z_MAX}"
        if UR3E_MAX_REACH and math.sqrt(x*x + y*y + z*z) > UR3E_MAX_REACH:
            return False, f"TCP reach > {UR3E_MAX_REACH}"
        return True, ""

    # not being used at the mment
    def check_joint_limits(self, q: list) -> tuple[bool, str]:
        """
        Check if the joint angles are within limits.

        Parameters
        ----------
        q : list
            The joint angles to check.

        Returns
        -------
        tuple[bool, str]
            A tuple indicating if the joint angles are within limits and a reason string.
        """
        q_list = q.toList() if hasattr(q, 'toList') else list(q)
        for i, (angle, limit) in enumerate(zip(q_list, JOINT_LIMITS)):
            if limit is None:
                continue
            lo, hi = limit
            if not (lo <= angle <= hi):
                return False, f"Joint {i} = {angle:.4f} outside [{lo}, {hi}]"
        return True, ""

    # -------------------------------------------------------------------------

    # Entry point of the pathfinding pipeline , checks safety, IK and cone orientation if needed
    # returns a new tcp valid if one has been found
    def validate_tcp(self, robot: SimRobotControl, tcp: TCP6D, qnear=None, margin=COLLISION_MARGIN,
                     check_obstacle: bool=True, orientation_search: bool=False,
                     max_cone_angle: float=math.radians(DRAWING_ANGLE), cone_step: float=math.radians(5)) -> tuple[bool, list, str, TCP6D]:
        """
        Validate the TCP pose for safety.

        Parameters
        ----------
        robot : SimRobotControl
            The robot control instance.
        tcp : TCP6D
            The TCP pose to validate.
        qnear : list, optional
            The nearest joint angles.
        margin : float, optional
            The collision margin.
        check_obstacle : bool, optional
            Whether to check obstacle collisions.
        orientation_search : bool, optional
            Whether to perform orientation search.
        max_cone_angle : float, optional
            The maximum cone angle for orientation search.
        cone_step : float, optional
            The step size for orientation search.

        Returns
        -------
        tuple[bool, list, str, TCP6D]
            A tuple indicating if the TCP is valid, the joint angles, a reason string, and the adjusted TCP pose.
        """
        ok, reason = self.check_workspace_bounds(tcp)
        if not ok:
            return False, None, reason, tcp

        ok, q, reason = self._try_ik_and_collision(robot, tcp, qnear, margin, check_obstacle)
        if ok:
            return True, q, "", tcp

        if orientation_search:
            result = self._cone_search(robot, tcp, qnear, margin, check_obstacle, max_cone_angle, cone_step)
            if result:
                q, adjusted_tcp = result
                return True, q, "", adjusted_tcp

        return False, None, reason, tcp


    # Reurns all IK solution, sort them by qnear, then returns the first safe waypoint
    def _try_ik_and_collision(self, robot: SimRobotControl, tcp: TCP6D, qnear: list, margin: float, check_obstacle: bool) -> tuple[bool, list, str]:
        """
        Try to find a valid IK solution and check for collisions.

        Parameters
        ----------
        robot : SimRobotControl
            The robot control instance.
        tcp : TCP6D
            The TCP pose to validate.
        qnear : list
            The nearest joint angles.
        margin : float
            The collision margin.
        check_obstacle : bool
            Whether to check obstacle collisions.

        Returns
        -------
        tuple[bool, list, str]
            A tuple indicating if a valid solution is found, the joint angles, and a reason string.
        """
        candidates = get_all_ik_solutions(tcp, robot._tcp_offset, robot._model_correction)
        if not candidates:
            return False, None, "IK has no solution"

        # Sort by joint-space distance to qnear so we prefer the closest config
        if qnear is not None:
            qnear_arr = np.array(qnear.toList())
            distances = []
            for c in candidates:
                diff = np.array(c.toList()) - qnear_arr
                distances.append(np.sum(diff ** 2))
            candidates = [c for _, c in sorted(zip(distances, candidates))]

        first_reason = None
        for q in candidates:
            ok, reason = self.check_joint_limits(q)
            if not ok:
                if first_reason is None:
                    first_reason = reason
                continue

            self.set_joint_angles(q.toList())

            # Per-link Z minimum
            link_ok = True
            for link_idx, z_min in (LINK_Z_MIN or {}).items():
                link_z = p.getLinkState(self.robot_id, link_idx, physicsClientId=self.cid)[0][2]
                if link_z < z_min:
                    info = p.getJointInfo(self.robot_id, link_idx, physicsClientId=self.cid)
                    if first_reason is None:
                        first_reason = f"Link {info[12].decode()} Z={link_z:.4f} < {z_min}"
                    link_ok = False
                    break
            if not link_ok:
                continue

            if self.in_self_collision():
                if first_reason is None:
                    first_reason = "Self-collision"
                continue

            if check_obstacle and self.in_obstacle_collision(margin):
                if first_reason is None:
                    first_reason = "Obstacle collision"
                continue

            return True, q, ""

        return False, None, first_reason or "IK has no solution"

    # Changes slightly the angle on which to search for analytical IK within a cone of angle [max_cone_angle]
    def _cone_search(self, robot: SimRobotControl, tcp: TCP6D, qnear: list, margin: float, check_obstacle: bool, max_cone_angle: float, cone_step: float) -> tuple[list, TCP6D]:
        """
        Search for a valid IK solution within a cone of orientations.

        Parameters
        ----------
        robot : SimRobotControl
            The robot control instance.
        tcp : TCP6D
            The TCP pose to validate.
        qnear : list
            The nearest joint angles.
        margin : float
            The collision margin.
        check_obstacle : bool
            Whether to check obstacle collisions.
        max_cone_angle : float
            The maximum cone angle for orientation search.
        cone_step : float
            The step size for orientation search.

        Returns
        -------
        tuple[list, TCP6D]
            A tuple containing the joint angles and the adjusted TCP pose.
        """
        tcp_xyz = np.array([tcp.x, tcp.y, tcp.z])
        original_rot = Rotation.from_rotvec([tcp.rx, tcp.ry, tcp.rz])

        tilt = cone_step
        while tilt <= max_cone_angle + 1e-9:
            n_azimuth = max(int(np.ceil(2 * math.pi / cone_step)), 1)
            for az_i in range(n_azimuth):
                azimuth = az_i * (2 * math.pi / n_azimuth)
                axis = np.array([math.sin(azimuth), math.cos(azimuth), 0.0])
                rv = (original_rot * Rotation.from_rotvec(axis * tilt)).as_rotvec()
                candidate_tcp = TCP6D.createFromMetersRadians(
                    *tcp_xyz.tolist(), float(rv[0]), float(rv[1]), float(rv[2])
                )
                ok, q, _ = self._try_ik_and_collision(robot, candidate_tcp, qnear, margin, check_obstacle)
                if ok:
                    return q, candidate_tcp
            tilt += cone_step
        return None

    # -------------------------------------------------------------------------


    def validate_path(self, robot: SimRobotControl, waypoints_tcp: list[TCP6D], margin: float = COLLISION_MARGIN,
                      qnear: list = None, check_obstacle: bool = True, orientation_search: bool = False) -> tuple[bool, int, str, list]:
        """
        Validate a path of TCP waypoints for collision avoidance.

        Parameters
        ----------
        robot : SimRobotControl
            The robot control instance.
        waypoints_tcp : list[TCP6D]
            The list of TCP poses to validate.
        margin : float
            The collision margin.
        qnear : list
            The nearest joint angles.
        check_obstacle : bool
            Whether to check obstacle collisions.
        orientation_search : bool
            Whether to perform orientation search.

        Returns
        -------
        tuple[bool, int, str, list]
            A tuple indicating if the path is valid, the index of the first invalid waypoint, a reason string, and the joint trajectory.
        """
        joint_trajectory = []

        for i, tcp in enumerate(waypoints_tcp):
            ok, q, reason, _ = self.validate_tcp(
                robot, tcp, qnear=qnear, margin=margin,
                check_obstacle=check_obstacle,
                orientation_search=orientation_search,
            )
            if not ok:
                return False, i, reason, joint_trajectory
            joint_trajectory.append(q)
            qnear = q

        return True, -1, "", joint_trajectory



def setup_checker(obstacle_stls: list, gui: bool = True) -> CollisionChecker:
    """
    Set up the collision checker with the given obstacle STL files.

    Parameters
    ----------
    obstacle_stls : list
        A list of paths to the obstacle STL files.
    gui : bool, optional
        Whether to display the PyBullet GUI, by default True.

    Returns
    -------
    CollisionChecker
        The initialized collision checker.
    """
    checker = CollisionChecker(obstacle_stls=obstacle_stls, gui=gui)

    print(f"PyBullet GUI running (cid={checker.cid})")
    print(f"Robot body id: {checker.robot_id}")
    print(f"Obstacle ids:  {checker.obstacle_ids}")

    for oid in checker.obstacle_ids:
        pos_ob, orn = p.getBasePositionAndOrientation(oid, physicsClientId=checker.cid)
        aabb_min, aabb_max = p.getAABB(oid, physicsClientId=checker.cid)
        print(f"\nObstacle {oid}:")
        print(f"  Position:    {pos_ob}")
        print(f"  Orientation: {orn}")
        print(f"  AABB min:    {[f'{v:.4f}' for v in aabb_min]}")
        print(f"  AABB max:    {[f'{v:.4f}' for v in aabb_max]}")
        size = (aabb_max[0]-aabb_min[0], aabb_max[1]-aabb_min[1], aabb_max[2]-aabb_min[2])
        print(f"  Size (m):    ({size[0]:.4f}, {size[1]:.4f}, {size[2]:.4f})")

    return checker