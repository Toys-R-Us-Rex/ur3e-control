"""SimRobotControl — mirrors the robot_control interface from ISCoin.

MIT License

Copyright (c) 2026 Pierre-Yves Savioz

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

Author:     Pierre-Yves Savioz, with assistance from Claude AI (Anthropic)
Course:     HES-SO Valais-Wallis, Engineering Track 304
URBasic:    API modelled on UrScriptExt by Martin Huus Bjerge,
            Rope Robotics ApS, Denmark (MIT License, 2017),
            modified by A. Amand, M. Richard, L. Azzalini (HES-SO).
            Uses data types from URBasic.waypoint6d (Joint6D, TCP6D).
"""

import logging
import math
import time
import numpy as np

from URBasic.waypoint6d import Joint6D, TCP6D, TCP6DDescriptor

log = logging.getLogger(__name__)

from .ros_bridge import read_joint_states, extract_6joints, publish_trajectory, estimate_duration
from .kinematics import (
    UR3E_DH, forward_kinematics, forward_kinematics_matrix,
    matrix_to_tcp6d, pose_to_matrix,
    analytical_ik, select_closest_ik,
)

T_DELAY = 0.1

T_DELAY = 0.1


class SimRobotControl:
    """Mirrors the robot_control interface from ISCoin (UrScriptExt)."""


    def __init__(self):
        self._tcp_offset = np.eye(4)  # no tool by default (identity)
        self._model_correction = np.eye(4)

    def set_tcp(self, pose):
        """Set the Tool Center Point offset (e.g. for a pen in the gripper).

        Args:
            pose: TCP6D offset from the flange to the tool tip.
                  For a pen of length L along Z: TCP6D.createFromMetersRadians(0, 0, L, 0, 0, 0)
        """
        self._tcp_offset = pose_to_matrix(pose)

    def set_model_correction(self, pose):
        """Correction for FK model inaccuracy (nominal DH vs real robot).

        Applied between FK flange and TCP offset:
            T_tcp = FK(joints) @ _model_correction @ _tcp_offset

        Note: a constant correction is only accurate near the calibration pose.

        Args:
            pose: TCP6D correction (typically a Z translation).
        """
        self._model_correction = pose_to_matrix(pose)

    # -- Read state ----------------------------------------------------------

    def get_actual_joint_positions(self, wait=True):
        """Read the current 6 joint angles from the simulator.

        Returns:
            Joint6D (same as the real ISCoin).
        """
        states = read_joint_states()
        values = extract_6joints(states["position"])
        return Joint6D.createFromRadians(*values)

    def get_actual_tcp_pose(self, wait=True):
        """Compute the current TCP pose using forward kinematics.

        Returns:
            TCP6D with [x, y, z, rx, ry, rz] (includes tool offset if set).
        """
        joints = self.get_actual_joint_positions(wait=wait)
        T_flange = forward_kinematics_matrix(joints.toList())
        T_tcp = T_flange @ self._model_correction @ self._tcp_offset
        return matrix_to_tcp6d(T_tcp)

    def get_fk(self, joints):
        """Compute the TCP pose for the given joint configuration.

        Args:
            joints: Joint6D with joint angles in radians.

        Returns:
            TCP6D with [x, y, z, rx, ry, rz] (includes tool offset if set).
        """
        T_flange = forward_kinematics_matrix(joints.toList())
        T_tcp = T_flange @ self._model_correction @ self._tcp_offset
        return matrix_to_tcp6d(T_tcp)

    def is_steady(self):
        """Check if the robot is not moving (all joint speeds near zero)."""
        states = read_joint_states()
        speeds = extract_6joints(states["velocity"])
        return all(abs(s) <= 0.01 for s in speeds)

    # -- Movement commands ---------------------------------------------------

    def movej(self, joints, a=1.4, v=1.05, t=0, r=0, wait=True):
        """Move the robot to a joint position (joint-space motion).

        Args:
            joints: A Joint6D target.
            a: acceleration (used to estimate duration if t=0).
            v: velocity (used to estimate duration if t=0).
            t: time in seconds. If > 0, used directly as duration.
            r: blend radius (not used in sim).
            wait: if True, waits for the movement to finish.

        Returns:
            True if the target was reached, False otherwise.
        """
        positions = joints.toList()

        if t > 0:
            duration_sec = t
        else:
            duration_sec = estimate_duration(v)

        publish_trajectory([{
            "positions": positions,
            "duration_sec": duration_sec,
        }])
        print(f"movej sent (duration={duration_sec}s)")

        if wait:
            return self._wait_until_motion_done(
                target_joints=joints,
                timeout_sec=max(5.0, float(duration_sec) + 10.0),
            )

        return True

    def movej_waypoints(self, waypoints, wait=True):
        """Move through multiple joint positions sequentially.

        Args:
            waypoints: list of Joint6DDescriptor (from the teacher's library).
            wait: if True, waits for all movements to finish.

        Returns:
            True if the final target was reached, False otherwise.
        """
        points = []
        cumulative_sec = 0

        for wp in waypoints:
            wp_dict = wp.getAsDict()
            positions = wp_dict["q"]
            v = wp_dict["v"]
            t_param = wp_dict["t"]

            if t_param > 0:
                duration = t_param
            else:
                duration = estimate_duration(v)

            cumulative_sec += duration
            points.append({
                "positions": positions,
                "duration_sec": cumulative_sec,
            })

        publish_trajectory(points)
        print(f"movej_waypoints sent ({len(points)} points, total={cumulative_sec}s)")

        if wait:
            final_target = Joint6D.createFromRadians(*points[-1]["positions"])
            return self._wait_until_motion_done(
                target_joints=final_target,
                timeout_sec=max(5.0, float(cumulative_sec) + 10.0),
            )

        return True

    def movel(self, pose, a=1.2, v=0.25, t=0, r=0, wait=True):
        """Move the robot in a straight line to a Cartesian pose.

        Computes inverse kinematics and then does a movej.

        Args:
            pose: A TCP6D target [x, y, z, rx, ry, rz].
            a: acceleration (not used directly).
            v: velocity (used to estimate duration).
            t: time in seconds.
            r: blend radius (not used in sim).
            wait: if True, waits for the movement to finish.

        Returns:
            True if the target was reached, False otherwise.
        """
        current_joints = self.get_actual_joint_positions()
        target_joints = self.get_inverse_kin(pose, qnear=current_joints)

        if target_joints is None:
            print("ERROR: movel failed — could not find inverse kinematics solution")
            return False

        if t > 0:
            duration = t
        else:
            current_tcp = self.get_actual_tcp_pose()
            dx = pose.x - current_tcp.x
            dy = pose.y - current_tcp.y
            dz = pose.z - current_tcp.z
            dist = math.sqrt(dx*dx + dy*dy + dz*dz)
            duration = max(0.3, dist / v)

        return self.movej(target_joints, t=duration, wait=wait)

    def movel_waypoints(self, waypoints, wait=True):
        """Move in a straight line through multiple TCP waypoints continuously.

        Args:
            waypoints: list of TCP6DDescriptor (from the teacher's library).
            wait: if True, waits for all movements to finish.

        Returns:
            True if the final target was reached, False otherwise.
        """
        if not isinstance(waypoints, list):
            raise ValueError("waypoints must be a list of TCP6DDescriptor objects")
        for w in waypoints:
            if not isinstance(w, TCP6DDescriptor):
                raise ValueError(f"waypoints must be a list of TCP6DDescriptor objects — got {type(w)}")

        points = []
        cumulative_sec = 0
        prev_joints = self.get_actual_joint_positions()
        prev_tcp = self.get_actual_tcp_pose()

        for wp in waypoints:
            wp_dict = wp.getAsDict()
            pose = TCP6D.createFromMetersRadians(*wp_dict["pose"])
            v = wp_dict["v"]
            t_param = wp_dict["t"]

            target_joints = self.get_inverse_kin(pose, qnear=prev_joints)
            if target_joints is None:
                print(f"ERROR: movel_waypoints — IK failed for {pose}")
                return False

            if t_param > 0:
                duration = t_param
            else:
                dx = pose.x - prev_tcp.x
                dy = pose.y - prev_tcp.y
                dz = pose.z - prev_tcp.z
                dist = math.sqrt(dx*dx + dy*dy + dz*dz)
                duration = max(0.3, dist / v)

            cumulative_sec += duration
            points.append({
                "positions": target_joints.toList(),
                "duration_sec": cumulative_sec,
            })

            prev_joints = target_joints
            prev_tcp = pose

        publish_trajectory(points)
        print(f"movel_waypoints sent ({len(points)} points, total={cumulative_sec}s)")

        if wait:
            final_target = Joint6D.createFromRadians(*points[-1]["positions"])
            return self._wait_until_motion_done(
                target_joints=final_target,
                timeout_sec=max(5.0, float(cumulative_sec) + 10.0),
            )

        return True

    def stopj(self, a=2.0, wait=True):
        """Stop the robot by sending the current position as target.

        Args:
            a: deceleration (not used in sim).
            wait: if True, waits briefly for the robot to stop.
        """
        current = self.get_actual_joint_positions()
        publish_trajectory([{
            "positions": current.toList(),
            "duration_sec": 0,
        }])
        print("stopj sent")

        if wait:
            time.sleep(0.5)

    def get_inverse_kin(self, pose, qnear=None):
        """Inverse kinematics using analytical closed-form solution for UR3e.

        Computes up to 8 exact solutions by exploiting the spherical wrist
        geometry, then picks the one closest to qnear.

        Args:
            pose: A TCP6D target.
            qnear: A Joint6D hint for selecting among multiple solutions
                   (optional, uses current position).

        Returns:
            Joint6D with the solution, or None if no solution found.
        """
        if qnear is not None:
            q0 = np.array(qnear.toList())
        else:
            q0 = np.zeros(6)

        T_desired = pose_to_matrix(pose)
        T_flange = T_desired @ np.linalg.inv(self._model_correction @ self._tcp_offset)
        solutions = analytical_ik(T_flange)

        if solutions:
            # Validate each solution with FK (applying tool offset) and keep only accurate ones
            valid = []
            target = np.array(pose.toList())
            for sol in solutions:
                T_check = forward_kinematics_matrix(sol.tolist()) @ self._model_correction @ self._tcp_offset
                tcp_check = matrix_to_tcp6d(T_check)
                err_pos = np.sum((np.array(tcp_check.toList()[:3]) - target[:3]) ** 2)
                if err_pos < 0.001:
                    valid.append(sol)

            best = select_closest_ik(valid if valid else solutions, q0)
            if best is not None:
                T_check = forward_kinematics_matrix(best.tolist()) @ self._model_correction @ self._tcp_offset
                tcp_check = matrix_to_tcp6d(T_check)
                err = np.sum((np.array(tcp_check.toList()[:3]) - np.array(pose.toList()[:3])) ** 2)
                if err < 0.001:
                    log.debug("IK OK for TCP=(%.4f, %.4f, %.4f), "
                              "%d/%d valid solutions",
                              pose.x, pose.y, pose.z, len(valid), len(solutions))
                    return Joint6D.createFromRadians(*best.tolist())
                log.debug("IK best solution has position error=%.6fm", np.sqrt(err))

        # Reachability diagnostic
        pos = np.array([pose.x, pose.y, pose.z])
        reach = np.linalg.norm(pos)
        n_raw = len(solutions) if solutions else 0
        flange_pos = T_flange[:3, 3]
        flange_reach = np.linalg.norm(flange_pos[:2])
        log.debug("IK failed for TCP=(%.4f, %.4f, %.4f), reach=%.4fm | "
                  "%d raw solutions | flange_r=%.4fm",
                  pose.x, pose.y, pose.z, reach, n_raw, flange_reach)

        return None

    def get_all_ik_solutions(self, pose):
        """Return all valid IK solutions for a TCP6D pose.

        Solutions are sorted by joint-space distance to the current
        robot configuration (closest first). All angles are wrapped to [-π, π].

        Returns:
            List of Joint6D solutions (empty if none found).
        """
        T_desired = pose_to_matrix(pose)
        T_flange = T_desired @ np.linalg.inv(self._model_correction @ self._tcp_offset)
        solutions = analytical_ik(T_flange)

        valid = []
        target = np.array(pose.toList())
        for sol in solutions:
            T_check = forward_kinematics_matrix(sol.tolist()) @ self._model_correction @ self._tcp_offset
            tcp_check = matrix_to_tcp6d(T_check)
            err_pos = np.sum((np.array(tcp_check.toList()[:3]) - target[:3]) ** 2)
            if err_pos < 0.001:
                valid.append(Joint6D.createFromRadians(*sol.tolist()))

        # Sort by distance to current configuration
        q_current = np.array(self.get_actual_joint_positions().toList())
        valid.sort(key=lambda j: np.sum((np.array(j.toList()) - q_current) ** 2))

        return valid

    # -- Internal helpers ----------------------------------------------------

    def _wait_until_motion_done(self, target_joints, timeout_sec=15.0, tolerance=0.05, poll_sec=0.1):
        """Block until robot is steady and close to target, or timeout."""
        deadline = time.time() + timeout_sec
        time.sleep(T_DELAY)

        while time.time() < deadline:
            if self.is_steady() and self._is_within_tolerance(target_joints, tolerance=tolerance):
                print("Movement OK — target reached")
                return True
            time.sleep(poll_sec)

        print(f"WARNING: Motion timeout after {timeout_sec:.1f}s")
        return self._verify_position(target_joints, tolerance=tolerance)

    def _is_within_tolerance(self, target_joints, tolerance=0.05):
        """Check target proximity without printing warnings."""
        actual = self.get_actual_joint_positions()
        target_list = target_joints.toList()
        actual_list = actual.toList()
        return all(abs(target_list[i] - actual_list[i]) <= tolerance for i in range(6))

    def _verify_position(self, target_joints, tolerance=0.05):
        """Check if the robot reached the target position.

        Args:
            target_joints: Joint6D with expected position.
            tolerance: max allowed error per joint in radians.

        Returns:
            True if all joints are within tolerance.
        """
        actual = self.get_actual_joint_positions()
        target_list = target_joints.toList()
        actual_list = actual.toList()

        all_ok = True
        for i in range(6):
            error = abs(target_list[i] - actual_list[i])
            if error > tolerance:
                print(f"WARNING: Joint {i+1} error = {math.degrees(error):.1f}° "
                      f"(target={target_list[i]:.4f}, actual={actual_list[i]:.4f})")
                all_ok = False

        if all_ok:
            print("Movement OK — target reached")
        else:
            print("Movement DONE — but target not fully reached (possible collision?)")

        return all_ok



    def get_tcp_force(self, wait=True) -> TCP6D:
        '''
        Returns the wrench (Force/Torque vector) at the TCP

        The external wrench is computed based on the error between the joint
        torques required to stay on the trajectory and the expected joint
        torques. The function returns "p[Fx (N), Fy(N), Fz(N), TRx (Nm), TRy (Nm),
        TRz (Nm)]". where Fx, Fy, and Fz are the forces in the axes of the robot
        base coordinate system measured in Newtons, and TRx, TRy, and TRz
        are the torques around these axes measured in Newton times Meters.
        
        Parameters:
        wait:     function return when movement is finished

        Return Value:
        the wrench (pose)
        '''
        if wait:
            time.sleep(0.5)
        return [1,2,3,0.1,0.2,0.3]
    
    def force(self, wait=True) -> float:
        '''
        Returns the force exerted at the TCP

        Return the current externally exerted force at the TCP. The force is the
        norm of Fx, Fy, and Fz calculated using get tcp force().
        
        Parameters:
        wait:     function return when movement is finished
        
        Return Value
        The force in Newtons (float) -> pi/2
        '''
        if wait:
            time.sleep(0.5)
        return np.pi/2
