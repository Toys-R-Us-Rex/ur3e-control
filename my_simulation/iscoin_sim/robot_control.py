"""SimRobotControl — mirrors the robot_control interface from ISCoin."""

import math
import time
import numpy as np

from URBasic.waypoint6d import Joint6D, TCP6D, TCP6DDescriptor

from .ros_bridge import read_joint_states, extract_6joints, publish_trajectory, estimate_duration
from .kinematics import (
    UR3E_DH, forward_kinematics, pose_to_matrix,
    analytical_ik, select_closest_ik,
)


class SimRobotControl:
    """Mirrors the robot_control interface from ISCoin (UrScriptExt)."""

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
            TCP6D with [x, y, z, rx, ry, rz].
        """
        joints = self.get_actual_joint_positions(wait=wait)
        return forward_kinematics(joints.toList())

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
            duration_sec = int(math.ceil(t))
        else:
            duration_sec = estimate_duration(v)

        publish_trajectory([{
            "positions": positions,
            "duration_sec": duration_sec,
        }])
        print(f"movej sent (duration={duration_sec}s)")

        if wait:
            time.sleep(duration_sec + 1)
            return self._verify_position(joints)

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
                duration = int(math.ceil(t_param))
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
            time.sleep(cumulative_sec + 1)
            final_target = Joint6D.createFromRadians(*points[-1]["positions"])
            return self._verify_position(final_target)

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
            duration = max(2, int(math.ceil(dist / v)))

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
                duration = max(2, int(math.ceil(dist / v)))

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
            time.sleep(cumulative_sec + 1)
            final_target = Joint6D.createFromRadians(*points[-1]["positions"])
            return self._verify_position(final_target)

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
            q0 = np.array(self.get_actual_joint_positions().toList())

        T_desired = pose_to_matrix(pose)
        solutions = analytical_ik(T_desired)

        if solutions:
            # Validate each solution with FK and keep only accurate ones
            valid = []
            target = np.array(pose.toList())
            for sol in solutions:
                tcp_check = forward_kinematics(sol.tolist())
                err_pos = np.sum((np.array(tcp_check.toList()[:3]) - target[:3]) ** 2)
                if err_pos < 0.001:
                    valid.append(sol)

            best = select_closest_ik(valid if valid else solutions, q0)
            if best is not None:
                tcp_check = forward_kinematics(best.tolist())
                err = np.sum((np.array(tcp_check.toList()[:3]) - np.array(pose.toList()[:3])) ** 2)
                if err < 0.001:
                    return Joint6D.createFromRadians(*best.tolist())
                print(f"WARNING: Analytical IK best solution has position error={np.sqrt(err):.6f}m")

        # Reachability diagnostic
        pos = np.array([pose.x, pose.y, pose.z])
        reach = np.linalg.norm(pos)
        max_reach = abs(UR3E_DH[1]["a"]) + abs(UR3E_DH[2]["a"])
        print(f"INFO: Analytical IK returned 0 valid solutions for target at "
              f"distance={reach:.4f}m (max reach ~{max_reach:.3f}m)")
        if reach > max_reach:
            print(f"WARNING: Target is likely outside the UR3e workspace "
                  f"({reach:.4f}m > {max_reach:.3f}m)")

        return None

    # -- Internal helpers ----------------------------------------------------

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
