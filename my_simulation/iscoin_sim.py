"""
ISCoinSim — A drop-in replacement for ISCoin that talks to the Gazebo simulator.

Usage:
    # Instead of:  from URBasic import ISCoin
    # Use:         from my_simulation import ISCoinSim as ISCoin

    iscoin = ISCoinSim()
    print(iscoin.robot_control.get_actual_joint_positions())
    iscoin.robot_control.movej(Joint6D.createFromRadians(1.19, -1.13, 1.05, -1.60, -1.52, 1.05))

How it works:
    This wrapper runs on your host machine and uses 'docker exec' to send
    ROS2 commands into the running iscoin_simulator container.
    No ROS2 installation needed on your machine.
"""

import subprocess
import time
import math
import numpy as np
from numpy import sin, cos, arctan2, arccos, sqrt, pi

# Reuse the teacher's data classes so return types are identical
from URBasic.waypoint6d import Joint6D, TCP6D, Joint6DDescriptor, TCP6DDescriptor

# Container name (must match docker-compose)
CONTAINER_NAME = "iscoin_simulator"

# Gripper constants
GRIPPER_JOINT_NAMES = [
    "robotiq_hande_left_finger_joint",
    "robotiq_hande_right_finger_joint",
]
GRIPPER_MAX_JOINT = 0.025  # meters, fully open
GRIPPER_CONTROLLER_NAMES = ["hande_controller_left", "hande_controller_right"]

# Joint names in the order the UR3e expects them
JOINT_NAMES = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]

# UR3e DH parameters (standard values from Universal Robots)
# [a, d, alpha] for each joint
UR3E_DH = [
    {"a": 0,        "d": 0.15185,  "alpha": math.pi / 2},   # joint 1
    {"a": -0.24355, "d": 0,        "alpha": 0},              # joint 2
    {"a": -0.2132,  "d": 0,        "alpha": 0},              # joint 3
    {"a": 0,        "d": 0.13105,  "alpha": math.pi / 2},    # joint 4
    {"a": 0,        "d": 0.08535,  "alpha": -math.pi / 2},   # joint 5
    {"a": 0,        "d": 0.0921,   "alpha": 0},              # joint 6
]


# ---------------------------------------------------------------------------
# Helper functions
# ---------------------------------------------------------------------------

def _docker_exec(cmd, timeout=15):
    """Run a bash command inside the simulator container."""
    result = subprocess.run(
        ["docker", "exec", CONTAINER_NAME, "bash", "-c", cmd],
        capture_output=True,
        text=True,
        timeout=timeout,
    )
    if result.returncode != 0:
        raise RuntimeError(f"Docker exec failed: {result.stderr.strip()}")
    return result.stdout


def _parse_joint_states(raw_output):
    """Parse the YAML output of 'ros2 topic echo /joint_states --once'.

    Returns a dict with 'name', 'position', 'velocity' lists.
    """
    names = []
    positions = []
    velocities = []
    current_section = None

    for line in raw_output.splitlines():
        stripped = line.strip()

        if stripped == "name:":
            current_section = "name"
            continue
        elif stripped == "position:":
            current_section = "position"
            continue
        elif stripped == "velocity:":
            current_section = "velocity"
            continue
        elif stripped.endswith(":") and not stripped.startswith("- "):
            current_section = None
            continue

        if stripped.startswith("- ") and current_section == "name":
            names.append(stripped[2:])
        elif stripped.startswith("- ") and current_section == "position":
            positions.append(float(stripped[2:]))
        elif stripped.startswith("- ") and current_section == "velocity":
            velocities.append(float(stripped[2:]))

    return {
        "name": names,
        "position": dict(zip(names, positions)),
        "velocity": dict(zip(names, velocities)),
    }


def _read_joint_states():
    """Read /joint_states from the simulator once. Returns parsed dict."""
    raw = _docker_exec(
        "source /opt/ros/humble/setup.bash && "
        "ros2 topic echo /joint_states --once"
    )
    return _parse_joint_states(raw)


def _extract_6joints(joint_dict):
    """Extract our 6 joint values in the correct order from a name->value dict."""
    values = []
    for name in JOINT_NAMES:
        if name not in joint_dict:
            raise RuntimeError(f"Joint '{name}' not found in /joint_states")
        values.append(joint_dict[name])
    return values


def _publish_trajectory(points, timeout=15):
    """Publish a trajectory message to the joint_trajectory_controller topic.

    Args:
        points: list of dicts, each with 'positions' (list of 6 floats)
                and 'duration_sec' (int).
    """
    # Build the points string for the ROS2 message
    points_str_parts = []
    for pt in points:
        pos_str = ", ".join(str(p) for p in pt["positions"])
        sec = pt["duration_sec"]
        points_str_parts.append(
            f"{{positions: [{pos_str}], "
            f"time_from_start: {{sec: {sec}, nanosec: 0}}}}"
        )
    points_str = ", ".join(points_str_parts)

    cmd = (
        "source /opt/ros/humble/setup.bash && "
        "ros2 topic pub --once "
        "/joint_trajectory_controller/joint_trajectory "
        "trajectory_msgs/msg/JointTrajectory "
        '"{joint_names: '
        "[shoulder_pan_joint, shoulder_lift_joint, elbow_joint, "
        "wrist_1_joint, wrist_2_joint, wrist_3_joint], "
        f'points: [{points_str}]}}"'
    )

    _docker_exec(cmd, timeout=timeout)


def _estimate_duration(v):
    """Estimate movement duration from velocity parameter."""
    return max(2, int(math.ceil(math.pi / v)))


def _forward_kinematics(joint_angles):
    """Compute the TCP pose from joint angles using UR3e DH parameters.

    Args:
        joint_angles: list of 6 joint angles in radians.

    Returns:
        TCP6D with [x, y, z, rx, ry, rz] (position in meters, rotation as axis-angle).
    """
    # Start with identity matrix
    T = np.eye(4)

    for i in range(6):
        theta = joint_angles[i]
        a = UR3E_DH[i]["a"]
        d = UR3E_DH[i]["d"]
        alpha = UR3E_DH[i]["alpha"]

        # Standard DH transformation matrix
        ct = math.cos(theta)
        st = math.sin(theta)
        ca = math.cos(alpha)
        sa = math.sin(alpha)

        Ti = np.array([
            [ct, -st * ca,  st * sa, a * ct],
            [st,  ct * ca, -ct * sa, a * st],
            [0,   sa,       ca,      d],
            [0,   0,        0,       1],
        ])

        T = T @ Ti

    # Extract position
    x, y, z = T[0, 3], T[1, 3], T[2, 3]

    # Extract rotation matrix and convert to axis-angle
    R = T[:3, :3]
    angle = math.acos(max(-1, min(1, (np.trace(R) - 1) / 2)))

    if abs(angle) < 1e-6:
        rx, ry, rz = 0.0, 0.0, 0.0
    elif abs(angle - math.pi) < 1e-6:
        # Special case: angle near pi
        rx = math.pi * math.sqrt((R[0, 0] + 1) / 2)
        ry = math.pi * math.sqrt((R[1, 1] + 1) / 2)
        rz = math.pi * math.sqrt((R[2, 2] + 1) / 2)
    else:
        # General case
        k = angle / (2 * math.sin(angle))
        rx = k * (R[2, 1] - R[1, 2])
        ry = k * (R[0, 2] - R[2, 0])
        rz = k * (R[1, 0] - R[0, 1])

    return TCP6D.createFromMetersRadians(x, y, z, rx, ry, rz)


def _dh_matrix(theta, a, d, alpha):
    """Build a single standard DH transformation matrix."""
    ct, st = cos(theta), sin(theta)
    ca, sa = cos(alpha), sin(alpha)
    return np.array([
        [ct, -st * ca,  st * sa, a * ct],
        [st,  ct * ca, -ct * sa, a * st],
        [0,   sa,       ca,      d     ],
        [0,   0,        0,       1     ],
    ])


def _pose_to_matrix(pose):
    """Convert a TCP6D (position + axis-angle) to a 4x4 homogeneous matrix.

    The axis-angle vector [rx, ry, rz] encodes both the axis (direction)
    and the angle (magnitude) of rotation.
    """
    x, y, z = pose.x, pose.y, pose.z
    rx, ry, rz = pose.rx, pose.ry, pose.rz

    angle = sqrt(rx*rx + ry*ry + rz*rz)
    T = np.eye(4)
    T[0, 3], T[1, 3], T[2, 3] = x, y, z

    if angle < 1e-10:
        return T  # identity rotation

    # Rodrigues' rotation formula
    kx, ky, kz = rx / angle, ry / angle, rz / angle
    c, s = cos(angle), sin(angle)
    v = 1 - c

    T[0, 0] = kx*kx*v + c
    T[0, 1] = kx*ky*v - kz*s
    T[0, 2] = kx*kz*v + ky*s
    T[1, 0] = kx*ky*v + kz*s
    T[1, 1] = ky*ky*v + c
    T[1, 2] = ky*kz*v - kx*s
    T[2, 0] = kx*kz*v - ky*s
    T[2, 1] = ky*kz*v + kx*s
    T[2, 2] = kz*kz*v + c

    return T


def _wrap_angle(a):
    """Wrap angle to [-pi, pi]."""
    return (a + pi) % (2 * pi) - pi


def _analytical_ik(T_desired):
    """Analytical (closed-form) inverse kinematics for the UR3e.

    Exploits the spherical wrist (joints 4, 5, 6 axes intersect)
    to decouple position from orientation, yielding up to 8 exact solutions.

    Based on the method from:
        Hawkins, K.P. "Analytic Inverse Kinematics for the Universal Robots"

    Args:
        T_desired: 4x4 homogeneous transformation matrix of the desired TCP pose.

    Returns:
        List of 6-element numpy arrays, each a valid joint solution.
        Empty list if no solution exists.
    """
    # DH constants
    d1 = UR3E_DH[0]["d"]   # 0.15185
    a2 = UR3E_DH[1]["a"]   # -0.24355
    a3 = UR3E_DH[2]["a"]   # -0.21325
    d4 = UR3E_DH[3]["d"]   # 0.13105
    d5 = UR3E_DH[4]["d"]   # 0.08535
    d6 = UR3E_DH[5]["d"]   # 0.0921

    T06 = T_desired

    # ── Step 1: Find the wrist center (origin of frame 5) ─────────────
    # P_05 = P_06 - d6 * z_6  (z_6 is the third column of R_06)
    P_06 = T06[:3, 3]
    z_6 = T06[:3, 2]
    P_05 = P_06 - d6 * z_6

    # ── Step 2: Solve θ1 (2 solutions) ────────────────────────────────
    # From the geometry: P_05x * sin(θ1) - P_05y * cos(θ1) = d4
    p05x, p05y = P_05[0], P_05[1]
    R = sqrt(p05x**2 + p05y**2)
    if R < 1e-10:
        return []  # singularity — wrist center on base z-axis

    # Check if |d4/R| <= 1 (reachability)
    if abs(d4 / R) > 1.0:
        return []

    phi1 = arctan2(p05y, p05x)
    phi2 = arccos(np.clip(d4 / R, -1.0, 1.0))
    theta1_options = [phi1 + phi2 + pi / 2, phi1 - phi2 + pi / 2]

    solutions = []

    for theta1 in theta1_options:
        # ── Step 3: Solve θ5 (2 solutions per θ1) ────────────────────
        # T_06[0,2]*sin(θ1) - T_06[1,2]*cos(θ1) = -cos(θ5) * ... but
        # the standard result for UR robots:
        # P_16z = d4 direction component
        # cos(θ5) = (P_06x*sin(θ1) - P_06y*cos(θ1) - d4) / d6
        numer5 = P_06[0] * sin(theta1) - P_06[1] * cos(theta1) - d4
        cos5 = np.clip(numer5 / d6, -1.0, 1.0)

        theta5_options = [arccos(cos5), -arccos(cos5)]

        for theta5 in theta5_options:
            sin5 = sin(theta5)

            # ── Step 4: Solve θ6 ──────────────────────────────────────
            if abs(sin5) < 1e-10:
                # Singular config — θ6 is arbitrary, set to 0
                theta6 = 0.0
            else:
                # From T_06 elements:
                # T_06[0,1]*sin(θ1) - T_06[1,1]*cos(θ1) = sin(θ5)*cos(θ6)  ... no
                # Standard formulation for UR:
                # θ6 = atan2(
                #   (-T_06[0,1]*sin(θ1) + T_06[1,1]*cos(θ1)) / sin(θ5),
                #   ( T_06[0,0]*sin(θ1) - T_06[1,0]*cos(θ1)) / sin(θ5)
                # )
                m = (-T06[0, 1]*sin(theta1) + T06[1, 1]*cos(theta1)) / sin5
                n = ( T06[0, 0]*sin(theta1) - T06[1, 0]*cos(theta1)) / sin5
                theta6 = arctan2(m, n)

            # ── Step 5: Solve θ2, θ3, θ4 (the planar 2R + orientation) ──
            # Build T01 and T45*T56 to isolate the middle chain
            T01 = _dh_matrix(theta1, UR3E_DH[0]["a"], UR3E_DH[0]["d"], UR3E_DH[0]["alpha"])
            T45 = _dh_matrix(theta5, UR3E_DH[4]["a"], UR3E_DH[4]["d"], UR3E_DH[4]["alpha"])
            T56 = _dh_matrix(theta6, UR3E_DH[5]["a"], UR3E_DH[5]["d"], UR3E_DH[5]["alpha"])

            # T_14 = T_01^-1 * T_06 * (T_45 * T_56)^-1
            T01_inv = np.linalg.inv(T01)
            T4556_inv = np.linalg.inv(T45 @ T56)
            T14 = T01_inv @ T06 @ T4556_inv

            # The position of frame 4 in frame 1 coordinates
            P14 = T14[:3, 3]

            # ── Solve θ3 (2 solutions) using law of cosines ──────────
            # The 2R arm (joints 2,3) operates in the x-y plane of frame 1.
            # P14_x = a2*cos(θ2) + a3*cos(θ2+θ3)
            # P14_y = a2*sin(θ2) + a3*sin(θ2+θ3)
            # P14_z ≈ d4 (offset along z1)
            D_sq = P14[0]**2 + P14[1]**2
            cos3_numer = D_sq - a2**2 - a3**2
            cos3_denom = 2 * a2 * a3

            if abs(cos3_denom) < 1e-10:
                continue

            cos3 = cos3_numer / cos3_denom
            if abs(cos3) > 1.0 + 1e-6:
                continue  # unreachable
            cos3 = np.clip(cos3, -1.0, 1.0)

            theta3_options = [arccos(cos3), -arccos(cos3)]

            for theta3 in theta3_options:
                # ── Solve θ2 ──────────────────────────────────────────
                # Standard 2-link planar arm in x-y plane of frame 1:
                # P14_x = (a2 + a3*c3)*c2 - a3*s3*s2
                # P14_y = (a2 + a3*c3)*s2 + a3*s3*c2
                s3 = sin(theta3)
                A = a2 + a3 * cos3
                B = a3 * s3

                # θ2 = atan2(A*P14_y - B*P14_x, A*P14_x + B*P14_y)
                theta2 = arctan2(
                    A * P14[1] - B * P14[0],
                    A * P14[0] + B * P14[1]
                )

                # ── Solve θ4 ──────────────────────────────────────────
                # θ234 = atan2(T14[1,0], T14[0,0])  (rotation about z)
                theta234 = arctan2(T14[1, 0], T14[0, 0])
                theta4 = theta234 - theta2 - theta3

                sol = np.array([
                    _wrap_angle(theta1),
                    _wrap_angle(theta2),
                    _wrap_angle(theta3),
                    _wrap_angle(theta4),
                    _wrap_angle(theta5),
                    _wrap_angle(theta6),
                ])
                solutions.append(sol)

    return solutions


def _select_closest_ik(solutions, qnear, joint_limits=None):
    """Pick the IK solution closest to qnear (weighted joint-space distance).

    Args:
        solutions: list of 6-element np arrays.
        qnear: 6-element np array (current/preferred joint config).
        joint_limits: optional list of (min, max) per joint.

    Returns:
        Best solution as np array, or None if no valid solution.
    """
    if not solutions:
        return None

    best = None
    best_dist = float("inf")

    for sol in solutions:
        # Optional: check joint limits
        if joint_limits is not None:
            valid = True
            for i, (lo, hi) in enumerate(joint_limits):
                if sol[i] < lo or sol[i] > hi:
                    valid = False
                    break
            if not valid:
                continue

        dist = np.sum((sol - qnear) ** 2)
        if dist < best_dist:
            best_dist = dist
            best = sol

    return best


# ---------------------------------------------------------------------------
# SimRobotControl — mirrors robot_control (UrScriptExt)
# ---------------------------------------------------------------------------

class SimRobotControl:
    """Mirrors the robot_control interface from ISCoin (UrScriptExt).

    Supported methods:
        - get_actual_joint_positions()
        - get_actual_joint_speeds()
        - get_actual_tcp_pose()
        - is_steady()
        - movej(joints, a, v, t, r, wait)
        - movej_waypoints(waypoints, wait)
        - movel(pose, a, v, t, r, wait)  — basic version
        - stopj(a, wait)
    """

    # -- Read state ----------------------------------------------------------

    def get_actual_joint_positions(self, wait=True):
        """Read the current 6 joint angles from the simulator.

        Returns:
            Joint6D (same as the real ISCoin).
        """
        states = _read_joint_states()
        values = _extract_6joints(states["position"])
        return Joint6D.createFromRadians(*values)

    def get_actual_joint_speeds(self, wait=True):
        """Read the current 6 joint velocities from the simulator.

        Returns:
            list of 6 floats (rad/s).
        """
        states = _read_joint_states()
        return _extract_6joints(states["velocity"])

    def get_actual_tcp_pose(self, wait=True):
        """Compute the current TCP pose using forward kinematics.

        Returns:
            TCP6D with [x, y, z, rx, ry, rz].
        """
        joints = self.get_actual_joint_positions(wait=wait)
        return _forward_kinematics(joints.toList())

    def is_steady(self):
        """Check if the robot is not moving (all joint speeds near zero).

        Returns:
            True if the robot is still, False otherwise.
        """
        speeds = self.get_actual_joint_speeds()
        for speed in speeds:
            if abs(speed) > 0.01:
                return False
        return True

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

        # Calculate duration
        if t > 0:
            duration_sec = int(math.ceil(t))
        else:
            duration_sec = _estimate_duration(v)

        # Send the trajectory
        _publish_trajectory([{
            "positions": positions,
            "duration_sec": duration_sec,
        }])
        print(f"movej sent (duration={duration_sec}s)")

        # Wait and verify
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
        # Build the list of trajectory points with cumulative time
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
                duration = _estimate_duration(v)

            cumulative_sec += duration
            points.append({
                "positions": positions,
                "duration_sec": cumulative_sec,
            })

        # Send all points as a single trajectory
        _publish_trajectory(points)
        print(f"movej_waypoints sent ({len(points)} points, total={cumulative_sec}s)")

        # Wait and verify final position
        if wait:
            time.sleep(cumulative_sec + 1)
            final_target = Joint6D.createFromRadians(*points[-1]["positions"])
            return self._verify_position(final_target)

        return True

    def movel(self, pose, a=1.2, v=0.25, t=0, r=0, wait=True):
        """Move the robot in a straight line to a Cartesian pose.

        This is a simplified version: it computes the inverse kinematics
        numerically and then does a movej. The real ISCoin does true
        linear interpolation in Cartesian space.

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
        # Get current joint positions as starting point for IK
        current_joints = self.get_actual_joint_positions()
        target_joints = self.get_inverse_kin(pose, qnear=current_joints)

        if target_joints is None:
            print("ERROR: movel failed — could not find inverse kinematics solution")
            return False

        # Estimate duration from Cartesian distance and requested velocity
        if t > 0:
            duration = t
        else:
            current_tcp = self.get_actual_tcp_pose()
            dx = pose.x - current_tcp.x
            dy = pose.y - current_tcp.y
            dz = pose.z - current_tcp.z
            dist = math.sqrt(dx*dx + dy*dy + dz*dz)
            # duration = distance / velocity, with a minimum of 2s
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

        # Build trajectory points with cumulative time
        points = []
        cumulative_sec = 0
        prev_joints = self.get_actual_joint_positions()
        prev_tcp = self.get_actual_tcp_pose()

        for wp in waypoints:
            wp_dict = wp.getAsDict()
            pose = TCP6D.createFromMetersRadians(*wp_dict["pose"])
            v = wp_dict["v"]
            t_param = wp_dict["t"]

            # Compute IK seeded from previous waypoint's joints
            target_joints = self.get_inverse_kin(pose, qnear=prev_joints)
            if target_joints is None:
                print(f"ERROR: movel_waypoints — IK failed for {pose}")
                return False

            # Estimate duration
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

        # Send all points as a single trajectory
        _publish_trajectory(points)
        print(f"movel_waypoints sent ({len(points)} points, total={cumulative_sec}s)")

        # Wait and verify final position
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
        _publish_trajectory([{
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

        Falls back to numerical optimization if the analytical solver
        returns no solutions.

        Args:
            pose: A TCP6D target.
            qnear: A Joint6D hint for selecting among multiple solutions
                   (optional, uses current position).

        Returns:
            Joint6D with the solution, or None if no solution found.
        """
        # Current joints as reference for picking the closest solution
        if qnear is not None:
            q0 = np.array(qnear.toList())
        else:
            q0 = np.array(self.get_actual_joint_positions().toList())

        # Convert TCP6D to 4x4 matrix
        T_desired = _pose_to_matrix(pose)

        # Analytical IK — exact closed-form solutions
        solutions = _analytical_ik(T_desired)

        if solutions:
            # Validate each solution with FK and keep only accurate ones
            valid = []
            target = np.array(pose.toList())
            for sol in solutions:
                tcp_check = _forward_kinematics(sol.tolist())
                err_pos = np.sum((np.array(tcp_check.toList()[:3]) - target[:3]) ** 2)
                if err_pos < 0.001:  # position error < ~3cm
                    valid.append(sol)

            best = _select_closest_ik(valid if valid else solutions, q0)
            if best is not None:
                # Final verification
                tcp_check = _forward_kinematics(best.tolist())
                err = np.sum((np.array(tcp_check.toList()[:3]) - np.array(pose.toList()[:3])) ** 2)
                if err < 0.001:
                    return Joint6D.createFromRadians(*best.tolist())
                print(f"WARNING: Analytical IK best solution has position error={sqrt(err):.6f}m")

        # Reachability diagnostic
        pos = np.array([pose.x, pose.y, pose.z])
        reach = np.linalg.norm(pos)
        max_reach = abs(UR3E_DH[1]["a"]) + abs(UR3E_DH[2]["a"])  # ~0.457m
        print(f"INFO: Analytical IK returned 0 valid solutions for target at "
              f"distance={reach:.4f}m (max reach ~{max_reach:.3f}m)")
        if reach > max_reach:
            print(f"WARNING: Target is likely outside the UR3e workspace "
                  f"({reach:.4f}m > {max_reach:.3f}m)")

        # Fallback: numerical optimization
        print("INFO: Trying numerical IK fallback...")
        return self._numerical_ik(pose, q0)

    def _numerical_ik(self, pose, q0):
        """Numerical IK fallback using scipy (multi-start L-BFGS-B)."""
        from scipy.optimize import minimize as sp_minimize

        target = np.array(pose.toList())

        def cost(q):
            tcp = _forward_kinematics(q.tolist())
            current = np.array(tcp.toList())
            pos_error = np.sum((current[:3] - target[:3]) ** 2)
            rot_error = np.sum((current[3:] - target[3:]) ** 2)
            return pos_error + 0.1 * rot_error

        best_result = None
        for attempt in range(8):
            q_start = q0 if attempt == 0 else q0 + np.random.uniform(-0.5, 0.5, 6)
            result = sp_minimize(cost, q_start, method="L-BFGS-B",
                                 options={"maxiter": 300})
            if best_result is None or result.fun < best_result.fun:
                best_result = result
            if best_result.fun < 1e-6:
                break

        if best_result.fun > 0.001:
            print(f"WARNING: IK solution may be inaccurate (error={best_result.fun:.6f})")
        if best_result.fun > 0.01:
            return None

        return Joint6D.createFromRadians(*best_result.x.tolist())

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


# ---------------------------------------------------------------------------
# Gripper helpers
# ---------------------------------------------------------------------------

def _pos_to_joint(pos):
    """Convert Robotiq position (0=open, 255=closed) to joint value in meters."""
    return GRIPPER_MAX_JOINT * (1.0 - max(0, min(255, pos)) / 255.0)


def _joint_to_pos(joint_val):
    """Convert joint value in meters to Robotiq position (0=open, 255=closed)."""
    clamped = max(0.0, min(GRIPPER_MAX_JOINT, joint_val))
    return int(round(255.0 * (1.0 - clamped / GRIPPER_MAX_JOINT)))


# ---------------------------------------------------------------------------
# SimGripper — mirrors RobotiqTwoFingersGripper
# ---------------------------------------------------------------------------

class SimGripper:
    """Simulated Robotiq HandE gripper that controls the Gazebo model.

    Provides the same public API as RobotiqTwoFingersGripper so user code
    works unchanged when switching between ISCoin and ISCoinSim.
    """

    class Status:
        """Status representing the gripper."""
        _OBJ_LABELS = {
            0: "In motion towards requested position",
            1: "Stopped due to contact while opening",
            2: "Stopped due to contact while closing",
            3: "Arrived to requested position. No object detected",
        }
        _STA_LABELS = {
            0: "Gripper is in reset (or automatic release) state",
            1: "Activation in progress",
            2: "Not used",
            3: "Activation is completed",
        }
        _GTO_LABELS = {
            0: "Stopped (or performing activation / automatic release)",
            1: "Go to Position Request mode",
        }
        _ACT_LABELS = {
            0: "Gripper stopped",
            1: "Gripper active",
        }

        def __init__(self, gOBJ, gSTA, gTO, gACT):
            self.gOBJ = gOBJ
            self.gSTA = gSTA
            self.gTO = gTO
            self.gACT = gACT

        def __str__(self):
            return (
                f"OBJ={self.gOBJ} ({self._OBJ_LABELS.get(self.gOBJ, '?')}), "
                f"STA={self.gSTA} ({self._STA_LABELS.get(self.gSTA, '?')}), "
                f"GTO={self.gTO} ({self._GTO_LABELS.get(self.gTO, '?')}), "
                f"ACT={self.gACT} ({self._ACT_LABELS.get(self.gACT, '?')})"
            )

        def __repr__(self):
            return self.__str__()

    class Fault:
        """Gripper fault representation."""
        _GFLT_LABELS = {
            0: "No Fault",
            5: "Action delayed; activation must be completed first",
            7: "The activation bit must be set prior to performing the action",
        }

        def __init__(self, kflt, gflt):
            self.kFLT = kflt
            self.gFLT = gflt

        def __str__(self):
            label = self._GFLT_LABELS.get(self.gFLT, f"Fault code {self.gFLT}")
            return f"kFLT=0x{self.kFLT:02X}, gFLT={self.gFLT} ({label})"

        def __repr__(self):
            return self.__str__()

    # -- Constructor ----------------------------------------------------------

    def __init__(self, opened_size_mm=50.0):
        self._opened_size_mm = opened_size_mm
        self._activated = False
        self._gto = 0       # go-to flag
        self._obj = 0       # object detection
        self._controllers_spawned = False

    # -- Controller spawning --------------------------------------------------

    def _ensure_controllers(self):
        """Spawn gripper controllers inside the container (once per session)."""
        if self._controllers_spawned:
            return
        for ctrl in GRIPPER_CONTROLLER_NAMES:
            try:
                _docker_exec(
                    "source /root/ws_moveit/install/setup.bash && "
                    f"ros2 run controller_manager spawner {ctrl} "
                    f"-c /controller_manager",
                    timeout=30,
                )
            except RuntimeError as e:
                # Controller may already be active
                if "already active" in str(e).lower() or "already loaded" in str(e).lower():
                    pass
                else:
                    raise
        self._controllers_spawned = True

    # -- Sending gripper commands ---------------------------------------------

    def _send_gripper_command(self, joint_value):
        """Send a GripperCommand action goal to both finger controllers."""
        self._ensure_controllers()
        for ctrl in GRIPPER_CONTROLLER_NAMES:
            _docker_exec(
                "source /root/ws_moveit/install/setup.bash && "
                f"ros2 action send_goal /{ctrl}/gripper_cmd "
                f"control_msgs/action/GripperCommand "
                f"\"{{command: {{position: {joint_value}, max_effort: 130.0}}}}\"",
                timeout=15,
            )

    def _read_gripper_joints(self):
        """Read gripper joint positions and velocities from /joint_states."""
        states = _read_joint_states()
        positions = []
        velocities = []
        for name in GRIPPER_JOINT_NAMES:
            positions.append(states["position"].get(name, 0.0))
            velocities.append(states["velocity"].get(name, 0.0))
        return positions, velocities

    # -- Activation -----------------------------------------------------------

    def activate(self):
        """Activate the gripper. Returns True."""
        self._ensure_controllers()
        self._activated = True
        print("SimGripper activated")
        return True

    def deactivate(self):
        """Deactivate the gripper. Returns True."""
        self._activated = False
        self._gto = 0
        print("SimGripper deactivated")
        return True

    def isActivated(self):
        """Check if the gripper is activated."""
        return self._activated

    def waitUntilActive(self):
        """Wait until the gripper is active (instant in sim)."""
        if not self._activated:
            self.activate()

    # -- Movement -------------------------------------------------------------

    def move(self, pos, speed=255, force=50):
        """Move the gripper to a position (0=open, 255=closed).

        Args:
            pos: Position 0-255 (0=open/50mm, 255=closed/0mm).
            speed: Speed 0-255 (not directly controllable in sim).
            force: Force 0-255 (not used in sim).

        Returns:
            True if command was sent successfully.
        """
        if not self._activated:
            print("WARNING: Gripper not activated. Call activate() first.")
            return False

        self._gto = 1
        self._obj = 0  # in motion
        joint_val = _pos_to_joint(pos)
        self._send_gripper_command(joint_val)
        self._obj = 3  # arrived, no object
        return True

    def open(self, speed=255, force=50):
        """Open the gripper."""
        return self.move(0, speed, force)

    def close(self, speed=255, force=50):
        """Close the gripper."""
        return self.move(255, speed, force)

    # -- Motion status --------------------------------------------------------

    def isMoving(self):
        """Check if the gripper is currently moving."""
        _, velocities = self._read_gripper_joints()
        return any(abs(v) > 0.001 for v in velocities)

    def waitUntilStopped(self):
        """Poll until gripper velocities are near zero."""
        for _ in range(50):  # up to ~5 seconds
            if not self.isMoving():
                return
            time.sleep(0.1)

    # -- Position / sensor readout --------------------------------------------

    def getPosition(self):
        """Get the gripper position (0=open, 255=closed)."""
        positions, _ = self._read_gripper_joints()
        # Average both fingers
        avg = sum(positions) / len(positions) if positions else 0.0
        return _joint_to_pos(avg)

    def getEstimatedPositionMm(self):
        """Get the estimated gripper opening in millimeters."""
        pos = self.getPosition()
        return self._opened_size_mm * (1.0 - pos / 255.0)

    def getCurrent(self):
        """Get the current register (no current sensing in sim)."""
        return 0

    def getEstimatedCurrentMA(self):
        """Get the estimated current in mA (no current sensing in sim)."""
        return 0.0

    # -- Object detection -----------------------------------------------------

    def hasDetectedObject(self):
        """Check if the gripper has detected an object (always False in sim)."""
        return False

    def closeAndCheckSize(self, expected_size_mm, plus_margin_mm,
                          minus_margin_mm, speed=255, force=50):
        """Close the gripper and check if the opening matches expectations."""
        self.close(speed, force)
        self.waitUntilStopped()
        actual_mm = self.getEstimatedPositionMm()
        return (expected_size_mm - minus_margin_mm) <= actual_mm <= (expected_size_mm + plus_margin_mm)

    def openAndCheckSize(self, expected_size_mm, plus_margin_mm,
                         minus_margin_mm, speed=255, force=50):
        """Open the gripper and check if the opening matches expectations."""
        self.open(speed, force)
        self.waitUntilStopped()
        actual_mm = self.getEstimatedPositionMm()
        return (expected_size_mm - minus_margin_mm) <= actual_mm <= (expected_size_mm + plus_margin_mm)

    # -- Status / fault -------------------------------------------------------

    def getStatus(self):
        """Get the gripper status."""
        return SimGripper.Status(
            gOBJ=self._obj,
            gSTA=3 if self._activated else 0,
            gTO=self._gto,
            gACT=1 if self._activated else 0,
        )

    def getOBJ(self):
        """Get the object status register."""
        return self._obj

    def getSTA(self):
        """Get the gripper state register."""
        return 3 if self._activated else 0

    def getGTO(self):
        """Get the go-to status register."""
        return self._gto

    def getACT(self):
        """Get the activation status register."""
        return 1 if self._activated else 0

    def getFault(self):
        """Get the fault status (always no-fault in sim)."""
        return SimGripper.Fault(kflt=0, gflt=0)


# ---------------------------------------------------------------------------
# ISCoinSim — main class (drop-in for ISCoin)
# ---------------------------------------------------------------------------

class ISCoinSim:
    """Drop-in replacement for ISCoin that talks to the Gazebo simulator.

    Supports:
        - robot_control: movej, movel, movej_waypoints, stopj,
          get_actual_joint_positions, get_actual_joint_speeds,
          get_actual_tcp_pose, is_steady, get_inverse_kin
        - gripper: simulated Robotiq HandE (activate, open, close, move, etc.)
        - camera: not available in simulator (prints warning)
    """

    def __init__(self, container_name=CONTAINER_NAME, opened_gripper_size_mm=50.0):
        global CONTAINER_NAME
        CONTAINER_NAME = container_name

        # Check that the container is running
        result = subprocess.run(
            ["docker", "inspect", "-f", "{{.State.Running}}", container_name],
            capture_output=True,
            text=True,
        )
        if "true" not in result.stdout:
            raise ConnectionError(
                f"Container '{container_name}' is not running. "
                "Start the simulator first:\n"
                "  cd ur3e-simulator/.docker && docker compose run --rm cpu\n"
                "  ros2 launch iscoin_simulation_gz iscoin_sim_control.launch.py"
            )

        self._robot_control = SimRobotControl()
        self._gripper = SimGripper(opened_size_mm=opened_gripper_size_mm)
        print(f"ISCoinSim connected to container '{container_name}'")

    @property
    def robot_control(self):
        return self._robot_control

    @property
    def gripper(self):
        return self._gripper

    @property
    def camera(self):
        print("WARNING: Camera is not available in the Gazebo simulator")
        return None

    def close(self):
        print("ISCoinSim closed")