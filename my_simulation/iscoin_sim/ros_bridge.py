"""Low-level helpers for communicating with the Gazebo simulator via Docker."""

import math
import subprocess

# Container name (must match docker-compose)
CONTAINER_NAME = "iscoin_simulator"

# Joint names in the order the UR3e expects them
JOINT_NAMES = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]


def docker_exec(cmd, timeout=15):
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


def parse_joint_states(raw_output):
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


def read_joint_states():
    """Read /joint_states from the simulator once. Returns parsed dict."""
    raw = docker_exec(
        "source /opt/ros/humble/setup.bash && "
        "ros2 topic echo /joint_states --once"
    )
    return parse_joint_states(raw)


def extract_6joints(joint_dict):
    """Extract our 6 joint values in the correct order from a name->value dict."""
    values = []
    for name in JOINT_NAMES:
        if name not in joint_dict:
            raise RuntimeError(f"Joint '{name}' not found in /joint_states")
        values.append(joint_dict[name])
    return values


def publish_trajectory(points, timeout=15):
    """Publish a trajectory message to the joint_trajectory_controller topic.

    Args:
        points: list of dicts, each with 'positions' (list of 6 floats)
                and 'duration_sec' (int).
    """
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

    docker_exec(cmd, timeout=timeout)


def estimate_duration(v):
    """Estimate movement duration from velocity parameter."""
    return max(2, int(math.ceil(math.pi / v)))
