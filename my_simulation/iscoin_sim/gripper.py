"""SimGripper — simulated Robotiq HandE gripper for the Gazebo simulator."""

import time

from .ros_bridge import docker_exec, read_joint_states

# Gripper constants
GRIPPER_JOINT_NAMES = [
    "robotiq_hande_left_finger_joint",
    "robotiq_hande_right_finger_joint",
]
GRIPPER_MAX_JOINT = 0.025  # meters, fully open
GRIPPER_CONTROLLER_NAMES = ["hande_controller_left", "hande_controller_right"]


def _pos_to_joint(pos):
    """Convert Robotiq position (0=open, 255=closed) to joint value in meters."""
    return GRIPPER_MAX_JOINT * (1.0 - max(0, min(255, pos)) / 255.0)


def _joint_to_pos(joint_val):
    """Convert joint value in meters to Robotiq position (0=open, 255=closed)."""
    clamped = max(0.0, min(GRIPPER_MAX_JOINT, joint_val))
    return int(round(255.0 * (1.0 - clamped / GRIPPER_MAX_JOINT)))


class SimGripper:
    """Simulated Robotiq HandE gripper that controls the Gazebo model.

    Provides the same public API as RobotiqTwoFingersGripper so user code
    works unchanged when switching between ISCoin and ISCoinSim.
    """

    def __init__(self, opened_size_mm=50.0):
        self._opened_size_mm = opened_size_mm
        self._activated = False
        self._gto = 0
        self._obj = 0
        self._controllers_spawned = False

    # -- Controller spawning --------------------------------------------------

    def _ensure_controllers(self):
        """Spawn gripper controllers inside the container (once per session)."""
        if self._controllers_spawned:
            return
        for ctrl in GRIPPER_CONTROLLER_NAMES:
            try:
                docker_exec(
                    "source /root/ws_moveit/install/setup.bash && "
                    f"ros2 run controller_manager spawner {ctrl} "
                    f"-c /controller_manager",
                    timeout=30,
                )
            except RuntimeError as e:
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
            docker_exec(
                "source /root/ws_moveit/install/setup.bash && "
                f"ros2 action send_goal /{ctrl}/gripper_cmd "
                f"control_msgs/action/GripperCommand "
                f"\"{{command: {{position: {joint_value}, max_effort: 130.0}}}}\"",
                timeout=15,
            )

    def _read_gripper_joints(self):
        """Read gripper joint positions and velocities from /joint_states."""
        states = read_joint_states()
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
        avg = sum(positions) / len(positions) if positions else 0.0
        return _joint_to_pos(avg)

    def getEstimatedPositionMm(self):
        """Get the estimated gripper opening in millimeters."""
        pos = self.getPosition()
        return self._opened_size_mm * (1.0 - pos / 255.0)

    # -- Object detection -----------------------------------------------------

    def hasDetectedObject(self):
        """Check if the gripper has detected an object (always False in sim)."""
        return False
