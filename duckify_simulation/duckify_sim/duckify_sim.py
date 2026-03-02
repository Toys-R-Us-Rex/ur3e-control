"""DuckifySim — A drop-in replacement for ISCoin that talks to the Gazebo simulator.

Usage:
    from my_simulation import DuckifySim

    iscoin = DuckifySim()
    print(iscoin.robot_control.get_actual_joint_positions())
    iscoin.robot_control.movej(Joint6D.createFromRadians(1.19, -1.13, 1.05, -1.60, -1.52, 1.05))

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
Inspired:   API modelled on ISCoin by Axel Amand,
            HES-SO Valais/Wallis (MIT License, 2024)
"""

import subprocess

from . import ros_bridge
from .robot_control import SimRobotControl
from .gripper import SimGripper


class DuckifySim:
    """Drop-in replacement for ISCoin that talks to the Gazebo simulator."""

    def __init__(self, container_name="iscoin_simulator", opened_gripper_size_mm=50.0):
        ros_bridge.CONTAINER_NAME = container_name

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
        print(f"DuckifySim connected to container '{container_name}'")

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
        print("DuckifySim closed")
