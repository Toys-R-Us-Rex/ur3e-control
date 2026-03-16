"""SimGripper — simulated Robotiq HandE gripper for the Gazebo simulator.

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
Inspired:   API modelled on RobotiqTwoFingersGripper from URBasic
            (devices/robotiq_two_fingers_gripper.py)
"""

import time

class SimGripper:
    """Simulated Robotiq HandE gripper for Gazebo.

    Provides the same public API as RobotiqTwoFingersGripper,
    but mocks actions
    """

    def __init__(self, opened_size_mm=50.0):
        self._opened_size_mm = opened_size_mm
        self._activated = False

    # -- Activation -----------------------------------------------------------

    def activate(self):
        """Activate the gripper. Returns True."""
        time.sleep(0.2)
        self._activated = True
        print("SimGripper activated")
        return True

    def deactivate(self):
        """Deactivate the gripper. Returns True."""
        time.sleep(0.2)
        self._activated = False
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

    # def move(self, pos, speed=255, force=50):
    #     """Move the gripper to a position (0=open, 255=closed).

    #     Args:
    #         pos: Position 0-255 (0=open/50mm, 255=closed/0mm).
    #         speed: Speed 0-255 (not directly controllable in sim).
    #         force: Force 0-255 (not used in sim).

    #     Returns:
    #         True if command was sent successfully.
    #     """
    #     if not self._activated:
    #         print("WARNING: Gripper not activated. Call activate() first.")
    #         return False

    #     self._gto = 1
    #     self._obj = 0  # in motion
    #     joint_val = _pos_to_joint(pos)
    #     self._send_gripper_command(joint_val)
    #     self._obj = 3  # arrived, no object
    #     return True

    def open(self, speed=255, force=50) -> bool:
        """
        Open the gripper if activated.
        Returns false on failure
        """

        time.sleep(0.1)
        return self.isActivated()

    def close(self, speed=255, force=50):
        """
        Clost the gripper if activated.
        Returns false on failure
        """
        time.sleep(0.1)
        return self.isActivated()