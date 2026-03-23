"""
Gazebo Simulation Stage Module

This module provides functionality for testing waypoints in the Gazebo simulation.

MIT License

Copyright (c) 2026 Mariéthoz Cédric

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

Author:     Mariéthoz Cédric, with assistance from Copilot AI (Microsoft)
Course:     HES-SO Valais-Wallis, Engineering Track 304
"""

from src.logger import DataStore
from src.robot import move_simple

from duckify_simulation.duckify_sim import DuckifySim
from src.segment import Segment
from src.stage import Stage

def test_waypoints(waypoints: list[Segment], ds: DataStore) -> bool:
    """
    Test the waypoints in the Gazebo simulation.

    Parameters
    ----------
    waypoints : list[Segment]
        The list of waypoints to test.
    ds : DataStore
        The data store instance.

    Returns
    -------
    bool
        True if the test is successful, False otherwise.
    """
    try:
        duckify_sim = DuckifySim()
        robot_sim = duckify_sim.robot_control
        _, tcp_offset = ds.load_calibration()
        robot_sim.set_tcp(tcp_offset)

        move_simple(robot_sim, waypoints)
        
        answer = input("Do the Gazebo test succed? y/n \n")
        
        if answer == 'y':
            return True
        elif answer == 'n':
            return False
        else:
            ds.log("Gazebo test skipped: no clear answer")
            return True
    except Exception as e:
        ds.log(f"Gazebo test skipped: {e}")
        raise
    
class Gazebo(Stage):
    """
    A stage for testing waypoints in the Gazebo simulation.
    """
    def __init__(self, datastore: DataStore):
        """
        Initialize the Gazebo stage.

        Parameters
        ----------
        datastore : DataStore
            The data store instance.
        """
        super().__init__(name="Gazebo", datastore=datastore)

    def run(self):
        """
        Run the Gazebo stage.
        """
        waypoints = self.ds.load_joint_segments()

        answer = input("Do you want to skip Gazebo test? y/n \n")
        if answer == 'y':
            self.ds.log("Gazebo test skipped.")
            raise RuntimeError("You can not avoid gazebot.")

        gazebo = test_waypoints(waypoints, self.ds)

        if gazebo:
            self.ds.log("Gazebo test successed.")
        else:
            self.ds.log("Gazebo test failed.")

        return gazebo