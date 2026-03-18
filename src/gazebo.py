from src.logger import DataStore
from src.robot import move_simple

from URBasic.urScript import UrScript
from URBasic.waypoint6d import TCP6D, Joint6D

from duckify_simulation.duckify_sim import DuckifySim
from duckify_simulation.duckify_sim.robot_control import SimRobotControl
from src.segment import Segment

def test_waypoints(waypoints: list[Segment], ds: DataStore):
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
    
class Gazebo:
    def __init__(self, datastore: DataStore):
        self.ds = datastore

    def run(self):
        waypoints = self.ds.load_tcp_segments()

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