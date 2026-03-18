from src.logger import DataStore, DataStoreForce

from URBasic.iscoin import ISCoin
from URBasic.urScript import UrScript
from URBasic.waypoint6d import TCP6D, Joint6D

from duckify_simulation.duckify_sim.robot_control import SimRobotControl

from src.config import *

def move_simple(robot: SimRobotControl|UrScript, motion, ds: DataStore = None):
    robot.movej(HOMEJ)
    for segment in motion:
        for m in segment.waypoints:
            print(m)
            if isinstance(m, TCP6D):
                if not robot.movel(m, wait=True) and ds:
                    ds.log("TCP not reached: ", str(m))
            
            elif isinstance(m, Joint6D):
                if not robot.movej(m, wait=True) and ds:
                    ds.log("JOINT not reached: ", str(m))
            
            else:
                raise NotImplemented("Only TCP6D or JOINT6D points allowed.")

class Robot:
    def __init__(self, datastore: DataStore, robot_ip: str):
        self.ds = datastore
        self.robot_ip = robot_ip
    
    def run(self):
        answer = input("Do you want to run the code on the REAL robot continue? y/n \n")

        if answer != "y":
            self.ds.log(f"Do not run on robot: {answer}")
            raise ValueError("You chose not the run on robot")

        iscoin = ISCoin(host=self.robot_ip, opened_gripper_size_mm=40)
        robot = iscoin.robot_control

        waypoint = self.ds.load_waypoints()

        force = DataStoreForce(robot)
        force.start_logging()

        try:
            move_simple(robot, waypoint, self.ds)
        except Exception as e:
            self.ds.log(f"Exception with robot: {e}")
            raise

        force.stop_logging()