from src.logger import DataStore, DataStoreForce

from URBasic.iscoin import ISCoin
from URBasic.urScript import UrScript
from URBasic.waypoint6d import TCP6D, Joint6D

from duckify_simulation.duckify_sim.robot_control import SimRobotControl

from src.config import *
from src.utils import ask_yes_no

import matplotlib.pyplot as plt

from src.stage import Stage

def move_simple(robot: SimRobotControl | UrScript, motion, ds: DataStore = None):
    """
    Move the robot to the specified waypoints.

    Parameters
    ----------
    robot : SimRobotControl | UrScript
        The robot control instance.
    motion : list[Segment]
        The list of segments to move through.
    ds : DataStore, optional
        The data store instance.

    Returns
    -------
    None
    """
    robot.movej(HOMEJ)

    # Create six empty joint lists
    j1, j2, j3, j4, j5, j6 = ([] for _ in range(6))

    for s, segment in enumerate(motion):
        for i, m in enumerate(segment.waypoints):
            print(s, i, m)
            if isinstance(m, TCP6D):
                if not robot.movel(m, wait=True) and ds:
                    ds.log("TCP not reached: ", str(m))

            elif isinstance(m, Joint6D):
                # Collect joint values
                j1.append(m.j1)
                j2.append(m.j2)
                j3.append(m.j3)
                j4.append(m.j4)
                j5.append(m.j5)
                j6.append(m.j6)

                if not robot.movej(m, wait=True) and ds:
                    ds.log("JOINT not reached: ", str(m))

            else:
                raise NotImplementedError("Only TCP6D or JOINT6D points allowed.")

    # Plot joint evolution
    fig, axs = plt.subplots(6, sharex=True)
    joint_lists = [j1, j2, j3, j4, j5, j6]

    for i, j in enumerate(joint_lists):
        axs[i].plot(j)
        axs[i].set_ylabel(f"J{i+1}")

    axs[-1].set_xlabel("Waypoint index")
    plt.tight_layout()
    plt.show()


class Robot(Stage):
    def __init__(self, datastore: DataStore, robot_ip: str):
        """
        Initialize the Robot stage.

        Parameters
        ----------
        datastore : DataStore
            The data store instance.
        robot_ip : str
            The IP address of the robot.
        """
        super().__init__(name="Robot", datastore=datastore)
        self.robot_ip = robot_ip
    
    def run(self):
        """
        Run the robot stage.
        """
        answer = ask_yes_no("Do you want to run the code on the REAL robot continue? y/n \n")

        if not answer:
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