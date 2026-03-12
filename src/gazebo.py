



from src.logger import LoggingLog

from URBasic.urScript import UrScript
from URBasic.waypoint6d import TCP6D, Joint6D

from duckify_simulation.duckify_sim import DuckifySim
from duckify_simulation.duckify_sim.robot_control import SimRobotControl

def move_simple(robot: SimRobotControl|UrScript, motion):
    for m in motion:
        if isinstance(m, TCP6D):
            robot.movel(m, wait=True)
        
        elif isinstance(m, Joint6D):
            robot.movej(m, wait=True)

def test_waypoints(waypoints, log: LoggingLog):
    try:
        duckify_sim = DuckifySim()
        robot_sim = duckify_sim.robot_control
        move_simple(robot_sim, waypoints)
        return True
    except Exception as e:
        log.log(f"Test skipped: {e}")
        return False