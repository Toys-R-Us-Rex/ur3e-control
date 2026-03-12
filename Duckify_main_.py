#!/usr/bin/env python3

from URBasic.iscoin import ISCoin
from URBasic.waypoint6d import TCP6D, Joint6D
import src.logger as Recording
from src.calibration import launch_calibration
from src.transformation import launch_transformation

from src.gazebo import test_waypoints


record = Recording.LoggingLog()
record._log("Start Duckify_main_.py")

# Create a new ISCoin object
# UR3e1 IP (closest to window): 10.30.5.158
# UR3e2 IP: 10.30.5.159
ROBOT_IP = "10.30.5.158"

JSON_OBJECT = "duckify_simulation/paths/duck_uv-dot-trace.json"

# ---------------------------------------------------------------------------
# Calibration TCP
# ---------------------------------------------------------------------------

while True:
    answer = input("Calibration, okay? y/n")
    if answer == "y":
        tcps, tcp_offset = record.load_calibration()
        break
    
    answer = input("True robot calibration okay? y/n")
    if answer == "y":
        launch_calibration(ROBOT_IP, record)
    else:    
        answer = input("Default calibration okay? y/n")
        if answer == "y":
            tcps, tcp_offset = record.load_calibration("save_data/calibration_default.pkl")
            break


# ---------------------------------------------------------------------------
# Transformation from object space to robot space
# ---------------------------------------------------------------------------

while True:
    answer = input("Object calibration, okay? y/n")
    if answer == "y":
        T_obj2robot, T_normal = record.load_transformation()
        break
    
    answer = input("True robot object calibration okay? y/n")
    if answer == "y":
        launch_transformation(ROBOT_IP, JSON_OBJECT, record)
    else:    
        answer = input("Default calibration okay? y/n")
        if answer == "y":
            T_obj2robot, T_normal = record.load_transformation("save_data/transformation_default.pkl")
            break


# ---------------------------------------------------------------------------
# Compute path
# ---------------------------------------------------------------------------

while True:
    answer = input("Draw path, okay? y/n")
    if answer == "y":
        waypoints = record.load_waypoints()
        break
    
    answer = input("Waypoints computation, okay? y/n")
    if answer == "y":
        #TODO
        pass
    else:    
        answer = input("Default calibration okay? y/n")
        if answer == "y":
            waypoints = record.load_waypoints("save_data/waypoints_default.pkl")
            break


# ---------------------------------------------------------------------------
# Control path with Gazebo
# ---------------------------------------------------------------------------

gazebo = test_waypoints(waypoints, record)

if gazebo:
    answer = input("Gazebo test failed. Do you want to continue? y/n")
else:
    answer = input("Gazebo test successed. Do you want to continue? y/n")

if answer != "y":
    exit(0)


# ---------------------------------------------------------------------------
# Run true robot
# ---------------------------------------------------------------------------

def move_simple(robot, motion):
    for m in motion:
        if isinstance(m, TCP6D):
            robot.movel(m, wait=True)
        
        elif isinstance(m, Joint6D):
            robot.movej(m, wait=True)


iscoin = ISCoin(host=ROBOT_IP, opened_gripper_size_mm=40)
robot = iscoin.robot_control

try:
    move_simple(robot, waypoints)
except Exception as e:
    record.log(f"Exception with robot: {e}")

iscoin.close()
