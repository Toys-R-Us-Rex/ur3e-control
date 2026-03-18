import sys
import json
from pathlib import Path
import os
import time
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

#from duckify_simulation.duckify_sim import DuckifySim as ISCoin
from URBasic import ISCoin  # <-- swap this line to use the real robot

# from src.safety import get_reachable
from URBasic import Joint6D, TCP6D
from URBasic.waypoint6d import TCP6DDescriptor
from math import radians, pi

# Connect


class Measurement :
    '''
    The minimal distance we must ensure before destructive reaction (unit in meter)
    The wood plate = 0.005
    The pen support = 0.046
    The pen length outside of the pen support = 0.101
    '''
    minimal_distance = 0.155
    length_between_pen = 0.05 # This distance comes from the design of the wood support for pen.
    facing_down = (pi, 0, 0) # To maintain the gripper facing down

    gripper_length = 0.101

m = Measurement()

def get_current_tcp():
    current_tcp = iscoin.robot_control.get_actual_tcp_pose()
    print(f"current tcp: ({current_tcp.x}, {current_tcp.y}, {current_tcp.z}, {current_tcp.rx}, {current_tcp.ry}, {current_tcp.rz})")
    return (current_tcp.x, current_tcp.y, current_tcp.z)

def home_position():
    
    tcp_x, tcp_y, tcp_z = get_current_tcp()

    home_down = TCP6D.createFromMetersRadians(
        tcp_x,
        tcp_y,
        tcp_z,
        m.facing_down[0],
        m.facing_down[1],
        m.facing_down[2]
    )

    return home_down


def get_pen_by_id(pen_id):
    """
    This function will upgrade the function "get pen" from Nathan Antonietti to be able to go to a pen base on an id we choose:
    
    The id = 0 will be the pen in the support with waypoint and every id > 0 will be at 50mm from the id 0 in Y axis
    """
    

    to_top_of_pen = TCP6D.createFromMetersRadians(
        PEN_POS_0[0],
        PEN_POS_0[1] - m.length_between_pen*pen_id,
        PEN_POS_0[2] + 0.05,
        m.facing_down[0],
        m.facing_down[1],
        m.facing_down[2]
    )

    to_pen = TCP6D.createFromMetersRadians(
        PEN_POS_0[0],
        PEN_POS_0[1] - m.length_between_pen*pen_id,
        PEN_POS_0[2],
        m.facing_down[0],
        m.facing_down[1],
        m.facing_down[2]
    )

    return to_top_of_pen, to_pen


def switch_pen(id2, id1 = None) :
    

    iscoin.gripper.activate()

    time.sleep(2)
    move_list = []

    v = 0.2

    home = home_position()

    waypoints_home = [
        TCP6DDescriptor(home, v=v),
    ]
    if id1 != None :
        pen_1 = get_pen_by_id(id1)
        waypoints_to_pen_1 = [
            TCP6DDescriptor(pen_1[0], v=v, r=0.01),
            TCP6DDescriptor(pen_1[1], v=v, r=0.01),
        ]
        waypoints_move_out_pen_1 = [
            TCP6DDescriptor(pen_1[0], v=v, r=0.01),
        ]

        move_list.append(waypoints_to_pen_1)
        move_list.append(0)
        move_list.append(waypoints_move_out_pen_1)


    pen_2 = get_pen_by_id(id2)

    waypoints_to_pen_2 = [
            TCP6DDescriptor(pen_2[0], v=v, r=0.01),
            TCP6DDescriptor(pen_2[1], v=v, r=0.01),
    ]
    
    waypoints_move_out_pen_2 = [
        TCP6DDescriptor(pen_2[0], v=v, r=0.01),
    ]

    waypoints_home = [
        TCP6DDescriptor(home, v=v, r=0.01),
    ]
    move_list.append(0)
    move_list.append(waypoints_to_pen_2)
    move_list.append(1)
    move_list.append(waypoints_move_out_pen_2)
    move_list.append(waypoints_home)

    print(move_list)
    for m in move_list:
        print(type(m))
        if type(m)==int:
            if m == 0:
                print("open")
                iscoin.gripper.open()
                time.sleep(1)
            else :
                print("close")
                iscoin.gripper.close()
                time.sleep(1)
        else :

            iscoin.robot_control.movel_waypoints(m, wait=True)

    print("n")
    iscoin.gripper.open()
    time.sleep(1)
    iscoin.gripper.deactivate()
    print("Deactivated")
    return True

    '''
    ok = iscoin.robot_control.movel_waypoints(waypoints_to_pen_1)
    if not ok:
        print("Pick approach failed")
        return False
    '''

if __name__ == "__main__" :

    iscoin = ISCoin(host="10.30.5.158") #(host="10.30.5.159")
    PEN_POS_0 =  [-0.250, -0.176, m.minimal_distance] # pen tip position in world frame -(SUPPORT_OFFSET - PEN_IN_SUPPORT_OFFSET)
    
    '''
    with open('calib.json', 'r', encoding='utf-8') as f:
        f.read()
        data = json.load(f)
        PEN_POS_0 = [data["x"], data["y"], data["z"]]
    '''

    tool_tcp = TCP6D.createFromMetersRadians(0.0, 0.0, m.gripper_length, 0.0, 0.0, 0.0)
    iscoin.robot_control.set_tcp(tool_tcp)
    switch_pen(0)
    