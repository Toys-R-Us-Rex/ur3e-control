import sys
import os
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from duckify_simulation.duckify_sim import DuckifySim as ISCoin
# from URBasic import ISCoin  # <-- swap this line to use the real robot

# from src.safety import get_reachable
from URBasic import Joint6D, TCP6D
from URBasic.waypoint6d import TCP6DDescriptor
from math import radians, pi

# Connect
iscoin = ISCoin() #(host="10.30.5.159")

HANDE_OFFSET = 0.101
PEN_OFFSET = 0.1
PEN_IN_SUPPORT_OFFSET = 0.101
SUPPORT_OFFSET = 0.046
DOWN_RX, DOWN_RY, DOWN_RZ = pi, 0.0, 0.0

PEN_DIST = 0.05

PEN_POS_0 = [0.3, -0.2, SUPPORT_OFFSET + PEN_IN_SUPPORT_OFFSET]  # pen tip position in world frame -(SUPPORT_OFFSET - PEN_IN_SUPPORT_OFFSET)

tool_tcp = TCP6D.createFromMetersRadians(0.0, 0.0, HANDE_OFFSET, 0.0, 0.0, 0.0)
iscoin.robot_control.set_tcp(tool_tcp)

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
        DOWN_RX,
        DOWN_RY,
        DOWN_RZ,
    )

    return home_down


def get_pen_by_id(pen_id):
    """
    This function will upgrade the function "get pen" from Nathan Antonietti to be able to go to a pen base on an id we choose:
    
    The id = 0 will be the pen in the support with waypoint and every id > 0 will be at 50mm from the id 0 in Y axis
    """
    

    to_top_of_pen = TCP6D.createFromMetersRadians(
        PEN_POS_0[0],
        PEN_POS_0[1] - PEN_DIST*pen_id,
        PEN_POS_0[2] + PEN_OFFSET,
        DOWN_RX,
        DOWN_RY,
        DOWN_RZ,
    )

    to_pen = TCP6D.createFromMetersRadians(
        PEN_POS_0[0],
        PEN_POS_0[1] - PEN_DIST*pen_id,
        PEN_POS_0[2],
        DOWN_RX,
        DOWN_RY,
        DOWN_RZ,
    )

    return to_top_of_pen, to_pen


def switch_pen(id1, id2) :
    """
    This function will make the robot transition between holding a pen to holding another pen. For that it will go through different steps :
    1. Get to a "base position" to have a reference position
    2. Move over a given pen by its id (id1)
    3. Move down to the position to get the pen
    4. Catch it (will not be done in the simulation)
    5. Move back to the position over the pen
    6. Move back to the base position
    7. Run step 2 and 3 again
    8. Open the hand to free the pen
    9. Run step 3
    10. Run step 3-4-5-6 for the second pen
    11. Move back to the "base" position
    """

    v = 0.2

    home = home_position()

    pen_1 = get_pen_by_id(id1)
    pen_2 = get_pen_by_id(id2)

    print("\n=== MOVEL: going over pen position ===")
    print()
    waypoints_to_pen_1 = [
        TCP6DDescriptor(home, v=v, r=0.01),
        TCP6DDescriptor(pen_1[0], v=v, r=0.01),
        TCP6DDescriptor(pen_1[1], v=v),
        
    ]

    waypoints_move_out_pen_1 = [
        TCP6DDescriptor(pen_1[0], v=v, r=0.01),
    ]

    waypoints_home = [
        TCP6DDescriptor(home, v=v),
    ]

    waypoints_pen_back = [
        TCP6DDescriptor(pen_1[0], v=v, r=0.01),
        TCP6DDescriptor(pen_1[1], v=v),
    ]

    waypoints_transition_pen = [
        TCP6DDescriptor(pen_1[0], v=v, r=0.01),
        TCP6DDescriptor(pen_2[0], v=v, r=0.01),
        TCP6DDescriptor(pen_2[1], v=v, r=0.01),
    ]

    waypoints_move_out_pen_2 = [
        TCP6DDescriptor(pen_2[0], v=v, r=0.01),
    ]

    waypoints_pen_back_2 = [
        TCP6DDescriptor(pen_2[0], v=v, r=0.01),
        TCP6DDescriptor(pen_2[1], v=v),
        TCP6DDescriptor(pen_2[0], v=v, r=0.01)
    ]

    iscoin.robot_control.movel_waypoints(waypoints_to_pen_1)

    # iscoin.gripper.close()
    os.system('pause')
    iscoin.robot_control.movel_waypoints(waypoints_move_out_pen_1)

    iscoin.robot_control.movel_waypoints(waypoints_home)
    
    iscoin.robot_control.movel_waypoints(waypoints_pen_back)

    # iscoin.gripper.open()
    
    iscoin.robot_control.movel_waypoints(waypoints_transition_pen)

    # iscoin.gripper.close()

    iscoin.robot_control.movel_waypoints(waypoints_move_out_pen_2)
    
    iscoin.robot_control.movel_waypoints(waypoints_home)
    
    iscoin.robot_control.movel_waypoints(waypoints_pen_back_2)

    #iscoin.gripper.open()
    
    iscoin.robot_control.movel_waypoints(waypoints_home)

    return True

    '''
    ok = iscoin.robot_control.movel_waypoints(waypoints_to_pen_1)
    if not ok:
        print("Pick approach failed")
        return False
    '''

switch_pen(2,1)