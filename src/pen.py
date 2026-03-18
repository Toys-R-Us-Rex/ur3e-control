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
from enum import Enum, auto

from duckify_simulation.duckify_sim import DuckifySim as ISCoin # (x,y,z)
# from URBasic import ISCoin  # <-- swap this line to use the real robot # (y,x,z)

from math import pi
from URBasic import TCP6D
from URBasic.waypoint6d import TCP6DDescriptor

MINIMAL_DISTANCE = 0.152
LEGNTH_BETWEEN_PENS = 0.05 # This distance comes from the design of the wood support for pen.
FACING_DOWN = (pi, 0, 0) # To maintain the gripper facing down

GRIPPER_LENGTH = 0.101
PEN_LENGTH = 0.128
PEN_POS_0 =  [-0.3, -0.3, MINIMAL_DISTANCE] # Position of pen at index 0

class GripperAction(Enum):
    CLOSE = auto()  # Close gripper
    OPEN  = auto()  # Open gripper

class PenState():
    """
    PenState handles to HandE Gripper pen transitions and state.
    ```
    from src.pen import PenState

    home = home_position()
    ps = PenState(home=home, robot=iscoin)

    ps.run_moves(
        ps.change_pen(target_pen_id=0)
    )

    ps.run_moves(
        ps.change_pen(target_pen_id=1)
    )
    ```
    """

    def __init__(self, home: TCP6D, robot: ISCoin):
        self.home = home
        self.gripper_tcp = TCP6D.createFromMetersRadians(0.0, 0.0, GRIPPER_LENGTH, 0.0, 0.0, 0.0)
        self.tool_tcp = TCP6D.createFromMetersRadians(0.0, 0.0, GRIPPER_LENGTH + PEN_LENGTH, 0.0, 0.0, 0.0)
        self.current_tcp = self.gripper_tcp

        self.active_pen_id = None
        self.robot = robot

    def get_tcp(self):
        return self.current_tcp
    
    def set_tcp(self, tcp: TCP6D):
        self.robot.robot_control.set_tcp(tcp)
        self.current_tcp = tcp

    def get_pen_by_id(self, pen_id: int):
        """
        This function will upgrade the function "get pen" from Nathan Antonietti to be able to go to a pen base on an id we choose:
        
        The id = 0 will be the pen in the support with waypoint and every id > 0 will be at 50mm from the id 0 in Y axis
        """

        print(f"PEN 0 POS: {PEN_POS_0}")

        to_top_of_pen = TCP6D.createFromMetersRadians(
            PEN_POS_0[0],
            PEN_POS_0[1] - LEGNTH_BETWEEN_PENS * pen_id,
            PEN_POS_0[2] + 0.05,
            FACING_DOWN[0],
            FACING_DOWN[1],
            FACING_DOWN[2]
        )

        to_pen = TCP6D.createFromMetersRadians(
            PEN_POS_0[0],
            PEN_POS_0[1] - LEGNTH_BETWEEN_PENS * pen_id,
            PEN_POS_0[2],
            FACING_DOWN[0],
            FACING_DOWN[1],
            FACING_DOWN[2]
        )

        return to_top_of_pen, to_pen

    def return_pen(self):
        move_list = []
        v = 0.2
    
        if not self.active_pen_id:
            return move_list

        top_active_pen, to_active_pen = self.get_pen_by_id(self.active_pen_id)
        waypoints_to_active_pen = [
            TCP6DDescriptor(top_active_pen, v=v),
            TCP6DDescriptor(to_active_pen, v=v),
        ]
        waypoints_move_out_active_pen = [
            TCP6DDescriptor(top_active_pen, v=v),
        ]
 
        move_list.append(waypoints_to_active_pen)
        move_list.append(GripperAction.OPEN)
        move_list.append(waypoints_move_out_active_pen)

        self.active_pen_id = None
        return move_list

    def change_pen(self, target_pen_id: int):
        move_list = []
        v = 0.2
        
        # return pen to support if it's active
        self.return_pen()

        # Get target pen position
        top_target_pen, to_target_pen = self.get_pen_by_id(target_pen_id)

        waypoints_to_target_pen = [
            TCP6DDescriptor(top_target_pen, v=v),
            TCP6DDescriptor(to_target_pen, v=v),
        ]
        
        waypoints_move_out_target_pen = [
            TCP6DDescriptor(top_target_pen, v=v),
        ]

        waypoints_home = [
            TCP6DDescriptor(self.home, v=v),
        ]

        move_list.append(waypoints_to_target_pen)
        move_list.append(GripperAction.CLOSE)
        move_list.append(waypoints_move_out_target_pen)
        move_list.append(waypoints_home)

        # set new active pen
        self.active_pen_id = target_pen_id

        return move_list
    
    def run_moves(self, move_list: list[list[TCP6DDescriptor | GripperAction]]):
        """
        move_list: list[list[TCP6DDescriptor]]
        Usage:
            `run_moves( change_pen(0, 1) )`
            `run_moves( return_pen(1) )`
        """
        for m in move_list:
            print(m)
            if m is GripperAction.CLOSE:
                self.robot.gripper.close()
                self.robot.gripper.waitUntilStopped()
                self.set_tcp(self.tool_tcp)
            elif m is GripperAction.OPEN:
                self.robot.gripper.open()
                self.robot.gripper.waitUntilStopped()
                self.set_tcp(self.gripper_tcp)
            else:
                self.robot.robot_control.movel_waypoints(m, wait=True)
