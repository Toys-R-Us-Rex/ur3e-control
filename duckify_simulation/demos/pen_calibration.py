import json

import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from duckify_simulation.duckify_sim import DuckifySim as ISCoin
# from URBasic import ISCoin  # <-- swap this line to use the real robot

# from src.safety import get_reachable
from URBasic import Joint6D, TCP6D
from URBasic.waypoint6d import TCP6DDescriptor
from pen_transition import Measurement

iscoin = ISCoin()

m = Measurement()

def pen_calibration():
    tool_tcp = TCP6D.createFromMetersRadians(0.0, 0.0, m.gripper_length, 0.0, 0.0, 0.0)
    iscoin.robot_control.set_tcp(tool_tcp)

    pos = iscoin.robot_control.get_actual_tcp_pose()

    coor = {
        "x" : pos[0],
        "y" : pos[1],
        "z" : pos[2]
    }

    with open('calib.json', 'w', encoding='utf-8') as f:
        json.dump(coor, f, ensure_ascii=False, indent=4)

    return pos

pen_calibration()

