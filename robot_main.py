#!/usr/bin/env python3

import time

from src.logger import DataStore
from src.segment import SideType
from src.stage import run_stage

from src.calibration import Calibration
from src.transformation import Transformation
from src.filter import Filter
from src.conversion import Conversion
from src.pathfinding_ import Pathfinding
from src.gazebo import Gazebo
from src.robot import Robot

# Create a new ISCoin object
# UR3e1 IP (closest to window): 10.30.5.158
# UR3e2 IP: 10.30.5.159
ROBOT_IP = "10.30.5.158"

JSON_OBJECT = "duckify_simulation/paths/duck_uv-dot-trace.json"
JSON_CALIBRATION = "duckify_simulation/paths/calibration_socle.json"

def main():
    day = time.strftime("%Y%m%d")
    ds = DataStore(f"save_data_test/{day}/")
    ds.log("Pipeline started")

    run_stage(Calibration(ds, ROBOT_IP), on_error="continue")
    run_stage(Transformation(ds, ROBOT_IP, JSON_CALIBRATION), on_error="fallback")
    run_stage(Filter(ds, JSON_OBJECT), on_error="stop")
    run_stage(Conversion(ds), on_error="stop")
    run_stage(Pathfinding(ds, side=SideType.RIGHT), on_error="stop")
    run_stage(Gazebo(ds), on_error="stop")
    run_stage(Robot(ds,ROBOT_IP), on_error="continue")

    ds.log("Pipeline finished")

if __name__ == "__main__":
    main()