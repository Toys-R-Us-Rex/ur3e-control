#!/usr/bin/env python3

import time

from src.logger import DataStore
from src.calibration import Calibration
from src.segment import SideType
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
JSON_CALIBRATION = "duckify_simulation/paths/socle.json"



# ---------------------------------------------------------------------------
# Helper
# ---------------------------------------------------------------------------
def run_stage(stage_name, stage_obj, datastore, on_error="stop"):
    datastore.log(f"Starting stage: {stage_name}")

    try:
        stage_obj.run()
        datastore.log(f"Stage completed: {stage_name}")

    except Exception as e:
        datastore.log(f"ERROR in {stage_name}: {e}")

        if on_error == "continue":
            print(f"Continuing despite error in {stage_name}:\n{e}")
            datastore.log(f"Continuing despite error in {stage_name}")
            return

        if on_error == "fallback":
            print(f"Attempting fallback for {stage_name}:\n{e}")
            datastore.log(f"Attempting fallback for {stage_name}")
            stage_obj.fallback()
            return

        if on_error == "stop":
            datastore.log(f"Stopping pipeline due to error in {stage_name}")
            raise

def main():
    day = time.strftime("%Y%m%d")
    ds = DataStore(f"save_data_test/{day}/")
    ds.log("Pipeline started")

    run_stage("Calibration TCP", Calibration(ds, ROBOT_IP), ds, on_error="continue")
    run_stage("ComputeTransformation", Transformation(ds, ROBOT_IP, JSON_CALIBRATION), ds, on_error="fallback")
    run_stage("Filter", Filter(ds, JSON_OBJECT), ds, on_error="stop")
    run_stage("Conversion", Conversion(ds), ds, on_error="stop")
    run_stage("Pathfinding", Pathfinding(ds, side=SideType.RIGHT), ds, on_error="stop")
    run_stage("Gazebo", Gazebo(ds), ds, on_error="stop")
    run_stage("Run Robot", Robot(ds,ROBOT_IP), ds, on_error="continue")

    ds.log("Pipeline finished")

if __name__ == "__main__":
    main()