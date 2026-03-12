from URBasic.urScript import UrScript
from duckify_simulation.duckify_sim.robot_control import SimRobotControl

import threading
import time
import csv
import numpy as np
import pickle
import os

from URBasic import TCP6D

class LoggingLog:
    def __init__(self, file_path="log.txt", data_path="save_data/"):
        self.file_path = file_path
        self.data_path = data_path

    def log_calibration(self, tcps: list[TCP6D], tcp_offset: TCP6D):
        s = "\n"
        for p in range(len(tcps)):
            s += str(tcps[p])
            s += "\n"
        self.log(f"Calibration measure:\n" + s)
        self.log(f"Calibred offset:" + str(tcp_offset))

    def log_worldtcp(self, pworld: list[TCP6D], tcps: list[TCP6D]):
        s = "\n"
        for p in range(len(pworld)):
            s += "("
            s += str(pworld[p])
            s += " => "
            s += str(tcps[p])
            s += ")\n"
        self.log(f"Object Calibration (world,robot):\n" + s)

    def log_transformation(self, AtoB):
        self.log(f"Transformation (world => robot):\n" + str(AtoB.T) + "\n" + str(AtoB.T_normal))

    def log_waypoint(self, waypoints):
        s = ""
        for p in waypoints:
            s += str(p.toList())
            s += ", "
        s += "\n"
        self.log(f"Path of the robot (waypoints):\n" + s)
        
    def log(self, message: str):
        timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
        entry = f"{timestamp} - {message}\n"
        
        with open(self.file_path, "a") as f:
            f.write(entry)
    

    # ----------------------------------------------------
    #                SAVE / LOAD NUMPY DATA
    # ----------------------------------------------------
    
    
    def save_calibration(self, tcps, tcp_offset, file_path=None):
        if not file_path:
            file_path = self.data_path + "calibration_data.pkl"
        
        # Ensure folder exists
        folder = os.path.dirname(file_path)
        if folder and not os.path.exists(folder):
            self.log(f"Create folder {folder}")
            os.makedirs(folder)

        with open(file_path, "wb") as f:
            pickle.dump({"tcps": tcps, "tcp_offset": tcp_offset}, f)
        self.log(f"Saved calibration data to file {file_path}")

    def load_calibration(self, file_path=None):
        if not file_path:
            file_path = self.data_path + "calibration_data.pkl"
        
        if not os.path.exists(file_path):
            self.log(f"Calibration data file not found {file_path}")
            return None, None

        with open(file_path, "rb") as f:
            data = pickle.load(f)
    
        tcps = data["tcps"]
        tcp_offset = data["tcp_offset"]
        self.log(f"Loaded calibration data from file {file_path}")
        return tcps, tcp_offset


    def save_worldtcp(self, pworld, tcps, file_path=None):
        if not file_path:
            file_path = self.data_path + "worldtcp_data.pkl"
    
        # Ensure folder exists
        folder = os.path.dirname(file_path)
        if folder and not os.path.exists(folder):
            self.log(f"Create folder {folder}")
            os.makedirs(folder)
        
        with open(file_path, "wb") as f:
            pickle.dump({"pworld":pworld, "tcps":tcps}, f)
        self.log(f"Saved worldtcp data to file {file_path}")

    def load_worldtcp(self, file_path=None):
        if not file_path:
            file_path = self.data_path + "worldtcp_data.pkl"

        if not os.path.exists(file_path):
            self.log(f"Worldtcp data file not found {file_path}")
            return None, None

        with open(file_path, "rb") as f:
            data = pickle.load(f)
    
        pworld = data["pworld"]
        tcps = data["tcps"]
        self.log(f"Loaded calibration data from file {file_path}")
        return pworld, tcps


    def save_transformation(self, AtoB, file_path=None):
        if not file_path:
            file_path = self.data_path + "transformation_data.pkl"

        # Ensure folder exists
        folder = os.path.dirname(file_path)
        if folder and not os.path.exists(folder):
            self.log(f"Create folder {folder}")
            os.makedirs(folder)

        T = AtoB.T
        T_normal = AtoB.T_normal
    
        with open(file_path, "wb") as f:
            pickle.dump({"T":T, "T_normal":T_normal}, f)
        self.log(f"Saved transformation data to file {file_path}")

    def load_transformation(self, file_path=None):
        if not file_path:
            file_path = self.data_path + "transformation_data.pkl"

        if not os.path.exists(file_path):
            self.log(f"Transformation data file not found {file_path}")
            return None, None

        with open(file_path, "rb") as f:
            data = pickle.load(f)

        T_obj = data["T"]
        T_normal = data["T_normal"]
        self.log(f"Loaded transformation data from file {file_path}")
        return T_obj, T_normal
    

    def save_waypoints(self, waypoints, file_path=None):
        if not file_path:
            file_path = self.data_path + "waypoints_data.pkl"

        # Ensure folder exists
        folder = os.path.dirname(file_path)
        if folder and not os.path.exists(folder):
            self.log(f"Create folder {folder}")
            os.makedirs(folder)

        with open(file_path, "wb") as f:
            pickle.dump({"waypoints":waypoints}, f)
        self.log(f"Saved waypoints data to file {file_path}")

    def load_waypoints(self, file_path=None):
        if not file_path:
            file_path = self.data_path + "waypoints_data.pkl"

        if not os.path.exists(file_path):
            self.log(f"Waypoints data file not found {file_path}")
            return None, None

        with open(file_path, "rb") as f:
            data = pickle.load(f)

        waypoints = data["waypoints"]
        self.log(f"Loaded waypoints data from file {file_path}")
        return waypoints



class LoggingForce:
    def __init__(self, robot: UrScript|SimRobotControl):
        self.robot = robot
        self.stop_event = None
        self.t = None
        self.current_file_path = None

    def start_logging(self, file_path="force_log.csv"):
        # Prevent starting twice
        if self.t is not None and self.t.is_alive():
            raise RuntimeError(
                f"Logging already running, writing to: {self.current_file_path}"
            )

        # Create a new stop event
        self.stop_event = threading.Event()
        self.current_file_path = file_path

        # Start thread
        self.t = threading.Thread(
            target=self._log_force,
            args=(self.robot, self.stop_event, file_path),
            daemon=True
        )
        self.t.start()

    def stop_logging(self):
        if self.stop_event is None:
            return  # Nothing to stop

        self.stop_event.set()
        if self.t is not None:
            self.t.join()

        # Reset state
        self.t = None
        self.stop_event = None
        self.current_file_path = None

    def _log_force(self, robot, stop_event, file_path):
        time_start = time.time()

        # Open file once and keep it open
        with open(file_path, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["time", "Fx", "Fy", "Fz", "Tx", "Ty", "Tz", "Force"])

            while not stop_event.is_set():
                tcp = robot.get_actual_tcp_pose(wait=False)
                wrench = robot.get_tcp_force(wait=False)
                magnitude = robot.force(wait=False)
                timestamp = time.time() - time_start

                row = [timestamp] + tcp.toList() + wrench.toList() + [magnitude]
                writer.writerow(row)
                f.flush()  # ensure data is physically written

                time.sleep(0.01)  # 100 Hz logging
