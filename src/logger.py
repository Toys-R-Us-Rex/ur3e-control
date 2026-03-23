"""

MIT License

Copyright (c) 2026 Mariéthoz Cédric

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

Author:     Mariéthoz Cédric, with assistance from Copilot AI (Microsoft)
Course:     HES-SO Valais-Wallis, Engineering Track 304

"""

import threading
import time
import csv
import pickle
import os

from src.segment import *
from src.utils import AtoB

from URBasic import TCP6D, Joint6D

from URBasic.urScript import UrScript
from duckify_simulation.duckify_sim.robot_control import SimRobotControl

class DataStore:
    """
    A class for storing and managing data logs.
    """
    def __init__(self, data_path: str="save_data/", log_file: str="log.txt"):
        """
        Initialize the DataStore.

        Parameters
        ----------
        data_path : str, optional
            The path to the directory where data will be stored.
        log_file : str, optional
            The name of the log file.
        """
        self.log_path = data_path + log_file
        self.data_path = data_path
        if not os.path.exists(data_path):
            os.makedirs(data_path)

    def log_calibration(self, tcps: list[TCP6D], tcp_offset: TCP6D):
        """
        Log the calibration measurements.
        
        Parameters
        ----------
        tcps : list[TCP6D]
            List of TCP points.
        tcp_offset : TCP6D
            The calibrated offset.
        """
        s = "\n"
        for p in range(len(tcps)):
            s += str(tcps[p])
            s += "\n"
        self.log(f"Calibration measure:\n" + s)
        self.log(f"Calibred offset:" + str(tcp_offset))

    def log_worldtcp(self, pworld: list[TCP6D], tcps: list[TCP6D]):
        """
        Log the world TCP points.
        
        Parameters
        ----------
        pworld : list[TCP6D]
            List of world TCP points.
        tcps : list[TCP6D]
            List of robot TCP points.
        """
        s = "\n"
        for p in range(len(pworld)):
            s += "("
            s += str(pworld[p])
            s += " => "
            s += str(tcps[p])
            s += ")\n"
        self.log(f"Object Calibration (world,robot):\n" + s)

    def log_transformation(self, AtoB: AtoB):
        """
        Log the transformation matrix.
        
        Parameters
        ----------
        AtoB : AtoB
            The transformation matrix.
        """
        self.log(f"Transformation (world => robot):\n" + str(AtoB.T_position) + "\n" + str(AtoB.T_orientation))

    def log_waypoint(self, waypoints: list[TCP6D]):
        """
        Log the waypoints.
        
        Parameters
        ----------
        waypoints : list[TCP6D]
            List of waypoints.
        """
        s = ""
        for p in waypoints:
            s += str(p.toList())
            s += ", "
        s += "\n"
        self.log(f"Path of the robot (waypoints):\n" + s)

    def log_trace_segment(self, segments: list[TraceSegment]):
        """
        Log the trace segments.
        
        Parameters
        ----------
        segments : list[TraceSegment]
            List of trace segments.
        """
        s = ""
        for i, seg in enumerate(segments):
            s += f"Segment {i}, {seg.side}, {seg.color}:\n"
            for p in seg.waypoints:
                s += str(p)
                s += ", "
            s += "\n"   
        s += "\n"
        self.log(f"Path of the robot (traces):\n" + s)
        pass
    
    def log_tcp_segment(self, segments: list[TCPSegment]):
        """
        Log the TCP segments.
        
        Parameters
        ----------
        segments : list[TCPSegment]
            List of TCP segments.
        """
        s = ""
        for i, seg in enumerate(segments):
            s += f"Segment {i}, {seg.side}, {seg.color}:\n"
            for p in seg.waypoints:
                s += str(p)
                s += ", "
            s += "\n"   
        s += "\n"
        self.log(f"Path of the robot (tcp waypoints):\n" + s)
        pass

    def log_joint_segment(self, segments: list[JointSegment]):
        """
        Log the joint segments.
        
        Parameters
        ----------
        segments : list[JointSegment]
            List of joint segments.
        """
        s = ""
        for i, seg in enumerate(segments):
            s += f"Segment {i}, {seg.side}, {seg.color}:\n"
            for p in seg.waypoints:
                s += str(p)
                s += ", "
            s += "\n"   
        s += "\n"
        self.log(f"Path of the robot (joint waypoints):\n" + s)
        pass

    def log(self, message: str):
        """
        Log a message to the log file.
        
        Parameters
        ----------
        message : str
            The message to log.
        """
        timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
        entry = f"{timestamp} - {message}\n"
        
        with open(self.log_path, "a") as f:
            f.write(entry)
    
    # ----------------------------------------------------
    #                SAVE / LOAD HELPER
    # ----------------------------------------------------

    def _indexed_file(self, key: str, index: int):
        """
        Get the file path for a indexed file.
        
        Parameters
        ----------
        key : str
            The key for the file.
        index : int
            The index for the file.
            
        Returns
        -------
        str
            The file path.
        """
        return os.path.join(self.data_path, f"{key}_{index}.pkl")

    def _next_index(self, key: str):
        """
        Get the next available index for a given key.
        
        Parameters
        ----------
        key : str
            The key for the file.
            
        Returns
        -------
        int
            The next available index.
        """
        files = [f for f in os.listdir(self.data_path) if f.startswith(key + "_")]
        if not files:
            return 0
        indices = [int(f.split("_")[-1].split(".")[0]) for f in files if f.split("_")[-1].split(".")[0].isdigit()]
        return max(indices) + 1
    
    def save_history(self, key: str, obj: dict):
        """
        Save a history entry.

        Parameters
        ----------
        key : str
            The key for the file.
        obj : dict
            The object to save.
        """
        idx = self._next_index(key)
        file_path = self._indexed_file(key, idx)
        with open(file_path, "wb") as f:
            pickle.dump(obj, f)
        self.log(f"Saved history entry {idx} for {key} -> {file_path}")

    def load_history_latest(self, key: str) -> dict|None:
        """
        Load the latest history entry for a given key.

        Parameters
        ----------
        key : str
            The key for the file.

        Returns
        -------
        dict or None
            The loaded object or None if not found.
        """
        idx = self._next_index(key) - 1
        if idx < 0:
            self.log(f"No history found for {key}")
            return None
        return self.load_history_index(key, idx)

    def load_history_index(self, key: str, index: int) -> dict|None:
        """
        Load a history entry for a given key and index.

        Parameters
        ----------
        key : str
            The key for the file.
        index : int
            The index for the file.

        Returns
        -------
        dict or None
            The loaded object or None if not found.
        """
        if index == -1:
            return self.load_history_latest(key)
        file_path = self._indexed_file(key, index)
        if not os.path.exists(file_path):
            self.log(f"History file not found: {file_path}")
            return None
        with open(file_path, "rb") as f:
            return pickle.load(f)

    # ----------------------------------------------------
    #                SAVE / LOAD DATA
    # ----------------------------------------------------


    def save_calibration(self, tcps: list[float], tcp_offset: float, file_path: str=None):
        """
        Save calibration data.

        Parameters
        ----------
        tcps : list[float]
            The TCP positions.
        tcp_offset : float
            The TCP offset.
        file_path : str, optional
            The file path to save the data to.
        """
        data = {"tcps": tcps, "tcp_offset": tcp_offset}
        if file_path:
            # Ensure folder exists
            folder = os.path.dirname(file_path)
            if folder and not os.path.exists(folder):
                self.log(f"Create folder {folder}")
                os.makedirs(folder)

            with open(file_path, "wb") as f:
                pickle.dump(data, f)
            self.log(f"Saved calibration data to file {file_path}")
        
        else:
            self.save_history("calibration", data)

    def load_calibration(self, file_path: str=None, index: int=-1) -> tuple[list[float], float]|tuple[None, None]:
        """
        Load calibration data.

        Parameters
        ----------
        file_path : str, optional
            The file path to load the data from.
        index : int, optional
            The index of the history entry to load.

        Returns
        -------
        tuple[list[float], float] or tuple[None, None]
            The loaded TCP positions and offset, or (None, None) if not found.
        """
        if file_path:            
            if not os.path.exists(file_path):
                self.log(f"Calibration data file not found {file_path}")
                return None, None

            with open(file_path, "rb") as f:
                data = pickle.load(f)
        
            self.log(f"Loaded calibration data from file {file_path}")
        else:
            data = self.load_history_index("calibration", index)

        tcps = data["tcps"]
        tcp_offset = data["tcp_offset"]
        return tcps, tcp_offset


    def save_worldtcp(self, pworld: list[float], tcps: list[float], file_path: str=None):
        """
        Save worldtcp data.

        Parameters
        ----------
        pworld : list[float]
            The world positions.
        tcps : list[float]
            The TCP positions.
        file_path : str, optional
            The file path to save the data to.
        """
        data = {"pworld": pworld, "tcps": tcps}
        if file_path:
            # Ensure folder exists
            folder = os.path.dirname(file_path)
            if folder and not os.path.exists(folder):
                self.log(f"Create folder {folder}")
                os.makedirs(folder)

            with open(file_path, "wb") as f:
                pickle.dump(data, f)
            self.log(f"Saved worldtcp data to file {file_path}")
        
        else:
            self.save_history("worldtcp", data)

    def load_worldtcp(self, file_path: str=None, index: int=-1) -> tuple[list[float], list[float]]|tuple[None, None]:
        """
        Load worldtcp data.

        Parameters
        ----------
        file_path : str, optional
            The file path to load the data from.
        index : int, optional
            The index of the history entry to load.

        Returns
        -------
        tuple[list[float], list[float]] or tuple[None, None]
            The loaded world positions and TCP positions, or (None, None) if not found.
        """
        if file_path:

            if not os.path.exists(file_path):
                self.log(f"Worldtcp data file not found {file_path}")
                return None, None

            with open(file_path, "rb") as f:
                data = pickle.load(f)
    
            self.log(f"Loaded worldtcp data from file {file_path}")
        else:
            data = self.load_history_index("worldtcp", index)
        pworld = data["pworld"]
        tcps = data["tcps"]
        return pworld, tcps


    def save_transformation(self, obj_to_robot: AtoB, file_path: str=None):
        """
        Save transformation data.

        Parameters
        ----------
        obj_to_robot : AtoB
            The transformation object.
        file_path : str, optional
            The file path to save the data to.
        """
        data = {"T_position": obj_to_robot.T_position, "T_normal": obj_to_robot.T_orientation}
        if file_path:
            # Ensure folder exists
            folder = os.path.dirname(file_path)
            if folder and not os.path.exists(folder):
                self.log(f"Create folder {folder}")
                os.makedirs(folder)

            with open(file_path, "wb") as f:
                pickle.dump(data, f)
            self.log(f"Saved transformation data to file {file_path}")
        
        else:
            self.save_history("transformation", data)

    def load_transformation(self, file_path: str=None, index: int=-1) -> AtoB|None:
        """
        Load transformation data.

        Parameters
        ----------
        file_path : str, optional
            The file path to load the data from.
        index : int, optional
            The index of the history entry to load.

        Returns
        -------
        AtoB
            The loaded transformation object.
        """
        if file_path:
            if not os.path.exists(file_path):
                self.log(f"Transformation data file not found {file_path}")
                return None

            with open(file_path, "rb") as f:
                data = pickle.load(f)
    
            self.log(f"Loaded transformation data from file {file_path}")
        else:
            data = self.load_history_index("transformation", index)
    
        T_position = data["T_position"]
        T_normal = data["T_normal"]
        return AtoB(T_position, T_normal)
    

    def save_waypoints(self, waypoints: list[TCP6D|Joint6D], file_path: str=None):
        """
        Save waypoints data.

        Parameters
        ----------
        waypoints : list[TCP6D|Joint6D]
            The list of waypoints to save.
        file_path : str, optional
            The file path to save the data to.
        """
        data = {"waypoints": waypoints}
        if file_path:
            # Ensure folder exists
            folder = os.path.dirname(file_path)
            if folder and not os.path.exists(folder):
                self.log(f"Create folder {folder}")
                os.makedirs(folder)

            with open(file_path, "wb") as f:
                pickle.dump(data, f)
            self.log(f"Saved waypoints data to file {file_path}")
        
        else:
            self.save_history("waypoints", data)

    def load_waypoints(self, file_path: str=None, index: int=-1) -> list[TCP6D|Joint6D]|None:
        """
        Load waypoints data.

        Parameters
        ----------
        file_path : str, optional
            The file path to load the data from.
        index : int, optional
            The index of the history entry to load.

        Returns
        -------
        list[TCP6D|Joint6D]
            The loaded waypoints.
        """
        if file_path:
            if not os.path.exists(file_path):
                self.log(f"Waypoints data file not found {file_path}")
                return None

            with open(file_path, "rb") as f:
                data = pickle.load(f)
    
            self.log(f"Loaded waypoints data from file {file_path}")
        else:
            data = self.load_history_index("waypoints", index)

        waypoints = data["waypoints"]
        return waypoints


    def save_tcp_segments(self, segments: list[TCPSegment], file_path: str=None):
        """
        Save TCP segments data.

        Parameters
        ----------
        segments : list[TCPSegment]
            The list of TCP segments to save.
        file_path : str, optional
            The file path to save the data to.
        """
        data = {"segments": segments}
        if file_path:
            # Ensure folder exists
            folder = os.path.dirname(file_path)
            if folder and not os.path.exists(folder):
                self.log(f"Create folder {folder}")
                os.makedirs(folder)

            with open(file_path, "wb") as f:
                pickle.dump(data, f)
            self.log(f"Saved TCP segments data to file {file_path}")
        
        else:
            self.save_history("tcp_segments", data)

    def load_tcp_segments(self, file_path: str=None, index: int=-1) -> list[TCPSegment]|None:
        """
        Load TCP segments data.

        Parameters
        ----------
        file_path : str, optional
            The file path to load the data from.
        index : int, optional
            The index of the history entry to load.

        Returns
        -------
        list[TCPSegment]
            The loaded TCP segments.
        """
        if file_path:
            if not os.path.exists(file_path):
                self.log(f"TCP segments data file not found {file_path}")
                return None

            with open(file_path, "rb") as f:
                data = pickle.load(f)
    
            self.log(f"Loaded TCP segments data from file {file_path}")
        else:
            data = self.load_history_index("tcp_segments", index)
            
        segments = data["segments"]
        return segments


    def save_joint_segments(self, segments: list[JointSegment], file_path: str=None):
        """
        Save Joint segments data.

        Parameters
        ----------
        segments : list[JointSegment]
            The list of Joint segments to save.
        file_path : str, optional
            The file path to save the data to.
        """
        data = {"segments": segments}
        if file_path:
            # Ensure folder exists
            folder = os.path.dirname(file_path)
            if folder and not os.path.exists(folder):
                self.log(f"Create folder {folder}")
                os.makedirs(folder)

            with open(file_path, "wb") as f:
                pickle.dump(data, f)
            self.log(f"Saved Joint segments data to file {file_path}")
        
        else:
            self.save_history("joint_segments", data)

    def load_joint_segments(self, file_path: str=None, index: int=-1) -> list[JointSegment]|None:
        """
        Load Joint segments data.

        Parameters
        ----------
        file_path : str, optional
            The file path to load the data from.
        index : int, optional
            The index of the history entry to load.

        Returns
        -------
        list[JointSegment]
            The loaded Joint segments.
        """
        if file_path:
            if not os.path.exists(file_path):
                self.log(f"Joint segments data file not found {file_path}")
                return None

            with open(file_path, "rb") as f:
                data = pickle.load(f)
    
            segments = data["segments"]
            self.log(f"Loaded Joint segments data from file {file_path}")
        else:
            data = self.load_history_index("joint_segments", index)
            
        segments = data["segments"]
        return segments


    def save_trace_segments(self, segments: list[TraceSegment], file_path: str=None):
        """
        Save Trace segments data.

        Parameters
        ----------
        segments : list[TraceSegment]
            The list of Trace segments to save.
        file_path : str, optional
            The file path to save the data to.
        """
        data = {"segments": segments}
        if file_path:
            # Ensure folder exists
            folder = os.path.dirname(file_path)
            if folder and not os.path.exists(folder):
                self.log(f"Create folder {folder}")
                os.makedirs(folder)

            with open(file_path, "wb") as f:
                pickle.dump(data, f)
            self.log(f"Saved Trace segments data to file {file_path}")
        
        else:
            self.save_history("trace_segments", data)

    def load_trace_segments(self, file_path: str=None, index: int=-1) -> list[TraceSegment]|None:
        """
        Load Trace segments data.

        Parameters
        ----------
        file_path : str, optional
            The file path to load the data from.
        index : int, optional
            The index of the history entry to load.

        Returns
        -------
        list[TraceSegment]
            The loaded Trace segments.
        """
        if file_path:
            if not os.path.exists(file_path):
                self.log(f"Trace segments data file not found {file_path}")
                return None

            with open(file_path, "rb") as f:
                data = pickle.load(f)
    
            self.log(f"Loaded Trace segments data from file {file_path}")
        else:
            data = self.load_history_index("trace_segments", index)
            
        segments = data["segments"]
        return segments


    def save_run_segments(self, segments: list[RunSegment], file_path: str=None):
        """
        Save Run segments data.

        Parameters
        ----------
        segments : list[RunSegment]
            The list of Run segments to save.
        file_path : str, optional
            The file path to save the data to.
        """
        data = {"segments": segments}
        if file_path:
            # Ensure folder exists
            folder = os.path.dirname(file_path)
            if folder and not os.path.exists(folder):
                self.log(f"Create folder {folder}")
                os.makedirs(folder)

            with open(file_path, "wb") as f:
                pickle.dump(data, f)
            self.log(f"Saved Run segments data to file {file_path}")
        
        else:
            self.save_history("run_segments", data)

    def load_run_segments(self, file_path: str=None, index: int=-1) -> list[RunSegment]|None:
        """
        Load Run segments data.

        Parameters
        ----------
        file_path : str, optional
            The file path to load the data from.
        index : int, optional
            The index of the history entry to load.

        Returns
        -------
        list[RunSegment]
            The loaded Run segments.
        """
        if file_path:
            if not os.path.exists(file_path):
                self.log(f"Run segments data file not found {file_path}")
                return None

            with open(file_path, "rb") as f:
                data = pickle.load(f)
    
            segments = data["segments"]
            self.log(f"Loaded Run segments data from file {file_path}")
        else:
            data = self.load_history_index("run_segments", index)
            
        segments = data["segments"]
        return segments


class DataStoreForce:
    """
    A class for managing force logging data.
    """
    def __init__(self, robot: UrScript|SimRobotControl):
        """
        Initialize the DataStoreForce.

        Parameters
        ----------
        robot : UrScript | SimRobotControl
            The robot instance to log force data for.
        """
        self.robot = robot
        self.stop_event = None
        self.t = None
        self.current_file_path = None

    def start_logging(self, file_path: str="force_log.csv"):
        """
        Start logging force data.

        Parameters
        ----------
        file_path : str, optional
            The file path to save the force data to.
        """
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
        """
        Stop logging force data.
        """
        if self.stop_event is None:
            return  # Nothing to stop

        self.stop_event.set()
        if self.t is not None:
            self.t.join()

        # Reset state
        self.t = None
        self.stop_event = None
        self.current_file_path = None

    def _log_force(self, robot: UrScript | SimRobotControl, stop_event: threading.Event, file_path: str):
        """
        Log force data.
        
        Parameters
        ----------
        robot : UrScript | SimRobotControl
            The robot instance to log force data for.
        stop_event : threading.Event
            The event to signal when to stop logging.
        file_path : str
            The file path to save the force data to.
        """
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

                row = [timestamp] + tcp.toList() + wrench + [magnitude]
                writer.writerow(row)
                f.flush()  # ensure data is physically written

                time.sleep(0.01)  # 100 Hz logging
