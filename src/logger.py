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
    def __init__(self, data_path="save_data/", log_file="log.txt"):
        self.log_path = data_path + log_file
        self.data_path = data_path
        if not os.path.exists(data_path):
            os.makedirs(data_path)

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
        self.log(f"Transformation (world => robot):\n" + str(AtoB.T_position) + "\n" + str(AtoB.T_orientation))

    def log_waypoint(self, waypoints):
        s = ""
        for p in waypoints:
            s += str(p.toList())
            s += ", "
        s += "\n"
        self.log(f"Path of the robot (waypoints):\n" + s)

    def log_trace_segment(self, segments: list[TraceSegment]):
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
        s = ""
        for i, seg in enumerate(segments):
            s += f"Segment {i}, {seg.side}, {seg.color}:\n"
            for p in seg.waypoints:
                s += str(p.toList())
                s += ", "
            s += "\n"   
        s += "\n"
        self.log(f"Path of the robot (tcp waypoints):\n" + s)
        pass

    def log_tcp_segment(self, segments: list[JointSegment]):
        s = ""
        for i, seg in enumerate(segments):
            s += f"Segment {i}, {seg.side}, {seg.color}:\n"
            for p in seg.waypoints:
                s += str(p.toList())
                s += ", "
            s += "\n"   
        s += "\n"
        self.log(f"Path of the robot (joint waypoints):\n" + s)
        pass

    def log(self, message: str):
        timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
        entry = f"{timestamp} - {message}\n"
        
        with open(self.log_path, "a") as f:
            f.write(entry)
    
    # ----------------------------------------------------
    #                SAVE / LOAD HELPER
    # ----------------------------------------------------

    def _indexed_file(self, key, index):
        return os.path.join(self.data_path, f"{key}_{index}.pkl")

    def _next_index(self, key):
        files = [f for f in os.listdir(self.data_path) if f.startswith(key + "_")]
        if not files:
            return 0
        indices = [int(f.split("_")[-1].split(".")[0]) for f in files]
        return max(indices) + 1
    
    def save_history(self, key, obj):
        idx = self._next_index(key)
        file_path = self._indexed_file(key, idx)
        with open(file_path, "wb") as f:
            pickle.dump(obj, f)
        self.log(f"Saved history entry {idx} for {key} -> {file_path}")

    def load_history_latest(self, key):
        idx = self._next_index(key) - 1
        if idx < 0:
            self.log(f"No history found for {key}")
            return None
        return self.load_history_index(key, idx)

    def load_history_index(self, key, index):
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


    def save_calibration(self, tcps, tcp_offset, file_path=None):
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

    def load_calibration(self, file_path=None, index=-1):
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


    def save_worldtcp(self, pworld, tcps, file_path=None):
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

    def load_worldtcp(self, file_path=None, index=-1):
        if file_path:
            if not os.path.exists(file_path):
                self.log(f"Worldtcp data file not found {file_path}")
                return None, None

            with open(file_path, "rb") as f:
                data = pickle.load(f)
    
            self.log(f"Loaded worldrcp data from file {file_path}")
        else:
            data = self.load_history_index("worldrcp", index)
        
        pworld = data["pworld"]
        tcps = data["tcps"]
        return pworld, tcps


    def save_transformation(self, obj_to_robot: AtoB, file_path=None):
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

    def load_transformation(self, file_path=None, index=-1) -> AtoB:
        if file_path:
            if not os.path.exists(file_path):
                self.log(f"Transformation data file not found {file_path}")
                return None, None

            with open(file_path, "rb") as f:
                data = pickle.load(f)
    
            self.log(f"Loaded transformation data from file {file_path}")
        else:
            data = self.load_history_index("transformation", index)
    
        T_position = data["T_position"]
        T_normal = data["T_normal"]
        return AtoB(T_position, T_normal)
    


    def save_waypoints(self, waypoints: list[TCP6D|Joint6D], file_path=None):
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

    def load_waypoints(self, file_path=None, index=-1):
        if file_path:
            if not os.path.exists(file_path):
                self.log(f"Waypoints data file not found {file_path}")
                return None, None

            with open(file_path, "rb") as f:
                data = pickle.load(f)
    
            self.log(f"Loaded waypoints data from file {file_path}")
        else:
            data = self.load_history_index("waypoints", index)

        waypoints = data["waypoints"]
        return waypoints


    def save_tcp_segments(self, segments: list[TCPSegment], file_path=None):
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

    def load_tcp_segments(self, file_path=None, index=-1):
        if file_path:
            if not os.path.exists(file_path):
                self.log(f"TCP segments data file not found {file_path}")
                return None, None

            with open(file_path, "rb") as f:
                data = pickle.load(f)
    
            self.log(f"Loaded TCP segments data from file {file_path}")
        else:
            data = self.load_history_index("tcp_segments", index)
            
        segments = data["segments"]
        return segments


    def save_joint_segments(self, segments: list[JointSegment], file_path=None):
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

    def load_joint_segments(self, file_path=None, index=-1):
        if file_path:
            if not os.path.exists(file_path):
                self.log(f"Joint segments data file not found {file_path}")
                return None, None

            with open(file_path, "rb") as f:
                data = pickle.load(f)
    
            segments = data["segments"]
            self.log(f"Loaded Joint segments data from file {file_path}")
        else:
            data = self.load_history_index("joint_segments", index)
            
        segments = data["segments"]
        return segments


    def save_trace_segments(self, segments: list[TraceSegment], file_path=None):
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

    def load_trace_segments(self, file_path=None, index=-1):
        if file_path:
            if not os.path.exists(file_path):
                self.log(f"Trace segments data file not found {file_path}")
                return None, None

            with open(file_path, "rb") as f:
                data = pickle.load(f)
    
            self.log(f"Loaded Trace segments data from file {file_path}")
        else:
            data = self.load_history_index("trace_segments", index)
            
        segments = data["segments"]
        return segments


    def save_run_segments(self, segments: list[RunSegment], file_path=None):
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

    def load_run_segments(self, file_path=None, index=-1):
        if file_path:
            if not os.path.exists(file_path):
                self.log(f"Run segments data file not found {file_path}")
                return None, None

            with open(file_path, "rb") as f:
                data = pickle.load(f)
    
            segments = data["segments"]
            self.log(f"Loaded Run segments data from file {file_path}")
        else:
            data = self.load_history_index("run_segments", index)
            
        segments = data["segments"]
        return segments


class DataStoreForce:
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

                row = [timestamp] + tcp.toList() + wrench + [magnitude]
                writer.writerow(row)
                f.flush()  # ensure data is physically written

                time.sleep(0.01)  # 100 Hz logging
