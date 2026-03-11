from URBasic.urScript import UrScript
from duckify_simulation.duckify_sim.robot_control import SimRobotControl

import threading
import time
import csv

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
