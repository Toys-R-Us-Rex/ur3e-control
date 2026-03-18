from src.logger import DataStore
from src.segment import TraceSegment, SideType
from src.computation import load_traces
from src.utils import ask_yes_no

class Filter:
    def __init__(self, dataStore: DataStore, json_path):
        self.ds = dataStore
        self.json_path = json_path

    def run(self):
        """
        Splits traces into left and right sides based on average Y-coordinate of points.
        Assumes traces have 'path' as list of [point, normal], where point is [x, y, z].
        Adjust the axis/threshold if the duck is oriented differently (e.g., use X or Z).
        """
        if ask_yes_no("Do you already extract the traces? y/n \n"):
            s = self.ds.load_trace_segments()
            self.ds.log_trace_segment(s)
            return
        
        traces, _ = load_traces(self.json_path)
        left_traces = []
        right_traces = []
        for trace in traces:
            path = trace['path']
            ys, color = [pt[1] for pt, _ in path], trace['color']
            avg_y = sum(ys) / len(ys) if ys else 0

            waypoints = [pt + pn for pt, pn in path]
        
            if avg_y >= 0:
                left_traces.append(
                    TraceSegment(color, SideType.LEFT, waypoints)
                )
            else:
                right_traces.append(
                    TraceSegment(color, SideType.RIGHT, waypoints)
                )
                
        
        s = left_traces+right_traces
        self.ds.save_trace_segments(s)
        self.ds.log_trace_segment(s)