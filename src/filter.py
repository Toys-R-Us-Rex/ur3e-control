from src.logger import DataStore
from src.segment import TraceSegment, SideType
from src.computation import load_traces

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
        traces, _ = load_traces(self.json_path)
        left_traces = []
        right_traces = []
        for trace in traces:
            path = trace['path']
            ys, color = [pt[1] for pt, _ in path], trace['color']
            avg_y = sum(ys) / len(ys) if ys else 0

            waypoints = [pt[0] + pt[1] for pt in path]
        
            if avg_y >= 0:
                left_traces.append(
                    TraceSegment(waypoints, color, SideType.LEFT)
                )
            else:
                right_traces.append(
                    TraceSegment(waypoints, color, SideType.RIGHT)
                )
            break

        self.ds.save_trace_segment(left_traces+right_traces)