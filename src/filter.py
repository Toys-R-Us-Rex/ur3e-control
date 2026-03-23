"""
Filter stage for processing trace segments.

MIT License

Copyright (c) 2026 Antonietti Nathan

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

Author:     Antonietti Nathan
Co-Author:  Mariéthoz Cédric, with assistance from Copilot AI (Microsoft)
Course:     HES-SO Valais-Wallis, Engineering Track 304
"""

from src.logger import DataStore
from src.segment import TraceSegment, SideType
from src.computation import load_traces
from src.stage import Stage
from src.utils import ask_yes_no

class Filter(Stage):
    def __init__(self, dataStore: DataStore, json_path):
        """
        Initialize the Filter stage.

        Parameters
        ----------
        dataStore : DataStore
            The data store instance.
        json_path : str
            The path to the JSON file containing the traces.
        """
        super().__init__(name="Filter", datastore=dataStore)
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
    
    def fallback(self):
        """
        Fallback method for the filter stage.
        """
        raise RuntimeError("Failed to filter traces.")