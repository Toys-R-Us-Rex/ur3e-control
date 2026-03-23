"""
Conversion Stage Module

This module provides functionality for converting trace segments to TCP segments.

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

from URBasic.waypoint6d import TCP6D, Joint6D
from src.logger import DataStore

from src.stage import Stage
from src.utils import ask_yes_no
from src.segment import TCPSegment, MotionType
from src.config import DRAW_A, DRAW_V


class Conversion(Stage):
    """
    A stage for converting trace segments to TCP segments.
    """
    def __init__(self, datastore: DataStore):
        """
        Initialize the Conversion stage.

        Parameters
        ----------
        datastore : DataStore
            The data store instance.
        """
        super().__init__(name="Conversion", datastore=datastore)

    def run(self):
        """
        Run the conversion stage.
        """
        if ask_yes_no("Do you have a conversion already saved? y/n\n"):
            segments = self.ds.load_tcp_segments()
            self.ds.log_tcp_segment(segments)
            return

        if not ask_yes_no("Do you want to  convert the traces? y/n \n"):
            raise RuntimeError("You chose not to convert the traces")

        objtorobot = self.ds.load_transformation()
        traces = self.ds.load_trace_segments()
        segments = []
        for t in traces:
            color = t.color
            side = t.side
            waypoints = t.waypoints
            segments.append(
                TCPSegment(
                    color=color,
                    side=side,
                    
                    waypoints=[TCP6D.createFromMetersRadians( *objtorobot(p)) for p in waypoints],
                    motion_type=MotionType.DRAW,
                    v=DRAW_V,
                    a=DRAW_A
                )
            )
        
        self.ds.save_tcp_segments(segments)
        self.ds.log_tcp_segment(segments)
    
    
    def fallback(self):
        """
        Fallback method for the conversion stage.
        """
        raise RuntimeError("Conversion fallback not implemented yet.")