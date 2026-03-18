from URBasic.waypoint6d import TCP6D, Joint6D
from src.logger import DataStore

from src.utils import ask_yes_no
from src.segment import TraceSegment, TCPSegment, MotionType
from src.config import DRAW_A, DRAW_V


class Conversion:
    def __init__(self, datastore: DataStore):
        self.ds = datastore
    
    def run(self):
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
        raise NotImplemented("Conversion fallback not implemnted yet.")