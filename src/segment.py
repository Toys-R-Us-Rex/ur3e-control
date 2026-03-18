from dataclasses import dataclass
from enum import Enum, auto
from URBasic import TCP6D, Joint6D

class MotionType(Enum):
    TRAVEL = auto()      # free-space collision-avoidant move
    APPROACH = auto()    # pen-down / pen-up (hover ↔ surface)
    DRAW = auto()        # on-surface stroke


class SideType(Enum):
    RIGHT = auto()  # y < 0
    LEFT = auto()   # y > 0

@dataclass
class Segment:
    color: int
    side: SideType

@dataclass
class TraceSegment(Segment):
    waypoints: list[list] = None # x,y,z,n1,n2,n3

@dataclass
class RunSegment(Segment):
    motion_type: MotionType = MotionType.DRAW
    waypoints: list[TCP6D] = None  # pre-computed TCP6D list

@dataclass
class TCPSegment(Segment):
    motion_type: MotionType
    v: float             # velocity m/s
    a: float             # acceleration m/s²
    r: float = 0.0       # blend radius
    waypoints: list[TCP6D] = None  # pre-computed TCP6D list

@dataclass
class JointSegment(Segment):
    motion_type: MotionType
    v: float             # velocity m/s
    a: float             # acceleration m/s²
    r: float = 0.0       # blend radius
    waypoints: list[Joint6D] = None  # optional pre-computed Joint6D list
    tcp_waypoints: list[TCP6D] = None  # corresponding TCP6D poses