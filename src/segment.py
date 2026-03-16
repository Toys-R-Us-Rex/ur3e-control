from dataclasses import dataclass
from enum import Enum, auto
from URBasic import TCP6D, Joint6D

class MotionType(Enum):
    TRAVEL = auto()      # free-space collision-avoidant move
    APPROACH = auto()    # pen-down / pen-up (hover ↔ surface)
    DRAW = auto()        # on-surface stroke


class SideType(Enum):
    RIGHT = auto()
    LEFT = auto()

class PointNormal():
    point: list  # position in list of dimension
    normal: list # normal in list of dimension

@dataclass
class Segment:
    color: int
    side: SideType

@dataclass
class TraceSegment(Segment):
    waypoints: list[list] | None = None  # optional pre-computed Joint6D list

@dataclass
class TCPSegment(Segment):
    motion_type: MotionType
    v: float             # velocity m/s
    a: float             # acceleration m/s²
    r: float = 0.0       # blend radius
    waypoints: list[TCP6D] | None = None  # optional pre-computed TCP6D list

@dataclass
class JointSegment(Segment):
    motion_type: MotionType
    v: float             # velocity m/s
    a: float             # acceleration m/s²
    r: float = 0.0       # blend radius
    waypoints: list[Joint6D] | None = None  # optional pre-computed Joint6D list