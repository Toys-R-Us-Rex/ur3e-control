"""
Segment Classes for UR-series Robot Control

MIT License

Copyright (c) 2026 Nathan Antonietti

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

Author:     Nathan Antonietti
Co-Author:  Mariéthoz Cédric, with assistance from Copilot AI (Microsoft)
Course:     HES-SO Valais-Wallis, Engineering Track 304
"""

from dataclasses import dataclass
from enum import Enum, auto
from URBasic import TCP6D, Joint6D

class MotionType(Enum):
    """
    Enum for different types of motion.
    """
    TRAVEL = auto()      # free-space collision-avoidant move
    APPROACH = auto()    # pen-down / pen-up (hover ↔ surface)
    DRAW = auto()        # on-surface stroke


class SideType(Enum):
    """
    Enum for different types of side.
    """
    RIGHT = auto()  # y < 0
    LEFT = auto()   # y > 0

@dataclass
class Segment:
    """
    Base class for all segment types.
    """
    color: int
    side: SideType

@dataclass
class TraceSegment(Segment):
    """
    Segment for trace motion.
    """
    waypoints: list[list] = None # x,y,z,n1,n2,n3

@dataclass
class RunSegment(Segment):
    """
    Segment for run motion.
    """
    motion_type: MotionType = MotionType.DRAW
    waypoints: list[TCP6D] = None  # pre-computed TCP6D list

@dataclass
class TCPSegment(Segment):
    """
    Segment for TCP motion.
    """
    motion_type: MotionType
    v: float             # velocity m/s
    a: float             # acceleration m/s²
    r: float = 0.0       # blend radius
    waypoints: list[TCP6D] = None  # pre-computed TCP6D list

@dataclass
class JointSegment(Segment):
    """
    Segment for joint motion.
    """
    motion_type: MotionType
    v: float             # velocity m/s
    a: float             # acceleration m/s²
    r: float = 0.0       # blend radius
    waypoints: list[Joint6D] = None  # optional pre-computed Joint6D list
    tcp_waypoints: list[TCP6D] = None  # corresponding TCP6D poses