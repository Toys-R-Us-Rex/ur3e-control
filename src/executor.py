'''
MIT License

Copyright (c) 2026 HES-SO Valais-Wallis, Engineering Track 304
'''

import logging

from src.computation import MotionType

log = logging.getLogger(__name__)


class ExecutionError(RuntimeError):
    """Raised when a movel command fails during drawing plan execution."""


def execute_drawing_plan(robot, segments, home_joints=None):
    """Execute a pre-computed list of Segments by calling movel commands.

    Parameters
    ----------
    robot : robot controller (movel, movel_waypoints, movej).
    segments : list[Segment] — fully pre-computed drawing plan from build_drawing_plan().
    home_joints : Joint6D — if given, movej to home before starting.
    """
    from URBasic import TCP6DDescriptor

    if home_joints is not None:
        robot.movej(home_joints)

    for i, seg in enumerate(segments):
        log.info("Segment %d: %s (%d waypoints, v=%.3f, a=%.3f)",
                 i, seg.motion_type.name, len(seg.waypoints), seg.v, seg.a)

        if seg.motion_type == MotionType.TRAVEL:
            # Skip first waypoint (robot is already there)
            wps = seg.waypoints[1:] if len(seg.waypoints) > 1 else seg.waypoints
            if wps:
                descriptors = [TCP6DDescriptor(wp) for wp in wps]
                robot.movel_waypoints(descriptors)

        elif seg.motion_type == MotionType.APPROACH:
            # Pen-down or pen-up: move to the target (last waypoint)
            target = seg.waypoints[-1]
            robot.movel(target, a=seg.a, v=seg.v)

        elif seg.motion_type == MotionType.DRAW:
            # Draw each surface point sequentially
            for wp in seg.waypoints:
                robot.movel(wp, a=seg.a, v=seg.v)
