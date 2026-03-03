'''
MIT License

Copyright (c) 2026 HES-SO Valais-Wallis, Engineering Track 304
'''

import numpy as np

from src.pathfinding import find_path


def is_reachable(tcp):
    raise NotImplementedError


def have_collision(tcp, robot):
    raise NotImplementedError


def divide_path(tcp1, tcp2):
    raise NotImplementedError


# -- Public function --

def compute_safe_waypoints(A, B, obstacles):
    """Find waypoints [A, ..., B] that go around all obstacles."""
    waypoints = [A, B]

    for mesh in obstacles:
        new_waypoints = []
        for i in range(len(waypoints) - 1):
            segment_path = find_path(waypoints[i], waypoints[i + 1], mesh)
            if new_waypoints:
                new_waypoints.extend(segment_path[1:])  # skip shared point
            else:
                new_waypoints.extend(segment_path)
        waypoints = new_waypoints

    return waypoints
