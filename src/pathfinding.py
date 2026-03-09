'''
MIT License

Copyright (c) 2026 HES-SO Valais-Wallis, Engineering Track 304
'''

import numpy as np
import trimesh

from src.config import SAFE_MARGIN, PUSH_STEP, MAX_DEPTH, MAX_PUSH, SURFACE_MARGIN


def segment_collides(A, B, mesh):
    """Check if segment A->B hits the mesh."""
    direction = B - A
    length = np.linalg.norm(direction)
    if length == 0:
        return False
    origins = np.array([A])
    directions = np.array([direction / length])
    hits = mesh.ray.intersects_any(origins, directions)
    if not hits[0]:
        return False
    # only count hits that are actually within our segment
    locations = mesh.ray.intersects_location(origins, directions)[0]
    if len(locations) == 0:
        return False
    distances = np.linalg.norm(locations - A, axis=1)
    return np.any(distances <= length + SAFE_MARGIN)


def perpendicular_up(A, B):
    """Get a direction perpendicular to A->B that points up."""
    seg = B - A
    up = np.array([0.0, 0.0, 1.0]) # only on z axis
    perp = up - seg * (np.dot(up, seg) / np.dot(seg, seg))
    norm = np.linalg.norm(perp)
    if norm < 1e-12:
        return up
    return perp / norm


def lift_midpoint(A, B, mesh):
    """Push midpoint just outside the mesh surface, then add margin."""
    mid = (A + B) / 2.0
    push_dir = perpendicular_up(A, B)
    total_push = 0.0

    # Push until midpoint is outside the mesh
    while mesh.contains([mid])[0] and total_push < MAX_PUSH:
        mid = mid + push_dir * PUSH_STEP
        total_push += PUSH_STEP

    if total_push >= MAX_PUSH:
        raise RuntimeError("lift_midpoint: could not clear mesh")

    # Add surface margin
    mid = mid + push_dir * SURFACE_MARGIN
    return mid


def find_path(A, B, mesh, depth=0):
    """Recursively split A->B around the mesh."""
    if not segment_collides(A, B, mesh):
        return [A, B]

    if depth >= MAX_DEPTH:
        raise RuntimeError("find_path: max depth reached")

    mid = lift_midpoint(A, B, mesh)
    left = find_path(A, mid, mesh, depth + 1)
    right = find_path(mid, B, mesh, depth + 1)

    # left ends with mid, right starts with mid -> skip duplicate
    return left + right[1:]
