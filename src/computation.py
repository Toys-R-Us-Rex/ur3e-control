'''
MIT License

Copyright (c) 2026 HES-SO Valais-Wallis, Engineering Track 304
'''

import logging

import numpy as np

from segment import Segment, MotionType
from src.config import (
    DRAW_V, DRAW_A, APPROACH_V, APPROACH_A, TRAVEL_V, TRAVEL_A,
    HOVER_OFFSET,
)
from src.utils import obj_to_stl, tcp_trans

log = logging.getLogger(__name__)

def compute_draw_motion(trace, obj2robot, hover_offset=None, max_pts=None):
    from URBasic import TCP6D

    if hover_offset is None:
        hover_offset = HOVER_OFFSET

    path = trace["path"]
    if max_pts is not None:
        path = path[:max_pts]

    motion = []

    for i, entry in enumerate(path):
        pt, normal = entry[0], entry[1]
        pt_stl = obj_to_stl(np.array(pt))
        normal_stl = obj_to_stl(np.array(normal))
        p_w = (*pt_stl, *normal_stl)
        x, y, z, rx, ry, rz = obj2robot(p_w)

        # Hover entry before first surface point
        if i == 0:
            x_t, y_t, z_t, _, _, _ = tcp_trans(
                [x, y, z, rx, ry, rz], hover_offset,
            )
            motion.append(
                TCP6D.createFromMetersRadians(x_t, y_t, z_t, rx, ry, rz),
            )

        # Surface point
        motion.append(TCP6D.createFromMetersRadians(x, y, z, rx, ry, rz))

        # Hover exit after last surface point
        if i == len(path) - 1:
            x_t, y_t, z_t, _, _, _ = tcp_trans(
                [x, y, z, rx, ry, rz], hover_offset,
            )
            motion.append(
                TCP6D.createFromMetersRadians(x_t, y_t, z_t, rx, ry, rz),
            )

    return motion


def compute_positioning_motion(robot, checker, start_tcp, end_tcp):
    from src.pathfinding import find_path

    waypoints = find_path(robot, checker, start_tcp, end_tcp)

    ok, fail_idx, reason, _joint_traj = checker.validate_path(
        robot, waypoints, orientation_search=True,
    )
    if not ok:
        raise RuntimeError(
            f"compute_positioning_motion: path validation failed at sample "
            f"{fail_idx}: {reason}"
        )

    return waypoints, _joint_traj


# ---------------------------------------------------------------------------
# Helpers for resilient drawing pipeline
# ---------------------------------------------------------------------------

def _hover_tcp(surface_tcp, hover_offset=None):
    """Compute a hover TCP above a surface point using tcp_trans."""
    from URBasic import TCP6D

    if hover_offset is None:
        hover_offset = HOVER_OFFSET

    tcp_list = [surface_tcp.x, surface_tcp.y, surface_tcp.z,
                surface_tcp.rx, surface_tcp.ry, surface_tcp.rz]
    x_t, y_t, z_t, _, _, _ = tcp_trans(tcp_list, hover_offset)
    return TCP6D.createFromMetersRadians(
        x_t, y_t, z_t, surface_tcp.rx, surface_tcp.ry, surface_tcp.rz,
    )


def _validate_surface_points(checker, robot, surface_tcps, qnear=None):
    """Validate each surface TCP with check_obstacle=False.

    Returns (valid_checklist, reasons, joint_solutions) where
    valid_checklist[i] is True if surface_tcps[i] passed validation.
    Chains qnear through valid points for IK consistency.
    """
    valid_checklist = []
    reasons = []
    joint_solutions = []

    for tcp in surface_tcps:
        ok, q, reason, _ = checker.validate_tcp(
            robot, tcp, qnear=qnear, check_obstacle=False,
            orientation_search=True,
        )
        valid_checklist.append(ok)
        reasons.append(reason)
        joint_solutions.append(q)
        if ok:
            qnear = q

    return valid_checklist, reasons, joint_solutions


def _split_into_runs(valid_checklist):
    """Convert boolean checklist to list of (start, end) index pairs of
    consecutive True values.

    E.g. [T, T, F, T, T, T] → [(0, 1), (3, 5)]
    """
    runs = []
    start = None
    for i, v in enumerate(valid_checklist):
        if v and start is None:
            start = i
        elif not v and start is not None:
            runs.append((start, i - 1))
            start = None
    if start is not None:
        runs.append((start, len(valid_checklist) - 1))
    return runs


def build_drawing_plan(robot, checker, traces, obj2robot,
                       start_tcp=None, home_joints=None,
                       hover_offset=None, max_pts=None):

    segments = []
    skip_report = []
    current_tcp = start_tcp

    if current_tcp is None and home_joints is not None:
        current_tcp = robot.get_fk(home_joints)

    for trace_idx, trace in enumerate(traces):
        draw_motion = compute_draw_motion(
            trace, obj2robot, hover_offset=hover_offset, max_pts=max_pts,
        )
        if not draw_motion:
            continue

        # Extract surface points (strip hover entry/exit from draw_motion)
        surface_pts = draw_motion[1:-1]
        if not surface_pts:
            continue

        # Per-point validation (no obstacle collision for surface points)
        valid_checklist, reasons, _ = _validate_surface_points(
            checker, robot, surface_pts,
        )

        # Build skip report for this trace
        skipped_indices = [i for i, ok in enumerate(valid_checklist) if not ok]
        skipped_reasons = [reasons[i] for i in skipped_indices]
        report = {
            "trace": trace_idx,
            "skipped_indices": skipped_indices,
            "reasons": skipped_reasons,
            "original_count": len(surface_pts),
            "drawn_count": sum(valid_checklist),
        }
        skip_report.append(report)

        if skipped_indices:
            log.warning(
                "Trace %d: skipping %d/%d points: %s",
                trace_idx, len(skipped_indices), len(surface_pts),
                skipped_reasons,
            )

        # Split into consecutive valid runs
        runs = _split_into_runs(valid_checklist)
        if not runs:
            log.warning("Trace %d: no valid points, skipping entire trace",
                        trace_idx)
            continue

        for run_start, run_end in runs:
            run_surface = surface_pts[run_start:run_end + 1]

            # Compute hover entry/exit for this run
            h_entry = _hover_tcp(run_surface[0], hover_offset)
            h_exit = _hover_tcp(run_surface[-1], hover_offset)

            # Validate hover points (in free space, check obstacles)
            ok, _, reason, _ = checker.validate_tcp(
                robot, h_entry, check_obstacle=True, orientation_search=True,
            )
            if not ok:
                log.warning(
                    "Trace %d run (%d-%d): hover entry invalid (%s), skip",
                    trace_idx, run_start, run_end, reason,
                )
                continue
            ok, _, reason, _ = checker.validate_tcp(
                robot, h_exit, check_obstacle=True, orientation_search=True,
            )
            if not ok:
                log.warning(
                    "Trace %d run (%d-%d): hover exit invalid (%s), skip",
                    trace_idx, run_start, run_end, reason,
                )
                continue

            # TRAVEL to hover entry
            if current_tcp is not None:
                travel_wps, _ = compute_positioning_motion(
                    robot, checker, current_tcp, h_entry,
                )
                segments.append(Segment(
                    motion_type=MotionType.TRAVEL,
                    waypoints=travel_wps,
                    v=TRAVEL_V,
                    a=TRAVEL_A,
                ))

            # APPROACH pen-down
            segments.append(Segment(
                motion_type=MotionType.APPROACH,
                waypoints=[h_entry, run_surface[0]],
                v=APPROACH_V,
                a=APPROACH_A,
            ))

            # DRAW on surface
            segments.append(Segment(
                motion_type=MotionType.DRAW,
                waypoints=run_surface,
                v=DRAW_V,
                a=DRAW_A,
            ))

            # APPROACH pen-up
            segments.append(Segment(
                motion_type=MotionType.APPROACH,
                waypoints=[run_surface[-1], h_exit],
                v=APPROACH_V,
                a=APPROACH_A,
            ))

            current_tcp = h_exit

    # Final TRAVEL back home
    if home_joints is not None and current_tcp is not None:
        home_tcp = robot.get_fk(home_joints)
        travel_wps, _ = compute_positioning_motion(
            robot, checker, current_tcp, home_tcp,
        )
        segments.append(Segment(
            motion_type=MotionType.TRAVEL,
            waypoints=travel_wps,
            v=TRAVEL_V,
            a=TRAVEL_A,
        ))

    return segments, skip_report


def load_traces(json_path):
    """Load trace JSON and normalize v1 → v2 format.

    Returns (traces, data) where traces is the normalized list
    and data is the full parsed dict (for calibration access etc.).
    """
    import json

    with open(json_path, "r", encoding="utf-8") as f:
        data = json.load(f)

    traces = data["traces"]

    # Normalize v1 -> v2: spread face normal to each waypoint
    for trace in traces:
        if "face" in trace:
            trace["path"] = [[pt, trace["face"]] for pt in trace["path"]]

    return traces, data


def compute_drawing_plan(robot, checker, json_path, obj2robot,
                         start_tcp=None, home_joints=None,
                         hover_offset=None, max_pts=None):
    """Load trace JSON and build drawing plan.

    Returns (segments, skip_report).
    """
    traces, _ = load_traces(json_path)

    return build_drawing_plan(
        robot, checker, traces, obj2robot,
        start_tcp=start_tcp, home_joints=home_joints,
        hover_offset=hover_offset, max_pts=max_pts,
    )
