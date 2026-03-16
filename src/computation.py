'''
MIT License

Copyright (c) 2026 HES-SO Valais-Wallis, Engineering Track 304
'''

import logging
from dataclasses import dataclass, field
from enum import Enum, auto

import numpy as np

from src.config import (
    DRAW_V, DRAW_A, APPROACH_V, APPROACH_A, TRAVEL_V, TRAVEL_A,
    HOVER_OFFSET,
)
from src.transformation import obj_to_stl, tcp_trans
from src.utils import normal_to_rotvec

log = logging.getLogger(__name__)


class MotionType(Enum):
    TRAVEL = auto()      # free-space collision-avoidant move
    APPROACH = auto()    # pen-down / pen-up (hover ↔ surface)
    DRAW = auto()        # on-surface stroke


@dataclass
class Segment:
    motion_type: MotionType
    waypoints: list      # list[TCP6D]
    v: float             # velocity m/s
    a: float             # acceleration m/s²
    r: float = 0.0       # blend radius
    joint_waypoints: list | None = None  # optional pre-computed Joint6D list


def trace_to_points(trace, max_pts=None):
    path = trace["path"]
    if max_pts is not None:
        path = path[:max_pts]
    points = np.array([entry[0] for entry in path])
    normals = np.array([entry[1] for entry in path])
    return points, normals


def transform_to_robot_frame(points, normals, obj2robot):
    T = obj2robot.T
    R_normal = obj2robot.T_normal[:3, :3]

    ones = np.ones((len(points), 1))
    points_h = np.hstack([points, ones])
    points_robot = (T @ points_h.T).T[:, :3]

    normals_robot = (R_normal @ normals.T).T
    norms = np.linalg.norm(normals_robot, axis=1, keepdims=True)
    normals_robot = normals_robot / norms

    return points_robot, normals_robot


def points_to_tcps(points_robot, normals_robot):
    from URBasic import TCP6D

    tcps = []
    for pt, n in zip(points_robot, normals_robot):
        rv = normal_to_rotvec(n)
        tcps.append(TCP6D.createFromMetersRadians(pt[0], pt[1], pt[2], rv[0], rv[1], rv[2]))
    return tcps


def trace_to_tcp(trace, obj2robot, max_pts=None):
    points, normals = trace_to_points(trace, max_pts=max_pts)
    points_robot, normals_robot = transform_to_robot_frame(points, normals, obj2robot)
    return points_to_tcps(points_robot, normals_robot)


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
# Helpers for drawing pipeline
# ---------------------------------------------------------------------------

def _hover_tcp(surface_tcp, hover_offset=None):
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


def _find_valid_hover(checker, robot, run_surface, surface_joints,
                      from_end, hover_offset=None):
    n = len(run_surface)
    indices = range(n - 1, -1, -1) if from_end else range(n)

    for trim, i in enumerate(indices):
        h_tcp = _hover_tcp(run_surface[i], hover_offset)
        qnear = surface_joints[i] if surface_joints is not None else None
        ok, q, reason, h_used = checker.validate_tcp(
            robot, h_tcp, qnear=qnear,
            check_obstacle=True, orientation_search=True,
        )
        if ok:
            return h_used, q, trim
    return None, None, n


def plan_drawing(robot, checker, traces, obj2robot,
                       start_tcp=None, home_joints=None,
                       hover_offset=None, max_pts=None):

    segments = []
    skip_report = []
    current_tcp = start_tcp

    if current_tcp is None and home_joints is not None:
        current_tcp = robot.get_fk(home_joints)

    for trace_idx, trace in enumerate(traces):
        surface_pts = trace_to_tcp(
            trace, obj2robot, max_pts=max_pts,
        )
        if not surface_pts:
            continue

        # Per-point validation (no obstacle collision for surface points)
        valid_checklist, reasons, surface_joints = _validate_surface_points(
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
            run_joints = surface_joints[run_start:run_end + 1]

            # Trim inward to find valid hover entry
            h_entry, _, entry_trim = _find_valid_hover(
                checker, robot, run_surface, run_joints,
                from_end=False, hover_offset=hover_offset,
            )
            if h_entry is None:
                log.warning(
                    "Trace %d run (%d-%d): no valid hover entry after "
                    "trimming all %d pts, discarding run",
                    trace_idx, run_start, run_end, len(run_surface),
                )
                continue

            # Trim inward to find valid hover exit
            h_exit, _, exit_trim = _find_valid_hover(
                checker, robot, run_surface, run_joints,
                from_end=True, hover_offset=hover_offset,
            )
            if h_exit is None:
                log.warning(
                    "Trace %d run (%d-%d): no valid hover exit after "
                    "trimming all %d pts, discarding run",
                    trace_idx, run_start, run_end, len(run_surface),
                )
                continue

            # Apply trimming
            trimmed = run_surface[entry_trim:len(run_surface) - exit_trim]
            if not trimmed:
                log.warning(
                    "Trace %d run (%d-%d): entry/exit trims overlap "
                    "(%d+%d >= %d), discarding run",
                    trace_idx, run_start, run_end,
                    entry_trim, exit_trim, len(run_surface),
                )
                continue

            if entry_trim or exit_trim:
                log.info(
                    "Trace %d run (%d-%d): trimmed %d entry + %d exit pts, "
                    "%d pts remain",
                    trace_idx, run_start, run_end,
                    entry_trim, exit_trim, len(trimmed),
                )

            run_surface = trimmed

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
    and data is the full parsed dict
    """
    import json

    with open(json_path, "r", encoding="utf-8") as f:
        data = json.load(f)

    traces = data["traces"]

    # Legacy usage: Normalize v1 -> v2: spread face normal to each waypoint
    for trace in traces:
        if "face" in trace:
            trace["path"] = [[pt, trace["face"]] for pt in trace["path"]]

    return traces, data


def load_and_convert_to_tcp(json_path, obj2robot, max_pts=None):
    traces, data = load_traces(json_path)
    surface_tcps_per_trace = []
    for t in traces:
        surface_tcps_per_trace.append(trace_to_tcp(t, obj2robot, max_pts=max_pts))
    return surface_tcps_per_trace, traces, data


def load_and_plan(robot, checker, json_path, obj2robot,
                  start_tcp=None, home_joints=None,
                  hover_offset=None, max_pts=None):
    """Load trace JSON and build drawing plan.

    Returns (segments, skip_report).
    """
    traces, _ = load_traces(json_path)

    return plan_drawing(
        robot, checker, traces, obj2robot,
        start_tcp=start_tcp, home_joints=home_joints,
        hover_offset=hover_offset, max_pts=max_pts,
    )


def assemble_segments(robot, checker, validated_runs, surface_joints_per_trace, home):
    from src.utils import fmt_tcp

    logging.getLogger("src.pathfinding").setLevel(logging.INFO)
    print("\nAssembling segments and computing travel paths...")

    segments = []
    current_tcp = robot.get_fk(home)
    current_label = "HOME"

    for run_i, (trace_i, run_start, run_end, h_entry, h_exit, run_surface, q_entry, q_exit) in enumerate(validated_runs):
        entry_label = f"Run{run_i} hover-entry"
        exit_label = f"Run{run_i} hover-exit"
        trace_surface_joints = surface_joints_per_trace[trace_i]

        print(f"\n  [{current_label}] -> [{entry_label}]")
        print(f"    from {fmt_tcp(current_tcp)}  to {fmt_tcp(h_entry)}")
        try:
            travel_wps, travel_joints = compute_positioning_motion(robot, checker, current_tcp, h_entry)
            segments.append(Segment(
                motion_type=MotionType.TRAVEL,
                waypoints=travel_wps, v=TRAVEL_V, a=TRAVEL_A,
                joint_waypoints=travel_joints,
            ))
            print(f"    TRAVEL OK ({len(travel_wps)} wps)")
        except RuntimeError as e:
            print(f"    TRAVEL FAILED: {e}")
            print(f"    Skipping run {run_i}.")
            continue

        print(f"  [{entry_label}] -> [Run{run_i} surface[0]]")
        segments.append(Segment(
            motion_type=MotionType.APPROACH,
            waypoints=[h_entry, run_surface[0]], v=APPROACH_V, a=APPROACH_A,
            joint_waypoints=[q_entry, trace_surface_joints[run_start]],
        ))
        print(f"    APPROACH down OK")

        print(f"  [Run{run_i} surface[0]] -> [Run{run_i} surface[{len(run_surface)-1}]]")
        segments.append(Segment(
            motion_type=MotionType.DRAW,
            waypoints=run_surface, v=DRAW_V, a=DRAW_A,
            joint_waypoints=trace_surface_joints[run_start:run_end + 1],
        ))
        print(f"    DRAW OK ({len(run_surface)} pts)")

        print(f"  [Run{run_i} surface[{len(run_surface)-1}]] -> [{exit_label}]")
        segments.append(Segment(
            motion_type=MotionType.APPROACH,
            waypoints=[run_surface[-1], h_exit], v=APPROACH_V, a=APPROACH_A,
            joint_waypoints=[trace_surface_joints[run_end], q_exit],
        ))
        print(f"    APPROACH up OK")

        current_tcp = h_exit
        current_label = exit_label

    home_tcp = robot.get_fk(home)
    print(f"\n  [{current_label}] -> [HOME]")
    print(f"    from {fmt_tcp(current_tcp)}  to {fmt_tcp(home_tcp)}")
    try:
        travel_wps, travel_joints = compute_positioning_motion(robot, checker, current_tcp, home_tcp)
        segments.append(Segment(
            motion_type=MotionType.TRAVEL,
            waypoints=travel_wps, v=TRAVEL_V, a=TRAVEL_A,
            joint_waypoints=travel_joints,
        ))
        print(f"    TRAVEL OK ({len(travel_wps)} wps)")
    except RuntimeError as e:
        print(f"    TRAVEL FAILED: {e}")

    print_segment_summary(segments)
    return segments


def print_segment_summary(segments):
    travel_count = sum(1 for s in segments if s.motion_type == MotionType.TRAVEL)
    approach_count = sum(1 for s in segments if s.motion_type == MotionType.APPROACH)
    draw_count = sum(1 for s in segments if s.motion_type == MotionType.DRAW)
    total_wps = sum(len(s.waypoints) for s in segments)
    print(f"\nPlan: {len(segments)} segments ({travel_count} TRAVEL, {approach_count} APPROACH, "
          f"{draw_count} DRAW), {total_wps} total waypoints")

    print(f"\nJoint plan:")
    for i, seg in enumerate(segments):
        n_joints = len(seg.joint_waypoints) if seg.joint_waypoints else 0
        print(f"  Segment {i}: {seg.motion_type.name:8s} - {n_joints:3d} joint waypoints")


def collect_joint_waypoints(segments):
    all_joint_waypoints = []
    for i, seg in enumerate(segments):
        has_joints = seg.joint_waypoints is not None
        print(f"\n  Segment {i}: {seg.motion_type.name}  ({len(seg.waypoints)} waypoints)")
        for j, wp in enumerate(seg.waypoints):
            if has_joints and j < len(seg.joint_waypoints):
                jw = seg.joint_waypoints[j]
                jnt_vals = jw.toList()
                all_joint_waypoints.append(jw)
                print(f"    WP {j:3d}  J=[{', '.join(f'{v:+.4f}' for v in jnt_vals)}]")

    print(f"\n  {len(segments)} segments, {len(all_joint_waypoints)} joint waypoints.")
    return all_joint_waypoints
