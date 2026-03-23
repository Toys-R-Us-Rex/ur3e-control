"""
Computational functions for handling robot motion planning and manipulation.

Usage
-----
This module is designed to be used with our Duckify simulation environment,
and the URBasic library from which it is derived:
    https://github.com/ISC-HEI/ur3e-control

MIT License

Copyright (c) 2026 Savioz Pierre-Yves

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

Author:     Savioz Pierre-Yves, with assistance from Claude AI (Anthropic)
Co-Author:  Mariéthoz Cédric, with assistance from Copilot AI (Microsoft)
Course:     HES-SO Valais-Wallis, Engineering Track 304
"""

import logging

import numpy as np

from URBasic import TCP6D, Joint6D

from URBasic.urScript import UrScript
from src.safety import CollisionChecker
from src.segment import *
from src.config import *
from src.utils import *
from src.pathfinding import find_path

log = logging.getLogger(__name__)

# TODO: Check utility
def compute_draw_motion(trace: dict, obj2robot: AtoB, hover_offset: float = None, max_pts: int = None):
    """
    Compute the motion for drawing a trace.

    Parameters
    ----------
    trace : dict
        The trace data containing the path.
    obj2robot : AtoB
        The transformation function from object to robot coordinates.
    hover_offset : float, optional
        The offset for the hover position.
    max_pts : int, optional
        Maximum number of points to consider from the path.

    Returns
    -------
    points : numpy.ndarray
        The computed points.
    normals : numpy.ndarray
        The computed normals.

    """

    if hover_offset is None:
        hover_offset = HOVER_OFFSET

    path = trace["path"]
    if max_pts is not None:
        path = path[:max_pts]
    points = np.array([entry[0] for entry in path])
    normals = np.array([entry[1] for entry in path])
    return points, normals

# TODO: Check utility
def points_to_tcps(points_robot: np.ndarray|list, normals_robot: np.ndarray|list):
    """
    Convert points and normals to a list of TCP6D objects.

    Parameters
    ----------
    points_robot : array_like
        The robot coordinates of the points.
    normals_robot : array_like
        The normals at each point.

    Returns
    -------
    tcps : list of TCP6D
        The converted TCP6D objects.
    """

    tcps = []
    for pt, n in zip(points_robot, normals_robot):
        rv = normal_to_rotvec(n)
        tcps.append(TCP6D.createFromMetersRadians(pt[0], pt[1], pt[2], rv[0], rv[1], rv[2]))
    return tcps

def trace_to_tcp(trace: dict, obj2robot: AtoB, max_pts: int=None):
    """
    Convert a trace to a list of TCP6D objects.

    Parameters
    ----------
    trace : dict
        The trace data containing the path.
    obj2robot : AtoB
        The transformation function from object to robot coordinates.
    max_pts : int, optional
        Maximum number of points to consider from the path.

    Returns
    -------
    surface_tcps : list of TCP6D
        The converted TCP6D objects.
    """

    path = trace["path"]
    if max_pts is not None:
        path = path[:max_pts]

    surface_tcps = []
    for entry in path:
        position_obj, normal_obj = entry[0], entry[1]
        robot_pose = obj2robot((*np.array(position_obj), *np.array(normal_obj)))
        x, y, z, rx, ry, rz = robot_pose
        surface_tcps.append(TCP6D.createFromMetersRadians(x, y, z, rx, ry, rz))

    return surface_tcps

def compute_positioning_motion(robot, checker: CollisionChecker, start_tcp: TCP6D, end_tcp: TCP6D):
    """
    Compute the motion for positioning the robot.

    Parameters
    ----------
    robot : UrScript?
        The robot instance.
    checker : CollisionChecker
        The collision checker instance.
    start_tcp : TCP6D
        The starting TCP position.
    end_tcp : TCP6D
        The ending TCP position.

    Returns
    -------
    waypoints : list of TCP6D
        The computed waypoints.
    _joint_traj : list of Joint6D
        The computed joint trajectory.
    """
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
    """
    """

    if hover_offset is None:
        hover_offset = HOVER_OFFSET

    tcp_list = [surface_tcp.x, surface_tcp.y, surface_tcp.z,
                surface_tcp.rx, surface_tcp.ry, surface_tcp.rz]
    x_t, y_t, z_t, _, _, _ = tcp_trans(tcp_list, hover_offset)
    return TCP6D.createFromMetersRadians(
        x_t, y_t, z_t, surface_tcp.rx, surface_tcp.ry, surface_tcp.rz,
    )


def _validate_surface_points(checker: CollisionChecker, robot: UrScript, surface_tcps, qnear=None):
    """
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
                segments.append(TCPSegment(
                    motion_type=MotionType.TRAVEL,
                    waypoints=travel_wps,
                    v=TRAVEL_V,
                    a=TRAVEL_A,
                ))

            # APPROACH pen-down
            segments.append(TCPSegment(
                motion_type=MotionType.APPROACH,
                waypoints=[h_entry, run_surface[0]],
                v=APPROACH_V,
                a=APPROACH_A,
            ))

            # DRAW on surface
            segments.append(TCPSegment(
                motion_type=MotionType.DRAW,
                waypoints=run_surface,
                v=DRAW_V,
                a=DRAW_A,
            ))

            # APPROACH pen-up
            segments.append(TCPSegment(
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
        segments.append(TCPSegment(
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


def assemble_segments(robot, checker, validated_runs, surface_joints_per_trace, home,
                      surface_tcps_per_trace=None):
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
        # Resolve surface TCPs for this trace (if available)
        trace_surface_tcps = surface_tcps_per_trace[trace_i] if surface_tcps_per_trace else None

        try:
            travel_wps, travel_joints = compute_positioning_motion(robot, checker, current_tcp, h_entry)
            segments.append(JointSegment(
                motion_type=MotionType.TRAVEL,
                color=1,
                side=SideType.LEFT,
                v=TRAVEL_V, a=TRAVEL_A,
                waypoints=travel_joints,
                tcp_waypoints=travel_wps,
            ))
            print(f"    TRAVEL OK ({len(travel_wps)} wps)")
        except RuntimeError as e:
            print(f"    TRAVEL FAILED: {e}")
            print(f"    Skipping run {run_i}.")
            continue

        print(f"  [{entry_label}] -> [Run{run_i} surface[0]]")
        approach_down_tcps = None
        if trace_surface_tcps is not None:
            approach_down_tcps = [h_entry, trace_surface_tcps[run_start]]
        segments.append(JointSegment(
            motion_type=MotionType.APPROACH,
            color=1,
            side=SideType.LEFT,
            waypoints=[q_entry, trace_surface_joints[run_start]],
            v=APPROACH_V, a=APPROACH_A,
            tcp_waypoints=approach_down_tcps,
        ))
        print(f"    APPROACH down OK")

        print(f"  [Run{run_i} surface[0]] -> [Run{run_i} surface[{len(run_surface)-1}]]")
        draw_tcps = trace_surface_tcps[run_start:run_end + 1] if trace_surface_tcps else None
        segments.append(JointSegment(
            motion_type=MotionType.DRAW,
            color=1,
            side=SideType.LEFT,
            waypoints=trace_surface_joints[run_start:run_end + 1],
            v=DRAW_V, a=DRAW_A,
            tcp_waypoints=draw_tcps,
        ))
        print(f"    DRAW OK ({len(run_surface)} pts)")

        print(f"  [Run{run_i} surface[{len(run_surface)-1}]] -> [{exit_label}]")
        approach_up_tcps = None
        if trace_surface_tcps is not None:
            approach_up_tcps = [trace_surface_tcps[run_end], h_exit]
        segments.append(JointSegment(
            motion_type=MotionType.APPROACH,
            color=1,
            side=SideType.LEFT,
            waypoints=[trace_surface_joints[run_end], q_exit],
            v=APPROACH_V, a=APPROACH_A,
            tcp_waypoints=approach_up_tcps,
        ))
        print(f"    APPROACH up OK")

        current_tcp = h_exit
        current_label = exit_label

    home_tcp = robot.get_fk(home)
    print(f"\n  [{current_label}] -> [HOME]")
    print(f"    from {fmt_tcp(current_tcp)}  to {fmt_tcp(home_tcp)}")
    try:
        travel_wps, travel_joints = compute_positioning_motion(robot, checker, current_tcp, home_tcp)
        segments.append(JointSegment(
            motion_type=MotionType.TRAVEL,
            color=1,
            side=SideType.LEFT,
            waypoints=travel_joints,
            v=TRAVEL_V, a=TRAVEL_A,
            tcp_waypoints=travel_wps,
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
        n_joints = len(seg.waypoints) if seg.waypoints else 0
        print(f"  Segment {i}: {seg.motion_type.name:8s} - {n_joints:3d} joint waypoints")


# going through the segments and recalculate each IK for each waypoint based on the previous pos
def smoothing(robot, checker, segments, home):

    qnear = home
    total_updated = 0
    total_failed = 0

    for seg_i, seg in enumerate(segments):
        if seg.tcp_waypoints is None:
            # No TCP data, keep existing joints, advance qnear to last waypoint
            if seg.waypoints:
                qnear = seg.waypoints[-1]
            continue

        seg_failed = 0
        new_waypoints = []
        for wp_i, tcp in enumerate(seg.tcp_waypoints):
            check_obs = seg.motion_type == MotionType.TRAVEL
            ok, q, reason, _ = checker.validate_tcp(
                robot, tcp, qnear=qnear, orientation_search=True,
                check_obstacle=check_obs,
            )
            if ok:
                new_waypoints.append(q)
                qnear = q
                total_updated += 1
            else:
                # Keep original joint if re-solve fails
                if seg.waypoints and wp_i < len(seg.waypoints):
                    original_q = seg.waypoints[wp_i]
                    new_waypoints.append(original_q)
                    # Check what's wrong with the original too
                    checker.set_joint_angles(original_q.toList())
                    orig_self = checker.in_self_collision()
                    orig_obs = checker.in_obstacle_collision()
                    qnear_list = [f"{v:+.4f}" for v in qnear.toList()]
                    orig_list = [f"{v:+.4f}" for v in original_q.toList()]
                    print(f"  Smoothing FAIL: seg {seg_i} ({seg.motion_type.name}) wp {wp_i}/{len(seg.tcp_waypoints)}")
                    print(f"    TCP:    ({tcp.x:.4f}, {tcp.y:.4f}, {tcp.z:.4f})")
                    print(f"    Reason: {reason}")
                    print(f"    qnear:  [{', '.join(qnear_list)}]")
                    print(f"    orig_q: [{', '.join(orig_list)}]")
                    print(f"    orig self-col: {orig_self}, orig obs-col: {orig_obs}")
                    qnear = original_q
                total_failed += 1
                seg_failed += 1

        seg.waypoints = new_waypoints
        if seg_failed:
            print(f"  Segment {seg_i} ({seg.motion_type.name}): {seg_failed}/{len(seg.tcp_waypoints)} failed")

        seg.waypoints = new_waypoints

    print(f"\nSmoothing done: {total_updated} updated, {total_failed} kept original")

def collect_joint_waypoints(segments):
    all_joint_waypoints = []
    for seg in segments:
        for jw in seg.waypoints:
            all_joint_waypoints.append(jw)
    return all_joint_waypoints
