'''
MIT License

Copyright (c) 2026 HES-SO Valais-Wallis, Engineering Track 304
'''

import logging

import numpy as np
from scipy.spatial.transform import Rotation as Rot, Slerp

from src.config import PUSH_STEP, MAX_DEPTH, MAX_PUSH

log = logging.getLogger(__name__)


def find_path(robot, checker, A_tcp, B_tcp, depth=0, qnear=None):

    if qnear is None:
        q = robot.get_inverse_kin(A_tcp)
        if q is not None:
            qnear = q

    # Try the direct segment
    ok, _fail_idx, _reason, _traj = checker.validate_path(
        robot, [A_tcp, B_tcp], qnear=qnear,
        orientation_search=True,
    )
    if ok:
        return [A_tcp, B_tcp]

    log.info("find_path depth=%d: direct A(%.4f,%.4f,%.4f)->B(%.4f,%.4f,%.4f) "
             "failed at sample %d: %s",
             depth, A_tcp.x, A_tcp.y, A_tcp.z,
             B_tcp.x, B_tcp.y, B_tcp.z, _fail_idx, _reason)

    if depth >= MAX_DEPTH:
        raise RuntimeError(
            f"find_path: max depth {MAX_DEPTH} reached, cannot find safe path"
        )

    # Lift the midpoint until it validates
    mid_tcp = lift_midpoint(robot, checker, A_tcp, B_tcp, qnear=qnear)

    left = find_path(robot, checker, A_tcp, mid_tcp, depth + 1, qnear=qnear)
    right = find_path(robot, checker, mid_tcp, B_tcp, depth + 1, qnear=qnear)

    # left ends with mid, right starts with mid — skip duplicate
    return left + right[1:]


def lift_midpoint(robot, checker, A_tcp, B_tcp, qnear=None):

    from URBasic import TCP6D

    a_pos = np.array([A_tcp.x, A_tcp.y, A_tcp.z])
    b_pos = np.array([B_tcp.x, B_tcp.y, B_tcp.z])
    mid_pos = (a_pos + b_pos) / 2.0

    # SLERP the orientations at t=0.5
    a_rotvec = np.array([A_tcp.rx, A_tcp.ry, A_tcp.rz])
    b_rotvec = np.array([B_tcp.rx, B_tcp.ry, B_tcp.rz])
    rots = Rot.from_rotvec([a_rotvec, b_rotvec])
    slerp = Slerp([0, 1], rots)
    mid_rotvec = slerp(0.5).as_rotvec()

    total_push = 0.0
    last_reason = ""
    while total_push < MAX_PUSH:
        tcp = TCP6D.createFromMetersRadians(
            float(mid_pos[0]), float(mid_pos[1]), float(mid_pos[2]),
            float(mid_rotvec[0]), float(mid_rotvec[1]), float(mid_rotvec[2]),
        )
        ok, _q, last_reason, _ = checker.validate_tcp(
            robot, tcp, qnear=qnear, orientation_search=True,
        )
        if ok:
            return tcp
        log.debug("lift z=%.4f failed: %s", mid_pos[2], last_reason)
        mid_pos[2] += PUSH_STEP
        total_push += PUSH_STEP

    raise RuntimeError(
        f"lift_midpoint: could not find valid midpoint after "
        f"pushing {MAX_PUSH}m upward (last reason: {last_reason})"
    )
