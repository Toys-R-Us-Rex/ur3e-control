'''
MIT License

Copyright (c) 2026 HES-SO Valais-Wallis, Engineering Track 304
'''

# Pathfinding parameters
SAFE_MARGIN = 0.005    # meters — min clearance from mesh
PUSH_STEP   = 0.005    # meters — midpoint push increment
MAX_DEPTH   = 10       # max recursion depth
MAX_PUSH    = 0.05     # meters — max total push before giving up
SURFACE_MARGIN = 0.01  # meters — clearance above mesh surface

# Free-space travel constraints
TCP_Y_MAX        = 0.0      # meters — TCP must stay at Y ≤ 0
TCP_Z_MIN        = 0.0      # meters — TCP must stay at Z ≥ 0
TCP_Z_MAX        = 0.5      # meters — TCP must stay at Z ≤ 0.5
LINK_Z_MIN       = {        # per-link Z minimum (meters) — links not listed are unconstrained
    3: 0.10,                 # upper_arm_link — must stay above 10 cm
    4: 0.10,                 # forearm_link   — must stay above 10 cm
}
UR3E_MAX_REACH   = None      # disabled — IK solver handles reachability
FREE_TRAVEL_STEP = 0.005    # meters — interpolation density for path validation

# Joint safe operating ranges (radians) — None means no limit
# UR3e hardware limits are ±2π for all joints.  We rely on
# self-collision detection rather than tight software limits.
JOINT_LIMITS = [
    None,              # shoulder_pan — no limit
    None,              # shoulder_lift — no limit (self-collision covers it)
    None,              # elbow — no limit (self-collision covers it)
    None,              # wrist_1 — no limit
    None,              # wrist_2 — no limit
    None,              # wrist_3 — no limit
]

# Motion parameters
DRAW_V = 0.1           # m/s — drawing velocity
DRAW_A = 0.5           # m/s² — drawing acceleration
APPROACH_V = 0.1       # m/s — pen-down / pen-up velocity
APPROACH_A = 0.5       # m/s² — pen-down / pen-up acceleration
TRAVEL_V = 0.25        # m/s — free-space travel velocity
TRAVEL_A = 1.2         # m/s² — free-space travel acceleration
HOVER_OFFSET = [0, 0, -0.01, 0, 0, 0]  # TCP-local offset for hover above surface
