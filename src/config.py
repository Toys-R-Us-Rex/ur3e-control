'''
MIT License

Copyright (c) 2026 HES-SO Valais-Wallis, Engineering Track 304
'''

# Pathfinding parameters
SAFE_MARGIN = 0.005    # meters — min clearance from mesh
PUSH_STEP   = 0.005    # meters — midpoint push increment
MAX_DEPTH   = 10       # max recursion depth
MAX_PUSH    = 0.5      # meters — max total push before giving up
SURFACE_MARGIN = 0.01  # meters — clearance above mesh surface

# Joint safe operating ranges (radians) — None means no limit
JOINT_LIMITS = [
    (-1.0,   1.0),    # shoulder_pan
    (-2.0,   0.1),    # shoulder_lift
    ( 0.1,   2.0),    # elbow
    (-2.5,  -0.5),    # wrist_1
    (-2.5,  -0.5),    # wrist_2
    None,              # wrist_3 — no limit
]
