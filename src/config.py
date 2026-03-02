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
