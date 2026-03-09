'''
MIT License

Copyright (c) 2026 HES-SO Valais-Wallis, Engineering Track 304
'''

from src.safety import free_space_travel

### VERIFY THOSE VALUES FOR THE REAL LIFE ROBOT ONLY TESTED IN SIMULATION NOW !!
# Drawing motion parameters
DRAW_A = 0.5   # acceleration (m/s²)
DRAW_V = 0.1   # velocity (m/s)
APPROACH_A = 0.5
APPROACH_V = 0.1


def pen_down(robot, hover_tcp, surface_tcp):
    robot.movel(surface_tcp, a=APPROACH_A, v=APPROACH_V)


def pen_up(robot, hover_tcp):
    robot.movel(hover_tcp, a=APPROACH_A, v=APPROACH_V)


def draw_stroke(robot, surface_tcps):
    for tcp in surface_tcps:
        robot.movel(tcp, a=DRAW_A, v=DRAW_V)


def draw_trace(robot, trace, obstacles):

    hover_entry = trace[0]
    hover_exit = trace[-1]
    surface_tcps = trace[1:-1]

    # Travel to hover entry
    current_tcp = robot.get_actual_tcp_pose()
    free_space_travel(robot, current_tcp, hover_entry, obstacles)

    # Approach surface
    pen_down(robot, hover_entry, surface_tcps[0])

    # Draw on surface
    draw_stroke(robot, surface_tcps)

    # Retract from surface
    pen_up(robot, hover_exit)


def draw_all_traces(robot, draw_motions, obstacles, home_joints=None):
    if home_joints is not None:
        robot.movej(home_joints)

    for trace in draw_motions:
        draw_trace(robot, trace, obstacles)

    # Return home
    if home_joints is not None:
        home_tcp = robot.get_fk(home_joints)
        current_tcp = robot.get_actual_tcp_pose()
        free_space_travel(robot, current_tcp, home_tcp, obstacles)
