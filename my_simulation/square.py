"""
Test: draw a 10cm square using movel_waypoints (continuous motion).

The robot moves through all 4 corners without stopping between them,
unlike sequential movel() calls which pause at each waypoint.

Prerequisites:
  1. Simulator is running (docker compose run --rm cpu)
  2. Gazebo is launched (ros2 launch iscoin_simulation_gz iscoin_sim_control.launch.py)
  3. You ran 'uv sync --python 3.11' in ur3e-control/

Run from ur3e-control/:
  PYTHONPATH=. uv run python my_simulation/square.py
"""

from my_simulation import ISCoinSim as ISCoin
from URBasic import Joint6D, TCP6D
from URBasic.waypoint6d import TCP6DDescriptor
from math import radians

# Connect
iscoin = ISCoin()

# Go to home position
print("\n=== MOVEJ: going to home position ===")
home = Joint6D.createFromRadians(1.1945, -1.1268, 1.0484, -1.5988, -1.5214, 1.0469)
iscoin.robot_control.movej(home, a=radians(80), v=radians(60))

# Read current TCP pose
tcp_before = iscoin.robot_control.get_actual_tcp_pose()
print(f"\nTCP before: {tcp_before}")

# Build 4 waypoints forming a 10cm square in XY plane
length = 0.15
v = 0.1

corner1 = TCP6D.createFromMetersRadians(
    tcp_before.x + length, tcp_before.y,
    tcp_before.z, tcp_before.rx, tcp_before.ry, tcp_before.rz,
)
corner2 = TCP6D.createFromMetersRadians(
    tcp_before.x + length, tcp_before.y + length,
    tcp_before.z, tcp_before.rx, tcp_before.ry, tcp_before.rz,
)
corner3 = TCP6D.createFromMetersRadians(
    tcp_before.x, tcp_before.y + length,
    tcp_before.z, tcp_before.rx, tcp_before.ry, tcp_before.rz,
)
corner4 = TCP6D.createFromMetersRadians(
    tcp_before.x, tcp_before.y,
    tcp_before.z, tcp_before.rx, tcp_before.ry, tcp_before.rz,
)

waypoints = [
    TCP6DDescriptor(corner1, v=v, r=0.01),
    TCP6DDescriptor(corner2, v=v, r=0.01),
    TCP6DDescriptor(corner3, v=v, r=0.01),
    TCP6DDescriptor(corner4, v=v),
]

# Draw the square in one continuous motion
print("\n=== MOVEL_WAYPOINTS: drawing square ===")
iscoin.robot_control.movel_waypoints(waypoints)

tcp_after = iscoin.robot_control.get_actual_tcp_pose()
print(f"\nTCP after: {tcp_after}")

print("\n=== DONE ===")
