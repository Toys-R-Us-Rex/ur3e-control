"""
Test: read joint angles, move the robot, read again.

Prerequisites:
  1. Simulator is running (docker compose run --rm cpu)
  2. Gazebo is launched (ros2 launch iscoin_simulation_gz iscoin_sim_control.launch.py)
  3. You ran 'uv sync --python 3.11' in ur3e-control/

Run from ur3e-control/:
  PYTHONPATH=. uv run python my_simulation/test.py
"""

from my_simulation import ISCoinSim as ISCoin
# from URBasic import ISCoin  # <-- swap this line to use the real robot

from URBasic import Joint6D, TCP6D
from math import radians

# Connect
iscoin = ISCoin()

# 1. Read current position
print("\n=== BEFORE MOVE ===")
joints = iscoin.robot_control.get_actual_joint_positions()
print(f"Current joints: {joints}")
tcp = iscoin.robot_control.get_actual_tcp_pose()
print(f"Current TCP:    {tcp}")


# # ── Test movej ──────────────────────────────────────────────────────────
# print("\n=== MOVEJ: moving to target ===")
# target = Joint6D.createFromRadians(1.7649, -1.1278, 1.0483, -1.5995, -1.4845, 1.0469)
# iscoin.robot_control.movej(target, a=radians(80), v=radians(60))
#
# joints = iscoin.robot_control.get_actual_joint_positions()
# print(f"After movej: {joints}")
#
# print("\n=== MOVEJ: moving back home ===")
# home = Joint6D.createFromRadians(1.1945, -1.1268, 1.0484, -1.5988, -1.5214, 1.0469)
# iscoin.robot_control.movej(home, a=radians(80), v=radians(60))
#
# joints = iscoin.robot_control.get_actual_joint_positions()
# print(f"Back home: {joints}")


# ── Test movel ──────────────────────────────────────────────────────────
print("\n=== MOVEL: reading current TCP pose ===")
tcp_before = iscoin.robot_control.get_actual_tcp_pose()
print(f"TCP before: {tcp_before}")

# Move 5cm in the X direction from the current pose
target_tcp = TCP6D.createFromMetersRadians(
    tcp_before.x + 0.15,
    tcp_before.y + 0.15,
    tcp_before.z + 0.15,
    tcp_before.rx,
    tcp_before.ry,
    tcp_before.rz,
)
print(f"\n=== MOVEL: moving to {target_tcp} ===")
iscoin.robot_control.movel(target_tcp, v=0.1)

tcp_after = iscoin.robot_control.get_actual_tcp_pose()
print(f"TCP after movel: {tcp_after}")

print(f"\n=== MOVEL: moving back ===")
iscoin.robot_control.movel(tcp_before, v=0.1)

tcp_final = iscoin.robot_control.get_actual_tcp_pose()
print(f"TCP final: {tcp_final}")


# ── Done ────────────────────────────────────────────────────────────────
print("\n=== DONE ===")