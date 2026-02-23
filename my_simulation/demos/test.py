"""
Test: read joint angles, move the robot, read again.

Prerequisites:
  1. Simulator is running (docker compose run --rm cpu)
  2. Gazebo is launched (ros2 launch iscoin_simulation_gz iscoin_sim_control.launch.py)
  3. You ran 'uv sync --python 3.11' in ur3e-control/

Run from ur3e-control/:
  uv run python my_simulation/demos/test.py
"""

import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

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


# ── Go to home position first ──────────────────────────────────────────
print("\n=== MOVEJ: going to home position ===")
home = Joint6D.createFromRadians(1.1945, -1.1268, 1.0484, -1.5988, -1.5214, 1.0469)
iscoin.robot_control.movej(home, a=radians(80), v=radians(60))

# ── Test movel ──────────────────────────────────────────────────────────
print("\n=== MOVEL: reading current TCP pose ===")
tcp_before = iscoin.robot_control.get_actual_tcp_pose()
print(f"TCP before: {tcp_before}")

length = 0.15

# Move 5cm in the X direction from the current pose
target_tcp_1 = TCP6D.createFromMetersRadians(
    tcp_before.x + length,
    tcp_before.y,
    tcp_before.z,
    tcp_before.rx,
    tcp_before.ry,
    tcp_before.rz,
)

target_tcp_2 = TCP6D.createFromMetersRadians(
    tcp_before.x + length,
    tcp_before.y + length,
    tcp_before.z,
    tcp_before.rx,
    tcp_before.ry,
    tcp_before.rz,
)

target_tcp_3 = TCP6D.createFromMetersRadians(
    tcp_before.x,
    tcp_before.y + length,
    tcp_before.z,
    tcp_before.rx,
    tcp_before.ry,
    tcp_before.rz,
)

target_tcp_4 = TCP6D.createFromMetersRadians(
    tcp_before.x,
    tcp_before.y,
    tcp_before.z,
    tcp_before.rx,
    tcp_before.ry,
    tcp_before.rz,
)


print(f"\n=== MOVEL 1: moving to {target_tcp_1} ===")
iscoin.robot_control.movel(target_tcp_1, v=0.1)

print(f"\n=== MOVEL 2: moving to {target_tcp_2} ===")
iscoin.robot_control.movel(target_tcp_2, v=0.1)

print(f"\n=== MOVEL 3: moving to {target_tcp_3} ===")
iscoin.robot_control.movel(target_tcp_3, v=0.1)

print(f"\n=== MOVEL 4: moving to {target_tcp_4} ===")
iscoin.robot_control.movel(target_tcp_4, v=0.1)

tcp_after = iscoin.robot_control.get_actual_tcp_pose()
print(f"TCP after movel: {tcp_after}")

# print(f"\n=== MOVEL: moving back ===")
# iscoin.robot_control.movel(tcp_before, v=0.1)
#
# tcp_final = iscoin.robot_control.get_actual_tcp_pose()
# print(f"TCP final: {tcp_final}")


# ── Done ────────────────────────────────────────────────────────────────
print("\n=== DONE ===")