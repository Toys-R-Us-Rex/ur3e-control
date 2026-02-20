# ISCoinSim Wrapper Report

## 1. Problem

We have two systems that speak different languages:

- **ISCoin / URBasic** controls the real UR3e via the RTDE protocol (TCP sockets on ports 30003/30004).
- **Gazebo** simulates the same robot but uses ROS2 with a `FollowJointTrajectory` action interface, running inside a Docker container.

We need a way to write code **once** and run it on either system without changes.

## 2. Solution: the ISCoinSim wrapper

We built `ISCoinSim`, a drop-in replacement for the teacher's `ISCoin` class. It exposes the exact same API but translates every call into ROS2 commands sent to Gazebo via `docker exec`.

Switching between real and simulated robot is a one-line swap:

```python
# Real robot
from URBasic import ISCoin

# Simulator (swap just this line)
from my_simulation import ISCoinSim as ISCoin
```

Everything else in the user's code stays identical.

## 3. Architecture

The wrapper lives in `my_simulation/iscoin_sim/` and is split into four modules:

| Module | Role |
|--------|------|
| `iscoin_sim.py` | Facade — creates `SimRobotControl` + `SimGripper` |
| `robot_control.py` | Motion commands (`movej`, `movel`, `get_inverse_kin`, ...) |
| `kinematics.py` | Forward kinematics + analytical inverse kinematics (pure math) |
| `gripper.py` | Robotiq HandE gripper simulation (`open`, `close`, `move`) |
| `ros_bridge.py` | All Docker/ROS2 communication (`docker exec` + topic parsing) |

See [architecture.drawio](architecture.drawio) for the full diagram.

## 4. Joint space vs Cartesian space

| | Joint space | Cartesian space |
|---|---|---|
| **Representation** | 6 angles `[q1..q6]` (radians) | Position + orientation `[x, y, z, rx, ry, rz]` |
| **Describes** | Motor positions | Tool (TCP) pose in the world |
| **Used by** | `movej()` | `movel()` |
| **Conversion** | FK (forward kinematics) | IK (inverse kinematics) |
| **Uniqueness** | One joint config = one pose | One pose = up to 8 joint configs |

The orientation `[rx, ry, rz]` uses the **axis-angle** convention: the direction of the vector is the rotation axis, its magnitude is the angle in radians.

## 5. Forward kinematics

**Goal:** given 6 joint angles, find the TCP pose.

We use the Denavit-Hartenberg (DH) convention. Each joint *i* has four parameters: the variable angle theta, an offset *d*, a link length *a*, and a twist *alpha*. The UR3e parameters are:

| Joint | a (m) | d (m) | alpha (rad) |
|-------|--------|--------|-------------|
| 1 | 0 | 0.15185 | pi/2 |
| 2 | -0.24355 | 0 | 0 |
| 3 | -0.2132 | 0 | 0 |
| 4 | 0 | 0.13105 | pi/2 |
| 5 | 0 | 0.08535 | -pi/2 |
| 6 | 0 | 0.0921 | 0 |

Each joint produces a 4x4 transformation matrix:

```
         [ cos(th)   -sin(th)*cos(al)    sin(th)*sin(al)    a*cos(th) ]
T_i  =  [ sin(th)    cos(th)*cos(al)   -cos(th)*sin(al)    a*sin(th) ]
         [ 0          sin(al)            cos(al)             d         ]
         [ 0          0                  0                   1         ]
```

We chain-multiply all six:

```
T = T_1 * T_2 * T_3 * T_4 * T_5 * T_6
```

The result is a 4x4 matrix where `T[0:3, 3]` gives the position `[x, y, z]` and `T[0:3, 0:3]` is the rotation matrix R. We convert R to axis-angle:

```
angle = arccos((trace(R) - 1) / 2)
k     = angle / (2 * sin(angle))
[rx, ry, rz] = k * [R[2,1]-R[1,2], R[0,2]-R[2,0], R[1,0]-R[0,1]]
```

## 6. Inverse kinematics (analytical)

**Goal:** given a desired TCP pose, find joint angles that achieve it.

The UR3e has a **spherical wrist** — joints 4, 5, 6 intersect at a single point. This lets us decouple the problem:

### Step 1 — Wrist center

Back off from the TCP by the tool offset d6 along the tool z-axis:

```
P_wrist = P_tcp - d6 * z_tool
```

### Step 2 — Solve theta1 (2 solutions)

Project the wrist center onto the base plane and use the wrist-1 offset d4:

```
theta1 = atan2(Py, Px) +/- acos(d4 / sqrt(Px^2 + Py^2)) + pi/2
```

### Step 3 — Solve theta5 (2 solutions per theta1)

```
cos(theta5) = (Px*sin(t1) - Py*cos(t1) - d4) / d6
theta5 = +/- acos(cos5)
```

### Step 4 — Solve theta6

Derived from the desired rotation matrix and theta1/theta5.

### Step 5 — Solve theta2, theta3, theta4

We build an intermediate frame T14 and solve a planar 2-link problem (links a2, a3) using the law of cosines:

```
cos(theta3) = (D^2 - a2^2 - a3^2) / (2*a2*a3)      # 2 solutions
theta2 = atan2(A*Py14 - B*Px14, A*Px14 + B*Py14)
theta4 = theta234 - theta2 - theta3
```

This gives **up to 8 solutions** (2 theta1 x 2 theta5 x 2 theta3). We validate each by re-running FK and checking the position error, then pick the solution **closest to the current joint configuration** to avoid wild arm movements.

## 7. Communication with Gazebo

All communication goes through `docker exec` into the `iscoin_simulator` container:

```
HOST                              DOCKER (iscoin_simulator)
+-----------------+               +---------------------------+
| Python script   |   docker      | ROS2 Humble               |
| ISCoinSim       |   exec        | Gazebo Ignition           |
| subprocess.run  | ------------> | joint_trajectory_controller|
|                 | <------------ | joint_state_broadcaster    |
+-----------------+   stdout      +---------------------------+
```

Two ROS2 topics are used:

| Topic | Direction | Purpose |
|-------|-----------|---------|
| `/joint_states` | Read | Current joint positions and velocities |
| `/joint_trajectory_controller/joint_trajectory` | Write | Send movement commands |

**Reading** — we run `ros2 topic echo /joint_states --once` inside the container, parse the YAML output, and reorder the joints to match the UR convention.

**Writing** — we run `ros2 topic pub --once` with a `JointTrajectory` message containing target positions and a duration.

**Gripper** — uses `ros2 action send_goal` on the `hande_controller_left/right` controllers.

Each `docker exec` call has ~0.5-1.5 s overhead, which is fine for simulation but would be too slow for real-time control.

## 8. Wrapper API summary

| Method | Space | Description |
|--------|-------|-------------|
| `movej(joints, a, v, t, r, wait)` | Joint | Move to target joint angles |
| `movel(pose, a, v, t, r, wait)` | Cartesian | Move TCP to target pose (IK then movej) |
| `movej_waypoints(waypoints, wait)` | Joint | Smooth multi-point joint trajectory |
| `movel_waypoints(waypoints, wait)` | Cartesian | Multi-point Cartesian trajectory (IK per point) |
| `get_actual_joint_positions()` | Joint | Read current 6 joint angles |
| `get_actual_tcp_pose()` | Cartesian | Compute current TCP via FK |
| `get_inverse_kin(pose, qnear)` | Both | Analytical IK, returns closest solution |
| `is_steady()` | — | True if all joint velocities < 0.01 rad/s |
| `stopj(a, wait)` | Joint | Emergency stop (publish current pos with t=0) |
| `gripper.open()` | — | Open the Robotiq HandE gripper |
| `gripper.close()` | — | Close the gripper |
| `gripper.move(pos)` | — | Move gripper to position (0=open, 255=closed) |
