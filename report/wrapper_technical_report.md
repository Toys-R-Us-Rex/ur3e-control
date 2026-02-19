# ISCoinSim Wrapper — Full Technical Report

## Table of Contents

1. [Project Overview](#1-project-overview)
2. [Architecture: How the Wrapper Works](#2-architecture-how-the-wrapper-works)
3. [Joint Space vs Cartesian Space](#3-joint-space-vs-cartesian-space)
4. [The 6 Joints of the UR3e](#4-the-6-joints-of-the-ur3e)
5. [Forward Kinematics — Full Math Breakdown](#5-forward-kinematics--full-math-breakdown)
6. [Inverse Kinematics — Full Math Breakdown](#6-inverse-kinematics--full-math-breakdown)
7. [Wrapper Functions Reference](#7-wrapper-functions-reference)
8. [Communication: How We Talk to Gazebo](#8-communication-how-we-talk-to-gazebo)

---

## 1. Project Overview

### The Problem

We have two systems that speak different languages:

- **Teacher's library (ISCoin/URBasic)**: Controls the real UR3e robot using the RTDE
  protocol over TCP sockets (ports 30003, 30004, 29999).
- **Gazebo simulator**: Simulates the same robot but uses ROS2 with a
  `FollowJointTrajectory` action interface.

### The Solution

The wrapper (`ISCoinSim`) provides the **same Python API** as the teacher's `ISCoin`
class, but translates every call into ROS2 commands sent to the Gazebo simulator
via `docker exec`.

```
Your Python code
      |
      |--- ISCoin (real robot) ---> RTDE protocol ---> UR3e hardware
      |
      |--- ISCoinSim (wrapper) ---> docker exec ---> ROS2 ---> Gazebo
```

### One Line Swap

```python
# For the real robot:
from URBasic import ISCoin

# For the simulator (swap just this line):
from my_simulation import ISCoinSim as ISCoin
```

Everything else in your code stays identical.

---

## 2. Architecture: How the Wrapper Works

### File Structure

```
ur3e-control/
  my_simulation/
    __init__.py         # Exports ISCoinSim
    iscoin_sim.py       # The wrapper (all the code)
    test.py             # Test script
  urbasic/
    URBasic/
      iscoin.py         # Teacher's real ISCoin class
      urScript.py       # Real robot commands (movej, movel, etc.)
      urScriptExt.py    # Extended commands
      waypoint6d.py     # Joint6D, TCP6D data classes (shared!)
      kinematic.py      # Forward/inverse kinematics (UR5/UR10 only)
```

### Key Design Decision: Reuse the Teacher's Data Classes

The wrapper imports `Joint6D`, `TCP6D`, and `Joint6DDescriptor` directly from
`URBasic.waypoint6d`. This means:

- `get_actual_joint_positions()` returns the **exact same type** (`Joint6D`) as
  the real robot
- You can use `Joint6D.createFromRadians(...)` and `Joint6D.createFromDegrees(...)`
  everywhere
- Properties like `.j1`, `.j2`, ..., `.j6` work identically

### Communication Flow

Every function in the wrapper follows this pattern:

```
1. Build a ROS2 command string
2. Run it inside the Docker container via subprocess + docker exec
3. Parse the text output
4. Return the same data types as the real ISCoin
```

---

## 3. Joint Space vs Cartesian Space

These are the two fundamental ways to describe where a robot arm is.

### Joint Space (6 angles)

Every position of the robot is described by **6 numbers**: the angle of each joint.

```
q = [q1, q2, q3, q4, q5, q6]   (all in radians)
```

- q1 = shoulder pan (base rotation)
- q2 = shoulder lift
- q3 = elbow
- q4 = wrist 1
- q5 = wrist 2
- q6 = wrist 3

This is **internal** — it tells you the state of each motor, but not directly
where the end-effector (tool/gripper) is in the real world.

**Example:**
```python
q = [1.19, -1.13, 1.05, -1.60, -1.52, 1.05]  # radians
# This means: base rotated 68.2°, shoulder at -64.7°, elbow at 60.2°, etc.
```

### Cartesian Space (position + orientation)

The position of the end-effector (TCP = Tool Center Point) in the real world,
described by **6 numbers**:

```
pose = [x, y, z, rx, ry, rz]
```

- **x, y, z**: Position in meters, relative to the robot's base frame
- **rx, ry, rz**: Orientation as an **axis-angle** vector (explained below)

The base frame is:
- Origin at the center of the robot's base
- Z pointing up
- X pointing forward (away from the cable)

### Axis-Angle Representation

The orientation (rx, ry, rz) is NOT Euler angles. It's an **axis-angle** vector:

```
rotation_axis = [rx, ry, rz] / ||[rx, ry, rz]||   (unit vector)
rotation_angle = ||[rx, ry, rz]||                   (magnitude in radians)
```

Example: `[0, 3.14, 0]` means "rotate 3.14 radians (180 degrees) around the Y axis."

### Why Two Representations?

| | Joint Space | Cartesian Space |
|---|---|---|
| **What it describes** | Motor positions | Tool position in the world |
| **Used for** | `movej()` | `movel()` |
| **Easy to command** | Yes (directly to motors) | Yes (intuitive for humans) |
| **Conversion** | Forward Kinematics --> | <-- Inverse Kinematics |
| **Unique?** | One joint config = one pose | One pose = multiple joint configs! |

---

## 4. The 6 Joints of the UR3e

### Physical Description

```
Joint 1: SHOULDER PAN   — Rotates the entire arm left/right (base)
         Axis: vertical (Z)
         Range: +/- 360°

Joint 2: SHOULDER LIFT   — Lifts the upper arm up/down
         Axis: horizontal
         Range: +/- 360°

Joint 3: ELBOW           — Bends the forearm
         Axis: horizontal (same direction as joint 2)
         Range: +/- 360°

Joint 4: WRIST 1         — Rotates the wrist
         Axis: horizontal
         Range: +/- 360°

Joint 5: WRIST 2         — Tilts the wrist
         Axis: perpendicular to wrist 1
         Range: +/- 360°

Joint 6: WRIST 3         — Rotates the tool flange
         Axis: along the tool direction
         Range: +/- 360°
```

### UR3e Physical Dimensions

The DH parameters encode the physical dimensions of each link:

```
Joint | a (m)     | d (m)    | Description
------+-----------+----------+------------------------------------------
  1   |  0        | 0.15185  | Base height (from floor to joint 2 axis)
  2   | -0.24355  | 0        | Upper arm length
  3   | -0.21325  | 0        | Forearm length
  4   |  0        | 0.13105  | Wrist 1 offset
  5   |  0        | 0.08535  | Wrist 2 offset
  6   |  0        | 0.0921   | Tool flange offset
```

Total reach: approximately 0.5 m from base.

---

## 5. Forward Kinematics — Full Math Breakdown

### What Is Forward Kinematics?

**Given:** 6 joint angles (q1, q2, q3, q4, q5, q6)
**Find:** The position and orientation of the end-effector (TCP) in the world.

This is the "easy" direction — there is always exactly one answer.

### Denavit-Hartenberg (DH) Convention

DH is a standard method to describe how each joint transforms the coordinate
frame from the base to the end-effector. Each joint has 4 parameters:

- **theta (θ)**: Joint angle (this is the variable we control)
- **d**: Offset along the previous Z axis
- **a**: Length along the new X axis (link length)
- **alpha (α)**: Twist angle around the new X axis

### The DH Transformation Matrix

For each joint i, the transformation from frame i-1 to frame i is a 4x4 matrix:

```
         [ cos(θ)   -sin(θ)*cos(α)    sin(θ)*sin(α)    a*cos(θ) ]
T_i  =   [ sin(θ)    cos(θ)*cos(α)   -cos(θ)*sin(α)    a*sin(θ) ]
         [ 0         sin(α)            cos(α)            d        ]
         [ 0         0                 0                 1        ]
```

Where:
- θ = joint angle (the variable)
- a, d, α = from the DH parameter table (constants for each joint)

### How It Works Step by Step

**Step 1:** Start with the identity matrix (we're at the base frame):

```
T = I = [ 1 0 0 0 ]
        [ 0 1 0 0 ]
        [ 0 0 1 0 ]
        [ 0 0 0 1 ]
```

**Step 2:** For each joint (1 through 6), compute T_i and multiply:

```
T = T * T_1       (after joint 1)
T = T * T_2       (after joint 2)
T = T * T_3       (after joint 3)
T = T * T_4       (after joint 4)
T = T * T_5       (after joint 5)
T = T * T_6       (after joint 6 — this is now the TCP frame)
```

The result T is a 4x4 matrix:

```
T = [ R(3x3)  p(3x1) ]    R = rotation matrix (3x3)
    [ 0 0 0   1      ]    p = position vector [x, y, z]
```

**Step 3:** Extract position from T:

```
x = T[0][3]
y = T[1][3]
z = T[2][3]
```

**Step 4:** Extract orientation (convert rotation matrix to axis-angle):

The rotation matrix R encodes the full orientation. To convert to axis-angle:

```
angle = arccos( (trace(R) - 1) / 2 )

where trace(R) = R[0][0] + R[1][1] + R[2][2]
```

If angle is not 0 or pi:

```
k = angle / (2 * sin(angle))

rx = k * (R[2][1] - R[1][2])
ry = k * (R[0][2] - R[2][0])
rz = k * (R[1][0] - R[0][1])
```

The axis-angle vector is [rx, ry, rz]. Its magnitude is the rotation angle,
and its direction is the rotation axis.

### Concrete Example with UR3e

For joint angles q = [0, -pi/2, 0, -pi/2, 0, 0]:

**Joint 1** (q1=0, a=0, d=0.15185, alpha=pi/2):

```
T_1 = [ 1    0   0   0       ]
      [ 0    0  -1   0       ]
      [ 0    1   0   0.15185 ]
      [ 0    0   0   1       ]
```

**Joint 2** (q2=-pi/2, a=-0.24355, d=0, alpha=0):

```
T_2 = [ 0    1   0   0       ]
      [ -1   0   0   0.24355 ]
      [ 0    0   1   0       ]
      [ 0    0   0   1       ]
```

And so on for joints 3-6. The final T gives us the TCP pose.

### The Code

```python
def _forward_kinematics(joint_angles):
    T = np.eye(4)  # Start with identity

    for i in range(6):
        theta = joint_angles[i]
        a     = UR3E_DH[i]["a"]
        d     = UR3E_DH[i]["d"]
        alpha = UR3E_DH[i]["alpha"]

        ct, st = cos(theta), sin(theta)
        ca, sa = cos(alpha), sin(alpha)

        Ti = np.array([
            [ct, -st*ca,  st*sa, a*ct],
            [st,  ct*ca, -ct*sa, a*st],
            [0,   sa,     ca,    d   ],
            [0,   0,      0,     1   ],
        ])

        T = T @ Ti  # Matrix multiplication

    # Extract position
    x, y, z = T[0,3], T[1,3], T[2,3]

    # Extract axis-angle from rotation matrix
    R = T[:3, :3]
    angle = arccos((trace(R) - 1) / 2)
    k = angle / (2 * sin(angle))
    rx = k * (R[2,1] - R[1,2])
    ry = k * (R[0,2] - R[2,0])
    rz = k * (R[1,0] - R[0,1])

    return TCP6D.createFromMetersRadians(x, y, z, rx, ry, rz)
```

---

## 6. Inverse Kinematics — Full Math Breakdown

### What Is Inverse Kinematics?

**Given:** A desired TCP pose [x, y, z, rx, ry, rz]
**Find:** The 6 joint angles that achieve that pose.

This is the "hard" direction because:
- There can be **multiple solutions** (up to 8 for a 6-DOF robot)
- There can be **no solution** (pose is out of reach)
- It involves solving nonlinear trigonometric equations

### Two Approaches

#### Approach A: Analytical (Closed-Form)

For specific robot geometries, you can derive exact equations. UR robots have a
special property called a **spherical wrist** (joints 4, 5, 6 axes intersect at
one point), which allows decoupling:

1. Solve joints 1-3 for the wrist center position
2. Solve joints 4-6 for the wrist orientation

This gives exact solutions but the derivation is complex and robot-specific.
The teacher's library uses this on the real robot via the UR controller's
built-in `get_inverse_kin()` function.

#### Approach B: Numerical (What Our Wrapper Uses)

Treat IK as an **optimization problem**: find joint angles that minimize the
distance between the current TCP and the target TCP.

This is what our wrapper implements because:
- It works for any robot geometry
- It's simpler to understand and code
- It's good enough for simulation

### The Optimization Approach — Step by Step

**Step 1: Define what we're minimizing**

We want to find joint angles q = [q1, q2, q3, q4, q5, q6] such that
forward_kinematics(q) is as close as possible to our target pose.

The **cost function** measures "how far are we from the target":

```
cost(q) = position_error + 0.1 * orientation_error
```

Where:

```
position_error    = (x_current - x_target)^2
                  + (y_current - y_target)^2
                  + (z_current - z_target)^2

orientation_error = (rx_current - rx_target)^2
                  + (ry_current - ry_target)^2
                  + (rz_current - rz_target)^2
```

We weight position more (factor 1.0) than orientation (factor 0.1) because
position accuracy is usually more important.

**Step 2: Choose a starting guess**

The optimizer needs a starting point. We use the **current joint positions**
(qnear parameter). This helps because:
- The solution closest to the current position is usually the best
  (avoids wild arm movements)
- The optimizer converges faster when starting close to a solution

**Step 3: Run the optimizer**

We use scipy's L-BFGS-B algorithm, which is a **gradient-based** optimizer.
It works by:

1. Evaluate cost(q) at the starting guess
2. Estimate the gradient (which direction reduces cost?)
3. Take a step in that direction
4. Repeat until cost is small enough (or max iterations reached)

```
L-BFGS-B:
  - L = Limited memory (doesn't store full Hessian matrix)
  - BFGS = Broyden-Fletcher-Goldfarb-Shanno (an approximation method)
  - B = Bounded (can enforce joint limits)
```

Internally, the gradient is computed by **finite differences**: the optimizer
calls forward_kinematics(q) with tiny variations in each joint to estimate
how the cost changes.

**Step 4: Validate the result**

After optimization, we check:
- If `cost < 0.001`: good solution
- If `0.001 < cost < 0.01`: warn about potential inaccuracy
- If `cost > 0.01`: no valid solution found, return None

### The Code

```python
def get_inverse_kin(self, pose, qnear=None):
    from scipy.optimize import minimize

    target = np.array(pose.toList())  # [x, y, z, rx, ry, rz]

    # Starting guess
    if qnear is not None:
        q0 = np.array(qnear.toList())
    else:
        q0 = np.array(self.get_actual_joint_positions().toList())

    def cost(q):
        tcp = _forward_kinematics(q.tolist())
        current = np.array(tcp.toList())
        pos_error = sum((current[:3] - target[:3])^2)
        rot_error = sum((current[3:] - target[3:])^2)
        return pos_error + 0.1 * rot_error

    result = minimize(cost, q0, method="L-BFGS-B", options={"maxiter": 200})

    if result.fun > 0.01:
        return None  # No solution

    return Joint6D.createFromRadians(*result.x.tolist())
```

### Limitations of the Numerical Approach

1. **Not guaranteed to find a solution** even if one exists (can get stuck in
   local minima)
2. **Slower** than analytical IK (runs FK many times)
3. **Depends on the starting guess** — bad starting point = bad or no solution
4. **Only finds one solution** — doesn't enumerate all possible configurations

For the simulator, this is perfectly fine. On the real robot, the UR controller's
built-in IK is analytical and much faster.

---

## 7. Wrapper Functions Reference

### 7.1 `get_actual_joint_positions()`

**What it does:** Reads the current 6 joint angles from the simulator.

**How it works:**
1. Runs `ros2 topic echo /joint_states --once` inside the container
2. Parses the YAML output to extract joint names and positions
3. Reorders to match UR convention (base, shoulder, elbow, wrist1-3)
4. Returns a `Joint6D` object

**Returns:** `Joint6D` with 6 angles in radians.

**Real ISCoin equivalent:** Reads from RTDE `actual_q` register.

---

### 7.2 `get_actual_joint_speeds()`

**What it does:** Reads the current 6 joint velocities.

**How it works:** Same as above, but extracts the `velocity` field from
`/joint_states`.

**Returns:** List of 6 floats (rad/s).

**Real ISCoin equivalent:** Reads from RTDE `actual_qd` register.

---

### 7.3 `get_actual_tcp_pose()`

**What it does:** Computes the current TCP position and orientation.

**How it works:**
1. Reads joint positions
2. Runs forward kinematics (see Section 5)
3. Returns a TCP6D

**Returns:** `TCP6D` with [x, y, z, rx, ry, rz].

**Real ISCoin equivalent:** Reads from RTDE `actual_TCP_pose` register
(the real robot computes FK internally).

---

### 7.4 `is_steady()`

**What it does:** Checks if the robot is standing still.

**How it works:**
1. Reads joint velocities
2. If all 6 are below 0.01 rad/s, returns True

**Returns:** Boolean.

**Real ISCoin equivalent:** Checks RTDE velocity data.

---

### 7.5 `movej(joints, a, v, t, r, wait)`

**What it does:** Moves the robot to a target joint configuration.

**Parameters:**
- `joints`: Joint6D — the target (6 angles in radians)
- `a`: acceleration in rad/s^2 (used for duration estimation)
- `v`: velocity in rad/s (used for duration estimation)
- `t`: time in seconds (if > 0, used directly as duration)
- `r`: blend radius (not used in simulator)
- `wait`: if True, blocks until movement is done

**How it works:**
1. Estimate duration: if `t > 0`, use it. Otherwise `duration = pi / v`.
2. Build a ROS2 trajectory message with the 6 joint names and target positions
3. Publish it to `/joint_trajectory_controller/joint_trajectory` via
   `ros2 topic pub --once`
4. If `wait=True`: sleep for the duration, then verify the robot reached
   the target (compare actual vs target positions)

**Duration estimation:**
The real robot uses trapezoidal velocity profiles with acceleration `a` and
max velocity `v`. In the simulator, we estimate: `duration = ceil(pi / v)`.
For the default `v=1.05 rad/s`, this gives about 3 seconds.

**Verification:**
After waiting, we read the actual joint positions and compare with the target.
If any joint differs by more than 0.05 rad (about 3 degrees), we print a
warning — this likely means a collision or joint limit was hit.

**Returns:** True if target was reached, False if not.

**Real ISCoin equivalent:** Sends a `movej` URScript command via the
Real-Time Client.

---

### 7.6 `movej_waypoints(waypoints, wait)`

**What it does:** Moves the robot through multiple joint positions.

**Parameters:**
- `waypoints`: list of `Joint6DDescriptor` objects, each containing a Joint6D
  plus velocity/acceleration parameters
- `wait`: if True, blocks until all waypoints are done

**How it works:**
1. For each waypoint, extract positions and estimate duration
2. Build cumulative timestamps (waypoint 1 at 3s, waypoint 2 at 6s, etc.)
3. Send ALL points in a **single** trajectory message
4. Gazebo's trajectory controller interpolates smoothly between them

**Why a single message?** If we sent separate `movej` calls, the robot would
stop between each waypoint. By sending all points in one trajectory, the
controller plans a smooth path through all of them.

**Returns:** True if the final waypoint was reached.

**Real ISCoin equivalent:** Sends a `movej` URScript with multiple waypoints
and blend radii.

---

### 7.7 `movel(pose, a, v, t, r, wait)`

**What it does:** Moves the end-effector to a Cartesian pose.

**Parameters:**
- `pose`: TCP6D — the target [x, y, z, rx, ry, rz]
- Other parameters same as movej

**How it works:**
1. Read current joint positions (as starting guess for IK)
2. Run `get_inverse_kin()` to find joint angles that achieve the target pose
3. Call `movej()` with those joint angles

**Important limitation:** The real `movel` interpolates in **Cartesian space**
(the TCP moves in a straight line). Our version just finds the target joints
and does a joint-space move, so the TCP path may be curved. For precise
straight-line motion, you'd need to interpolate many intermediate Cartesian
points, compute IK for each, and send them all as waypoints.

**Returns:** True if target was reached, False if IK failed or target not reached.

**Real ISCoin equivalent:** Sends a `movel` URScript command (true linear
interpolation in Cartesian space on the UR controller).

---

### 7.8 `stopj(a, wait)`

**What it does:** Immediately stops the robot.

**How it works:**
1. Reads the current joint positions
2. Publishes a trajectory with those positions and `time_from_start = 0`
3. This tells the controller "be at THIS position NOW", which effectively
   stops any ongoing movement

**Real ISCoin equivalent:** Sends `stopj(a)` URScript command.

---

### 7.9 `get_inverse_kin(pose, qnear)`

**What it does:** Finds joint angles for a desired TCP pose.

**Parameters:**
- `pose`: TCP6D — the desired end-effector pose
- `qnear`: Joint6D — starting guess (optional, defaults to current position)

**How it works:** See Section 6 for full math breakdown.

**Returns:** Joint6D with the solution, or None if no solution found.

**Real ISCoin equivalent:** Runs a URScript program that calls the controller's
built-in `get_inverse_kin()` function and reads the result from output registers.

---

### 7.10 `_verify_position(target, tolerance)`

**What it does:** Internal helper that checks if the robot reached its target.

**How it works:**
1. Read actual joint positions
2. For each of the 6 joints, compute `|actual - target|`
3. If any exceeds `tolerance` (default 0.05 rad = ~3 degrees), print a warning

This is how we detect collisions in the simulator: if the robot can't reach
the target because something is in the way, the actual position won't match
the commanded position.

---

## 8. Communication: How We Talk to Gazebo

### The Docker Architecture

```
HOST MACHINE                    DOCKER CONTAINER (iscoin_simulator)
+-------------------+          +--------------------------------+
| Your Python code  |          | ROS2 Humble                    |
|   ISCoinSim       |  docker  | Gazebo Ignition                |
|   subprocess.run  | -------> | joint_trajectory_controller    |
|   "docker exec"   |  exec    | joint_state_broadcaster         |
+-------------------+          +--------------------------------+
                                (network_mode: host)
```

### How `docker exec` Works

Every ROS2 command is executed via:

```python
subprocess.run(
    ["docker", "exec", "iscoin_simulator", "bash", "-c", "<ros2 command>"],
    capture_output=True,
    text=True,
    timeout=15,
)
```

This starts a bash shell inside the container, runs the command, captures
stdout/stderr, and returns the output as a string.

### ROS2 Topics Used

| Topic | Type | Direction | Purpose |
|-------|------|-----------|---------|
| `/joint_states` | `sensor_msgs/JointState` | Read | Current positions and velocities |
| `/joint_trajectory_controller/joint_trajectory` | `trajectory_msgs/JointTrajectory` | Write | Send movement commands |

### Reading: `/joint_states`

Command:
```bash
ros2 topic echo /joint_states --once
```

Output (YAML):
```yaml
header:
  stamp: {sec: 4553, nanosec: 512000000}
  frame_id: base_link
name:
- shoulder_pan_joint
- shoulder_lift_joint
- wrist_1_joint
- wrist_2_joint
- wrist_3_joint
- robotiq_hande_left_finger_joint
- elbow_joint
- robotiq_hande_right_finger_joint
position:
- 1.194510077457171
- -1.1268384369324844
- ...
velocity:
- 0.0
- 0.0
- ...
```

Note: the joint order in the message is **not** the same as the UR convention.
Our parser builds a dict `{name: value}` and then extracts them in the right
order using the `JOINT_NAMES` list.

### Writing: Trajectory Messages

Command:
```bash
ros2 topic pub --once \
  /joint_trajectory_controller/joint_trajectory \
  trajectory_msgs/msg/JointTrajectory \
  "{joint_names: [shoulder_pan_joint, shoulder_lift_joint, elbow_joint,
    wrist_1_joint, wrist_2_joint, wrist_3_joint],
    points: [{positions: [1.76, -1.13, 1.05, -1.60, -1.48, 1.05],
              time_from_start: {sec: 3, nanosec: 0}}]}"
```

The `--once` flag means "publish this message once and exit."
The controller receives it and starts moving the robot toward the target,
arriving in the specified duration.

### Performance

Each `docker exec` call takes approximately 0.5-1.5 seconds of overhead:
- Starting a bash shell in the container
- Sourcing the ROS2 setup
- Running the ROS2 command
- Returning the output

This is acceptable for simulation but would be too slow for real-time control.
On the real robot, RTDE runs at 500 Hz with sub-millisecond latency.
