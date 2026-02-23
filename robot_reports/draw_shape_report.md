# Drawing Shapes with the UR3e — TCP Offset for Pen Drawing

## Context

The UR3e robot arm ends with a Robotiq Hand-E 2-finger gripper. To draw shapes
on a surface, we grip a pen (stylo) that extends straight out from the gripper.
Without any correction, forward and inverse kinematics compute the position of
the **flange** (end of joint 6), not the pen tip. This means all Cartesian
coordinates would be off by the length of the pen.

## The Tool Center Point (TCP)

The **TCP** (Tool Center Point) defines the offset from the robot flange to the
actual working point of the tool. On the real robot, the UR controller handles
this internally via `set_tcp()`. In our ISCoinSim simulator, we implement the
same concept in software.

### How it works

The TCP offset is represented as a 4x4 homogeneous transformation matrix
`T_tool`. For a pen of length `L` mounted straight along the tool Z-axis:

```
T_tool = [[1, 0, 0, 0],
          [0, 1, 0, 0],
          [0, 0, 1, L],
          [0, 0, 0, 1]]
```

This is set via:
```python
robot.set_tcp(TCP6D.createFromMetersRadians(0, 0, L, 0, 0, 0))
```

### Forward kinematics (reading the current pose)

To get the pen tip position:

```
T_tcp = T_flange @ T_tool
```

The flange pose is computed from joint angles via the DH chain, then the tool
offset is appended. The resulting `T_tcp` gives the position and orientation of
the pen tip.

### Inverse kinematics (planning a move)

To move the pen tip to a desired Cartesian pose, we first compute where the
flange needs to be:

```
T_flange = T_desired @ inv(T_tool)
```

Then we solve IK for `T_flange` using the standard analytical solver.

## Usage in the draw_shape notebook

```python
from my_simulation.iscoin_sim import ISCoinSim as ISCoin
from URBasic import TCP6D

iscoin = ISCoin()
robot = iscoin.robot_control

# Set TCP offset for a 10 cm pen
pen_length = 0.10  # meters
robot.set_tcp(TCP6D.createFromMetersRadians(0, 0, pen_length, 0, 0, 0))

# Now all movel / get_actual_tcp_pose calls reference the pen tip
tcp = robot.get_actual_tcp_pose()
```

## Verification

Two quick checks confirm correctness:

1. **Z offset check**: After `set_tcp(0, 0, 0.1, 0, 0, 0)`, `get_actual_tcp_pose()`
   should return a Z value approximately 0.1 m greater than without the offset
   (for a vertically-oriented tool).

2. **Round-trip check**: `get_inverse_kin(get_actual_tcp_pose())` should return
   joint angles close to the current joints, proving FK and IK are consistent
   with the offset applied.
