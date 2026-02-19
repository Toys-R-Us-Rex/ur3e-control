# TCP Calibration Guide — Holding a Stylo with the UR3e

## Table of Contents

1. [The Setup: Robot + Gripper + Stylo](#1-the-setup-robot--gripper--stylo)
2. [What is TCP and Why It Matters](#2-what-is-tcp-and-why-it-matters)
3. [The Geometry: Stylo Perpendicular to Gripper](#3-the-geometry-stylo-perpendicular-to-gripper)
4. [Computing the TCP Offset](#4-computing-the-tcp-offset)
5. [Setting the TCP in Code](#5-setting-the-tcp-in-code)
6. [Calibration Method 1: Fixed Point (4-Point Method)](#6-calibration-method-1-fixed-point-4-point-method)
7. [Calibration Method 2: Camera + Checkerboard](#7-calibration-method-2-camera--checkerboard)
8. [Testing Your Calibration](#8-testing-your-calibration)
9. [Common Mistakes and Troubleshooting](#9-common-mistakes-and-troubleshooting)

---

## 1. The Setup: Robot + Gripper + Stylo

### Physical Configuration

```
                    JOINT 6 (wrist 3)
                        |
                   TOOL FLANGE
                        |
                  ROBOTIQ HAND-E
                   GRIPPER JAWS
                    |       |
                    +--PEN--+
                        |
                        |  (pen body, held perpendicular)
                        |
                        *  <-- PEN TIP (this is what we want to control)
```

The UR3e's default TCP (Tool Center Point) is at the **tool flange** — the flat
metal disk where you bolt attachments. When you add a gripper + pen, the actual
point you care about (the pen tip) is somewhere else in space.

### The Coordinate Frames

Every UR robot uses these frames:

```
BASE FRAME (world)         FLANGE FRAME (end of joint 6)
   Z ^                        Z ^  (points away from flange)
     |                          |
     |                          |
     +----> X                   +----> X
    /                          /
   Y                          Y
```

The flange frame moves with the robot. Its origin is at the center of the
tool flange, and its Z axis points outward (away from the robot, along the
last link).

### Gripper Frame

The Robotiq Hand-E gripper attaches to the flange. The gripper adds some
height along the flange Z axis. The jaws open/close along one axis
(let's call it the gripper X axis).

```
FLANGE (Z pointing down when arm points down)
   |
   |  ~50mm (gripper body height)
   |
GRIPPER FINGERTIPS
```

---

## 2. What is TCP and Why It Matters

### Definition

The **TCP (Tool Center Point)** is the point in space that the robot considers
as "the tool." When you command `movel` to position [0.3, 0.2, 0.1, ...],
the robot moves so that the **TCP** arrives at that position.

### Without TCP Offset

If you don't set a TCP offset, the robot thinks the tool is at the flange center.
So `get_actual_tcp_pose()` returns the position of the flange, not the pen tip.

```
You command: "go to [0.3, 0.0, 0.05]"
What arrives at that point: the center of the flange
Where the pen tip is: somewhere 15-20cm away from that point!
```

### With TCP Offset

After `set_tcp(offset)`, the robot accounts for the tool. Now:

```
You command: "go to [0.3, 0.0, 0.05]"
What arrives at that point: the pen tip
The flange position: automatically computed to place the tip correctly
```

This is critical for drawing — you want to control where the **pen tip** is,
not where the flange is.

---

## 3. The Geometry: Stylo Perpendicular to Gripper

### Understanding "Perpendicular"

When we say the stylo is held perpendicular to the gripper, we mean:

```
TOP VIEW (looking down the flange Z axis):

    Gripper jaw   Gripper jaw
        |             |
        |    STYLO    |
        |    ====     |
        |             |

The pen lies ACROSS the gripper, perpendicular to the jaw direction.
```

```
SIDE VIEW:

    FLANGE
       |
    GRIPPER BODY
       |
    ---PEN--- (horizontal, perpendicular to flange Z)
       |
    PEN TIP (hangs below or extends to the side)
```

### Possible Pen Orientations

There are two common ways to hold a pen:

**Option A: Pen aligned with flange Z (pointing straight down)**

```
    FLANGE
       |
    GRIPPER
     | P |
     | E |
     | N |
       *  tip

TCP offset: [0, 0, Lpen + Lgripper, 0, 0, 0]
No rotation needed — pen is along the default Z axis.
```

**Option B: Pen perpendicular to flange Z (horizontal)**

```
    FLANGE
       |
    GRIPPER
    ----PEN----*
              tip

TCP offset: [Lpen, 0, Lgripper, 0, -pi/2, 0]
Needs a 90° rotation because the tip is along X, not Z.
```

For drawing on a flat table, **Option A** (pen pointing down) is much simpler.
The robot just moves the arm above the paper and lowers the pen tip to the
surface. With Option B, you'd need to tilt the whole arm sideways to draw.

### Recommendation

Hold the pen **along the flange Z axis** (pointing down when the arm reaches
down). Even if the gripper naturally holds the pen horizontally, you can
reorient joint 5 or 6 to make the pen point downward. This makes the TCP
offset much simpler.

---

## 4. Computing the TCP Offset

### What You Need to Measure

The TCP offset is a 6D vector: `[x, y, z, rx, ry, rz]`

- **x, y, z** (meters): Position of the pen tip relative to the flange center
- **rx, ry, rz** (radians): Rotation of the tool frame relative to the flange frame

### Case 1: Pen Along Flange Z Axis (Simplest)

If the pen points straight out from the flange (along Z):

```
FLANGE CENTER (origin)
       |
       |  d_gripper (gripper body depth, ~50mm for Hand-E)
       |
GRIPPER TIP
       |
       |  d_pen (pen length sticking out, measure this!)
       |
PEN TIP

TCP offset = [0, 0, d_gripper + d_pen, 0, 0, 0]
```

**Measurements needed:**
- `d_gripper`: distance from flange to gripper fingertips (~0.05m for Hand-E
  when closed). Check the Robotiq Hand-E datasheet.
- `d_pen`: length of pen sticking out below the gripper jaws. Measure with a
  ruler.

**Example:**
```python
# Gripper depth = 50mm, pen sticking out = 120mm
d_gripper = 0.050  # meters
d_pen = 0.120      # meters
offset = TCP6D.createFromMetersRadians(0, 0, d_gripper + d_pen, 0, 0, 0)
# offset = [0, 0, 0.170, 0, 0, 0]
```

### Case 2: Pen Perpendicular to Flange Z

If the pen is held horizontally by the gripper:

```
FLANGE CENTER (origin)
       |
       | d_gripper (along Z)
       |
GRIPPER CENTER
       +------------- d_pen (along X) ------------ * PEN TIP
```

The pen tip is offset along both Z (gripper depth) and X (pen length).
Plus, the tool direction has rotated 90° — the "working direction" is now
along X instead of Z.

```python
# Gripper depth = 50mm, pen extends 120mm along X
offset = TCP6D.createFromMetersRadians(
    0.120,   # x: pen length along gripper X
    0,       # y: centered
    0.050,   # z: gripper depth
    0,       # rx: no rotation around X
    -1.5708, # ry: -90° rotation (pi/2) to point tool along X
    0        # rz: no rotation around Z
)
```

### Case 3: Pen at an Angle

If the pen isn't perfectly aligned (which is realistic), you need all 6
components. This is where calibration becomes essential — measuring by hand
isn't accurate enough.

---

## 5. Setting the TCP in Code

### On the Real Robot

```python
from URBasic import ISCoin, TCP6D

iscoin = ISCoin(host="10.30.5.158", opened_gripper_size_mm=40)

# Measure your pen setup
d_gripper = 0.050  # Robotiq Hand-E closed depth
d_pen = 0.120      # Pen sticking out below gripper

# Set TCP (pen along Z axis)
tcp_offset = TCP6D.createFromMetersRadians(0, 0, d_gripper + d_pen, 0, 0, 0)
iscoin.robot_control.set_tcp(tcp_offset)

# Now get_actual_tcp_pose() returns the PEN TIP position
pen_position = iscoin.robot_control.get_actual_tcp_pose()
print(f"Pen tip is at: x={pen_position.x:.3f}, y={pen_position.y:.3f}, z={pen_position.z:.3f}")

# movel now moves the PEN TIP to the target
target = TCP6D.createFromMetersRadians(0.3, 0.0, 0.02, 0, 3.14, 0)
iscoin.robot_control.movel(target)
# The pen tip arrives at [0.3, 0.0, 0.02] — just above the paper
```

### In the Simulator (our wrapper)

The `set_tcp` function needs to be added to our wrapper. It modifies the
forward kinematics by adding one more transformation at the end:

```python
# In ISCoinSim:
iscoin = ISCoinSim()
iscoin.robot_control.set_tcp(tcp_offset)

# Now get_actual_tcp_pose() accounts for the pen
# And movel() targets the pen tip, not the flange
```

---

## 6. Calibration Method 1: Fixed Point (4-Point Method)

This is the standard industrial method. No camera needed.

### Concept

If you touch the **same fixed point** from **multiple different orientations**,
the TCP offset is the point where all approaches converge.

```
    Approach 1:    Approach 2:    Approach 3:    Approach 4:

       /              |              \              |
      / arm          | arm           \ arm        | arm
     /               |                \           |
    * fixed       * fixed          * fixed     * fixed
    point         point            point       point

All 4 approaches touch the same point, but with different joint angles.
```

### Step-by-Step Procedure

**Setup:**
1. Fix a sharp pointed object on the table (a nail, a screw, or a pin).
   This is your reference point. It must NOT move during calibration.
2. Attach the pen to the gripper.
3. Close the gripper to hold the pen firmly.

**Collect 4 poses:**

For each of the 4 measurements:

1. Use teach mode (freedrive) or jog the robot manually
2. Move the robot so the PEN TIP touches the fixed point
3. Approach from a **different direction each time** (tilt the wrist differently)
4. Record the flange pose: `pose_i = iscoin.robot_control.get_actual_tcp_pose()`
   (this is the FLANGE pose since TCP hasn't been set yet)

**Important:** The pen tip must touch the EXACT same point every time.
The flange will be in 4 different positions/orientations, but the pen tip
is at the same world coordinate.

**The Math:**

Each pose gives us a flange transformation matrix `T_flange_i`.
The TCP offset `T_offset` is the same for all 4 poses.
The fixed point `P_world` is the same for all 4 poses.

For each measurement:
```
P_world = T_flange_i * T_offset * [0, 0, 0, 1]^T
```

This gives us 4 equations (one per measurement). We know `T_flange_i` (recorded)
and `P_world` is the same unknown for all. We solve for `T_offset`.

Since we have 4 equations and 6 unknowns (x, y, z, rx, ry, rz of the offset),
we use least-squares optimization. In practice, for a pen that's roughly aligned
with Z, we only need x, y, z (3 unknowns), making 4 measurements more than
enough.

**Simplified solver (pen roughly along Z, no rotation):**

```python
import numpy as np

def calibrate_tcp_4point(poses):
    """
    Compute TCP offset from 4 flange poses that all touch the same point.

    Args:
        poses: list of 4 flange poses, each as a 4x4 numpy matrix.

    Returns:
        [x, y, z] offset from flange to TCP.
    """
    # We solve: T_flange_i @ [x, y, z, 1]^T = P_world  for all i
    # Rearranged: R_i @ [x,y,z] + p_i = P_world
    # So: (R_1 - R_2) @ [x,y,z] = p_2 - p_1  (subtract pairs)

    A = []  # coefficient matrix
    b = []  # right-hand side

    for i in range(len(poses) - 1):
        R1 = poses[i][:3, :3]
        p1 = poses[i][:3, 3]
        R2 = poses[i+1][:3, :3]
        p2 = poses[i+1][:3, 3]

        A.append(R1 - R2)
        b.append(p2 - p1)

    A = np.vstack(A)  # (9x3) matrix
    b = np.concatenate(b)  # (9,) vector

    # Least squares solution
    result = np.linalg.lstsq(A, b, rcond=None)
    tcp_offset = result[0]

    return tcp_offset  # [x, y, z] in meters
```

**Usage:**

```python
# Record 4 poses (each as a 4x4 matrix)
# For now, manually jog the robot and record:
pose1 = iscoin.robot_control.get_actual_tcp_pose()  # flange pose, approach 1
# ... move robot, touch same point from different angle ...
pose2 = iscoin.robot_control.get_actual_tcp_pose()  # approach 2
pose3 = iscoin.robot_control.get_actual_tcp_pose()  # approach 3
pose4 = iscoin.robot_control.get_actual_tcp_pose()  # approach 4

# Convert to 4x4 matrices (using the existing library functions)
# Then solve
offset = calibrate_tcp_4point([mat1, mat2, mat3, mat4])
print(f"TCP offset: x={offset[0]:.4f}, y={offset[1]:.4f}, z={offset[2]:.4f}")

# Apply it
tcp = TCP6D.createFromMetersRadians(offset[0], offset[1], offset[2], 0, 0, 0)
iscoin.robot_control.set_tcp(tcp)
```

### Accuracy

This method typically achieves **< 1mm** accuracy if:
- The fixed point is truly sharp and fixed
- You approach from sufficiently different angles (at least 45° apart)
- You touch precisely (take your time)

---

## 7. Calibration Method 2: Camera + Checkerboard

This method is more complex but calibrates both the camera AND the TCP at once.
Useful if you'll later use the camera for vision tasks.

### What You Need

- The Robotiq wrist camera (mounted on the robot)
- A printed checkerboard pattern (e.g., 7x5, with known square size)
- OpenCV (already installed: `opencv-python`)
- The camera must see the checkerboard from multiple robot positions

### The Hand-Eye Calibration Problem

The camera is rigidly attached to the flange. We need to find the
transformation from the flange to the camera: `T_flange_to_camera`.

```
WORLD FRAME
    |
    | T_base_to_flange (known — from robot FK)
    v
FLANGE FRAME
    |
    | T_flange_to_camera (UNKNOWN — what we're solving for)
    v
CAMERA FRAME
    |
    | T_camera_to_board (known — from OpenCV solvePnP)
    v
CHECKERBOARD FRAME (fixed on table)
```

### Step-by-Step Procedure

**Step 1: Print and place the checkerboard**

- Print a checkerboard pattern (7 columns x 5 rows of internal corners)
- Measure the square size accurately (e.g., 25mm = 0.025m)
- Tape it flat on the table. It must NOT move during calibration.

**Step 2: Collect multiple observations**

Move the robot to 10-15 different positions where the camera can see the
checkerboard. At each position:

```python
import cv2
import numpy as np

# Checkerboard parameters
BOARD_SIZE = (7, 5)      # internal corners
SQUARE_SIZE = 0.025      # meters

# Prepare 3D object points (the board corners in the board's frame)
objp = np.zeros((BOARD_SIZE[0] * BOARD_SIZE[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:BOARD_SIZE[0], 0:BOARD_SIZE[1]].T.reshape(-1, 2)
objp *= SQUARE_SIZE

# At each robot position:
image = iscoin.camera.getImageAsOpenCvNpArray()  # grab frame
gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

ret, corners = cv2.findChessboardCorners(gray, BOARD_SIZE)
if ret:
    # Refine corner positions for sub-pixel accuracy
    corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1),
        criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))

    # Record this observation
    flange_pose = iscoin.robot_control.get_actual_tcp_pose()
    observations.append({
        'corners': corners,
        'flange_pose': flange_pose,
    })
    print(f"Observation {len(observations)} recorded")
else:
    print("Checkerboard not detected — try moving closer or adjusting lighting")
```

**Step 3: Camera intrinsic calibration**

Before hand-eye calibration, we need the camera's internal parameters
(focal length, distortion). If the Robotiq camera provides these, use them.
Otherwise, calibrate from the checkerboard images:

```python
all_object_points = [objp] * len(observations)
all_image_points = [obs['corners'] for obs in observations]

ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
    all_object_points, all_image_points, gray.shape[::-1], None, None
)

print(f"Camera matrix:\n{camera_matrix}")
print(f"Distortion coefficients: {dist_coeffs}")
# camera_matrix contains:
#   fx  0  cx
#   0  fy  cy
#   0   0   1
# where fx, fy = focal lengths in pixels, cx, cy = principal point
```

**Step 4: Compute camera-to-board transforms**

For each observation, compute where the camera is relative to the board:

```python
R_camera_to_board = []
t_camera_to_board = []

for i, obs in enumerate(observations):
    ret, rvec, tvec = cv2.solvePnP(
        objp, obs['corners'], camera_matrix, dist_coeffs
    )
    R, _ = cv2.Rodrigues(rvec)  # Convert rotation vector to matrix
    R_camera_to_board.append(R)
    t_camera_to_board.append(tvec)
```

**Step 5: Compute flange-to-base transforms**

Convert each recorded flange pose to a rotation matrix + translation:

```python
R_base_to_flange = []
t_base_to_flange = []

for obs in observations:
    pose = obs['flange_pose']
    # Convert axis-angle to rotation matrix
    angle = np.sqrt(pose.rx**2 + pose.ry**2 + pose.rz**2)
    if angle > 1e-6:
        axis = np.array([pose.rx, pose.ry, pose.rz]) / angle
        rvec = axis * angle
    else:
        rvec = np.array([0.0, 0.0, 0.0])
    R, _ = cv2.Rodrigues(rvec)
    t = np.array([[pose.x], [pose.y], [pose.z]])

    R_base_to_flange.append(R)
    t_base_to_flange.append(t)
```

**Step 6: Hand-eye calibration**

OpenCV provides the solver directly:

```python
R_flange_to_cam, t_flange_to_cam = cv2.calibrateHandEye(
    R_base_to_flange,   # rotation: base -> flange (list)
    t_base_to_flange,   # translation: base -> flange (list)
    R_camera_to_board,  # rotation: camera -> board (list)
    t_camera_to_board,  # translation: camera -> board (list)
    method=cv2.CALIB_HAND_EYE_TSAI  # Tsai-Lenz method (robust)
)

print(f"Camera offset from flange:")
print(f"  Translation: {t_flange_to_cam.flatten()}")
print(f"  Rotation matrix:\n{R_flange_to_cam}")
```

**Step 7: Derive TCP offset**

Now you know where the camera is relative to the flange.
If you know where the pen tip is relative to the camera (e.g., by looking
at the pen in the camera image), you can chain the transforms:

```
T_flange_to_tip = T_flange_to_camera * T_camera_to_tip
```

But for a simpler approach: combine the hand-eye result with a manual
measurement of the pen-tip-to-camera distance.

### When to Use This Method

- When you need the camera calibrated anyway (for object detection later)
- When the tool geometry is complex or unknown
- When you need high accuracy and can't do the 4-point method easily

---

## 8. Testing Your Calibration

### Test 1: Touch Test

After setting the TCP, verify it manually:

1. Place a piece of paper on the table
2. Mark a small X on the paper
3. Manually jog the robot so the pen tip is exactly above the X
4. Record the TCP position: `pos1 = iscoin.robot_control.get_actual_tcp_pose()`
5. Move away, approach the X from a completely different angle
6. Record again: `pos2 = iscoin.robot_control.get_actual_tcp_pose()`
7. Compare: `pos1.x - pos2.x`, `pos1.y - pos2.y`, `pos1.z - pos2.z`

If the calibration is good, the differences should be < 1mm.

### Test 2: Draw a Known Shape

```python
# Draw a small square (10cm x 10cm)
# The pen tip should trace the shape on paper
from URBasic import TCP6D
from math import radians

z_paper = 0.02   # height of paper surface
z_safe = 0.05    # safe height above paper

# Starting corner
start = TCP6D.createFromMetersRadians(0.3, -0.1, z_paper, 0, 3.14, 0)

# Four corners of the square
corners = [
    TCP6D.createFromMetersRadians(0.3,  -0.1, z_paper, 0, 3.14, 0),
    TCP6D.createFromMetersRadians(0.4,  -0.1, z_paper, 0, 3.14, 0),
    TCP6D.createFromMetersRadians(0.4,   0.0, z_paper, 0, 3.14, 0),
    TCP6D.createFromMetersRadians(0.3,   0.0, z_paper, 0, 3.14, 0),
    TCP6D.createFromMetersRadians(0.3,  -0.1, z_paper, 0, 3.14, 0),  # close
]

# Move to safe height above start
iscoin.robot_control.movel(
    TCP6D.createFromMetersRadians(0.3, -0.1, z_safe, 0, 3.14, 0))

# Lower to paper
iscoin.robot_control.movel(corners[0], v=0.05)

# Draw the square
for corner in corners[1:]:
    iscoin.robot_control.movel(corner, v=0.02)  # slow for drawing

# Lift pen
iscoin.robot_control.movel(
    TCP6D.createFromMetersRadians(0.3, -0.1, z_safe, 0, 3.14, 0))
```

Measure the drawn square with a ruler. It should be 10cm x 10cm if the
calibration is correct.

### Test 3: Repeatability Test

Move the pen tip to a specific point, lift, move away, come back to the
same point. Repeat 10 times. Mark each touch point. They should all overlap
within ~0.5mm.

---

## 9. Common Mistakes and Troubleshooting

### Mistake 1: Forgetting to Set TCP After Robot Restart

The TCP offset is stored in the robot controller's memory. After a power
cycle or program restart, it resets to the default (flange center).

**Fix:** Always call `set_tcp()` at the start of your program.

### Mistake 2: Wrong Units

The UR3e uses **meters** for position and **radians** for angles.
A 15cm pen = 0.15m, not 15 or 150.

```python
# WRONG
offset = TCP6D.createFromMetersRadians(0, 0, 150, 0, 0, 0)  # 150 meters!

# CORRECT
offset = TCP6D.createFromMetersRadians(0, 0, 0.15, 0, 0, 0)  # 0.15 meters
```

### Mistake 3: TCP Direction Confusion

The TCP Z axis defines the "approach direction" of the tool. For a pen
pointing down, the pen tip is along +Z in the flange frame. Make sure
you understand which direction is positive.

```
If pen points ALONG flange Z:      offset z = positive
If pen points OPPOSITE to flange Z: offset z = negative
If pen points along flange X:       offset x = positive, needs rotation
```

### Mistake 4: Pen Moves in Gripper

If the pen slips or rotates in the gripper during use, your calibration
is invalidated. Solutions:
- Grip harder (increase gripper force)
- Use tape or rubber bands for extra friction
- Re-calibrate if the pen moves

### Mistake 5: Ignoring Pen Tip Shape

A ballpoint pen tip is round — the contact point depends on the pen angle
relative to the paper. For maximum accuracy:
- Keep the pen perpendicular to the paper surface
- Use a fine-tip pen (0.3mm or less)
- Apply consistent pressure

### Troubleshooting: Calibration Seems Off

| Symptom | Likely Cause | Fix |
|---------|-------------|-----|
| Offset is correct but drifts over time | Pen slipping in gripper | Tighten grip, add friction |
| Offset wrong by several cm | Wrong measurement units | Check meters vs mm |
| Z offset wrong, X/Y correct | Forgot gripper depth | Add gripper body length to Z |
| Everything rotated | Wrong axis-angle convention | Check flange frame orientation |
| Inconsistent across workspace | Robot needs calibration | Contact UR support / recalibrate robot |