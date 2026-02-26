# Orientation Tolerance Report — UR3e Control Paths

## 1. How IK and Orientation Are Handled Today

### 1.1 Real Robot (URBasic)

The real robot path sends 6D Cartesian poses `[x, y, z, rx, ry, rz]` (position + axis-angle orientation) directly to the UR controller via URScript commands like `movel`. The UR controller solves inverse kinematics internally using its own firmware solver.

Python-side IK (`kinematic.py:Invkine_manip`) exists for **planning and validation only**. It uses a numerical Newton-Raphson method with screw-theory Jacobians (`manipulation.py:IKinFixed`), iterating up to 100 times with tight convergence thresholds:

- Position tolerance: **0.0001 m**
- Orientation tolerance: **0.001 rad** (~0.06 deg)

Because the UR controller handles the final IK, the Python code does not directly control how strictly orientation is enforced at execution time. The robot faithfully tracks the exact orientation specified in each waypoint.

### 1.2 Simulator (iscoin_sim)

The simulator must solve IK entirely in Python. It uses an **analytical (closed-form) solver** (`kinematics.py:analytical_ik`) based on the Hawkins method for UR-series robots. This exploits the spherical wrist geometry to decouple position from orientation, producing up to 8 candidate solutions.

Solution selection (`select_closest_ik`) picks the candidate closest to the current joint configuration. Valid solutions are filtered by forward-kinematics round-trip with a position error threshold of **0.001 m** (10x looser than URBasic).

Key difference: the simulator has **no explicit orientation error check** during solution validation — only position error is verified. This means a solution can be accepted even if the achieved orientation deviates slightly from the target, provided the position is close enough.

### 1.3 Common Orientation Representation

Both paths use the same **axis-angle** format `[rx, ry, rz]` in radians, where the vector magnitude encodes the rotation angle and the direction encodes the rotation axis. Conversion to/from rotation matrices uses Rodrigues' formula. The `TCP6D` class is shared across both paths.

TCP offset (tool length) is handled identically: `T_flange = T_desired @ inv(T_tcp)`, decoupling tool geometry from the IK target.

---

## 2. Why Exact Normals Fail

When following a path on a curved or tilted surface, each waypoint ideally has a tool orientation aligned with the **surface normal** at that point. In practice this creates problems:

### 2.1 IK Feasibility

The UR3e has a limited workspace (~0.457 m reach). Demanding an exact orientation at every waypoint constrains the IK problem more tightly. Near workspace boundaries or at steep angles, no joint solution may exist that simultaneously satisfies the exact position **and** exact orientation. The analytical solver returns an empty solution set; the numerical solver fails to converge.

### 2.2 Joint-Space Discontinuities

Even when individual waypoints have valid IK solutions, consecutive waypoints with rapidly changing orientation (e.g. following a curve's normals) can require large joint-space jumps. This leads to:

- Solution-branch flipping (the "closest solution" heuristic picks a different IK branch)
- Non-smooth joint trajectories that the robot cannot execute safely
- Abrupt wrist rotations, especially around singularities

### 2.3 Orientation Precision vs. Task Requirements

For tasks like drawing, pen contact with the surface matters far more than sub-degree alignment of the tool axis with the true surface normal. Demanding exact normals over-constrains the problem relative to the actual task requirements.

---

## 3. Proposed Approach: Cone-Based Orientation Tolerance

### 3.1 Core Idea

Instead of demanding that the tool Z-axis aligns exactly with the surface normal **n**, accept any orientation whose Z-axis falls within a **cone** of half-angle **alpha** around **n**:

```
cos(angle between tool_z and n) >= cos(alpha)
```

This relaxes the orientation constraint from a single direction to a cone of directions, dramatically expanding the set of feasible IK solutions.

### 3.2 Implementation Strategy: Pre-Adjustment in Python

The tolerance is best implemented **before** the IK call, not inside the solver. The workflow:

1. **Compute the ideal orientation** (surface normal) for each waypoint.
2. **Attempt IK** with the ideal orientation.
3. **If IK fails**, search within the tolerance cone for a feasible orientation:
   - Sample orientations on concentric rings within the cone (e.g. 8 samples at alpha/3, alpha*2/3, alpha).
   - For each sample, attempt IK. Accept the first solution that succeeds and is closest to the previous joint configuration.
4. **If all samples fail**, flag the waypoint as unreachable.

```
def find_feasible_orientation(position, ideal_normal, alpha_max, q_near):
    """
    Try the ideal orientation first, then search the tolerance cone.
    Returns (tcp6d, joint_solution) or None.
    """
    # 1. Try ideal
    tcp = build_tcp(position, ideal_normal)
    q = ik_solve(tcp, q_near)
    if q is not None:
        return tcp, q

    # 2. Sample the cone
    for alpha in [alpha_max/3, 2*alpha_max/3, alpha_max]:
        for phi in linspace(0, 2*pi, 8, endpoint=False):
            n_tilted = rotate_around_cone(ideal_normal, alpha, phi)
            tcp = build_tcp(position, n_tilted)
            q = ik_solve(tcp, q_near)
            if q is not None:
                return tcp, q

    return None  # Unreachable
```

### 3.3 Why Pre-Adjustment, Not Solver Modification

- **URBasic path**: Python cannot modify the UR controller's internal IK. The only lever is the pose sent to `movel`. Pre-adjusting the orientation is the only option.
- **iscoin_sim path**: While modifying `analytical_ik` is theoretically possible, the analytical solver produces exact solutions for exact targets. Adding tolerance inside it would require switching to a numerical approach, losing the speed advantage. Pre-adjustment keeps the solver clean.
- **Consistency**: The same pre-adjustment logic works for both paths, keeping the codebase uniform.

### 3.4 Integration Points

| Component | Change |
|-----------|--------|
| Path generation (demos) | Call `find_feasible_orientation` when building waypoints |
| `TCP6D` / `TCP6DDescriptor` | No change — they already accept any valid orientation |
| IK solvers | No change — they receive adjusted (feasible) poses |
| `movel` / `movel_waypoints` | No change — they forward whatever pose is given |

The tolerance logic lives entirely in the **path planning layer**, upstream of both control paths.

---

## 4. Limitations and Trade-Offs

### 4.1 Orientation Deviation Is Visible

With a tolerance cone of e.g. 15 deg, the tool may visibly tilt away from the ideal normal. For drawing tasks this is acceptable (pen still contacts surface). For tasks requiring precise tool alignment (e.g. welding, machining), the tolerance must be kept very small, reducing its benefit.

### 4.2 Sampling Cost

Searching the cone requires multiple IK calls per waypoint. With 8 azimuthal samples and 3 radial rings, that is up to 25 IK attempts per failing waypoint. For the analytical solver this is fast (~microseconds each). For the numerical solver used in validation, it may add noticeable latency on long paths.

### 4.3 No Guarantee of Smooth Transitions

Accepting different orientations at adjacent waypoints can introduce joint-space discontinuities between them. Mitigations:

- **Continuity bias**: when multiple cone samples succeed, prefer the one closest to the previous waypoint's orientation (not just the previous joint state).
- **Post-smoothing**: after generating the full path, apply a smoothing pass that interpolates orientations and re-validates IK.

### 4.4 Cone Axis Depends on Surface Model

The tolerance cone is centered on the surface normal, which assumes you have a surface model. For flat surfaces (current demos) the normal is constant and trivial. For curved surfaces, normal estimation adds complexity and potential error.

### 4.5 Real Robot vs. Simulator Parity

The real robot's internal IK may find solutions that the Python-side IK rejects (or vice versa), since they use different algorithms and tolerances. Pre-adjustment in Python may reject a waypoint that the UR controller could actually reach. Conservative pre-filtering is acceptable for safety, but it means the simulator may be slightly more restrictive than the real robot.

---

## 5. Summary

| Aspect | Current State | With Tolerance |
|--------|--------------|----------------|
| Orientation constraint | Exact (single direction) | Cone (configurable half-angle) |
| IK failure rate on complex paths | High near workspace limits | Significantly reduced |
| Code changes required | — | Path planning layer only |
| Solver modifications | — | None |
| Applicable to both paths | — | Yes (pre-adjustment strategy) |
| Risk | — | Visible tilt; possible joint discontinuities |

The recommended next step is to implement `find_feasible_orientation` as a utility function in the path planning module and integrate it into the demo notebooks that generate surface-following waypoints.
