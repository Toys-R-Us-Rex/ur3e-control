# Plan: Multi-Line Drawing on 3D Object Faces

## Context

Currently `draw_shape_live.ipynb` can draw a single continuous line on a single flat surface with one fixed tilt angle. We need to generalize this to draw **multiple lines on multiple faces** of a 3D object, where each face may have a different orientation.

**Input:** A list of lines in the format `[c1, c2, n1, n2, color]`:
- `c1 = (x1, y1, z1)` — start point in world coordinates
- `c2 = (x2, y2, z2)` — end point in world coordinates
- `n1, n2` — normals at each endpoint (always equal, since all faces are flat)
- `color` — pen color

The line coordinates and normals are **already given** — we don't need to detect faces or project anything. We only need the mesh for **collision avoidance during travel** between lines.

Everything stays in **one fixed world coordinate system** (the robot base frame).

## Step-by-step plan

### Step 1: Extract pathfinding into a reusable module

Create `my_simulation/drawing/pathfinding.py` — extract from `pathfinding.ipynb`:
- `segment_collides(A, B, mesh)`
- `perpendicular_up(A, B)`
- `closest_distance_to_mesh(point, mesh)`
- `lift_midpoint(A, B, mesh, safe_margin, push_step)`
- `find_path(A, B, mesh, safe_margin, push_step, max_depth)`

No logic changes — just extraction.

### Step 2: Create geometry helpers

Create `my_simulation/drawing/geometry.py`:

- **`normal_to_tcp_orientation(face_normal)`** — converts a face outward normal to axis-angle `(rx, ry, rz)`. The pen must point INTO the surface (along `-normal`). Builds a rotation matrix `R = [x_tool | y_tool | z_tool]` where `z_tool = -normal`, then converts to axis-angle using the same approach as `kinematics.py:matrix_to_tcp6d()`.
  - Validation: normal `[0,0,1]` (horizontal, pen down) → `ry ≈ pi` (matches existing default)

- **`world_point_to_tcp(point, normal, offset=0.0)`** — combines a world position + normal into a `TCP6D`. Offsets the point along the normal (positive = away from surface).

### Step 3: Create the draw job sequencer

Create `my_simulation/drawing/draw_job.py`:

**Input data:**
```python
# Each line is [c1, c2, n1, n2, color]
# n1 == n2 always (flat faces)
lines = [
    [np.array([x1,y1,z1]), np.array([x2,y2,z2]), normal, normal, "red"],
    ...
]
```

**`execute_draw_lines(robot, lines, mesh, config)`** — the main sequencer:

```
For each line [c1, c2, n, n, color]:
  1. TRAVEL to above c1:
     - Lift current position along previous normal by safe_clearance
     - If same normal as previous line → simple lift-move-lower (no pathfinding)
     - If different normal → use find_path() to navigate around the mesh
     - Travel with safe vertical orientation (0, pi, 0)
  2. APPROACH c1:
     - Rotate to face orientation (normal_to_tcp_orientation(n))
     - Lower to ~5mm above surface along normal
     - Lower to drawing contact height (~1mm along normal)
  3. DRAW c1 → c2:
     - movel() from c1 to c2 at draw_speed with correct face orientation
  4. LIFT from c2:
     - Raise along normal by safe_clearance
```

After all lines: return home.

**Key detail — same-face optimization:** If consecutive lines share the same normal, skip pathfinding. Just lift, move laterally at lifted height, lower back down. Compare normals with `np.allclose(n_prev, n_curr)`.

### Step 4: Scale and place the mesh

The mesh is only needed for travel collision avoidance, not for drawing coordinates (those are already given). At load time:
```python
mesh = trimesh.load("trapez.stl")
mesh.apply_scale(0.001)            # mm -> m (if needed)
mesh.apply_transform(T_placement)  # position in robot workspace
```

Pathfinding parameters scale accordingly: `safe_margin=0.02m`, `push_step=0.005m`.

### Step 5: Create the demo notebook

Create `my_simulation/demos/draw_multiface.ipynb`:
1. Connect to simulator, set home
2. Load and place mesh (for collision avoidance only)
3. Define the list of lines `[c1, c2, n1, n2, color]`
4. Visualize: mesh + all lines in 3D (matplotlib with azimuth/elevation sliders)
5. Execute `execute_draw_lines()` and observe in simulator

### Step 6: Update report

Add a section to `robot_reports/pathfinding_report.md` or create a new report documenting the multi-face drawing integration.

## New files to create

| File | Purpose |
|---|---|
| `my_simulation/drawing/__init__.py` | Package init |
| `my_simulation/drawing/pathfinding.py` | Extracted collision detection + pathfinding |
| `my_simulation/drawing/geometry.py` | `normal_to_tcp_orientation()`, `world_point_to_tcp()` |
| `my_simulation/drawing/draw_job.py` | `execute_draw_lines()` sequencer |
| `my_simulation/demos/draw_multiface.ipynb` | Demo notebook |

## Critical existing files

| File | What we reuse |
|---|---|
| `my_simulation/demos/draw_shape_live.ipynb` | Drawing pattern: approach/contact/draw/lift sequence |
| `my_simulation/demos/pathfinding.ipynb` | `segment_collides()`, `find_path()` to extract |
| `my_simulation/iscoin_sim/kinematics.py` | `matrix_to_tcp6d()` — reuse rotation→axis-angle logic |
| `my_simulation/iscoin_sim/robot_control.py` | `movel()`, `movel_waypoints()`, `set_tcp()` |

## Verification

1. **`normal_to_tcp_orientation()`**: test with known normals — `[0,0,1]` → `ry≈pi`, `[1,0,0]` → pen along `-X`
2. **Pathfinding extraction**: import from module, run same test cases, verify identical results
3. **Demo**: load trapezoid, define lines on 2+ different faces, visualize, execute on simulator — confirm correct pen angle per face and collision-free travel between faces