# Pathfinding Around Obstacles — Recursive Midpoint Lifting

## Context

When the UR3e robot needs to move from point A to point B, the straight-line
path might go through an obstacle. We need an algorithm that finds a
collision-free path around the object. Instead of using a complex library
(like RRT or A*), we implemented a simple hand-made algorithm that
recursively splits the path and lifts waypoints over the obstacle.

## Tools used

- **trimesh**: load STL meshes, check if points are inside, ray-cast for
  segment intersections, compute closest distance to surface
- **numpy**: all the vector math
- **matplotlib**: 3D visualization of the mesh and paths

## How collision detection works

We check if a segment A->B collides with a mesh in two steps:

1. **Endpoint check**: if A or B is inside the mesh, it's a collision
2. **Ray intersection**: cast a ray from A in the direction of B, check if
   any intersection point lies within the segment length

```python
def segment_collides(A, B, mesh):
    if mesh.contains(np.array([A, B])).any():
        return True
    direction = B - A
    length = np.linalg.norm(direction)
    d_unit = direction / length
    locations, _, _ = mesh.ray.intersects_location(
        ray_origins=np.array([A]), ray_directions=np.array([d_unit])
    )
    if len(locations) == 0:
        return False
    return bool(np.any(np.linalg.norm(locations - A, axis=1) <= length))
```

We validated this with 8 test cases (through, inside, miss above, miss side,
etc.) and all returned the expected result.

## The pathfinding algorithm

### Idea

The idea is simple: if the direct path collides, split it in half and push
the midpoint upward until it clears the obstacle. Then check each half
recursively and do the same thing if needed.

### Parameters

| Parameter     | Value | Description                                |
|---------------|-------|--------------------------------------------|
| `safe_margin` | 20    | Minimum distance from the mesh surface     |
| `push_step`   | 5     | How far we push the midpoint each iteration|
| `max_depth`   | 10    | Maximum recursion depth                    |

### Step by step

1. Check if segment A->B collides with the mesh
2. If **no collision**: keep the segment as-is, we're done
3. If **collision**:
   a. Compute the midpoint M = (A + B) / 2
   b. Compute a push direction perpendicular to A->B, biased toward +Z
      (we want to go *over* the obstacle, not sideways)
   c. Push M upward in steps of `push_step` until:
      - M is outside the mesh AND
      - M is at least `safe_margin` away from the mesh surface
   d. Now we have two sub-segments: A->M and M->B
   e. Recursively apply the same algorithm to each sub-segment
4. Stop recursing when `max_depth` is reached

### The perpendicular-up direction

To push the midpoint upward, we need a direction perpendicular to the
segment that points as much toward +Z as possible:

```python
def perpendicular_up(A, B):
    AB_norm = (B - A) / np.linalg.norm(B - A)
    z_hat = np.array([0.0, 0.0, 1.0])
    perp = z_hat - np.dot(z_hat, AB_norm) * AB_norm
    return perp / np.linalg.norm(perp)
```

This projects the Z-axis onto the plane perpendicular to the segment. If the
segment happens to be vertical, we fall back to pushing along Y.

### The lift function

```python
def lift_midpoint(A, B, mesh, safe_margin, push_step):
    M = (A + B) / 2.0
    up = perpendicular_up(A, B)
    offset = 0.0
    while offset <= max_push:
        candidate = M + up * offset
        dist = closest_distance_to_mesh(candidate, mesh)
        if not is_inside(mesh, candidate) and dist >= safe_margin:
            return candidate
        offset += push_step
    return None
```

### The recursive pathfinder

```python
def find_path(A, B, mesh, safe_margin, push_step, max_depth, _depth=0):
    if not segment_collides(A, B, mesh):
        return [A, B]
    if _depth >= max_depth:
        return [A, B]
    M = lift_midpoint(A, B, mesh, safe_margin, push_step)
    left_path  = find_path(A, M, mesh, ...)
    right_path = find_path(M, B, mesh, ...)
    return left_path + right_path[1:]  # merge without duplicating M
```

## Visualization

The notebook produces a 3D plot showing:
- The obstacle mesh (gray, transparent)
- The direct path A->B (red dashed line — collides)
- The computed path with waypoints (blue solid line — collision-free)
- Start (green triangle) and goal (red star)

## Limitations

- Only works well for convex or simple obstacles. For complex concave shapes,
  pushing straight up might not be enough.
- The path is not optimized for length — it just avoids collisions.
- `push_step` is fixed. A smaller step gives more precision but is slower.
- No smoothing: the path has sharp corners at each waypoint.
- Only considers a single obstacle mesh.

## Possible improvements

- Path smoothing (e.g. cubic spline interpolation between waypoints)
- Adaptive push step (start large, refine if needed)
- Support for multiple obstacles
- Use a proper distance field for faster clearance checks
