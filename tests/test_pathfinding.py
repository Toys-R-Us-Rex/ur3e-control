import numpy as np
import pytest

from src.config import SURFACE_MARGIN
from src.pathfinding import segment_collides, perpendicular_up, lift_midpoint, find_path


# ── perpendicular_up ────────────────────────────────────────────────

class TestPerpendicularUp:
    def test_horizontal_segment_returns_up(self):
        A = np.array([0.0, 0.0, 0.0])
        B = np.array([1.0, 0.0, 0.0])
        result = perpendicular_up(A, B)
        np.testing.assert_allclose(result, [0, 0, 1], atol=1e-9)

    def test_diagonal_segment_perpendicular_and_positive_z(self):
        A = np.array([0.0, 0.0, 0.0])
        B = np.array([1.0, 1.0, 1.0])
        result = perpendicular_up(A, B)
        seg = B - A
        assert abs(np.dot(result, seg)) < 1e-9, "result must be perpendicular to A→B"
        assert result[2] > 0, "result must point upward"
        np.testing.assert_allclose(np.linalg.norm(result), 1.0, atol=1e-9)

    def test_vertical_segment_returns_fallback(self):
        A = np.array([0.0, 0.0, 0.0])
        B = np.array([0.0, 0.0, 1.0])
        result = perpendicular_up(A, B)
        np.testing.assert_allclose(result, [0, 0, 1], atol=1e-9)


# ── segment_collides ────────────────────────────────────────────────

class TestSegmentCollides:
    def test_segment_through_box(self, box_mesh):
        A = np.array([-1.0, 0.0, 0.0])
        B = np.array([1.0, 0.0, 0.0])
        assert segment_collides(A, B, box_mesh)

    def test_segment_misses_box(self, box_mesh):
        A = np.array([-1.0, 1.0, 0.0])
        B = np.array([1.0, 1.0, 0.0])
        assert segment_collides(A, B, box_mesh) is False

    def test_zero_length_segment(self, box_mesh):
        A = np.array([0.0, 0.0, 0.0])
        assert segment_collides(A, A, box_mesh) is False


# ── lift_midpoint ───────────────────────────────────────────────────

class TestLiftMidpoint:
    def test_colliding_segment_midpoint_clears(self, box_mesh):
        A = np.array([-1.0, 0.0, 0.0])
        B = np.array([1.0, 0.0, 0.0])
        mid = lift_midpoint(A, B, box_mesh)
        assert not segment_collides(A, mid, box_mesh)
        assert not segment_collides(mid, B, box_mesh)

    def test_clear_segment_midpoint_adds_margin(self, box_mesh):
        A = np.array([-1.0, 1.0, 0.0])
        B = np.array([1.0, 1.0, 0.0])
        mid = lift_midpoint(A, B, box_mesh)
        geometric = (A + B) / 2.0
        push_dir = perpendicular_up(A, B)
        expected = geometric + push_dir * SURFACE_MARGIN
        np.testing.assert_allclose(mid, expected, atol=1e-9)


# ── find_path ───────────────────────────────────────────────────────

class TestFindPath:
    def test_clear_path_returns_two_points(self, box_mesh):
        A = np.array([-1.0, 1.0, 0.0])
        B = np.array([1.0, 1.0, 0.0])
        path = find_path(A, B, box_mesh)
        assert len(path) == 2
        np.testing.assert_allclose(path[0], A)
        np.testing.assert_allclose(path[1], B)

    def test_obstructed_path_has_waypoints(self, box_mesh):
        A = np.array([-1.0, 0.0, 0.0])
        B = np.array([1.0, 0.0, 0.0])
        path = find_path(A, B, box_mesh)
        assert len(path) >= 3
        # no waypoint should be inside the box
        for pt in path:
            assert not box_mesh.contains([pt])[0], f"waypoint {pt} is inside the box"

    def test_max_depth_raises(self, box_mesh):
        A = np.array([-1.0, 0.0, 0.0])
        B = np.array([1.0, 0.0, 0.0])
        with pytest.raises(RuntimeError, match="max depth"):
            find_path(A, B, box_mesh, depth=100)
