import numpy as np
import pytest
from unittest.mock import MagicMock, call

from src.pathfinding import find_path, lift_midpoint
from src.computation import (
    build_drawing_plan, compute_drawing_plan,
    _split_into_runs, _validate_surface_points, _hover_tcp,
)


class FakeTCP6D:
    """Minimal TCP6D stand-in for tests."""
    def __init__(self, x, y, z, rx=0.0, ry=np.pi, rz=0.0):
        self.x, self.y, self.z = x, y, z
        self.rx, self.ry, self.rz = rx, ry, rz

    @staticmethod
    def createFromMetersRadians(x, y, z, rx, ry, rz):
        return FakeTCP6D(x, y, z, rx, ry, rz)


@pytest.fixture(autouse=True)
def _patch_urbasic(monkeypatch):
    """Provide a fake URBasic.TCP6D so tests don't need the real module."""
    import types
    fake_mod = types.ModuleType("URBasic")
    fake_mod.TCP6D = FakeTCP6D
    monkeypatch.setitem(
        __import__("sys").modules, "URBasic", fake_mod,
    )


@pytest.fixture
def robot():
    """Mock robot with get_inverse_kin returning a dummy Joint6D."""
    r = MagicMock()
    r.get_inverse_kin.return_value = MagicMock(
        toList=lambda: [0.0, -1.0, 1.0, -1.5, -1.5, 0.0],
    )
    return r


@pytest.fixture
def clear_checker():
    """Checker where everything validates OK (no obstacles)."""
    checker = MagicMock()
    checker.validate_tcp.return_value = (True, MagicMock(), "", MagicMock())
    checker.validate_path.return_value = (True, -1, "", [])
    return checker


@pytest.fixture
def blocking_checker():
    """Checker where the direct path fails but lifted midpoints pass."""
    checker = MagicMock()

    # validate_path: fail on first call (direct), pass on subsequent (halves)
    call_count = {"n": 0}
    def fake_validate_path(robot, waypoints, **kw):
        call_count["n"] += 1
        if call_count["n"] == 1:
            return (False, 5, "Collision detected", [])
        return (True, -1, "", [])

    checker.validate_path.side_effect = fake_validate_path
    checker.validate_tcp.return_value = (True, MagicMock(), "", MagicMock())
    return checker


# ── find_path ───────────────────────────────────────────────────────

class TestFindPath:
    def test_clear_path_returns_two_points(self, robot, clear_checker):
        A = FakeTCP6D(-0.1, -0.3, 0.2)
        B = FakeTCP6D(0.1, -0.3, 0.2)
        path = find_path(robot, clear_checker, A, B)
        assert len(path) == 2
        assert path[0] is A
        assert path[1] is B

    def test_obstructed_path_has_waypoints(self, robot, blocking_checker):
        A = FakeTCP6D(-0.1, -0.3, 0.1)
        B = FakeTCP6D(0.1, -0.3, 0.1)
        path = find_path(robot, blocking_checker, A, B)
        assert len(path) >= 3

    def test_max_depth_raises(self, robot):
        """When validate_path always fails, max depth should be hit."""
        checker = MagicMock()
        checker.validate_path.return_value = (False, 3, "collision", [])
        checker.validate_tcp.return_value = (True, MagicMock(), "", MagicMock())

        A = FakeTCP6D(-0.1, -0.3, 0.1)
        B = FakeTCP6D(0.1, -0.3, 0.1)
        with pytest.raises(RuntimeError, match="max depth"):
            find_path(robot, checker, A, B)


# ── lift_midpoint ───────────────────────────────────────────────────

class TestLiftMidpoint:
    def test_returns_tcp_above_midpoint(self, robot):
        """lift_midpoint should push Z upward until validate_tcp passes."""
        checker = MagicMock()
        # Fail twice, then pass
        checker.validate_tcp.side_effect = [
            (False, None, "collision", MagicMock()),
            (False, None, "collision", MagicMock()),
            (True, MagicMock(), "", MagicMock()),
        ]
        A = FakeTCP6D(-0.1, -0.3, 0.1)
        B = FakeTCP6D(0.1, -0.3, 0.1)
        result = lift_midpoint(robot, checker, A, B)
        # Midpoint Z should be higher than the average
        avg_z = (A.z + B.z) / 2.0
        assert result.z > avg_z

    def test_raises_when_cannot_clear(self, robot):
        """If validate_tcp never passes, should raise."""
        checker = MagicMock()
        checker.validate_tcp.return_value = (False, None, "always blocked", MagicMock())

        A = FakeTCP6D(-0.1, -0.3, 0.1)
        B = FakeTCP6D(0.1, -0.3, 0.1)
        with pytest.raises(RuntimeError, match="could not find valid midpoint"):
            lift_midpoint(robot, checker, A, B)


# ── _split_into_runs ────────────────────────────────────────────────

class TestSplitIntoRuns:
    def test_all_valid(self):
        assert _split_into_runs([True, True, True]) == [(0, 2)]

    def test_all_invalid(self):
        assert _split_into_runs([False, False]) == []

    def test_gap_in_middle(self):
        mask = [True, True, False, True, True, True]
        assert _split_into_runs(mask) == [(0, 1), (3, 5)]

    def test_single_valid(self):
        assert _split_into_runs([False, True, False]) == [(1, 1)]

    def test_empty(self):
        assert _split_into_runs([]) == []


# ── build_drawing_plan draw validation ────────────────────────────────

def _make_trace():
    """Return a minimal v2 trace dict with two path points (per-point normals)."""
    normal = [0.0, 0.0, 1.0]
    return {
        "path": [
            [[0.0, 0.0, 0.0], normal],
            [[0.01, 0.0, 0.0], normal],
        ],
    }


def _identity_obj2robot(p):
    """Pass-through transformation (6-tuple in, 6-tuple out)."""
    return tuple(p)


class TestDrawValidation:
    def test_approach_draw_validated(self, robot, clear_checker):
        """validate_tcp must be called for surface points."""
        traces = [_make_trace()]
        start = FakeTCP6D(0.0, 0.0, 0.3)
        robot.get_fk.return_value = start

        segments, report = build_drawing_plan(
            robot, clear_checker, traces, _identity_obj2robot,
            start_tcp=start, max_pts=2,
        )

        # validate_tcp is called for each surface point + hover points
        assert clear_checker.validate_tcp.call_count >= 2

    def test_draw_validation_skip_report(self, robot):
        """A failing surface point should be reported in skip_report, not raise."""
        checker = MagicMock()
        # validate_tcp: fail on second call (second surface point)
        call_count = {"n": 0}
        def fake_validate_tcp(robot, tcp, **kw):
            call_count["n"] += 1
            if call_count["n"] == 2:
                return (False, None, "IK singular", MagicMock())
            return (True, MagicMock(), "", MagicMock())

        checker.validate_tcp.side_effect = fake_validate_tcp
        checker.validate_path.return_value = (True, -1, "", [])

        # 3 surface points so one can fail
        normal = [0.0, 0.0, 1.0]
        trace = {
            "path": [
                [[0.0, 0.0, 0.0], normal],
                [[0.01, 0.0, 0.0], normal],
                [[0.02, 0.0, 0.0], normal],
            ],
        }
        start = FakeTCP6D(0.0, 0.0, 0.3)

        segments, report = build_drawing_plan(
            robot, checker, [trace], _identity_obj2robot,
            start_tcp=start,
        )

        # The skip report should record the skipped point
        assert len(report) == 1
        assert 1 in report[0]["skipped_indices"]
        assert report[0]["drawn_count"] < report[0]["original_count"]


class TestSkipUnreachable:
    def test_skip_unreachable_produces_report(self, robot):
        """Mock checker fails on specific point, verify skip_report content."""
        checker = MagicMock()
        # Fail only on the 2nd surface point (index 1)
        validate_call = {"n": 0}
        def fake_validate_tcp(robot, tcp, **kw):
            validate_call["n"] += 1
            # Calls: surface pt 0, surface pt 1 (fail), surface pt 2,
            #        then hover validations
            if validate_call["n"] == 2:
                return (False, None, "IK has no solution", MagicMock())
            return (True, MagicMock(), "", MagicMock())

        checker.validate_tcp.side_effect = fake_validate_tcp
        checker.validate_path.return_value = (True, -1, "", [])

        normal = [0.0, 0.0, 1.0]
        trace = {
            "path": [
                [[0.0, 0.0, 0.0], normal],
                [[0.01, 0.0, 0.0], normal],
                [[0.02, 0.0, 0.0], normal],
            ],
        }
        start = FakeTCP6D(0.0, 0.0, 0.3)

        segments, report = build_drawing_plan(
            robot, checker, [trace], _identity_obj2robot,
            start_tcp=start,
        )

        assert len(report) == 1
        r = report[0]
        assert r["trace"] == 0
        assert r["skipped_indices"] == [1]
        assert "IK has no solution" in r["reasons"]
        assert r["original_count"] == 3
        assert r["drawn_count"] == 2

    def test_trace_split_on_gap(self, robot):
        """Middle point fails → two DRAW segments produced."""
        checker = MagicMock()
        # Fail only the 2nd surface point (index 1)
        validate_call = {"n": 0}
        def fake_validate_tcp(robot, tcp, **kw):
            validate_call["n"] += 1
            if validate_call["n"] == 2:
                return (False, None, "Self-collision detected", MagicMock())
            return (True, MagicMock(), "", MagicMock())

        checker.validate_tcp.side_effect = fake_validate_tcp
        checker.validate_path.return_value = (True, -1, "", [])

        normal = [0.0, 0.0, 1.0]
        trace = {
            "path": [
                [[0.0, 0.0, 0.0], normal],
                [[0.01, 0.0, 0.0], normal],
                [[0.02, 0.0, 0.0], normal],
            ],
        }
        start = FakeTCP6D(0.0, 0.0, 0.3)

        segments, report = build_drawing_plan(
            robot, checker, [trace], _identity_obj2robot,
            start_tcp=start,
        )

        from src.computation import MotionType
        draw_segments = [s for s in segments if s.motion_type == MotionType.DRAW]
        # Gap in the middle → 2 separate DRAW segments
        assert len(draw_segments) == 2
        # Each DRAW segment has 1 point (pt 0 alone, pt 2 alone)
        assert len(draw_segments[0].waypoints) == 1
        assert len(draw_segments[1].waypoints) == 1


# ── compute_drawing_plan (JSON → segments) ───────────────────────────

class TestComputeDrawingPlan:
    def test_v1_json_normalizes_and_produces_segments(
        self, robot, clear_checker, tmp_path,
    ):
        """Write a v1 JSON, call compute_drawing_plan, verify segments."""
        import json

        v1_data = {
            "traces": [
                {
                    "face": [0.0, 0.0, 1.0],
                    "path": [[0.0, 0.0, 0.0], [0.01, 0.0, 0.0]],
                },
            ],
        }
        json_file = tmp_path / "trace.json"
        json_file.write_text(json.dumps(v1_data))

        start = FakeTCP6D(0.0, 0.0, 0.3)
        robot.get_fk.return_value = start

        segments, report = compute_drawing_plan(
            robot, clear_checker, str(json_file), _identity_obj2robot,
            start_tcp=start, max_pts=2,
        )

        # Should have TRAVEL, APPROACH (down), DRAW, APPROACH (up)
        assert len(segments) >= 3
        from src.computation import MotionType
        types = [s.motion_type for s in segments]
        assert MotionType.DRAW in types
        assert types.count(MotionType.APPROACH) == 2

    def test_v2_json_works_directly(
        self, robot, clear_checker, tmp_path,
    ):
        """v2 JSON (per-point normals, no face key) should work without normalization."""
        import json

        v2_data = {
            "traces": [
                {
                    "path": [
                        [[0.0, 0.0, 0.0], [0.0, 0.0, 1.0]],
                        [[0.01, 0.0, 0.0], [0.0, 0.0, 1.0]],
                    ],
                },
            ],
        }
        json_file = tmp_path / "trace_v2.json"
        json_file.write_text(json.dumps(v2_data))

        start = FakeTCP6D(0.0, 0.0, 0.3)
        robot.get_fk.return_value = start

        segments, report = compute_drawing_plan(
            robot, clear_checker, str(json_file), _identity_obj2robot,
            start_tcp=start, max_pts=2,
        )

        from src.computation import MotionType
        types = [s.motion_type for s in segments]
        assert MotionType.DRAW in types
