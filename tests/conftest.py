import pytest
import trimesh


@pytest.fixture
def box_mesh():
    """A 0.1 m cube centered at the origin."""
    return trimesh.creation.box(extents=(0.1, 0.1, 0.1))
