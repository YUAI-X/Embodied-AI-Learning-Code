import numpy as np
import pytest

from so101_vision.camera_geometry import (
    backproject_pixels,
    depth_to_pointcloud,
    project_points,
)
from so101_vision.synthetic_data import default_intrinsics, synthetic_rgbd


def test_projection_round_trip():
    intrinsics = default_intrinsics()
    pixels = np.array([[10.5, 20.0], [159.5, 119.5], [280.0, 180.0]])
    depths = np.array([0.3, 0.6, 1.1])
    points = backproject_pixels(pixels, depths, intrinsics)
    assert np.allclose(project_points(points, intrinsics), pixels)


def test_resolution_scales_focal_length():
    full = default_intrinsics(640, 480)
    assert full.fx == pytest.approx(560.0)
    assert full.fy == pytest.approx(560.0)


def test_invalid_depth_is_filtered():
    color, depth, intrinsics, _ = synthetic_rgbd()
    points, colors = depth_to_pointcloud(depth, intrinsics, color)
    assert len(points) == len(colors)
    assert np.isfinite(points).all()
    assert np.all(points[:, 2] > 0.0)


def test_depth_and_camera_info_size_must_match():
    with pytest.raises(ValueError, match="CameraInfo"):
        depth_to_pointcloud(np.ones((20, 30)), default_intrinsics())
