import cv2
import numpy as np
# 作者：宇哥的具身笔记


from so101_vision.pointcloud import (
    icp_point_to_point,
    ransac_plane,
    voxel_downsample,
)
from so101_vision.synthetic_data import synthetic_tabletop_cloud
from so101_vision.transforms import make_transform, transform_points


def test_voxel_downsample_reduces_points():
    points, _ = synthetic_tabletop_cloud()
    reduced = voxel_downsample(points, 0.01)
    assert 0 < len(reduced) < len(points)


def test_ransac_finds_horizontal_table():
    points, labels = synthetic_tabletop_cloud()
    model, inliers = ransac_plane(points, 0.005)
    assert abs(model[2]) > 0.98
    assert inliers[labels == 0].mean() > 0.95


def test_icp_recovers_small_transform():
    rng = np.random.default_rng(4)
    source = rng.normal(size=(250, 3)) * [0.03, 0.02, 0.01]
    rotation, _ = cv2.Rodrigues(np.array([0.02, -0.03, 0.04]))
    truth = make_transform(rotation, [0.008, -0.006, 0.004])
    target = transform_points(truth, source)
    result = icp_point_to_point(source, target, max_correspondence=0.04)
    assert result.converged
    assert result.rmse < 1e-4
    assert np.allclose(result.transform, truth, atol=2e-3)
