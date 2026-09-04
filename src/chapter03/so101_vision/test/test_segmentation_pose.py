import numpy as np
import pytest
# 作者：宇哥的具身笔记


from so101_vision.camera_geometry import project_points
from so101_vision.pose_estimation import cube_model, make_demo_pose, solve_pose_pnp
from so101_vision.segmentation import masked_target_cloud, segment_color_prompt
from so101_vision.synthetic_data import default_intrinsics, synthetic_rgbd
from so101_vision.transforms import rotation_error_deg, transform_points


def test_red_prompt_produces_target_cloud():
    color, depth, intrinsics, _ = synthetic_rgbd()
    mask = segment_color_prompt(color, "红色杯子")
    points, _, _ = masked_target_cloud(color, depth, mask, intrinsics)
    assert len(points) > 100
    assert np.median(points[:, 2]) == pytest.approx(0.46)


def test_pnp_recovers_pose_with_one_outlier():
    intrinsics = default_intrinsics(640, 480)
    model = cube_model()
    truth = make_demo_pose()
    pixels = project_points(transform_points(truth, model), intrinsics)
    pixels[0] += [20.0, -15.0]
    result = solve_pose_pnp(model, pixels, intrinsics)
    assert result.success
    assert len(result.inliers) >= 6
    assert np.linalg.norm(result.transform_camera_object[:3, 3] - truth[:3, 3]) < 0.005
    assert rotation_error_deg(result.transform_camera_object, truth) < 2.0
