import numpy as np
import pytest

from so101_leisaac_course.transforms import (
    matrix_to_pose7,
    pose7_to_matrix,
    relative_transform,
    transform_to_action6,
)
from so101_leisaac_course.umi import UmiEpisode, filter_for_workspace, relative_actions


def sample_episode():
    yaw_10 = np.deg2rad(10) / 2
    return UmiEpisode(
        timestamps=np.array([1.0, 1.1, 1.2]),
        poses=np.array(
            [
                [0.20, 0.00, 0.15, 0, 0, 0, 1],
                [0.21, 0.00, 0.15, 0, 0, np.sin(yaw_10), np.cos(yaw_10)],
                [0.22, 0.00, 0.15, 0, 0, np.sin(yaw_10), np.cos(yaw_10)],
            ]
        ),
        gripper_width_m=np.array([0.06, 0.04, 0.04]),
        task="move cup",
    )


def test_pose_matrix_roundtrip():
    pose = sample_episode().poses[1]
    recovered = matrix_to_pose7(pose7_to_matrix(pose))
    assert np.allclose(recovered, pose, atol=1e-7)


def test_relative_transform_translation_and_rotation():
    episode = sample_episode()
    first = pose7_to_matrix(episode.poses[0])
    second = pose7_to_matrix(episode.poses[1])
    action = transform_to_action6(relative_transform(first, second))
    assert np.allclose(action[:3], [0.01, 0, 0], atol=1e-7)
    assert action[5] == pytest.approx(np.deg2rad(10), abs=1e-7)


def test_umi_relative_actions_and_workspace_filter():
    episode = sample_episode()
    actions = relative_actions(episode, "previous")
    assert actions.shape == (3, 7)
    assert np.allclose(actions[0, :6], 0)
    assert filter_for_workspace(episode) == []


def test_umi_rejects_bad_timestamps():
    episode = sample_episode()
    with pytest.raises(ValueError, match="严格递增"):
        UmiEpisode(
            timestamps=np.array([1.0, 1.0, 1.2]),
            poses=episode.poses,
            gripper_width_m=episode.gripper_width_m,
            task=episode.task,
        )
