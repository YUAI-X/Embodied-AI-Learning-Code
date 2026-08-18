import numpy as np
from so101_math.kinematics import forward_kinematics, solve_position_ik


def test_fk_returns_homogeneous_matrix():
    transform = forward_kinematics(np.zeros(5))
    assert transform.shape == (4, 4)
    assert np.allclose(transform[3], [0, 0, 0, 1])


def test_position_ik_can_recover_reachable_target():
    target_joints = np.array([0.2, -0.5, 0.9, -0.3, 0.1])
    target = forward_kinematics(target_joints)[:3, 3]
    result = solve_position_ik(target)
    assert result.success
    assert result.error < 1e-3
