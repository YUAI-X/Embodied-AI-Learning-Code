import numpy as np

from so101_tasks.task_geometry import (
    NAMED_POSES,
    PICK_CUBE_POSITION,
    PLACE_CUBE_POSITION,
    cube_center_in_base,
    geometry_errors,
)


def test_grasp_and_place_are_aligned_with_scene_cube_positions():
    """防止命名姿态和Planning Scene坐标再次各自修改后失配。"""
    pick_error, place_error = geometry_errors()
    assert pick_error < 0.002
    assert place_error < 0.002


def test_preposes_keep_xy_and_have_safe_vertical_clearance():
    """接近与撤离姿态保持XY；较长的官方夹爪在放置侧需10 cm间隙。"""
    pregrasp = cube_center_in_base(NAMED_POSES["pregrasp"])
    preplace = cube_center_in_base(NAMED_POSES["preplace"])
    assert np.allclose(
        pregrasp, PICK_CUBE_POSITION + [0.0, 0.0, 0.05], atol=0.002
    )
    assert np.allclose(
        preplace, PLACE_CUBE_POSITION + [0.0, 0.0, 0.10], atol=0.002
    )
