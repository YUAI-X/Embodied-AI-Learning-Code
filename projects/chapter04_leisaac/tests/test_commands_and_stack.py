from pathlib import Path

from so101_leisaac_course.commands import LeIsaacCommandBuilder, render_command
from so101_leisaac_course.leisaac_stack import format_stack
from so101_leisaac_course.splits import split_episode_ids


def make_upstream(root: Path) -> None:
    scripts = (
        "scripts/environments/list_envs.py",
        "scripts/environments/teleoperation/teleop_se3_agent.py",
        "scripts/environments/teleoperation/replay.py",
        "scripts/convert/isaaclab2lerobotv3.py",
    )
    for relative in scripts:
        path = root / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.touch()


def test_commands_follow_leisaac_official_scripts(tmp_path):
    make_upstream(tmp_path)
    builder = LeIsaacCommandBuilder(tmp_path)
    assert builder.list_envs()[-1].endswith("list_envs.py")
    record = builder.teleop(dataset_file=Path("data/lift.hdf5"))
    assert "--device=cuda" in record
    assert "--task=LeIsaac-SO101-LiftCube-v0" in record
    assert "--record" in record
    assert "teleop_se3_agent.py" in render_command(record)
    assert "replay.py" in render_command(builder.replay(Path("data/lift.hdf5")))
    assert "isaaclab2lerobotv3.py" in render_command(
        builder.convert_v3(Path("data/lift.hdf5"))
    )


def test_stack_terms_are_all_connected_to_leisaac():
    output = format_stack()
    for name in ("PhysX", "Isaac Sim", "Isaac Lab", "LeIsaac", "LeRobot"):
        assert name in output


def test_episode_split_has_no_overlap():
    split = split_episode_ids(list(range(20)), seed=7)
    assert set(split.train).isdisjoint(split.validation)
    assert set(split.train).isdisjoint(split.test)
    assert set(split.validation).isdisjoint(split.test)
