from pathlib import Path
# 作者：宇哥的具身笔记


from so101_leisaac_course.commands import LeIsaacCommandBuilder, render_command
from so101_leisaac_course.leisaac_stack import format_stack
from so101_leisaac_course.splits import split_episode_ids


def make_upstream(root: Path) -> None:
    scripts = (
        "scripts/environments/list_envs.py",
        "scripts/environments/teleoperation/teleop_se3_agent.py",
        "scripts/environments/teleoperation/replay.py",
        "scripts/convert/isaaclab2lerobotv3.py",
        "scripts/datagen/state_machine/generate.py",
        "source/leisaac/leisaac/datagen/state_machine/pick_orange.py",
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
    generated = builder.state_machine_generate(
        dataset_file=Path("data/orange.hdf5"), num_demos=12, seed=7, headless=True
    )
    assert "generate.py" in render_command(generated)
    assert "--task=LeIsaac-SO101-PickOrange-v0" in generated
    assert "--num_demos=12" in generated
    assert "--seed=7" in generated
    assert "--headless" in generated
    lerobot_generated = builder.state_machine_generate(
        num_demos=1,
        lerobot_repo_id="local/pick_orange_one",
        camera_width=320,
        camera_height=240,
    )
    assert "--use_lerobot_recorder" in lerobot_generated
    assert "--lerobot_dataset_repo_id=local/pick_orange_one" in lerobot_generated
    assert "--camera_width=320" in lerobot_generated
    replay = builder.replay(
        Path("data/orange.hdf5"),
        task="LeIsaac-SO101-PickOrange-v0",
        task_type="so101_state_machine",
    )
    assert "--task_type=so101_state_machine" in replay
    converted = builder.convert_v3(
        Path("data/orange.hdf5"),
        repo_id="local/pick_orange_state_machine",
        task="LeIsaac-SO101-PickOrange-v0",
        task_type="so101_state_machine",
    )
    assert "--task_type=so101_state_machine" in converted


def test_state_machine_command_rejects_unsupported_or_unbounded_jobs(tmp_path):
    make_upstream(tmp_path)
    builder = LeIsaacCommandBuilder(tmp_path)
    try:
        builder.state_machine_generate(task="LeIsaac-SO101-LiftCube-v0")
    except ValueError as error:
        assert "PickOrange" in str(error)
    else:
        raise AssertionError("未注册的状态机任务应被拒绝")

    try:
        builder.state_machine_generate(num_demos=0)
    except ValueError as error:
        assert "正整数" in str(error)
    else:
        raise AssertionError("课程入口不应生成无限录制命令")


def test_stack_terms_are_all_connected_to_leisaac():
    output = format_stack()
    for name in ("PhysX", "Isaac Sim", "Isaac Lab", "LeIsaac", "LeRobot"):
        assert name in output


def test_episode_split_has_no_overlap():
    split = split_episode_ids(list(range(20)), seed=7)
    assert set(split.train).isdisjoint(split.validation)
    assert set(split.train).isdisjoint(split.test)
    assert set(split.validation).isdisjoint(split.test)
