"""Tests for launching external ROS2 nodes and launch files via the Launcher.

Covers Launcher.add_ros_node and Launcher.include_launch_file — used by
recipes to bring up nodes such as a MoveIt move_group alongside components.
"""

import pytest
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node as NodeLaunchAction

from ros_sugar import Launcher


@pytest.fixture
def launcher():
    return Launcher()


def test_add_ros_node_appends_action(launcher):
    action = launcher.add_ros_node(
        package="demo_nodes_cpp",
        executable="talker",
        name="my_talker",
        remappings=[("chatter", "/chat")],
    )
    assert isinstance(action, NodeLaunchAction)
    assert action in launcher._description.entities


def test_include_launch_file_from_path(launcher, tmp_path):
    launch_file = tmp_path / "external.launch.py"
    launch_file.write_text(
        "from launch import LaunchDescription\n"
        "def generate_launch_description():\n"
        "    return LaunchDescription()\n"
    )
    action = launcher.include_launch_file(
        package=None, launch_file=str(launch_file), launch_args={"use_rviz": False}
    )
    assert isinstance(action, IncludeLaunchDescription)
    assert action in launcher._description.entities
    # launch args are stringified for the launch system
    assert ("use_rviz", "False") in [tuple(pair) for pair in action.launch_arguments]


def test_include_launch_file_missing_file_raises(launcher):
    with pytest.raises(FileNotFoundError):
        launcher.include_launch_file(
            package=None, launch_file="/nonexistent.launch.py"
        )


def test_include_launch_file_missing_in_package_raises(launcher):
    # automatika_ros_sugar is installed but has no such launch file
    with pytest.raises(FileNotFoundError):
        launcher.include_launch_file(
            package="automatika_ros_sugar", launch_file="no_such.launch.py"
        )
