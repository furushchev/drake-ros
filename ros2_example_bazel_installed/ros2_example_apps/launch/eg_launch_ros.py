from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """
    Launch file demonstrating use of launch_ros.actions.Node with Bazel.

    This launch file uses Node actions instead of ExecuteProcess, which is
    possible because the executables are registered as part of a ROS package
    via the ros_package rule.

    See bazel_ros2_rules/lib/README.md, Launch Files section for details.
    """
    return LaunchDescription([
        # Running a talker written in python.
        Node(
            package='ros2_example_nodes',
            executable='talker',
            output='screen',
        ),
        # Running a listener written in cpp.
        Node(
            package='ros2_example_nodes',
            executable='listener',
            output='screen',
        ),
    ])
