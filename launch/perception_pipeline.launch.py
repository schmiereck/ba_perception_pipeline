"""Launch the perception pipeline node.

Starts ``perception_pipeline_node`` which combines depth estimation,
VLM target detection, and backprojection in a single process.

The node is launched via the venv Python interpreter so that torch,
transformers, groq, and google-genai are on sys.path.

Trigger:  ros2 topic pub --once /perception/detect_request std_msgs/String "data: 'the red cup'"
Result:   /perception/target_pose  (geometry_msgs/PoseStamped)
Status:   /perception/status       (std_msgs/String)
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


_DEFAULT_VENV_PYTHON = os.path.expanduser(
    '~/venvs/ba_depth_node/bin/python3')


def generate_launch_description() -> LaunchDescription:
    default_params = PathJoinSubstitution([
        FindPackageShare('ba_perception_pipeline'),
        'config',
        'perception_pipeline.yaml',
    ])

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params,
        description='Path to the parameter YAML.',
    )

    venv_python_arg = DeclareLaunchArgument(
        'venv_python',
        default_value=_DEFAULT_VENV_PYTHON,
        description='Path to the venv Python interpreter.',
    )

    pipeline_node = Node(
        package='ba_perception_pipeline',
        executable='perception_pipeline_node',
        name='perception_pipeline',
        output='screen',
        emulate_tty=True,
        parameters=[LaunchConfiguration('params_file')],
        prefix=[LaunchConfiguration('venv_python'), ' '],
    )

    return LaunchDescription([
        params_file_arg,
        venv_python_arg,
        pipeline_node,
    ])
