import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    pkg_share = get_package_share_directory('ba_perception_pipeline')

    # --- venv handling (Auto-Fix for ModuleNotFoundError) ---
    # Append the venv site-packages to the environment's PYTHONPATH.
    venv_site = os.path.expanduser('~/venvs/ba_depth_node/lib/python3.10/site-packages')
    env = os.environ.copy()
    if os.path.exists(venv_site):
        existing_pp = env.get('PYTHONPATH', '')
        if existing_pp:
            env['PYTHONPATH'] = f"{venv_site}:{existing_pp}"
        else:
            env['PYTHONPATH'] = venv_site

    # Parameters
    config_file = LaunchConfiguration('config_file')
    depth_cal_file = LaunchConfiguration('depth_cal_file')

    declare_config_file = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(pkg_share, 'config', 'perception_pipeline.yaml'),
        description='Full path to the main configuration file'
    )

    declare_depth_cal_file = DeclareLaunchArgument(
        'depth_cal_file',
        default_value=os.path.join(pkg_share, 'config', 'depth_calibration.yaml'),
        description='Full path to the depth calibration file'
    )

    # 1. Perception Pipeline Node
    perception_node = Node(
        package='ba_perception_pipeline',
        executable='perception_pipeline_node',
        name='ba_perception_pipeline',
        output='screen',
        parameters=[config_file, {'depth_calibration_file': depth_cal_file}],
        env=env  # Use modified environment
    )

    # 2. Goal Generator Node
    goal_node = Node(
        package='ba_perception_pipeline',
        executable='goal_generator_node',
        name='ba_goal_generator',
        output='screen',
        parameters=[{
            'planning_group': 'bracket_arm',
            'base_frame': 'base_link',
            'z_offset': 0.10,
        }],
        env=env  # Use modified environment
    )

    ld = LaunchDescription()
    ld.add_action(declare_config_file)
    ld.add_action(declare_depth_cal_file)
    ld.add_action(perception_node)
    ld.add_action(goal_node)

    return ld
