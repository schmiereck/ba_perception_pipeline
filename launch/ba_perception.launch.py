import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression

def generate_launch_description():
    pkg_share = get_package_share_directory('ba_perception_pipeline')

    # --- venv handling (Auto-Fix for ModuleNotFoundError) ---
    venv_site = os.path.expanduser('~/venvs/ba_depth_node/lib/python3.10/site-packages')
    env = os.environ.copy()
    if os.path.exists(venv_site):
        existing_pp = env.get('PYTHONPATH', '')
        if existing_pp:
            env['PYTHONPATH'] = f"{venv_site}:{existing_pp}"
        else:
            env['PYTHONPATH'] = venv_site

    # --- Arguments ---
    config_file = LaunchConfiguration('config_file')
    depth_cal_file = LaunchConfiguration('depth_cal_file')
    auto_execute = LaunchConfiguration('auto_execute')
    target_plane_z = LaunchConfiguration('target_plane_z')

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

    declare_auto_execute = DeclareLaunchArgument(
        'auto_execute',
        default_value='false',
        description='If true, the robot will automatically move to the target'
    )

    declare_target_plane_z = DeclareLaunchArgument(
        'target_plane_z',
        default_value='0.00',
        description='Ray-plane intersection Z in robot frame (meters). VLM empirically latches on object bottom edge at steep angles.'
    )

    velocity_scaling = LaunchConfiguration('velocity_scaling')
    acceleration_scaling = LaunchConfiguration('acceleration_scaling')

    declare_velocity_scaling = DeclareLaunchArgument(
        'velocity_scaling',
        default_value='0.3',
        description='MoveGroup max_velocity_scaling_factor (0.0-1.0). RViz equivalent: "Velocity Scaling".'
    )

    declare_acceleration_scaling = DeclareLaunchArgument(
        'acceleration_scaling',
        default_value='0.3',
        description='MoveGroup max_acceleration_scaling_factor (0.0-1.0). RViz equivalent: "Accel. Scaling".'
    )

    # 1. Perception Pipeline Node
    perception_node = Node(
        package='ba_perception_pipeline',
        executable='perception_pipeline_node',
        name='ba_perception_pipeline',
        output='screen',
        parameters=[
            config_file,
            {'depth_calibration_file': depth_cal_file},
            {'target_plane_z': PythonExpression(['float("', target_plane_z, '")'])},
        ],
        env=env
    )

    # 2. Goal Generator Node
    goal_node = Node(
        package='ba_perception_pipeline',
        executable='goal_generator_node',
        name='ba_goal_generator',
        output='screen',
        parameters=[{
            'planning_group': 'arm',
            'base_frame': 'base_link',
            'tip_link': 'grasp_link',
            'z_offset': 0.0,
            'max_reach': 0.35,
            'auto_execute': auto_execute,
            'velocity_scaling': PythonExpression(['float("', velocity_scaling, '")']),
            'acceleration_scaling': PythonExpression(['float("', acceleration_scaling, '")']),
        }],
        env=env
    )

    ld = LaunchDescription()
    ld.add_action(declare_config_file)
    ld.add_action(declare_depth_cal_file)
    ld.add_action(declare_auto_execute)
    ld.add_action(declare_target_plane_z)
    ld.add_action(declare_velocity_scaling)
    ld.add_action(declare_acceleration_scaling)
    ld.add_action(perception_node)
    ld.add_action(goal_node)

    return ld
