import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def get_share_file(package_name, file_name):
    return os.path.join(get_package_share_directory(package_name), file_name)

def generate_launch_description():
    mav_name = LaunchConfiguration('mav_name', default='firefly')
    world_name = LaunchConfiguration('world_name', default='basic')
    enable_logging = LaunchConfiguration('enable_logging', default='false')
    enable_ground_truth = LaunchConfiguration('enable_ground_truth', default='true')
    log_file = LaunchConfiguration('log_file', default=mav_name)
    debug = LaunchConfiguration('debug', default='false')
    gui = LaunchConfiguration('gui', default='false')
    paused = LaunchConfiguration('paused', default='false')
    verbose = LaunchConfiguration('verbose', default='false')

    config_dir = get_share_file('mav_trajectory_generation_example', 'config/')
    rviz_config_file = os.path.join(config_dir, f'rviz_view_{mav_name}.rviz')
    params_file = os.path.join(config_dir, f'{mav_name}_params.yaml')

    # Planner node
    planner_node = Node(
        package='mav_trajectory_generation_example',
        executable='trajectory_generation_readme_example',
        output='screen',
        parameters=[params_file],
        remappings=[('uav_pose', 'odometry_sensor1/odometry')],
        emulate_tty=True,
    )

    # Sampler node
    sampler_node = Node(
        package='mav_trajectory_generation_ros',
        executable='trajectory_sampler_node',
        output='screen',
        remappings=[('path_segments_4D', 'trajectory')],
        emulate_tty=True,
    )

    # RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', rviz_config_file],
        output='screen',
    )

    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument('mav_name', default_value=mav_name))
    ld.add_action(DeclareLaunchArgument('world_name', default_value=world_name))
    ld.add_action(DeclareLaunchArgument('enable_logging', default_value=enable_logging))
    ld.add_action(DeclareLaunchArgument('enable_ground_truth', default_value=enable_ground_truth))
    ld.add_action(DeclareLaunchArgument('log_file', default_value=log_file))
    ld.add_action(DeclareLaunchArgument('debug', default_value=debug))
    ld.add_action(DeclareLaunchArgument('gui', default_value=gui))
    ld.add_action(DeclareLaunchArgument('paused', default_value=paused))
    ld.add_action(DeclareLaunchArgument('verbose', default_value=verbose))

    ld.add_action(planner_node)
    # ld.add_action(sampler_node)
    # ld.add_action(rviz_node)

    return ld
