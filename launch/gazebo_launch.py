"""Launch Gz Harmonic simulation with BEN spawned in an ocean world.

Thin wrapper: starts Gazebo with ocean.sdf, then includes spawn_ben_launch.py.
For spawning BEN into an externally launched world, use spawn_ben_launch.py
directly with the appropriate world_name argument.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_ben_gazebo = get_package_share_directory('ben_gazebo')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Launch arguments
    namespace = LaunchConfiguration('namespace')
    world = LaunchConfiguration('world')
    rviz = LaunchConfiguration('rviz')
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    z = LaunchConfiguration('z')
    R = LaunchConfiguration('R')
    P = LaunchConfiguration('P')
    Y = LaunchConfiguration('Y')

    declare_namespace = DeclareLaunchArgument(
        'namespace', default_value='ben')
    declare_world = DeclareLaunchArgument(
        'world', default_value=os.path.join(
            pkg_ben_gazebo, 'worlds', 'ocean.sdf'))
    declare_rviz = DeclareLaunchArgument(
        'rviz', default_value='false')
    declare_x = DeclareLaunchArgument('x', default_value='0')
    declare_y = DeclareLaunchArgument('y', default_value='0')
    declare_z = DeclareLaunchArgument('z', default_value='0.17')
    declare_R = DeclareLaunchArgument('R', default_value='0')
    declare_P = DeclareLaunchArgument('P', default_value='0')
    declare_Y = DeclareLaunchArgument('Y', default_value='0')

    # Start Gz sim
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': [
                '-r ',
                '--physics-engine gz-physics-bullet-featherstone-plugin ',
                world,
            ],
            'on_exit_shutdown': 'true',
        }.items(),
    )

    # Spawn BEN (robot_state_publisher + entity spawner + bridge)
    spawn_ben = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ben_gazebo, 'launch', 'spawn_ben_launch.py')),
        launch_arguments={
            'namespace': namespace,
            'world_name': 'ocean',
            'x': x, 'y': y, 'z': z,
            'R': R, 'P': P, 'Y': Y,
        }.items(),
    )

    # RViz (optional)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(pkg_ben_gazebo, 'rviz', 'ben.rviz')],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(rviz),
    )

    return LaunchDescription([
        declare_namespace,
        declare_world,
        declare_rviz,
        declare_x, declare_y, declare_z,
        declare_R, declare_P, declare_Y,
        gz_sim,
        spawn_ben,
        rviz_node,
    ])
