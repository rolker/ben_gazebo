"""Launch BEN in the ocean world at default spawn position."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_ben_gazebo = get_package_share_directory('ben_gazebo')

    rviz = LaunchConfiguration('rviz')
    declare_rviz = DeclareLaunchArgument('rviz', default_value='false')

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ben_gazebo, 'launch', 'gazebo_launch.py')),
        launch_arguments={
            'x': '158',
            'y': '108',
            'z': '0.5',
            'Y': '-2.76',
            'rviz': rviz,
        }.items(),
    )

    return LaunchDescription([
        declare_rviz,
        gazebo_launch,
    ])
