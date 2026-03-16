"""Spawn BEN into an already-running Gazebo world."""

import os
import subprocess
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _spawn_ben(context, *args, **kwargs):
    """Generate spawn node using -string with pre-converted SDF.

    The -topic approach (reading URDF from robot_description) has a Gz Harmonic
    bug where the model is added to the server but never appears in the GUI.
    Converting to SDF first and using -string works correctly.
    """
    pkg_ben_gazebo = get_package_share_directory('ben_gazebo')
    ns = LaunchConfiguration('namespace').perform(context)
    x = LaunchConfiguration('x').perform(context)
    y = LaunchConfiguration('y').perform(context)
    z = LaunchConfiguration('z').perform(context)
    R = LaunchConfiguration('R').perform(context)
    P = LaunchConfiguration('P').perform(context)
    Y = LaunchConfiguration('Y').perform(context)

    xacro_file = os.path.join(pkg_ben_gazebo, 'urdf', 'ben.xacro')

    # xacro → URDF → SDF
    # gz sdf -p requires a real file path, so we use a temp file.
    urdf = subprocess.check_output(
        ['xacro', xacro_file, f'namespace:={ns}'],
        text=True,
    )
    with tempfile.NamedTemporaryFile(
        mode='w', suffix='.urdf', delete=False
    ) as f:
        f.write(urdf)
        urdf_path = f.name
    try:
        sdf = subprocess.check_output(
            ['gz', 'sdf', '-p', urdf_path],
            text=True,
        )
    finally:
        os.unlink(urdf_path)

    return [
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-string', sdf,
                '-name', ns,
                '-x', x, '-y', y, '-z', z,
                '-R', R, '-P', P, '-Y', Y,
            ],
            output='screen',
        ),
    ]


def generate_launch_description():
    pkg_ben_gazebo = get_package_share_directory('ben_gazebo')

    # Launch arguments
    namespace = LaunchConfiguration('namespace')
    world_name = LaunchConfiguration('world_name')

    declare_namespace = DeclareLaunchArgument(
        'namespace', default_value='ben')
    declare_world_name = DeclareLaunchArgument(
        'world_name', default_value='ocean',
        description='Gz world name (used in bridge topic paths)')
    declare_x = DeclareLaunchArgument('x', default_value='0')
    declare_y = DeclareLaunchArgument('y', default_value='0')
    declare_z = DeclareLaunchArgument('z', default_value='0.17')
    declare_R = DeclareLaunchArgument('R', default_value='0')
    declare_P = DeclareLaunchArgument('P', default_value='0')
    declare_Y = DeclareLaunchArgument('Y', default_value='0')

    # Process xacro for robot_state_publisher (needs URDF, not SDF)
    xacro_file = os.path.join(pkg_ben_gazebo, 'urdf', 'ben.xacro')
    robot_description = ParameterValue(
        Command(['xacro ', xacro_file, ' namespace:=', namespace]),
        value_type=str,
    )

    # Robot state publisher (publishes TF from URDF)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=namespace,
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
        }],
        output='screen',
    )

    # ros_gz_bridge for all sensor and thruster topics
    # world_name is used in Gz topic paths instead of hardcoded 'ocean'
    bridge_args = [
        # Clock
        '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        # GPS NavSat
        ['/world/', world_name, '/model/', namespace,
         '/link/', namespace, '/gps/sensor/', namespace,
         '_gps_navsat/navsat'
         '@sensor_msgs/msg/NavSatFix[gz.msgs.NavSat'],
        # Heading IMU
        ['/world/', world_name, '/model/', namespace,
         '/link/', namespace, '/heading/sensor/', namespace,
         '_heading_imu/imu'
         '@sensor_msgs/msg/Imu[gz.msgs.IMU'],
        # POS MV NavSat → sensors/nav/position
        ['/world/', world_name, '/model/', namespace,
         '/link/', namespace, '/motion_sensor/sensor/', namespace,
         '_posmv_navsat/navsat'
         '@sensor_msgs/msg/NavSatFix[gz.msgs.NavSat'],
        # POS MV IMU → sensors/nav/orientation
        ['/world/', world_name, '/model/', namespace,
         '/link/', namespace, '/motion_sensor/sensor/', namespace,
         '_posmv_imu/imu'
         '@sensor_msgs/msg/Imu[gz.msgs.IMU'],
        # Lidar point cloud
        ['/world/', world_name, '/model/', namespace,
         '/link/', namespace, '/lidar/sensor/lidar_sensor/scan/points'
         '@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked'],
        # Forward camera image
        ['/world/', world_name, '/model/', namespace,
         '/link/', namespace,
         '/forward_camera/sensor/forward_camera_sensor/image'
         '@sensor_msgs/msg/Image[gz.msgs.Image'],
        # Forward camera info
        ['/world/', world_name, '/model/', namespace,
         '/link/', namespace,
         '/forward_camera/sensor/forward_camera_sensor/camera_info'
         '@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo'],
        # Thruster command (ROS → Gz)
        ['/', namespace, '/thrusters/main/thrust'
         '@std_msgs/msg/Float64]gz.msgs.Double'],
        # Steering command (ROS → Gz)
        ['/', namespace, '/thrusters/main/pos'
         '@std_msgs/msg/Float64]gz.msgs.Double'],
        # MBES ram position command (ROS → Gz)
        ['/', namespace, '/mbes/ram/pos'
         '@std_msgs/msg/Float64]gz.msgs.Double'],
    ]

    # Add panoramic cameras (6 cameras, image + camera_info each)
    for i in range(1, 7):
        bridge_args.extend([
            ['/world/', world_name, '/model/', namespace,
             '/link/', namespace,
             f'/pano_{i}/sensor/pano_{i}_sensor/image'
             '@sensor_msgs/msg/Image[gz.msgs.Image'],
            ['/world/', world_name, '/model/', namespace,
             '/link/', namespace,
             f'/pano_{i}/sensor/pano_{i}_sensor/camera_info'
             '@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo'],
        ])

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=bridge_args,
        remappings=[
            # GPS
            (['/world/', world_name, '/model/', namespace,
              '/link/', namespace, '/gps/sensor/', namespace,
              '_gps_navsat/navsat'],
             [namespace, '/sensors/gps/fix']),
            # Heading
            (['/world/', world_name, '/model/', namespace,
              '/link/', namespace, '/heading/sensor/', namespace,
              '_heading_imu/imu'],
             [namespace, '/sensors/heading/data']),
            # POS MV NavSat → sensors/nav/position (matches ben_sim.yaml)
            (['/world/', world_name, '/model/', namespace,
              '/link/', namespace, '/motion_sensor/sensor/', namespace,
              '_posmv_navsat/navsat'],
             [namespace, '/sensors/nav/position']),
            # POS MV IMU → sensors/nav/orientation (matches ben_sim.yaml)
            (['/world/', world_name, '/model/', namespace,
              '/link/', namespace, '/motion_sensor/sensor/', namespace,
              '_posmv_imu/imu'],
             [namespace, '/sensors/nav/orientation']),
            # Lidar
            (['/world/', world_name, '/model/', namespace,
              '/link/', namespace,
              '/lidar/sensor/lidar_sensor/scan/points'],
             [namespace, '/sensors/lidar/points']),
            # Forward camera
            (['/world/', world_name, '/model/', namespace,
              '/link/', namespace,
              '/forward_camera/sensor/forward_camera_sensor/image'],
             [namespace, '/sensors/cameras/forward_camera/image_raw']),
            (['/world/', world_name, '/model/', namespace,
              '/link/', namespace,
              '/forward_camera/sensor/forward_camera_sensor/camera_info'],
             [namespace, '/sensors/cameras/forward_camera/camera_info']),
        ] + [
            remap
            for i in range(1, 7)
            for remap in [
                (['/world/', world_name, '/model/', namespace,
                  '/link/', namespace,
                  f'/pano_{i}/sensor/pano_{i}_sensor/image'],
                 [namespace, f'/sensors/cameras/pano_{i}/image_raw']),
                (['/world/', world_name, '/model/', namespace,
                  '/link/', namespace,
                  f'/pano_{i}/sensor/pano_{i}_sensor/camera_info'],
                 [namespace, f'/sensors/cameras/pano_{i}/camera_info']),
            ]
        ],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    return LaunchDescription([
        declare_namespace,
        declare_world_name,
        declare_x, declare_y, declare_z,
        declare_R, declare_P, declare_Y,
        robot_state_publisher,
        OpaqueFunction(function=_spawn_ben),
        bridge,
    ])
