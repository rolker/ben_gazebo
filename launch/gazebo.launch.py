"""Launch Gz Harmonic simulation with BEN spawned in an ocean world."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


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
    declare_z = DeclareLaunchArgument('z', default_value='0.5')
    declare_R = DeclareLaunchArgument('R', default_value='0')
    declare_P = DeclareLaunchArgument('P', default_value='0')
    declare_Y = DeclareLaunchArgument('Y', default_value='0')

    # Process xacro
    xacro_file = os.path.join(pkg_ben_gazebo, 'urdf', 'ben.xacro')
    robot_description = ParameterValue(
        Command(['xacro ', xacro_file, ' namespace:=', namespace]),
        value_type=str,
    )

    # Start Gz sim
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': [
            '-r ',
            '--physics-engine gz-physics-bullet-featherstone-plugin ',
            world,
        ]}.items(),
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

    # Spawn model into Gz from the robot_description topic
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', namespace,
            '-topic', [namespace, '/robot_description'],
            '-x', x, '-y', y, '-z', z,
            '-R', R, '-P', P, '-Y', Y,
        ],
        output='screen',
    )

    # ros_gz_bridge for all sensor and thruster topics
    bridge_args = [
        # Clock
        '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        # GPS NavSat
        ['/world/ocean/model/', namespace,
         '/link/', namespace, '/gps/sensor/', namespace,
         '_gps_navsat/navsat'
         '@sensor_msgs/msg/NavSatFix[gz.msgs.NavSat'],
        # Heading IMU
        ['/world/ocean/model/', namespace,
         '/link/', namespace, '/heading/sensor/', namespace,
         '_heading_imu/imu'
         '@sensor_msgs/msg/Imu[gz.msgs.IMU'],
        # POS MV NavSat
        ['/world/ocean/model/', namespace,
         '/link/', namespace, '/motion_sensor/sensor/', namespace,
         '_posmv_navsat/navsat'
         '@sensor_msgs/msg/NavSatFix[gz.msgs.NavSat'],
        # POS MV IMU
        ['/world/ocean/model/', namespace,
         '/link/', namespace, '/motion_sensor/sensor/', namespace,
         '_posmv_imu/imu'
         '@sensor_msgs/msg/Imu[gz.msgs.IMU'],
        # Lidar point cloud
        ['/world/ocean/model/', namespace,
         '/link/', namespace, '/lidar/sensor/lidar_sensor/scan/points'
         '@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked'],
        # Forward camera image
        ['/world/ocean/model/', namespace,
         '/link/', namespace,
         '/forward_camera/sensor/forward_camera_sensor/image'
         '@sensor_msgs/msg/Image[gz.msgs.Image'],
        # Forward camera info
        ['/world/ocean/model/', namespace,
         '/link/', namespace,
         '/forward_camera/sensor/forward_camera_sensor/camera_info'
         '@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo'],
        # Thruster command (ROS → Gz)
        ['/', namespace, '/thrusters/main/thrust'
         '@std_msgs/msg/Float64]gz.msgs.Double'],
        # Steering command (ROS → Gz)
        ['/', namespace, '/thrusters/main/pos'
         '@std_msgs/msg/Float64]gz.msgs.Double'],
    ]

    # Add panoramic cameras (6 cameras, image + camera_info each)
    for i in range(1, 7):
        bridge_args.extend([
            ['/world/ocean/model/', namespace,
             '/link/', namespace,
             f'/pano_{i}/sensor/pano_{i}_sensor/image'
             '@sensor_msgs/msg/Image[gz.msgs.Image'],
            ['/world/ocean/model/', namespace,
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
            (['/world/ocean/model/', namespace,
              '/link/', namespace, '/gps/sensor/', namespace,
              '_gps_navsat/navsat'],
             [namespace, '/sensors/gps/fix']),
            # Heading
            (['/world/ocean/model/', namespace,
              '/link/', namespace, '/heading/sensor/', namespace,
              '_heading_imu/imu'],
             [namespace, '/sensors/heading/data']),
            # POS MV NavSat
            (['/world/ocean/model/', namespace,
              '/link/', namespace, '/motion_sensor/sensor/', namespace,
              '_posmv_navsat/navsat'],
             [namespace, '/sensors/posmv/fix']),
            # POS MV IMU
            (['/world/ocean/model/', namespace,
              '/link/', namespace, '/motion_sensor/sensor/', namespace,
              '_posmv_imu/imu'],
             [namespace, '/sensors/posmv/imu/data']),
            # Lidar
            (['/world/ocean/model/', namespace,
              '/link/', namespace,
              '/lidar/sensor/lidar_sensor/scan/points'],
             [namespace, '/sensors/lidar/points']),
            # Forward camera
            (['/world/ocean/model/', namespace,
              '/link/', namespace,
              '/forward_camera/sensor/forward_camera_sensor/image'],
             [namespace, '/sensors/cameras/forward_camera/image_raw']),
            (['/world/ocean/model/', namespace,
              '/link/', namespace,
              '/forward_camera/sensor/forward_camera_sensor/camera_info'],
             [namespace, '/sensors/cameras/forward_camera/camera_info']),
        ] + [
            remap
            for i in range(1, 7)
            for remap in [
                (['/world/ocean/model/', namespace,
                  '/link/', namespace,
                  f'/pano_{i}/sensor/pano_{i}_sensor/image'],
                 [namespace, f'/sensors/cameras/pano_{i}/image_raw']),
                (['/world/ocean/model/', namespace,
                  '/link/', namespace,
                  f'/pano_{i}/sensor/pano_{i}_sensor/camera_info'],
                 [namespace, f'/sensors/cameras/pano_{i}/camera_info']),
            ]
        ],
        parameters=[{'use_sim_time': True}],
        output='screen',
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
        robot_state_publisher,
        spawn_entity,
        bridge,
        rviz_node,
    ])
