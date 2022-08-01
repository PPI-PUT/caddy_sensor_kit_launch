import os
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, IncludeLaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, Command
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    urdf_file_path = os.path.join(get_package_share_directory('caddy_sensor_kit_description'), 'urdf', 'sensors.xacro')
    rviz_config_path = os.path.join(get_package_share_directory('caddy_sensor_kit_launch'), 'config/rviz', 'display.rviz')


    # Temporary transforms
    # bl_imu_publisher_node = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     arguments=["0.7", "0.0", "2.0", "0.0", "0", "0", "base_link", "imu_link"]
    # )
    #
    # bl_lsf_publisher_node = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     arguments=["2.0", "0.0", "2.12", "3.14159", "0", "0", "base_link", "lidar_sensor_front"]
    # )
    #
    # bl_lsr_publisher_node = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     arguments=["-0.2", "0.0", "2.12", "0.0", "0", "0", "base_link", "lidar_sensor_rear"]
    # )
    #
    # bl_camera_publisher_node = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     arguments=["2.1", "0.0", "2.0", "0.0", "0", "0", "base_link", "blackfly_s"]
    # )

    # Lidar
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            launch_file_path=PathJoinSubstitution([
                FindPackageShare('caddy_sensor_kit_launch'), 'launch', 'lidar.launch.py'
            ]),
        )
    )

    # IMU
    imu_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            launch_file_path=PathJoinSubstitution([
                FindPackageShare('caddy_sensor_kit_launch'), 'launch', 'imu.launch.py'
            ]),
        )
    )

    # Camera
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            launch_file_path=PathJoinSubstitution([
                FindPackageShare('caddy_sensor_kit_launch'), 'launch', 'camera.launch.py'
            ]),
        )
    )

    # Rviz
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=[["-d"], [rviz_config_path]],
    )

    state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='sensors_state_publisher',
        namespace='sensors',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_description': Command(['xacro', ' ', urdf_file_path])
        }]
    )

    return LaunchDescription([
        state_publisher_node,
        lidar_launch,
        imu_launch,
        camera_launch,
        rviz2_node
    ])

