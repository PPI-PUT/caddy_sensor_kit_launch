import os
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, IncludeLaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ld = LaunchDescription()

    # Set env var to print messages to stdout immediately
    arg = SetEnvironmentVariable('RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED', '1')
    ld.add_action(arg)

    parameters_file_path = Path(get_package_share_directory('caddy_sensor_kit_launch'), 'config/imu', 'xsens_mti.param.yaml')
    xsens_mti_node = Node(
            package='bluespace_ai_xsens_mti_driver',
            executable='xsens_mti_node',
            name='xsens_mti_node',
            namespace='imu',
            output='screen',
            parameters=[parameters_file_path],
            arguments=[]
            )
    ld.add_action(xsens_mti_node)

    # # Robot State Publisher node
    # urdf_file_path = os.path.join(get_package_share_directory('bluespace_ai_xsens_mti_driver'), 'urdf', 'MTi_10.urdf')
    # state_publisher_node = Node(
    #     package='robot_state_publisher',
    #     executable='robot_state_publisher',
    #     name='xsens_state_publisher',
    #     namespace='imu',
    #     output='screen',
    #     arguments=[urdf_file_path],
    # )
    # ld.add_action(state_publisher_node)

    return ld
