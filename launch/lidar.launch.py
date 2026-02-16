from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    sllidar_pkg = FindPackageShare('sllidar_ros2')

    serial_port = LaunchConfiguration('serial_port')
    declare_serial_port = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB1',
        description='Serial port for RPLiDAR (e.g. /dev/ttyUSB1 on LeKiwi)')

    sllidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([sllidar_pkg, 'launch', 'sllidar_a1_launch.py'])
        ),
        launch_arguments={'serial_port': serial_port}.items()
    )

    return LaunchDescription([
        declare_serial_port,
        sllidar_launch,
    ])
