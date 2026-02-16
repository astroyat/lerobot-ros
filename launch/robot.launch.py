import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    sllidar_pkg = FindPackageShare('sllidar_ros2')

    # Arguments
    serial_port = LaunchConfiguration('serial_port')
    declare_serial_port = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB1',
        description='Serial port for RPLiDAR A1 (default /dev/ttyUSB1)')

    launch_motor_host = LaunchConfiguration('launch_motor_host')
    declare_launch_motor_host = DeclareLaunchArgument(
        'launch_motor_host',
        default_value='true',
        description='Start the LeKiwi motor host daemon (ZMQ port 5555)')

    launch_lidar = LaunchConfiguration('launch_lidar')
    declare_launch_lidar = DeclareLaunchArgument(
        'launch_lidar',
        default_value='true',
        description='Start the RPLiDAR A1 driver (/scan)')

    launch_odom = LaunchConfiguration('launch_odom')
    declare_launch_odom = DeclareLaunchArgument(
        'launch_odom',
        default_value='true',
        description='Start rf2o laser odometry node on the robot (/odom)')

    launch_tf = LaunchConfiguration('launch_tf')
    declare_launch_tf = DeclareLaunchArgument(
        'launch_tf',
        default_value='true',
        description='Publish static TF base_link -> laser on the robot')

    # 1. LeKiwi Motor Host Daemon (ZMQ on port 5555)
    # Uses ~/lerobot/examples/lekiwi/lekiwi_host.sh to ensure correct conda/python env
    host_script = os.path.expanduser('~/lerobot/examples/lekiwi/lekiwi_host.sh')
    motor_host_proc = ExecuteProcess(
        cmd=['bash', host_script],
        output='screen',
        condition=IfCondition(launch_motor_host),
    )

    # 2. RPLiDAR A1 Driver (/scan)
    sllidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([sllidar_pkg, 'launch', 'sllidar_a1_launch.py'])
        ),
        launch_arguments={'serial_port': serial_port}.items(),
        condition=IfCondition(launch_lidar),
    )

    # 3. Static TF (base_link -> laser)
    tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_pub_laser',
        arguments=[
            '--x', '-0.1',
            '--y', '0.0',
            '--z', '0.3',
            '--yaw', '0.0',
            '--pitch', '0.0',
            '--roll', '0.0',
            '--frame-id', 'base_link',
            '--child-frame-id', 'laser',
        ],
        output='screen',
        condition=IfCondition(launch_tf),
    )

    # 4. Laser Odometry (rf2o)
    rf2o_node = Node(
        package='rf2o_laser_odometry',
        executable='rf2o_laser_odometry_node',
        name='rf2o_laser_odometry',
        output='screen',
        parameters=[{
            'laser_scan_topic': '/scan',
            'odom_topic': '/odom',
            'publish_tf': True,
            'base_frame_id': 'base_link',
            'odom_frame_id': 'odom',
            'init_pose_from_topic': '',
            'freq': 10.0
        }],
        condition=IfCondition(launch_odom),
    )

    return LaunchDescription([
        declare_serial_port,
        declare_launch_motor_host,
        declare_launch_lidar,
        declare_launch_odom,
        declare_launch_tf,

        motor_host_proc,
        sllidar_launch,
        tf_node,
        rf2o_node,
    ])
