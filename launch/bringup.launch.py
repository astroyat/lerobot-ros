from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('lerobot_ros')
    sllidar_pkg = FindPackageShare('sllidar_ros2')
    rf2o_pkg = FindPackageShare('rf2o_laser_odometry')
    nav2_pkg = FindPackageShare('nav2_bringup')
    slam_pkg = FindPackageShare('slam_toolbox')

    # Arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    mode = LaunchConfiguration('mode')
    declare_mode = DeclareLaunchArgument(
        'mode',
        default_value='slam',
        description='Operation mode: "slam" (mapping), "nav" (navigation), or "idle" (sensors and odom only)')

    serial_port = LaunchConfiguration('serial_port')
    declare_serial_port = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB1',
        description='Serial port for RPLiDAR (e.g. /dev/ttyUSB1 if motor bus is on /dev/ttyUSB0)')

    # LIDAR
    sllidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([sllidar_pkg, 'launch', 'sllidar_a1_launch.py'])
        ),
        launch_arguments={'serial_port': serial_port}.items()
    )

    # Odometry
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
    )

    # TF (base_link -> laser)
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
    )

    # SLAM Toolbox (mode:=slam)
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([slam_pkg, 'launch', 'online_async_launch.py'])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam_params_file': PathJoinSubstitution([pkg_share, 'config', 'slam.yaml'])
        }.items(),
        condition=IfCondition(PythonExpression(["'", mode, "' == 'slam'"]))
    )

    # Navigation Nav2 (mode:=nav)
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([nav2_pkg, 'launch', 'navigation_launch.py'])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': PathJoinSubstitution([pkg_share, 'config', 'nav2.yaml'])
        }.items(),
        condition=IfCondition(PythonExpression(["'", mode, "' == 'nav'"]))
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_mode,
        declare_serial_port,

        # Always run sensors and TF
        sllidar_launch,
        rf2o_node,
        tf_node,

        # Conditional SLAM or Nav2
        slam_launch,
        nav2_launch,
    ])
