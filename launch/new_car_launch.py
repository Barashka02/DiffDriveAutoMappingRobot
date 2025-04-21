from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, AppendEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from pathlib import Path
from launch.actions import TimerAction

import os

def generate_launch_description():
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    robot_path = get_package_share_directory('robot')

    # Use the combined SDF file (with correct texture paths)
    sdf_file = os.path.join(robot_path, 'models', 'robot', 'combined.sdf')
    with open(sdf_file, 'r') as infp:
        robot_desc = infp.read()

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-r ' + os.path.join(robot_path, 'worlds/track.sdf')}.items(),
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/camera@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/scan/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/navsat@sensor_msgs/msg/NavSatFix[gz.msgs.NavSat',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/joint_state@sensor_msgs/msg/JointState[gz.msgs.JointState'



        ],
        remappings=[('/world/track/model/combined/joint_state', '/joint_states')],
        output='screen'
    )

    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_node',
        output='screen',
        parameters=[os.path.join(robot_path, 'config', 'ekf.yaml'), {'use_sim_time': True}]
    )

    joy = Node(
        package='joy',
        executable='joy_node',
    )

    keyboard = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        output='screen',
        prefix='xterm -e'
    )

    rviz = Node(
        package='rviz2',
        namespace='',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(robot_path, 'config', 'config_file.rviz')],
        output='screen'
    )

    executive_node = Node(
        package='robot',
        executable='executive',
        name='executive_node',
        output='screen'
    )

    executive_delayed = TimerAction(
        period=7.0,
        actions=[executive_node]
    )

    pose_node = Node(
        package='robot',
        executable='pose_srv',
        name='pose_node',
        output='screen'
    )
    
    # Publish robot description from the combined SDF file
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[
            {'robot_description': robot_desc},
            {'use_sim_time': True},
        ]
    )

    apriltags = Node(
        package='apriltag_ros',
        executable='apriltag_node',
        remappings=[
            ('image_rect', '/camera'),
            ('camera_info', '/camera_info'),
        ],
        parameters=[os.path.join(robot_path, 'config', 'apriltags.yaml')]
    )

    # Add the missing TF publishers
    odom_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=["1", "0", "0", "0", "0", "0", "odom", "base_link"],
        name='odom_tf'
    )
    map_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=["1", "0", "0", "0", "0", "0", "map", "odom"],
        name='map_tf'
    )
    world_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=["1", "0", "0", "0", "0", "0", "world", "map"],
        name='world_tf'
    )

    nav2 = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')),
            launch_arguments={'use_sim_time': 'true', 'params_file': os.path.join(robot_path,'config/nav2_params.yaml')}.items(),
        )

    slam = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('slam_toolbox'), 'launch', 'online_sync_launch.py')),
            launch_arguments={'use_sim_time': 'true', 'slam_params_file': os.path.join(robot_path,'config/mapper_params_localization.yaml')}.items(),
        )

    return LaunchDescription([
        AppendEnvironmentVariable(name='GZ_SIM_RESOURCE_PATH', value=robot_path),
        AppendEnvironmentVariable(name='GZ_SIM_RESOURCE_PATH', value=str(Path(os.path.join(robot_path)).parent.resolve())),
        gz_sim,
        bridge,
        # Uncomment "joy" if needed:
        # joy,
        rviz,
        keyboard,
        robot_state_publisher,
        odom_tf,
        
        map_tf,
        world_tf,
        apriltags,
        robot_localization_node,
        slam,
        nav2,
        executive_delayed,
        pose_node,
        # executive_node,
    ])
