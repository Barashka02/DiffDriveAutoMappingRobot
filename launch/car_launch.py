from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    # Get package directories
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    robot_path = get_package_share_directory('robot')

    sdf_file = os.path.join(robot_path, 'models', 'robot', 'combined.sdf')
    with open(sdf_file, 'r') as infp:
        robot_desc = infp.read()

    # Launch Gazebo with your world file
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': os.path.join(robot_path, 'worlds/track.sdf')}.items(),
        #launch_arguments={'gz_args': os.path.join(robot_path, 'worlds/ware_house.sdf')}.items(),

    )



    # Launch teleop_keyboard node
    teleop_keyboard = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_keyboard',
        output='screen',
        prefix='xterm -e',  # Opens teleop in a new terminal window.
        remappings=[('/cmd_vel', '/cmd_vel')]
    )

    # Define RViz node
    rviz_node = Node(
        package='rviz2',
        namespace='',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(get_package_share_directory('robot'), 'config', 'robot_config.rviz')],
        output='screen'
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
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'
        ],
        output='screen'
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[
            {'use_sim_time': True},
            {'robot_description': robot_desc},
        ]
    )


    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_node',
        output='screen',
        parameters=[
            os.path.join(robot_path, 'config/ekf.yaml'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )

    #-------------

    #-----------------TF TREE INFO--------------------------

    odom_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=["0", "0", "0", "0", "0", "0", "odom", "base_link"],
        name='odom_to_base_link'
    )

    map_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
        name='map_to_odom'
    )

    world_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=["0", "0", "0", "0", "0", "0", "world", "map"],
        name='world_to_map'
    )

    chassis_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=["0", "0", "0", "0", "0", "0", "base_link", "chassis"],
        name='base_link_to_chassis'
        )
    apriltags = Node(
            package='apriltag_ros',
            executable='apriltag_node',
            remappings=[
                ('image_rect', '/camera'),
                ('camera_info', '/camera_info'),
                ],
            parameters=[robot_path+'/config/apriltags.yaml']
        )

    nav2 = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')),
            launch_arguments={'slam_params_file': os.path.join(robot_path,'config/nav2params.yaml')}.items(),
        )

    slam = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('slam_toolbox'), 'launch', 'online_sync_launch.py')),
            launch_arguments={'slam_params_file': os.path.join(robot_path,'config/mapper_params_localization.yaml')}.items(),
        )
    return LaunchDescription([
        # Set the resource path so Gazebo finds your models
        SetEnvironmentVariable(name='GZ_SIM_RESOURCE_PATH', value=robot_path),

        # Optional argument to launch RViz
        DeclareLaunchArgument('rviz', default_value='true', description='Open RViz.'),

        DeclareLaunchArgument(name='use_sim_time', default_value='True', description='Flag to enable use_sim_time'),

        # Launch components
        gz_sim,
        bridge,
        teleop_keyboard,
        rviz_node,
        #joint_state_publisher_gui,
        #odom_tf,
        #map_tf,
        world_tf,

        chassis_tf,
        robot_state_publisher,
        apriltags,

        robot_localization_node,
        nav2, 
        slam
    ])
