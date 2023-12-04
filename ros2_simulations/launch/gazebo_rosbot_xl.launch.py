from launch import LaunchDescription, LaunchContext
from launch.actions import (
    RegisterEventHandler,
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    PathJoinSubstitution,
    Command,
    PythonExpression,
    FindExecutable,
    LaunchConfiguration,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare

from ament_index_python.packages import get_package_share_directory


def launch_gz_bridge(context: LaunchContext, *args, **kwargs):
    camera_model = context.perform_substitution(LaunchConfiguration('camera_model'))
    lidar_model = context.perform_substitution(LaunchConfiguration('lidar_model'))

    gz_args = ['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock']
    gz_remapping = []
    depth_camera_parent_tf = None

    # Add camera topic
    if camera_model.startswith('intel_realsense'):
        gz_args.append(
            '/camera/color/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo'
        )
        gz_args.append('/camera/color/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image')
        gz_args.append('/camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo')
        gz_args.append('/camera/depth@sensor_msgs/msg/Image[ignition.msgs.Image')
        gz_args.append(
            '/camera/depth/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked'
        )

        gz_remapping.append(('/camera/camera_info', '/camera/depth/camera_info'))
        gz_remapping.append(('/camera/depth', '/camera/depth/image_raw'))

        depth_camera_parent_tf = 'camera_depth_frame'
    else:
        pass

    # Add lidar topic
    if lidar_model.startswith('slamtec_rplidar'):
        gz_args.append('/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan')

    else:
        pass

    gz_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        arguments=gz_args,
        remappings=gz_remapping,
        output='screen',
    )

    # The frame of the point cloud from ignition gazebo 6 isn't provided by <frame_id>.
    # See https://github.com/gazebosim/gz-sensors/issues/239
    point_cloud_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='point_cloud_tf',
        output='log',
        arguments=[
            '0.0',
            '0.0',
            '0.0',
            '0.0',
            '0.0',
            '0.0',
            depth_camera_parent_tf,
            'rosbot_xl/base_link/camera_depth',
        ],
    )

    if depth_camera_parent_tf:
        return [gz_bridge_node, point_cloud_tf]
    else:
        return [gz_bridge_node]


def generate_launch_description():
    mecanum = LaunchConfiguration('mecanum')
    declare_mecanum_arg = DeclareLaunchArgument(
        'mecanum',
        default_value='True',
        description=(
            'Whether to use mecanum drive controller (otherwise diff drive controller is used)'
        ),
    )

    camera_model = LaunchConfiguration('camera_model')
    declare_camera_model_arg = DeclareLaunchArgument(
        'camera_model',
        default_value='intel_realsense_d435',
        description='Add camera model to the robot URDF',
        choices=[
            'None',
            'intel_realsense_d435',
        ],
    )

    lidar_model = LaunchConfiguration('lidar_model')
    declare_lidar_model_arg = DeclareLaunchArgument(
        'lidar_model',
        default_value='slamtec_rplidar_s1',
        description='Add LiDAR model to the robot URDF',
        choices=[
            'None',
            'slamtec_rplidar_a2',
            'slamtec_rplidar_a3',
            'slamtec_rplidar_s1',
        ],
    )

    include_camera_mount = LaunchConfiguration('include_camera_mount')
    declare_include_camera_mount_arg = DeclareLaunchArgument(
        'include_camera_mount',
        default_value='False',
        description='Whether to include camera mount to the robot URDF',
    )

    world_package = get_package_share_directory('husarion_office_gz')
    world_file = PathJoinSubstitution([world_package, 'worlds', 'husarion_world.sdf'])
    world_cfg = LaunchConfiguration('world')
    declare_world_arg = DeclareLaunchArgument(
        'world', default_value=world_file, description='SDF world file'
    )

    simulation_engine = LaunchConfiguration('simulation_engine')
    declare_simulation_engine_arg = DeclareLaunchArgument(
        'simulation_engine',
        default_value='ignition-gazebo',
        description='Which simulation engine will be used',
    )

    controller_config_name = PythonExpression([
        "'mecanum_drive_controller.yaml' if ",
        mecanum,
        " else 'diff_drive_controller.yaml'",
    ])

    robot_controllers = PathJoinSubstitution([
        FindPackageShare('rosbot_xl_controller'),
        'config',
        controller_config_name,
    ])

    headless = LaunchConfiguration('headless')
    declare_headless_arg = DeclareLaunchArgument(
        'headless',
        default_value='False',
        description='Run Gazebo Ignition in the headless mode',
    )

    params_file = LaunchConfiguration('params_file')
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([
            get_package_share_directory('ros2_simulations'),
            'params',
            'gazebo_params_rosbot_xl.yaml',
        ]),
        description='Full path to the ROS2 parameters file to use for all launched nodes',
    )

    use_rviz = LaunchConfiguration('use_rviz')
    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='True',
        description='Whether to start RVIZ'
    )

    rviz_config_file = LaunchConfiguration('rviz_config_file')
    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=PathJoinSubstitution([
            get_package_share_directory('ros2_simulations'),
            'rviz',
            'rviz_config_rosbot_xl.rviz',
        ]),
        description='Full path to the RVIZ config file to use'
    )

    slam = LaunchConfiguration('slam')
    declare_slam_cmd = DeclareLaunchArgument(
        'slam',
        default_value='True',
        description='Whether run a SLAM',
    )

    map_file = LaunchConfiguration('map_file')
    declare_map_file_cmd = DeclareLaunchArgument(
        'map_file',
        default_value='',
        description='Full path to map yaml file to load',
    )

    headless_cfg = PythonExpression([
        "'--headless-rendering -s -r' if ",
        headless,
        " else '-r'",
    ])
    gz_args = [headless_cfg, ' ', world_cfg]

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py',
            ])
        ),
        launch_arguments={
            'gz_args': gz_args,
            'on_exit_shutdown': 'True',
        }.items(),
    )

    #######################
    # ROSbot XL           #
    #######################

    # Gazebo

    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name',
            'rosbot_xl',
            '-allow_renaming',
            'true',
            '-topic',
            'robot_description',
            '-x',
            '0',
            '-y',
            '2.0',
            '-z',
            '0.2',
        ],
        output='screen',
    )

    # BringUp

    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            PathJoinSubstitution([
                get_package_share_directory('rosbot_xl_bringup'), 'config', 'ekf.yaml'
            ])
        ],
    )

    laser_filter_node = Node(
        package='laser_filters',
        executable='scan_to_scan_filter_chain',
        parameters=[
            PathJoinSubstitution([
                get_package_share_directory('rosbot_xl_bringup'),
                'config',
                'laser_filter.yaml',
            ])
        ],
    )

    # Controller

    # Get URDF via xacro
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        PathJoinSubstitution([
            get_package_share_directory('rosbot_xl_description'),
            'urdf',
            'rosbot_xl.urdf.xacro',
        ]),
        ' mecanum:=',
        mecanum,
        ' lidar_model:=',
        lidar_model,
        ' camera_model:=',
        camera_model,
        ' include_camera_mount:=',
        include_camera_mount,
        ' use_sim:=',
        'True',
        ' simulation_engine:=',
        simulation_engine,
        ' simulation_controllers_config_file:=',
        robot_controllers,
    ])
    robot_description = {'robot_description': robot_description_content}

    robot_state_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[robot_description],
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager',
            '/controller_manager',
            '--controller-manager-timeout',
            '120',
        ],
    )

    robot_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'rosbot_xl_base_controller',
            '--controller-manager',
            '/controller_manager',
            '--controller-manager-timeout',
            '120',
        ],
    )

    # Delay start of robot_controller after joint_state_broadcaster
    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_controller_spawner],
        )
    )

    imu_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'imu_broadcaster',
            '--controller-manager',
            '/controller_manager',
            '--controller-manager-timeout',
            '120',
        ],
    )

    # Delay start of imu_broadcaster after robot_controller
    # when spawning without delay ros2_control_node sometimes crashed
    delay_imu_broadcaster_spawner_after_robot_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=robot_controller_spawner,
            on_exit=[imu_broadcaster_spawner],
        )
    )

    #######################
    # Navigation          #
    #######################

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                get_package_share_directory('nav2_bringup'),
                'launch',
                'navigation_launch.py',
            ])
        ),
        launch_arguments={
            'params_file': params_file,
            'use_sim_time': 'True',
        }.items(),
    )

    #######################
    # Localization        #
    #######################

    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                get_package_share_directory('nav2_bringup'),
                'launch',
                'localization_launch.py',
            ])
        ),
        condition=IfCondition(PythonExpression(['not ', slam])),
        launch_arguments={
            'map': map_file,
            'use_sim_time': 'True',
        }.items(),
    )

    #######################
    # Mapping             #
    #######################

    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                get_package_share_directory('nav2_bringup'),
                'launch',
                'slam_launch.py',
            ])
        ),
        condition=IfCondition(slam),
        launch_arguments={
            'params_file' : params_file,
            'use_sim_time': 'True',
        }.items(),
    )

    #######################
    # RViz                #
    #######################

    rviz_node = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen',
    )

    return LaunchDescription([
        declare_mecanum_arg,
        declare_lidar_model_arg,
        declare_camera_model_arg,
        declare_include_camera_mount_arg,
        declare_simulation_engine_arg,
        declare_world_arg,
        declare_headless_arg,
        declare_params_file_cmd,
        declare_use_rviz_cmd,
        declare_rviz_config_file_cmd,
        declare_slam_cmd,
        declare_map_file_cmd,
        SetParameter(name='use_sim_time', value=True),
        gz_sim,
        OpaqueFunction(function=launch_gz_bridge),
        gz_spawn_entity,
        robot_localization_node,
        laser_filter_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        delay_robot_controller_spawner_after_joint_state_broadcaster_spawner,
        delay_imu_broadcaster_spawner_after_robot_controller_spawner,
        navigation_launch,
        localization_launch,
        slam_launch,
        rviz_node,
    ])
