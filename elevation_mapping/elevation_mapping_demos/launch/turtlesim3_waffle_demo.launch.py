import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Define launch arguments
    x_pos = LaunchConfiguration('x_pos', default='-3.0')
    y_pos = LaunchConfiguration('y_pos', default='1.0')
    z_pos = LaunchConfiguration('z_pos', default='0.0')
    
    # Get package directories
    turtlebot3_gazebo_share_dir = get_package_share_directory('turtlebot3_gazebo')
    gazebo_ros_share_dir = get_package_share_directory('gazebo_ros')
    xacro_share_dir = get_package_share_directory('xacro')
    turtlebot3_description_share_dir = get_package_share_directory('turtlebot3_description')
    elevation_mapping_demos_share_dir = get_package_share_directory('elevation_mapping_demos')

    # Create launch description
    ld = LaunchDescription()

    # Define launch arguments
    ld.add_action(DeclareLaunchArgument('x_pos', default_value='-3.0'))
    ld.add_action(DeclareLaunchArgument('y_pos', default_value='1.0'))
    ld.add_action(DeclareLaunchArgument('z_pos', default_value='0.0'))

    # Include gazebo empty world launch
    gazebo_launch_file = os.path.join(gazebo_ros_share_dir, 'launch', 'spawn_entity_demo.launch.py')
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch_file),
        launch_arguments={
            'world_name': os.path.join(turtlebot3_gazebo_share_dir, 'worlds', 'turtlebot3_house.world'),
            'paused': 'false',
            'use_sim_time': 'true',
            'gui': 'false',
            'headless': 'false',
            'debug': 'false',
            'verbose': 'false'
        }.items()
    ))

    # Load robot description
    ld.add_action(Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='waffle_state_publisher',
        output='screen',
        parameters=[{'robot_description': LaunchConfiguration('robot_description')}]
    ))

    # Spawn turtlebot
    ld.add_action(Node(
        package='gazebo_ros',
        executable='spawn_model',
        name='spawn_urdf',
        output='screen',
        arguments=['-urdf', '-model', 'turtlebot3',
                   '-x', x_pos, '-y', y_pos, '-z', z_pos,
                   '-param', 'robot_description']
    ))

    # Publish tf 'base_footprint' as pose
    ld.add_action(Node(
        package='elevation_mapping_demos',
        executable='tf_to_pose_publisher.py',
        name='waffle_pose_publisher',
        output='screen',
        parameters=[{'from_frame': 'odom'}, {'to_frame': 'base_footprint'}]
    ))

    # Run a passthrough filter to down-sample the sensor point cloud
    ld.add_action(Node(
        package='nodelet',
        executable='nodelet',
        name='pcl_manager',
        output='screen',
        arguments=['manager']
    ))
    ld.add_action(Node(
        package='nodelet',
        executable='nodelet',
        name='voxel_grid',
        output='screen',
        arguments=['load', 'pcl/VoxelGrid', 'pcl_manager'],
        remappings=[('input', '/camera/depth/points'), ('output', '/camera/depth/points_downsampled')],
        parameters=[{
            'filter_field_name': 'z',
            'filter_limit_min': 0.01,
            'filter_limit_max': 6.0,
            'filter_limit_negative': False,
            'leaf_size': 0.05
        }]
    ))

    # Launch elevation mapping node
    ld.add_action(Node(
        package='elevation_mapping',
        executable='elevation_mapping',
        name='elevation_mapping',
        output='screen',
        parameters=[os.path.join(elevation_mapping_demos_share_dir, 'config', 'robots', 'waffle_robot.yaml'),
                    os.path.join(elevation_mapping_demos_share_dir, 'config', 'postprocessing', 'postprocessor_pipeline.yaml')]
    ))

    # Launch RViz with the demo configuration
    rviz_config_file = os.path.join(elevation_mapping_demos_share_dir, 'rviz', 'turtlebot3_waffle_demo.rviz')
    ld.add_action(Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        output='screen',
        arguments=['-d', rviz_config_file]
    ))

    return ld
