import os

from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import SetEnvironmentVariable


def generate_launch_description():
    # 1. set the TURTLEBOT3_MODEL environment variable to 'waffle'.
    turtlebot_model = SetEnvironmentVariable(name='TURTLEBOT3_MODEL',
                                             value=['waffle'])

    # 2. Launch turtlebot3 simulation in a house environment.
    x_pose = LaunchConfiguration('x_pose', default='-3.0')
    y_pose = LaunchConfiguration('y_pose', default='1.0')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    gazebo_world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('elevation_mapping_demos'),
                         'launch', 'turtlebot3_house.launch.py')
        ]),
        launch_arguments={
            'x_pose': x_pose,
            'y_pose': y_pose,
            'use_sim_time': use_sim_time
        }.items())

    # 3. Launch Elevation Mapping node.
    param_path = os.path.join(
        get_package_share_directory('elevation_mapping_demos'), 'config')
    waffle_robot_param_path = os.path.join(param_path, 'robots',
                                           'waffle_robot.yaml')
    postprocessing_param_path = os.path.join(param_path, 'postprocessing',
                                             'postprocessor_pipeline.yaml')
    elevation_mapping_node = Node(
        package='elevation_mapping',
        executable='elevation_mapping',
        name='elevation_mapping_node',
        output='screen',
        parameters=[waffle_robot_param_path, postprocessing_param_path])

    # 4. Run tf-to-pose publisher script.
    from_frame = LaunchConfiguration('from_frame', default='odom')
    to_frame = LaunchConfiguration('to_frame', default='base_footprint')
    tf_to_pose_publisher = Node(
        package='elevation_mapping_demos',
        executable='tf_to_pose_publisher.py',
        name='tf_to_pose_publisher',
        output='screen',
        parameters=[{
            'from_frame': from_frame,
            'to_frame': to_frame
        }])

    # 4. Launch RViz.
    rviz = Node(package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                arguments=[
                    '-d',
                    os.path.join(
                        get_package_share_directory('elevation_mapping_demos'),
                        'rviz', 'turtlebot3_waffle_demo.rviz')
                ])

    ld = LaunchDescription()
    ld.add_action(turtlebot_model)
    ld.add_action(gazebo_world_launch)
    ld.add_action(elevation_mapping_node)
    ld.add_action(tf_to_pose_publisher)
    ld.add_action(rviz)

    return ld
