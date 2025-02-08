import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

import xacro


def generate_launch_description():

    # Check if we're told to use sim time
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Process the URDF file

    pkg_path = os.path.join(get_package_share_directory('vektor'))
    xacro_file = os.path.join(pkg_path,'urdf','omniwheel.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)

    rviz_config = os.path.join(pkg_path,'config','view_omniwheel.rviz')
    
    # Create a robot_state_publisher node

    params = {'robot_description': robot_description_config.toxml(), 'use_sim_time': use_sim_time}
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    node_joint_state_publisher = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui"
    )

    node_rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        arguments=['-d', rviz_config]
    )


    default_world = os.path.join(pkg_path,'worlds','empty.world')

    world = LaunchConfiguration('world')

    world_arg = DeclareLaunchArgument(
        'world',
        default_value=default_world,
        description='World to load'
        )

    # Include the Gazebo launch file, provided by the ros_gz_sim package

    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
                    launch_arguments={'gz_args': ['-r -v4 ', world], 'on_exit_shutdown': 'true'}.items()
             )

    # Run the spawner node from the ros_gz_sim package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(package='ros_gz_sim', executable='create',
                        arguments=['-topic', 'robot_description',
                                   '-name', 'vektor', 
                                   '-z', '0.1'],
                        output='screen')


    # Launch!
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),
        node_joint_state_publisher,
        node_rviz2,
        spawn_entity,
        world_arg,
        # ros_gz_bridge,
        gazebo,
        node_robot_state_publisher
    ])
