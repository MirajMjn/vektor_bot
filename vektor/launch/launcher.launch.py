from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    params = os.path.join(get_package_share_directory('vektor'),'config','parameters.yaml')

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[params],
    )
    

    teleop_node = Node(
        package='vektor', 
        executable='teleop_node',
        name='teleop_node',
        output='screen',
        # parameters=[{'input_mode': LaunchConfiguration('input_mode')}],
    )

    kinematics_node = Node(
        package='vektor', 
        executable='kinematics_node',
        name='kinematics_node',
        output='screen',
    )

    motor1_PID_node = Node(
        package='vektor', 
        executable='PID_control_node',
        name='motor1_PID_node',
        output='screen',
        parameters=[params],
        remappings=[
            ('/PID_output', '/motor_1/PID_output')
        ]
    )

    motor1_interface_node = Node(
        package='vektor', 
        executable='motor_interface_node',
        name='motor1_interface_node',
        output='screen',
        parameters=[params],
    )

    motor2_PID_node = Node(
        package='vektor', 
        executable='PID_control_node',
        name='motor2_PID_node',
        output='screen',
        parameters=[params],
        remappings=[
            ('/PID_output', '/motor_2/PID_output')
        ]
    )

    motor2_interface_node = Node(
        package='vektor', 
        executable='motor_interface_node',
        name='motor2_interface_node',
        output='screen',
        parameters=[params],
    )

    motor3_PID_node = Node(
        package='vektor', 
        executable='PID_control_node',
        name='motor3_PID_node',
        output='screen',
        parameters=[params],
        remappings=[
            ('/PID_output', '/motor_3/PID_output')
        ]
    )

    motor3_interface_node = Node(
        package='vektor', 
        executable='motor_interface_node',
        name='motor3_interface_node',
        output='screen',
        parameters=[params],
    )

    return LaunchDescription([
        joy_node,       
        teleop_node,
        kinematics_node,
        motor1_PID_node,
        motor2_PID_node,
        motor3_PID_node,
        motor1_interface_node,
        motor2_interface_node,
        motor3_interface_node,
    ])