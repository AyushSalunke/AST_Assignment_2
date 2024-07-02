#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    robot_safety_state_machine_node = Node(
        package='my_pkg',
        executable='robot_safety_state_machine',
        output='screen',
        emulate_tty=True
    )

    return LaunchDescription([
        robot_safety_state_machine_node
    ])