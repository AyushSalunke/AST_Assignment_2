#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    multi_robot_follower = Node(
        package='my_pkg',
        executable='follower_robot',
        output='screen',
        emulate_tty=True
    )

    return LaunchDescription([
        multi_robot_follower
    ])