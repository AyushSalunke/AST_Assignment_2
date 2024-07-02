from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    robot_safety_behaviour_tree_node = Node(
        package='my_pkg',
        executable='robot_safety_behaviour_tree',
        output='screen',
        emulate_tty=True
    )

    test_behaviour_node = Node(
        package='my_pkg',
        executable='test_behaviours',
        # cmd=['python3', 'path_to_your_test_file/test_behaviours.py'],
        output='screen',
        emulate_tty=True
    )

    return LaunchDescription([
        robot_safety_behaviour_tree_node,
        test_behaviour_node
    ])