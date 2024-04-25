from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    traverse_node = Node(
        package="turtlebot3_autocontrol",
        executable="traverse",
        name='traverse',
        remappings=[
            ("cmd_vel", "cmd_vel"),
            ("scan", "scan"),
            ("odom", "odom")
        ],
    )
    ld.add_action(traverse_node)
    return ld