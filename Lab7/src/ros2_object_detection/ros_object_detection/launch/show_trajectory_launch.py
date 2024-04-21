from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()
    show_tfs_node = Node(
        package="ros_object_detection",
        executable="show_trajectory",
        name='show_trajectory',
        remappings=[
            ("bounding_box", "traffic/bounding_box"),
            ("trajectory", "traffic/trajectory")
        ]
    )
    ld.add_action(show_tfs_node)
    return ld
