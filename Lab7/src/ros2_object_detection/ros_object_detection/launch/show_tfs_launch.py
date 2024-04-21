from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()
    show_tfs_node = Node(
        package="ros_object_detection",
        executable="show_tfs",
        name='show_tfs',
        remappings=[
            ("bounding_box", "traffic/bounding_box")
        ],
        parameters=[
            {"parent_tf": 'map'}
        ]
    )
    ld.add_action(show_tfs_node)
    return ld
