from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()
    show_tfs_node = Node(
        package="ros_object_detection",
        executable="show_on_pcl",
        name='show_on_pcl',
        remappings=[
            ("point_cloud", "traffic/point_cloud"),
            ("point_cloud_detection", "traffic/point_cloud_detection"),
            ("bounding_box", "traffic/bounding_box")
        ],
    )
    ld.add_action(show_tfs_node)
    return ld
