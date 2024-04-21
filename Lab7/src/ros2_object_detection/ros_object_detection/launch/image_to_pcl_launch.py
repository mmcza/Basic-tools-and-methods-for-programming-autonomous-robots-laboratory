from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()
    show_tfs_node = Node(
        package="ros_object_detection",
        executable="image_to_pcl",
        name='image_to_pcl',
        remappings=[
            ("image", "traffic/image_annotated"),
            ("point_cloud", "traffic/point_cloud")
        ]

    )
    ld.add_action(show_tfs_node)
    return ld
