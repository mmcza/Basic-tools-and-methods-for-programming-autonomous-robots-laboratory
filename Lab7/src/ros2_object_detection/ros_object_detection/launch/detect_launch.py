from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()
    show_tfs_node = Node(
        package="ros_object_detection",
        executable="detect",
        name='detect',
        remappings=[
            ("image", "traffic/image"),
            ("image_annotated", "traffic/image_annotated"),
            ("bounding_box", "traffic/bounding_box")
        ],
        parameters=[
            {"label_map": '/root/Shared/ros_object_detection/label_map.pbtxt'},
            {"model": '/root/Shared/ros_object_detection/traffic_inference_graph'},
            {"score_thresh": 0.5}
        ]

    )
    ld.add_action(show_tfs_node)
    return ld
