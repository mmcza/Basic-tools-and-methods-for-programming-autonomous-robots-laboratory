from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()
    show_tfs_node = Node(
        package="ros_object_detection",
        executable="publish_video",
        name='publish_video',
        remappings=[
            ("image", "traffic/image")
        ],
        parameters=[
            {"video_path": '/root/Shared/ros_object_detection/stmarc_video.avi'},
            {"loop": True},
            {"scale": 0.5},
            {"fps": 10}
        ]

    )
    ld.add_action(show_tfs_node)
    return ld