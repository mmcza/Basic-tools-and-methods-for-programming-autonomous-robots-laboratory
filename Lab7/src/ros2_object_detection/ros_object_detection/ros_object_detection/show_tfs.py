#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
import tf2_ros
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

from ros_object_detection_msgs.msg import BoundingBoxes


class ShowTfsNode(Node):
    def __init__(self):
        super().__init__('show_tfs')
        self.declare_parameter('parent_tf')
        self.boxes = None
        self.pcl_subscriber = self.create_subscription(BoundingBoxes, "/bounding_box", self.box_callback,
                                                       qos_profile=qos_profile_sensor_data)
        self.br = TransformBroadcaster(self)
        self.parent_tf = self.get_parameter('parent_tf').value

    def box_callback(self, data):
        self.boxes = data
        for i, box in enumerate(self.boxes.data):
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = self.parent_tf
            t.child_frame_id = '{}_{}'.format(box.data, i)

            ### Write your code here ###
            ### box.xmin, box.xmax, box.ymin, box.ymax
            t.transform.translation.x = ((box.xmax-0.5)*2.0 + (box.xmin-0.5)*2.0)/2.0
            t.transform.translation.y = ((box.ymax-0.5)*2.0 + (box.ymin - 0.5)*2.0)/2.0
            t.transform.translation.z = 0.0
            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = 0.0
            t.transform.rotation.w = 1.0

            self.br.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = ShowTfsNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
