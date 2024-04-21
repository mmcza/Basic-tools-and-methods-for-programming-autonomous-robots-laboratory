#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import numpy as np
import cv2
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Image

from ros_object_detection_msgs.msg import BoundingBoxes


class ShowTrajectoryNode(Node):
    def __init__(self):
        super().__init__('show_trajectory')
        self.boxes = None
        self.pcl_subscriber = self.create_subscription(BoundingBoxes, "/bounding_box", self.box_callback,
                                                       qos_profile=qos_profile_sensor_data)
        self.trajectory_publisher = self.create_publisher(Image, "/traffic/trajectory", 10)
        self.width = 100
        self.height = 100
        self.trajectory_image = np.zeros((self.width,self.height,3), dtype=np.uint8)   

    def box_callback(self, data):
        self.boxes = data
        for i, box in enumerate(self.boxes.data):

            x = int((box.xmax + box.xmin)*self.width/2.0)
            y = int((box.ymax + box.ymin)*self.height/2.0)
            
            color = [0, 0, 0]

            if box.data == "car":
                color = np.array([255, 0, 0])
            if box.data == "pedestrians":
                color = np.array([0, 255, 0])
            if box.data == "bicycle":
                color = np.array([0, 0, 255])
            
            self.trajectory_image[x, y, :] = color
            message = CvBridge().cv2_to_imgmsg(self.trajectory_image)
            self.trajectory_publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = ShowTrajectoryNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
