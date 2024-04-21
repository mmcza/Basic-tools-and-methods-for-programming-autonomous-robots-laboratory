#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import numpy as np
import cv2
from .point_cloud2 import create_cloud
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import struct


class ImageToPclNode(Node):
    def __init__(self):
        super().__init__('image_to_pcl')
        self.image_subscriber = self.create_subscription(Image, '/image', self.convert,
                                                         qos_profile=qos_profile_sensor_data)
        self.pcl_publisher = self.create_publisher(PointCloud2, '/point_cloud', 10)

    def convert(self, data):
        image = CvBridge().imgmsg_to_cv2(data)
        h, w = np.shape(image)[:2]

        image = cv2.resize(image, (100, 100 * w // h))
        h, w = np.shape(image)[:2]

        points = []

        for i, row in enumerate(image):
            for j, color in enumerate(row):
                x = (float(j) / w - 0.5) * 2
                y = -(float(i) / h - 0.5) * 2
                z = 0
                b = int(color[0])
                g = int(color[1])
                r = int(color[2])
                a = 255
                rgb = struct.unpack('I', struct.pack('BBBB', b, g, r, a))[0]
                pt = [x, y, z, rgb]
                points.append(pt)

        fields = [PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                  PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                  PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
                  # PointField('rgb', 12, PointField.UINT32, 1),
                  PointField(name='rgba', offset=12, datatype=PointField.UINT32, count=1),
                  ]

        header = Header()
        header.frame_id = "map"
        pc2 = create_cloud(header, fields, points)
        self.pcl_publisher.publish(pc2)


def main(args=None):
    rclpy.init(args=args)
    node = ImageToPclNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
