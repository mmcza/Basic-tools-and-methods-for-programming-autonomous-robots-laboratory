#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
import threading


class VideoPublisher(Node):
    def __init__(self):
        super().__init__('video_publisher')
        self.publisher_ = self.create_publisher(Image, '/image', 10)
        self.declare_parameter('video_path')
        self.declare_parameter('loop')
        self.declare_parameter('my_double_array')
        self.declare_parameter('fps')
        self.declare_parameter('scale')

    def run(self):
        video_path = self.get_parameter('video_path').value
        loop = self.get_parameter('loop').value
        scale = self.get_parameter('scale').value
        fps = self.get_parameter('fps').value


        while rclpy.ok():
            cap = cv2.VideoCapture(video_path)
            if fps is None:
                fps = cap.get(cv2.CAP_PROP_FPS)
            rate = self.create_rate(fps)

            success, image = cap.read()
            while success and rclpy.ok():
                width = int(image.shape[1] * scale)
                height = int(image.shape[0] * scale)
                image = cv2.resize(image, (width, height))
                message = CvBridge().cv2_to_imgmsg(image)
                self.publisher_.publish(message)
                rate.sleep()
                success, image = cap.read()

            if not loop:
                break


def main(args=None):
    rclpy.init(args=args)

    vp_node = VideoPublisher()

    thread = threading.Thread(target=rclpy.spin, args=(vp_node,), daemon=True)
    thread.start()
    vp_node.run()

    vp_node.destroy_node()
    rclpy.shutdown()
    thread.join()


if __name__ == '__main__':
    main()
