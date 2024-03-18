#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState
import cv2
from cv_bridge import CvBridge, CvBridgeError
import threading
import numpy as np


class SampleJointStates(Node):
    def __init__(self):
        super().__init__('sample_joint_states')
        self.publisher_ = self.create_publisher(JointState, '/joint_states', 10)

    def run(self):
        msg = JointState()
        msg.name.append("single_rrbot_joint1")
        msg.name.append("single_rrbot_joint2")
        msg.name.append("single_rrbot_joint3")
        msg.name.append("single_rrbot_joint4")
        msg.position.append(0.0)
        msg.position.append(0.0)
        msg.position.append(0.0)
        msg.position.append(0.0)
        counter = 0.0
        rate = self.create_rate(50)
        while rclpy.ok():
            counter += 0.01

            # tu zmieniac zeby byly ok zakresy
            if counter <= np.pi/2:
                
                msg.position[0] = np.pi*np.sin(counter)/2
                msg.position[1] = -np.pi*np.sin(counter*2)/2
                msg.position[2] = counter*2
                msg.position[3] = -counter*2/np.pi

            else:
                msg.position[0] = np.pi/2
                msg.position[1] = 0
                msg.position[2] = np.pi
                msg.position[3] = -1


            msg.header.stamp = self.get_clock().now().to_msg()
            self.publisher_.publish(msg)
            rate.sleep()


def main(args=None):
    rclpy.init(args=args)

    node = SampleJointStates()

    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()
    node.run()

    node.destroy_node()
    rclpy.shutdown()
    thread.join()


if __name__ == '__main__':
    main()