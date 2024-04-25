#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, HistoryPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import threading
from tf_transformations import quaternion_from_euler # requires installing tf_transformations


class Traverse(Node):
    def __init__(self):
        super().__init__('traverse')
        self.laser_scan: LaserScan = None
        self.odom: Odometry = None

        self.new_scan = False
        self.new_odom = False

        self.laser_scan_sub = self.create_subscription(LaserScan, "/scan", self.scan_callback,
                                                       qos_profile=qos_profile_sensor_data)
        
        self.odometry_sub = self.create_subscription(Odometry, "/odom", self.odom_callback,
                                                       qos_profile=qos_profile_sensor_data)
        
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", qos_profile=HistoryPolicy.KEEP_LAST)
        self.state = 0
        self.initial_orientation = 0

    
    def stop(self):
        msg = Twist()
        msg.angular.z = 0.0
        msg.linear.x = 0.0
        
        self.cmd_vel_pub.publish(msg)

    def run_forward(self, vel=0.1):
        msg = Twist()
        msg.angular.z = 0.0
        msg.linear.x = vel
        
        self.cmd_vel_pub.publish(msg)

    def turn_left(self, vel = 0.1, ang_vel = 0.5):
        msg = Twist()
        msg.angular.z = ang_vel
        msg.linear.x = vel
        
        self.cmd_vel_pub.publish(msg)

    def turn_right(self, vel = 0.1, ang_vel = .5):
        msg = Twist()
        msg.angular.z = -ang_vel
        msg.linear.x = vel
        
        self.cmd_vel_pub.publish(msg)

    def scan_callback(self, data):
        self.laser_scan = data
        self.new_scan = True

    def odom_callback(self, data):
        self.odom = data
        self.new_odom = True


    def process(self):
        if self.new_scan and self.new_odom:
            self.new_scan = False
            self.new_odom = False

            angle_min = self.laser_scan.angle_min
            angle_max = self.laser_scan.angle_max
            angle_inc = self.laser_scan.angle_increment

            ranges = self.laser_scan.ranges

            position = self.odom.pose.pose.position
            orientation = self.odom.pose.pose.orientation

            # self.get_logger().info(str(angle_min))
            # self.get_logger().info(str(angle_max))
            # self.get_logger().info(str(angle_inc))
            # self.get_logger().info(str(orientation.z))
            # self.get_logger().info(str(ranges))

            if self.state == 0:
                for i in range(len(ranges)):
                    if ranges[i] == float('inf'):
                        ranges[i] = -1
                
                max_range_index =  ranges.index(max(ranges))
                max_range_angle = angle_min + max_range_index * angle_inc

                quaternion = quaternion_from_euler(0.0, 0.0, max_range_angle)
                self.initial_orientation = orientation.z + quaternion[2]
                
                self.state = 1

            if self.state == 1:
                if abs(orientation.z - self.initial_orientation) > 0.1:
                    self.turn_right(vel=0.0)
                else:
                    self.state = 2 

            if self.state == 2:
                if ranges[30] == float('inf'):
                    dist_30 = 100
                else:
                    dist_30 = ranges[30]
                
                if ranges[-30] == float('inf'):
                    dist_330 = 100
                else:
                    dist_330 = ranges[-30]
                
                if ranges[60] == float('inf'):
                    dist_60 = 100
                else:
                    dist_60 = ranges[60]
                
                if ranges[-60] == float('inf'):
                    dist_300 = 100
                else:
                    dist_300 = ranges[-60]

                direction_30 = dist_30/dist_330
                direction_60 = dist_60/dist_300

                if ranges[90] == float('inf') and ranges[-90] == float('inf'):
                    self.get_logger().info("Got to the end of the maze. Current position:")
                    self.get_logger().info(str(position))
                    self.state = 3

                # self.get_logger().info(str(direction_30))

                if direction_30 > 1.2:
                    self.turn_left(vel=1.0)
                if direction_30 > 0.8 and direction_30 <= 1.2:
                    self.run_forward(vel = 1.0)
                if direction_30 <= 0.8:
                    self.turn_right(vel=1.0)

            if self.state == 3:
                self.stop()



def main(args=None):
    rclpy.init(args=args)

    node = Traverse()
    rate = node.create_rate(100)

    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()

    while rclpy.ok():
        node.process()
        rate.sleep()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()