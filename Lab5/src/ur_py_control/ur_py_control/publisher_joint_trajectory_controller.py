# Copyright 2022 Stogl Robotics Consulting UG (haftungsbeschränkt)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Authors: Denis Štogl, Lovro Ivanov
#

import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.qos import qos_profile_sensor_data

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState


class PublisherJointTrajectory(Node):
    def __init__(self):
        super().__init__("publisher_position_trajectory_controller")
        # Declare all parameters

        controller_name = "joint_trajectory_controller"
        wait_sec_between_publish = 6.0
        self.joints = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint',
                       'wrist_3_joint']
        self.positions = []
        self.positions.append([0.0, -1.57, 0.0, -1.57, 0.0, 0.0])
        self.positions.append([0.785, -1.57, 0.785, 0.785, 0.785, 0.785])
        self.positions.append([0.0, -1.57, 0.0, 0.0, 0.0, 0.0])
        self.positions.append([0.0, -1.57, 0.0, 0.0, -0.785, 0.0])
        self.positions.append([0.0, -1.57, 0.0, 0.0, 0.0, 0.0])
        self.positions.append([0.0, -1.57, 0.0, -1.57, 0.0, 0.0])
        self.robot_in_goal = True

        # Read all positions from parameters
        self.goals = []  # List of JointTrajectoryPoint
        for pos in self.positions:
            point = JointTrajectoryPoint()
            point.positions = pos

            point.time_from_start = Duration(sec=4)
            self.goals.append(point)
            self.get_logger().info(f'Goal  has definition {point}')

        publish_topic = "/" + controller_name + "/" + "joint_trajectory"

        self.publisher_ = self.create_publisher(JointTrajectory, publish_topic, 1)
        self.timer = self.create_timer(wait_sec_between_publish, self.timer_callback)
        self.i = 0
        self.subscriber_ = self.create_subscription(JointState, '/joint_states', self.joints_callback, qos_profile=qos_profile_sensor_data)

    def timer_callback(self):
        traj = JointTrajectory()
        traj.joint_names = self.joints
        traj.points.append(self.goals[self.i])
        self.publisher_.publish(traj)

    def joints_callback(self, msg: JointState):
        self.robot_in_goal = True
        for index, position in enumerate(msg.position):
            if abs(position - self.positions[self.i][(index+1)%len(self.joints)]) > 0.01:
                self.robot_in_goal = False
        
        if self.robot_in_goal:
            self.i += 1
            self.i %= len(self.goals)
            self.timer_callback

        # for position in msg.position:
        #     self.get_logger().info(str(position))
        # for position, name in zip(msg.position, msg.name):
        #     if name not in self.state.keys():
        #         self.state[name] = [position]
        #     else:
        #         self.state[name].append(position)


def main(args=None):
    rclpy.init(args=args)

    publisher_joint_trajectory = PublisherJointTrajectory()

    rclpy.spin(publisher_joint_trajectory)
    publisher_joint_trajectory.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
