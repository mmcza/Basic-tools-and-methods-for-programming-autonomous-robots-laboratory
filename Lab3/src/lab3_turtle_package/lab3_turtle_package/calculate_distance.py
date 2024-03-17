import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import math

from geometry_msgs.msg import Twist

class Turtle(Node):
    def __init__(self):
        super().__init__('turtle')
        self.publisher_ = self.create_publisher(String, '/turtle1/distance', 10)
        self.subscription = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.listener_callback,
            10)
        self.subscription
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.distance = 0
        self.prev_x = 0
        self.prev_y = 0
        self.calc_dist = False

    def listener_callback(self, msg):
        if not self.calc_dist:
           self.prev_x = msg.x
           self.prev_y = msg.y
           self.calc_dist = True
        else:
           self.distance += math.sqrt((self.prev_x - msg.x)**2+(self.prev_y - msg.y)**2)
           self.prev_x = msg.x
           self.prev_y = msg.y
        #self.get_logger().info('Heard: "%.2f"' % self.distance)

    def timer_callback(self):
        msg = String()
        msg.data = '%.3f' % self.distance
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    turtle = Turtle()
    rclpy.spin(turtle)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatica>
    # when the garbage collector destroys the node obj>
    turtle.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
