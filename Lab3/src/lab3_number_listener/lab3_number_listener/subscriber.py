import rclpy
from rclpy.node import Node
from std_msgs.msg import String




class MinimalSubscriber(Node):


    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.num_array =[]

    def listener_callback(self, msg):
        self.num_array.append(int(msg.data))
        if len(self.num_array) >= 8:
          sum1=0
          sum2=0
          for i in range(4):
            sum1+= self.num_array[i]
          for j in range(4, 8):
            sum2+= self.num_array[j]
          result = sum1*sum2
          self.get_logger().info('Result for last 8 numbers is: "%s"' % result)
          self.num_array.pop(0)
        else:
          self.get_logger().info('Waiting for more data')



def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()




if __name__ == '__main__':
    main()
