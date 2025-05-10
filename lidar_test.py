import rclpy
import numpy as np
from rclpy.node import Node

from sensor_msgs.msg import LaserScan


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            LaserScan,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        R = np.array(msg.ranges)
        theta = np.linspace( msg.angle_min,msg.angle_max,len(R) )
        self.get_logger().info(R)
        # plan was to use this to see if I was querying the msg structure 
        # correctly, but something was getting stuck and neither returning
        # any data nor throwing an error, so I have no idea how to debug it
        # Hopefully, this is at least a useful starting point for someone 
        # who has any clue how this stuff actually works.



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
