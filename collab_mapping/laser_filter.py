from cmath import inf
import argparse
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry


class LaserFilter(Node):

    def __init__(self, args):
        super().__init__('laser_filter')
        parser = argparse.ArgumentParser(
            description='Controls the Turtlebot3 based on a predifined set of points.')

        parser.add_argument('-namespace', type=str, metavar='NAME_SPACE',
                            help='Namespace of robot')

        self.args = parser.parse_args(args[1:])

        self.subscription = self.create_subscription(
            LaserScan,
            f'/{self.args.namespace}/scan',
            self.laser_cb,
            10)
        
        self.publisher = self.create_publisher(
            LaserScan,
            f'/{self.args.namespace}/filtered/scan',
            10
        )

    def laser_cb(self, msg):
        max = msg.range_max

        for i in range(len(msg.ranges)):
            if msg.ranges[i] == inf:
                msg.ranges[i] = max - 0.01
        
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    args_without_ros = rclpy.utilities.remove_ros_args(args)

    minimal_subscriber = LaserFilter(args_without_ros)

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()