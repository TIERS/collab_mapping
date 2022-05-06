#!/usr/bin/env python

import sys
import time
import numpy
import random
import argparse
import threading

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry


class WaypointRecorder(Node) :

    def __init__(self, args) :

        parser = argparse.ArgumentParser(
            description='Controls the Turtlebot3 based on a predifined set of points.')

        parser.add_argument('-file', type=str, metavar='FILE_NAME',
                            help='Load waypoints file in csv format.')
        parser.add_argument('-namespace', type=str, metavar='NAME_SPACE',
                            help='Namespace of robot')

        self.args = parser.parse_args(args[1:])

        # Init node
        super().__init__('turtlebot_{}'.format(random.randint(0,100000)))

        # Odom Subscriber
        self.vio_sub = self.create_subscription(Odometry, f"/{self.args.namespace}/odom", self.odom_cb, 10)

        self.last_pos = [0, 0]
        self.poses = []
    

    def odom_cb(self, msg) :
        '''
            Mainly used for orientation from VIO
        '''
        self.last_pos = [ msg.pose.pose.position.x, msg.pose.pose.position.y ]

    def control(self) :
        '''
            Records the position
        '''

        # Skip if it has not started
        if self.last_pos == None:
            pass

        self.poses.append(self.last_pos)
        

    def run(self) :
        
        # Set filter update timer at 6 Hz
        time.sleep(1)
        rospy_check_rate = self.create_rate(10.0)
        self.control_timer = self.create_timer(1.0, self.control)
        
        # Start 
        try:
            while rclpy.ok() :
                rospy_check_rate.sleep()

        except KeyboardInterrupt :
            self.get_logger().error("Keyboard Intterupt!")

            # Save positions to a csv file
            numpy.savetxt(self.args.file, numpy.array(self.poses), '%.4f', delimiter=',')


def main(args=sys.argv):
    rclpy.init(args=args)
    args_without_ros = rclpy.utilities.remove_ros_args(args)
    recorder = WaypointRecorder(args_without_ros)
    
    # Spin in a separate thread
    thread = threading.Thread(target=rclpy.spin, args=(recorder, ), daemon=True)
    thread.start()
    
    recorder.get_logger().info('Turtlebot Controller started')
    exit_code = recorder.run()
    sys.exit(exit_code)


if __name__ == '__main__':
    main()

