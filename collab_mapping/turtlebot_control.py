#!/usr/bin/env python

import os
import sys
import time
import math
import numpy
import random
import argparse
import threading

import rclpy
from rclpy.node import Node

from std_msgs.msg       import String
from nav_msgs.msg       import Odometry
from geometry_msgs.msg  import Twist
from ament_index_python.packages import get_package_share_directory


class TurtlebotControl(Node) :

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

        # Predefined trajectory
        file_ = os.path.join(get_package_share_directory('collab_mapping'), 'waypoints', self.args.file)
        self.trajectory = numpy.genfromtxt(file_, float, delimiter=',')
        self.objective_idx = 0
        self.yaw = 0

        self.pos = numpy.zeros(2)

        # Params
        self.dist_threshold = 0.2
        self.angular_threshold = 0.1
        self.angular_speed = 0.3
        self.linear_speed = 0.6
    
        # Odom Subscriber
        self.vio_sub = self.create_subscription(Odometry, f"/{self.args.namespace}/odom", self.odom_cb, 10)

        # Velocity Publisher
        self.vel_pub = self.create_publisher(Twist, f"/{self.args.namespace}/cmd_vel", 10)

        # Done Publisher
        self.done_pub = self.create_publisher(String, f"/mapping_done", 10)


    def orientation_to_euler(self, orientation):
        x = orientation.x
        y = orientation.y
        z = orientation.z
        w = orientation.w
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
        
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
        
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
        
        return yaw_z # in radians

    def odom_cb(self, msg) :
        '''
            Mainly used for orientation from VIO
        '''
        self.pos[0] = msg.pose.pose.position.x
        self.pos[1] = msg.pose.pose.position.y
        self.yaw = self.orientation_to_euler(msg.pose.pose.orientation)

    def pos_cb(self, msg) :
        '''
            Update robot's position
        '''
        self.yaw = self.orientation_to_euler(msg.pose.pose.orientation)


    def control(self) :
        '''
            Controls predefined trajectory based on UWB
        '''

        vel = Twist()

        if numpy.linalg.norm(self.pos - self.trajectory[self.objective_idx]) < self.dist_threshold :
            self.objective_idx = self.objective_idx + 1

        # Robot stops when traversed over all the trajcetory points
        if self.objective_idx == len(self.trajectory):
            self.get_logger().info("Reached the destination!")

            # Send done msg
            msg = String()
            msg.data = self.args.namespace
            self.done_pub.publish(msg)

            # Send stop cmd and cancel the timer
            self.vel_pub.publish(vel)
            self.control_timer.cancel()
            return
        
        theta = math.atan2(
            self.trajectory[self.objective_idx][1] - self.pos[1],
            self.trajectory[self.objective_idx][0] - self.pos[0]
        )

        # self.get_logger().info("We want to go angle {}, now at {}".format(theta, self.yaw))

        diff = abs(self.yaw - theta)
        clockwise = 1
        if diff > math.pi:
            clockwise = -1
            diff = 2 * math.pi - diff

        if diff > self.angular_threshold:
            if self.yaw < theta :
                vel.angular.z = clockwise * self.angular_speed
            else :
                vel.angular.z = clockwise * -self.angular_speed
                    
        else :
            vel.linear.x = self.linear_speed

        # self.get_logger().info("We are at {} moving to {} with vels {},{}".format(self.pos, self.trajectory[self.objective_idx], vel.linear.x, vel.angular.z))
        
        self.vel_pub.publish(vel)

    def run(self) :
        
        # Set filter update timer at 6 Hz
        time.sleep(20)
        rospy_check_rate = self.create_rate(10.0)
        self.control_timer = self.create_timer(1.0/10, self.control)
        
        # Start 
        try:
            while rclpy.ok() :
                rospy_check_rate.sleep()

        except KeyboardInterrupt :
            self.get_logger().error("Keyboard Intterupt!")
            self.get_logger().info("Finished!")


def main(args=sys.argv):
    rclpy.init(args=args)
    args_without_ros = rclpy.utilities.remove_ros_args(args)
    controller = TurtlebotControl(args_without_ros)
    
    # Spin in a separate thread
    thread = threading.Thread(target=rclpy.spin, args=(controller, ), daemon=True)
    thread.start()
    
    controller.get_logger().info('Turtlebot Controller started')
    exit_code = controller.run()
    sys.exit(exit_code)


if __name__ == '__main__':
    main()