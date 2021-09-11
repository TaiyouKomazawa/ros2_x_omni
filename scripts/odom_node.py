#! /usr/bin/env python3

import math

import numpy as np

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry

from tf2_ros import TransformBroadcaster

class XOmniOdomNode(Node):

    def __init__(self):
        super().__init__('x_omni_odom_node')

        self.br = TransformBroadcaster(self)
        self.lasttime = self.get_clock().now()
        self.lastmsg = Twist()
        self.pose_x = 0
        self.pose_y = 0
        self.pose_th = 0

        self.subscription = self.create_subscription(
            Twist,
            'x_omni/odom_vel',
            self.got_odom_callback,
            10)
        self.subscription  # prevent unused variable warning


    def got_odom_callback(self, msg):

        now = self.get_clock().now()
        diff = now - self.lasttime
        self.lasttime = now

        self.pose_th += (msg.angular.z+self.lastmsg.angular.z)/2.0*diff.to_msg().nanosec/1.0E+9

        xvec = msg.linear.x
        yvec = msg.linear.y

        msg.linear.y = yvec * np.cos(self.pose_th) - xvec * np.sin(self.pose_th)
        msg.linear.x = yvec * np.cos(self.pose_th) + xvec * np.cos(self.pose_th)

        self.pose_y += (msg.linear.y+self.lastmsg.linear.y)/2.0*diff.to_msg().nanosec/1.0E+9
        self.pose_x += (msg.linear.x+self.lastmsg.linear.x)/2.0*diff.to_msg().nanosec/1.0E+9

        self.lastmsg = msg


        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = self.pose_x
        t.transform.translation.y = self.pose_y
        t.transform.translation.z = 0.0

        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = np.sin(self.pose_th/2)
        t.transform.rotation.w = np.cos(self.pose_th/2)

        self.br.sendTransform(t)




def main():
    rclpy.init()

    node = XOmniOdomNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()

if __name__ == '__main__':
    main()
