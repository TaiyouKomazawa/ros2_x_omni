#! /usr/bin/env python3

import math

import numpy as np

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, MagneticField

from ros2_x_omni_msgs.msg import ImuAxis9

from tf2_ros import TransformBroadcaster

class XOmniOdomNode(Node):

    def __init__(self):
        super().__init__('x_odom_node')

        self.br = TransformBroadcaster(self)
        self.lasttime = self.get_clock().now()
        self.lastmsg = Twist()
        self.pose_x = 0
        self.pose_y = 0
        self.pose_th = 0

        self.imu_calibration(500)

        self.odom_sub = self.create_subscription(
            Twist,
            'x_omni/odom_vel/raw',
            self.got_odom_callback,
            10)
        self.odom_sub  # prevent unused variable warning
        self.imu_sub = self.create_subscription(
            ImuAxis9,
            'x_omni/imu/raw',
            self.got_imu_callback,
            10)
        self.imu_sub
        self.imu_pub = self.create_publisher(
            Imu,
            'x_omni/imu',
            5)
        self.mag_pub = self.create_publisher(
            MagneticField,
            'x_omni/magnetic_field',
            5)

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

    def got_imu_callback(self, msg):
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = "imu_frame"
        imu_msg.linear_acceleration.x = msg.data[0]
        imu_msg.linear_acceleration.y = msg.data[1]
        imu_msg.linear_acceleration.z = msg.data[2]
        imu_msg.angular_velocity.x = msg.data[3]
        imu_msg.angular_velocity.y = msg.data[4]
        imu_msg.angular_velocity.z = msg.data[5]
        if self.samples == 0:
            imu_msg.angular_velocity.x -= self.offset_gyro_x
            imu_msg.angular_velocity.y -= self.offset_gyro_y
            imu_msg.angular_velocity.z -= self.offset_gyro_z
        self.imu_pub.publish(imu_msg)
        mag_msg = MagneticField()
        mag_msg.header.stamp = self.get_clock().now().to_msg()
        mag_msg.header.frame_id = "mag_frame"
        mag_msg.magnetic_field.x = msg.data[6]
        mag_msg.magnetic_field.y = msg.data[7]
        mag_msg.magnetic_field.z = msg.data[8]
        self.mag_pub.publish(mag_msg)

    def imu_calibration(self, samples):
        self.get_logger().warn('Calibrating the IMU. Please do not tilt for a while.')
        self.samples = samples
        self.calibration_cnt = 0
        self.offset_gyro_x = 0
        self.offset_gyro_y = 0
        self.offset_gyro_z = 0
        self.imu_sub_ = self.create_subscription(
            ImuAxis9,
            'x_omni/imu/raw',
            self.cal_imu_callback,
            10)
        self.imu_sub_

    def cal_imu_callback(self, msg):
        if self.calibration_cnt < self.samples:
            self.offset_gyro_x += msg.data[3]
            self.offset_gyro_y += msg.data[4]
            self.offset_gyro_z += msg.data[5]
            self.calibration_cnt += 1
        else:
            self.destroy_subscription(self.imu_sub_)
            self.offset_gyro_x /= self.samples
            self.offset_gyro_y /= self.samples
            self.offset_gyro_z /= self.samples
            self.samples = 0
            self.calibration_cnt = 0
            self.get_logger().info('IMU calibration done.')







def main():
    rclpy.init()

    node = XOmniOdomNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
