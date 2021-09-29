#! /usr/bin/env python3

import math

import numpy as np

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, MagneticField

from ros2_x_omni_msgs.msg import ImuAxis9

from std_srvs.srv import Empty

from tf2_ros import TransformBroadcaster

class XOmniOdomNode(Node):

    def __init__(self):
        super().__init__('x_odom_node')

        self.br = TransformBroadcaster(self)
        self.imu_msg = Imu()
        self.mag_msg = MagneticField()
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

        self.odom_rst_srv = self.create_service(
            Empty,
            'x_omni/odom_reset',
            self.odom_reset_callback)

        self.imu_cal_srv = self.create_service(
            Empty,
            'x_omni/imu_calibration',
            self.imu_calibration_callback)

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
        self.imu_msg.header.stamp = self.get_clock().now().to_msg()
        self.imu_msg.header.frame_id = "imu_frame"
        self.imu_msg.linear_acceleration.x = msg.data[0]
        self.imu_msg.linear_acceleration.y = msg.data[1]
        self.imu_msg.linear_acceleration.z = msg.data[2]
        self.imu_msg.angular_velocity.x = msg.data[3]
        self.imu_msg.angular_velocity.y = msg.data[4]
        self.imu_msg.angular_velocity.z = msg.data[5]
        if self.samples == 0:
            self.imu_msg.angular_velocity.x -= self.offset_gyro_x
            self.imu_msg.angular_velocity.y -= self.offset_gyro_y
            self.imu_msg.angular_velocity.z -= self.offset_gyro_z
        self.imu_pub.publish(self.imu_msg)
        self.mag_msg.header.stamp = self.get_clock().now().to_msg()
        self.mag_msg.header.frame_id = "mag_frame"
        self.mag_msg.magnetic_field.x = msg.data[6]
        self.mag_msg.magnetic_field.y = msg.data[7]
        self.mag_msg.magnetic_field.z = msg.data[8]
        self.mag_pub.publish(self.mag_msg)

    def imu_calibration(self, samples):
        self.get_logger().warn('Calibrating the IMU. Please do not tilt for a while.')
        self.samples = samples
        self.calibration_cnt = 0
        self.offset_gyro_x = 0
        self.offset_gyro_y = 0
        self.offset_gyro_z = 0
        self.gyro_samples = []
        self.acc_samples = []
        self.mag_samples = []
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
            self.gyro_samples.append([msg.data[3],msg.data[4],msg.data[5]])
            self.acc_samples.append([msg.data[0], msg.data[1], msg.data[2]])
            self.mag_samples.append([msg.data[6], msg.data[7], msg.data[8]])
            self.calibration_cnt += 1
        else:
            self.destroy_subscription(self.imu_sub_)
            self.offset_gyro_x /= self.samples
            self.offset_gyro_y /= self.samples
            self.offset_gyro_z /= self.samples

            gyro_cov = np.cov(self.gyro_samples, rowvar=False, bias=True)
            self.imu_msg.angular_velocity_covariance = gyro_cov.reshape((1, 9))[0]

            acc_cov = np.cov(self.acc_samples, rowvar=False, bias=True)
            self.imu_msg.linear_acceleration_covariance = acc_cov.reshape((1, 9))[0]

            mag_cov = np.cov(self.mag_samples, rowvar=False, bias=True)
            self.mag_msg.magnetic_field_covariance = mag_cov.reshape((1, 9))[0]

            self.samples = 0
            self.calibration_cnt = 0
            self.get_logger().info('IMU calibration done.')

    def odom_reset_callback(self, request, response):
        self.pose_th = 0.0
        self.pose_x = 0.0
        self.pose_y = 0.0
        return response

    def imu_calibration_callback(self, request, response):
        self.imu_calibration(500)
        return response






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
