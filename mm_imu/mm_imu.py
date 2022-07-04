#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from .odom_func import OdomFunc
from tf2_ros.transform_broadcaster import TransformBroadcaster


class OdomNode(Node):

    ##################
    # Initialization #
    ##################
    def __init__(self):
        super().__init__('Odom_Simulator')

        # initialize publisher
        self.OdometryPub_ = self.create_publisher(Odometry, '/odom', 10)
        self._tf_Odompublisher = TransformBroadcaster(self)

        # initialize subscriber
        self.ImuSub = self.create_subscription(
            Imu,
            '/imu/data_raw',
            self.sub_imu_callback,
            10)

        self.CmdvelSub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.sub_cmdvel_callback,
            10)

        self.CmdvelSub  # prevent unused variable warning

        # timer configuration
        timer_period = 0.02  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        # self.path_follow_func = Path_follow_func()
        self.cnt = 0

        self.odomFunc = OdomFunc()

    def timer_callback(self):
        # publish odom data
        # cmdvel = self.pathfollowerFunc.update_cmd_vel()
        # odom = Odometry()
        odom, tf_odom = self.odomSimFunc.update_odom()
        # calc cmdvel
        odom.header.stamp = self.get_clock().now().to_msg()
        self.OdometryPub_.publish(odom)
        tf_odom.header.stamp = self.get_clock().now().to_msg()
        self._tf_Odompublisher.sendTransform(tf_odom)
        self.cnt += 1
        if self.cnt % 10 == 0:
            self.get_logger().info('Publishing Count: "%s"' % self.cnt)

    # subscriber call back for Odometry
    def sub_imu_callback(self, msg):
        self.odomFunc.set_imu_from_subscriber(msg)

    # subscriber call back for Odometry
    def sub_cmdvel_callback(self, msg):
        self.odomFunc.set_cmdvel_from_subscriber(msg)


def main(args=None):
    rclpy.init(args=args)

    odomSimNode = OdomNode()
    rclpy.spin(odomSimNode)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    odomSimNode.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
