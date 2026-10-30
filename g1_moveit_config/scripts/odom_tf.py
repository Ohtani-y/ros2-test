#!/usr/bin/env python3
"""Relay Gazebo /model/g1/odometry to TF (odom -> pelvis)."""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class OdomTF(Node):
    def __init__(self):
        super().__init__('odom_tf')
        self.br = TransformBroadcaster(self)
        self.create_subscription(Odometry, '/model/g1/odometry', self.cb, 10)

    def cb(self, m):
        t = TransformStamped()
        t.header = m.header
        t.child_frame_id = m.child_frame_id
        p = m.pose.pose.position
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = p.x, p.y, p.z
        t.transform.rotation = m.pose.pose.orientation
        self.br.sendTransform(t)


def main():
    rclpy.init()
    rclpy.spin(OdomTF())


if __name__ == '__main__':
    main()
