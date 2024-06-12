#!/usr/bin/env python3
# Reffered from https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Writing-A-Tf2-Listener-Py.html .

import math

import geometry_msgs.msg as geometry_msgs

import rclpy
from rclpy.node import Node

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from turtlesim.srv import Spawn


class FrameListener(Node):
 
    def __init__(self):
        super().__init__('frame_listener')

        self.from_frame = self.declare_parameter(
            "from_frame", "odom").get_parameter_value().string_value
        self.to_frame = self.declare_parameter(
            "to_frame", "base_footprint").get_parameter_value().string_value

        # Create a buffer and listener to listen to tf2 transforms
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Create a client to spawn a turtle
        self.spawner = self.create_client(Spawn, 'spawn')

        # Create a publisher.
        self.publisher = self.create_publisher(
            geometry_msgs.PoseWithCovarianceStamped, self.to_frame + '_pose',
            10)

        # Call on_timer function every second
        self.timer = self.create_timer(0.05, self.on_timer)

    def on_timer(self):
        # Look up for the transformation between target_frame and turtle2 frames
        # and send velocity commands for turtle2 to reach target_frame
        try:
            t = self.tf_buffer.lookup_transform(self.from_frame, self.to_frame,
                                                rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {self.to_frame} to {self.from_frame}: {ex}'
            )
            return

        msg = geometry_msgs.PoseWithCovarianceStamped()
        msg.header = t.header
        msg.pose.pose.position.x = t.transform.translation.x
        msg.pose.pose.position.y = t.transform.translation.y
        msg.pose.pose.position.z = t.transform.translation.z
        msg.pose.pose.orientation.x = t.transform.rotation.x
        msg.pose.pose.orientation.y = t.transform.rotation.y
        msg.pose.pose.orientation.z = t.transform.rotation.z
        msg.pose.pose.orientation.w = t.transform.rotation.w
        # Since tf transforms do not have a covariance, pose is filled with
        # a zero covariance.
        msg.pose.covariance = [0.0] * 36

        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    tf_publisher = FrameListener()
    try:
        rclpy.spin(tf_publisher)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == "__main__":
    main()
