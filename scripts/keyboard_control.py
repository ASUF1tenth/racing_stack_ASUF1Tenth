#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDriveStamped


class CmdVelToAckermann(Node):

    def __init__(self):

        super().__init__('cmd_vel_to_ackermann')

        self.declare_parameter('topic_name', '/teleop')
        self.declare_parameter('max_speed', 1.0)
        self.declare_parameter('steering_angle', 0.34)

        topic_name = self.get_parameter('topic_name').value
        self.max_speed = self.get_parameter('max_speed').value
        self.max_steering = self.get_parameter('steering_angle').value

        self.sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.callback,
            10
        )

        self.pub = self.create_publisher(
            AckermannDriveStamped,
            topic_name,
            10
        )

    def callback(self, msg):

        drive = AckermannDriveStamped()

        drive.header.stamp = self.get_clock().now().to_msg()

        # forward/reverse speed with safety max_speed clamping
        speed = msg.linear.x
        if speed > 0:
            speed = min(speed, self.max_speed)
        elif speed < 0:
            speed = max(speed, -self.max_speed)

        drive.drive.speed = speed

        # steering angle in radians
        angular = msg.angular.z
        if angular < 0.0:
            drive.drive.steering_angle = -self.max_steering
        elif angular > 0.0:
            drive.drive.steering_angle = self.max_steering
        else:
            drive.drive.steering_angle = 0.0

        self.pub.publish(drive)


def main():

    rclpy.init()

    node = CmdVelToAckermann()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
