import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

import math


class CircleTrajectory(Node):

    def __init__(self):

        super().__init__('circle_trajectory')

        # Publishers
        self.cmd_pub = self.create_publisher(
            TwistStamped,
            '/cmd_vel',
            10
        )

        self.path_pub = self.create_publisher(
            Path,
            '/planned_path',
            10
        )

        # Timer
        self.timer = self.create_timer(
            0.1,
            self.timer_callback
        )

        # Path
        self.path = Path()
        self.path.header.frame_id = 'odom'

        # Motion settings
        self.linear_speed = 0.2
        self.radius = 1.0

        # Angular velocity
        self.angular_speed = (
            self.linear_speed /
            self.radius
        )

        # Time tracking
        self.time = 0.0

        # One full revolution
        self.total_time = (
            2 * math.pi /
            self.angular_speed
        )

    def timer_callback(self):

        dt = 0.1

        # Publish velocity command
        cmd = TwistStamped()

        cmd.twist.linear.x = self.linear_speed

        cmd.twist.angular.z = self.angular_speed

        self.cmd_pub.publish(cmd)

        # PERFECT analytical circle
        theta = self.angular_speed * self.time

        x = self.radius * math.sin(theta)

        y = self.radius - (
            self.radius * math.cos(theta)
        )
        
        # Publish planned path
        pose = PoseStamped()

        pose.header.frame_id = 'odom'

        pose.pose.position.x = x
        pose.pose.position.y = y

        self.path.poses.append(pose)

        self.path_pub.publish(self.path)

        self.time += dt

        # Stop after one circle
        if self.time >= self.total_time:

            stop = TwistStamped()

            stop.twist.linear.x = 0.0
            stop.twist.angular.z = 0.0

            self.cmd_pub.publish(stop)

            self.get_logger().info(
                'Circle trajectory complete'
            )

            self.timer.cancel()


def main(args=None):

    rclpy.init(args=args)

    node = CircleTrajectory()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()