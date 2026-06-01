import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

import math


class LemniscateTrajectory(Node):

    def __init__(self):

        super().__init__('lemniscate_trajectory')

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
        self.dt = 0.1

        self.timer = self.create_timer(
            self.dt,
            self.timer_callback
        )

        # Planned path
        self.path = Path()
        self.path.header.frame_id = 'odom'

        # Lemniscate settings
        self.a = 1.0
        self.omega = 0.2

        # Time
        self.time = 0.0

        # Previous heading
        self.prev_theta = 0.0

    def timer_callback(self):

        t = self.omega * self.time

        # Lemniscate of Gerono
        x = self.a * math.sin(t)

        y = self.a * math.sin(t) * math.cos(t)

        # Analytical derivatives
        dx = self.a * math.cos(t)

        dy = self.a * (
            math.cos(t)**2 -
            math.sin(t)**2
        )

        # Scale derivatives by omega
        dx *= self.omega
        dy *= self.omega

        # Desired heading
        theta = math.atan2(dy, dx)

        # Heading error unwrap
        angle_error = theta - self.prev_theta

        angle_error = math.atan2(
            math.sin(angle_error),
            math.cos(angle_error)
        )

        # Robot velocities
        linear_speed = math.sqrt(
            dx**2 + dy**2
        )

        angular_speed = (
            angle_error / self.dt
        )

        # Smooth steering
        angular_speed *= 0.5

        # Publish velocity command
        cmd = TwistStamped()

        cmd.twist.linear.x = linear_speed
        cmd.twist.angular.z = angular_speed

        self.cmd_pub.publish(cmd)

        # Publish PERFECT planned path
        pose = PoseStamped()

        pose.header.frame_id = 'odom'

        pose.pose.position.x = x
        pose.pose.position.y = y

        self.path.poses.append(pose)

        self.path_pub.publish(self.path)

        # Update state
        self.prev_theta = theta

        self.time += self.dt

        # Stop after full traversal
        if t >= 2 * math.pi:

            stop = TwistStamped()

            stop.twist.linear.x = 0.0
            stop.twist.angular.z = 0.0

            self.cmd_pub.publish(stop)

            self.get_logger().info(
                'Lemniscate complete'
            )

            self.timer.cancel()


def main(args=None):

    rclpy.init(args=args)

    node = LemniscateTrajectory()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()