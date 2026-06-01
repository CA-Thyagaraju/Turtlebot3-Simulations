import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

import math


class SquareTrajectory(Node):

    def __init__(self):

        super().__init__('square_trajectory')

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

        self.timer = self.create_timer(
            0.1,
            self.timer_callback
        )

        self.path = Path()
        self.path.header.frame_id = 'odom'

        # Motion settings
        self.linear_speed = 0.2
        self.angular_speed = 0.5

        self.side_length = 2.0

        self.forward_time = (
            self.side_length /
            self.linear_speed
        )

        self.turn_time = (
            (math.pi / 2) /
            self.angular_speed
        )

        # FSM
        self.state = 'forward'

        self.state_time = 0.0

        self.completed_sides = 0

    def timer_callback(self):

        dt = 0.1

        cmd = TwistStamped()

        # FORWARD
        if self.state == 'forward':

            cmd.twist.linear.x = self.linear_speed
            cmd.twist.angular.z = 0.0

            t = self.state_time

            # PERFECT analytical square
            if self.completed_sides == 0:

                x = self.linear_speed * t
                y = 0.0

            elif self.completed_sides == 1:

                x = self.side_length
                y = self.linear_speed * t

            elif self.completed_sides == 2:

                x = (
                    self.side_length -
                    self.linear_speed * t
                )

                y = self.side_length

            elif self.completed_sides == 3:

                x = 0.0

                y = (
                    self.side_length -
                    self.linear_speed * t
                )

            # Transition to turn
            if self.state_time >= self.forward_time:

                self.state = 'turn'

                self.state_time = 0.0

        # TURN
        elif self.state == 'turn':

            cmd.twist.linear.x = 0.0
            cmd.twist.angular.z = self.angular_speed

            # Hold exact corner positions
            if self.completed_sides == 0:

                x = self.side_length
                y = 0.0

            elif self.completed_sides == 1:

                x = self.side_length
                y = self.side_length

            elif self.completed_sides == 2:

                x = 0.0
                y = self.side_length

            elif self.completed_sides == 3:

                x = 0.0
                y = 0.0

            # Finish turn
            if self.state_time >= self.turn_time:

                self.state = 'forward'

                self.state_time = 0.0

                self.completed_sides += 1

        # Publish velocity command
        self.cmd_pub.publish(cmd)

        # Publish PERFECT planned path
        pose = PoseStamped()

        pose.header.frame_id = 'odom'

        pose.pose.position.x = x
        pose.pose.position.y = y

        self.path.poses.append(pose)

        self.path_pub.publish(self.path)

        self.state_time += dt

        # Finish square
        if self.completed_sides >= 4:

            stop = TwistStamped()

            self.cmd_pub.publish(stop)

            self.get_logger().info(
                'Square trajectory complete'
            )

            self.timer.cancel()


def main(args=None):

    rclpy.init(args=args)

    node = SquareTrajectory()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()