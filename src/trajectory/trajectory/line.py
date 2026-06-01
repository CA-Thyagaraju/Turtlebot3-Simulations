import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped


class LineTrajectory(Node):

    def __init__(self):

        super().__init__('line_trajectory')

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

        self.velocity = 0.2
        self.total_time = 20.0

        self.time = 0.0

    def timer_callback(self):

        dt = 0.1

        # Publish velocity command
        cmd = TwistStamped()

        cmd.twist.linear.x = self.velocity
        cmd.twist.angular.z = 0.0

        self.cmd_pub.publish(cmd)

        # PERFECT analytical trajectory
        x = self.velocity * self.time
        y = 0.0

        pose = PoseStamped()

        pose.header.frame_id = 'odom'

        pose.pose.position.x = x
        pose.pose.position.y = y

        self.path.poses.append(pose)

        self.path_pub.publish(self.path)

        self.time += dt

        # Stop after total time
        if self.time >= self.total_time:

            stop = TwistStamped()

            stop.twist.linear.x = 0.0
            stop.twist.angular.z = 0.0

            self.cmd_pub.publish(stop)

            self.get_logger().info(
                'Line trajectory complete'
            )

            self.timer.cancel()


def main(args=None):

    rclpy.init(args=args)

    node = LineTrajectory()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()