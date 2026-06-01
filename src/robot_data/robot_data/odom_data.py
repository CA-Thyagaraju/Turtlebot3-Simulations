

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import random

class OdomDataNode(Node):
    def __init__(self):
        self.x_bias = 0.0
        self.y_bias = 0.0
        self.theta_drift = 0.0

        super().__init__('odom_data_node')
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)         # Subscribe to original odometry topic
        self.odom_pub = self.create_publisher(Odometry, '/odom_data', 10)           # Publish duplicated odometry data for EKF
        self.get_logger().info('Odom Data Node Started')

    def odom_callback(self, msg):
        # Duplicate odometry message
        odom_msg = Odometry()
        odom_msg.header = msg.header
        odom_msg.child_frame_id = (msg.child_frame_id)
        odom_msg.pose = msg.pose
        odom_msg.twist = msg.twist

        # Future noise can be added here if needed for testing EKF robustness
        # Slowly accumulating drift
        self.x_bias += random.gauss(0, 0.001)
        self.y_bias += random.gauss(0, 0.0005)
        self.theta_drift += 0.002
        odom_msg.pose.pose.position.x += self.x_bias
        odom_msg.pose.pose.position.y += self.y_bias
        odom_msg.pose.pose.position.y += (0.02 * self.theta_drift)


        self.odom_pub.publish(odom_msg)

def main(args=None):
    rclpy.init(args=args)
    node = OdomDataNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

