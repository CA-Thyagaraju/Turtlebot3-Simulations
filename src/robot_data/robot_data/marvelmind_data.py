import random 
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

class MarvelmindDataNode(Node):
    def __init__(self):
        super().__init__('marvelmind_data_node')
        # Subscribe to ground-truth odometry to generate fake beacon data
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)         
        # Publish noisy PoseStamped data for the EKF
        self.pose_pub = self.create_publisher(PoseStamped, '/marvelmind_data', 10)          
        self.get_logger().info('Marvelmind Data Node Started')

    def odom_callback(self, msg):
        pose_msg = PoseStamped()
        
        # Keep matching timestamp and coordinate frames
        pose_msg.header = msg.header         

        # Add realistic independent indoor GPS noise (~2cm standard deviation)
        pose_msg.pose.position.x = msg.pose.pose.position.x + random.gauss(0, 0.02)
        pose_msg.pose.position.y = msg.pose.pose.position.y + random.gauss(0, 0.02)
        pose_msg.pose.position.z = msg.pose.pose.position.z
        
        # Orientation from beacons usually has some noise, but we pass it cleanly for now
        pose_msg.pose.orientation = msg.pose.pose.orientation
    
        self.pose_pub.publish(pose_msg)

def main(args=None):
    rclpy.init(args=args)
    node = MarvelmindDataNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()