
import random
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

class ImuDataNode(Node):
    def __init__(self):
        super().__init__('imu_data_node')
        self.create_subscription(Imu, '/imu', self.imu_callback, 10)         # Subscribe to original IMU topic
        self.imu_pub = self.create_publisher(Imu, '/imu_data', 10)          # Publish duplicated IMU data for EKF
        self.get_logger().info('IMU Data Node Started')

    def imu_callback(self, msg):
        # Duplicate IMU message
        imu_msg = Imu()
        imu_msg.header = msg.header
        imu_msg.orientation = (msg.orientation)
        imu_msg.orientation_covariance = (msg.orientation_covariance)
        imu_msg.angular_velocity = (msg.angular_velocity)
        imu_msg.angular_velocity_covariance = (msg.angular_velocity_covariance)
        imu_msg.linear_acceleration = (msg.linear_acceleration)
        imu_msg.linear_acceleration_covariance = (msg.linear_acceleration_covariance)

        # Future noise can be added here if needed for testing EKF robustness
        imu_msg.angular_velocity.z += (0.003 + random.gauss(0, 0.002))  # Simulate gyro bias + noise
        self.imu_pub.publish(imu_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ImuDataNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()