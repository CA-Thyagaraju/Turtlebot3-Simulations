import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped
from tf_transformations import euler_from_quaternion
import numpy as np
from collections import deque
from extended_kalman_filter.models import (
    motion_model,
    jacobian_F,
    process_noise_Q,
    measurement_noise_R,
    measurement_matrix_H,
    normalize_angle,
    initial_covariance_P)

class EstimationNode(Node):
    def __init__(self):
        super().__init__('estimation_node')

        # Subscribers
        self.create_subscription(Odometry, '/odom_data', self.odom_callback, 10)
        self.create_subscription(Imu, '/imu_data', self.imu_callback, 10)
        self.create_subscription(PoseStamped, '/marvelmind_data', self.marvelmind_callback, 10)

        # Publishers
        self.prediction_pub = self.create_publisher(Path, '/prediction_path', 10)
        self.path_pub = self.create_publisher(Path, '/state_estimate', 10)

        # Prediction Path
        self.prediction_path = Path()
        self.prediction_path.header.frame_id = 'odom'
        self.pred_poses_deque = deque(maxlen=5000)

        # Estimated Path
        self.path = Path()
        self.path.header.frame_id = 'odom'
        self.est_poses_deque = deque(maxlen=5000)

        # State vector
        self.state = np.array([0.0, 0.0, 0.0])          # [x, y, theta]

        # Imports from models.py for EKF implementation
        self.P = initial_covariance_P()               # Covariance matrix
        self.Q = process_noise_Q()                    # Process noise
        self.H = measurement_matrix_H()               # Measurement model
        self.R = measurement_noise_R()                # Measurement noise

        # Inputs
        self.linear_velocity = 0.0
        self.angular_velocity = None  

        # Measurements
        self.mx = 0.0
        self.my = 0.0
        self.mtheta = 0.0

        # Startup sync
        self.odom_received = False
        self.imu_received = False
        self.marvelmind_received = False

        # Timing
        self.dt = 0.02

        # EKF Timer
        self.create_timer(self.dt, self.ekf_step)
        self.get_logger().info('EKF Estimation Node Started')

    def odom_callback(self, msg):
        self.odom_received = True
        self.linear_velocity = msg.twist.twist.linear.x         
        
    def imu_callback(self, msg):
        self.imu_received = True
        self.angular_velocity = msg.angular_velocity.z          

    def marvelmind_callback(self, msg):
        x = msg.pose.position.x
        y = msg.pose.position.y
        if abs(x) < 0.08 and abs(y) < 0.08:
            return
        self.marvelmind_received = True
        self.mx = msg.pose.position.x                           
        self.my = msg.pose.position.y
        q = msg.pose.orientation
        quaternion = [q.x, q.y, q.z, q.w]
        _, _, self.mtheta = euler_from_quaternion(quaternion)

    def ekf_step(self):
        # 1. Prevent startup garbage before running any logic
        if not self.odom_received or not self.imu_received:
            return

        # Capture a single cohesive timestamp for this loop iteration
        current_time = self.get_clock().now().to_msg()

        # =================================
        # Prediction Step
        # =================================
        control = np.array([self.linear_velocity, self.angular_velocity])

        x_pred_raw = motion_model(self.state, control, self.dt)         
        F = jacobian_F(self.state, control, self.dt)                    
        self.P = F @ self.P @ F.T + self.Q                            

        # Build prediction pose
        pred_pose = PoseStamped()
        pred_pose.header.frame_id = 'odom'
        pred_pose.header.stamp = current_time
        pred_pose.pose.position.x = float(x_pred_raw[0])  
        pred_pose.pose.position.y = float(x_pred_raw[1])
        
        self.pred_poses_deque.append(pred_pose)
        self.prediction_path.header.stamp = current_time
        self.prediction_path.poses = list(self.pred_poses_deque)
        self.prediction_pub.publish(self.prediction_path)

        # Enforce column vector representation for update step matrix math
        x_pred = x_pred_raw.reshape(3, 1)

        # =================================
        # Measurement Update
        # =================================
        if self.marvelmind_received:    
            z = np.array([
                [self.mx],
                [self.my],
                [self.mtheta]])

            innovation = z - self.H @ x_pred                           
            innovation[2, 0] = normalize_angle(innovation[2, 0])         
            
            S = self.H @ self.P @ self.H.T + self.R                    
            K = self.P @ self.H.T @ np.linalg.inv(S)                   
            
            x_est = x_pred + K @ innovation                            
            x_est[2, 0] = normalize_angle(x_est[2, 0])                   

            # Covariance update
            I = np.eye(3)
            self.P = (I - K @ self.H) @ self.P  

            self.marvelmind_received = False
        else:
            x_est = x_pred  

        # Sync corrected state back to 1D array
        self.state = x_est.flatten()  

        # =================================
        # Publish Estimated Path
        # =================================
        pose = PoseStamped()
        pose.header.frame_id = 'odom'
        pose.header.stamp = current_time
        pose.pose.position.x = float(self.state[0])
        pose.pose.position.y = float(self.state[1])
        
        self.est_poses_deque.append(pose)
        self.path.header.stamp = current_time
        self.path.poses = list(self.est_poses_deque)
        self.path_pub.publish(self.path)

        # Debug
        logger = self.get_logger()
        if logger.get_effective_level() <= rclpy.logging.LoggingSeverity.DEBUG:
            logger.debug(
                f'X: {self.state[0]:.2f}, '
                f'Y: {self.state[1]:.2f}, '
                f'Theta: {self.state[2]:.2f}')

def main(args=None):
    """Initialize and spin the EKF estimation ROS2 node."""
    rclpy.init(args=args)
    node = EstimationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()