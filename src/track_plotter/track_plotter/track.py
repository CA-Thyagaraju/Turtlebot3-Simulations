
# PLOT WITH PREDICTION AND EKF ESTIMATE INCLUDED
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import Imu
from tf_transformations import euler_from_quaternion
import matplotlib.pyplot as plt
from matplotlib.widgets import CheckButtons, Button
import math

class OdomTracker(Node):
    def __init__(self):
        super().__init__('track_plotter')
        # Subscribers
        self.create_subscription(Path, '/planned_path', self.path_callback, 10)
        self.create_subscription(Odometry, '/odom_data', self.odom_callback, 10)
        self.create_subscription(Imu, '/imu_data', self.imu_callback, 10)
        self.create_subscription(PoseStamped, '/marvelmind_data', self.marvelmind_callback, 10)
        self.create_subscription(Path, '/prediction_path', self.prediction_callback, 10)
        self.create_subscription(Path, '/state_estimate', self.estimate_callback, 10)

        # Plot setup
        plt.ion()
        self.fig, self.ax = plt.subplots()
        plt.subplots_adjust(left=0.25, bottom=0.15)

        # Data storage
        self.odom_x = []
        self.odom_y = []

        self.path_x = []
        self.path_y = []

        self.imu_x = []
        self.imu_y = []

        self.meas_x = []
        self.meas_y = []

        self.pred_x = []
        self.pred_y = []

        self.est_x = []
        self.est_y = []

        # IMU integration states
        self.imu_pos_x = 0.0
        self.imu_pos_y = 0.0

        self.imu_vel_x = 0.0
        self.imu_vel_y = 0.0

        self.prev_time = None

        # Plot lines
        self.odom_line, = self.ax.plot(
            [],
            [],
            'b-',
            label='Odometry'
        )

        self.path_line, = self.ax.plot(
            [],
            [],
            'r--',
            label='Planned Path'
        )

        self.imu_line, = self.ax.plot(
            [],
            [],
            'g-',
            label='IMU Estimate'
        )

        self.meas_line, = self.ax.plot(
            [],
            [],
            'y-',
            label='Marvelmind Measurements'
        )

        self.pred_line, = self.ax.plot(
            [],
            [],
            'm-',
            label='Prediction'
        )

        self.est_line, = self.ax.plot(
            [],
            [],
            'k-',
            linewidth=2,
            label='EKF Estimate'
        )

        # Visibility states
        self.visibility = {

            'Planned Path': True,

            'Odometry': True,

            'IMU Estimate': True,

            'Marvelmind Measurements': True,

            'Prediction': True,

            'EKF Estimate': True
        }

        # Checkboxes
        rax = plt.axes([0.02, 0.30, 0.18, 0.35])

        self.check = CheckButtons(
            rax,
            [
                'Planned Path',

                'Odometry',

                'IMU Estimate',

                'Marvelmind Measurements',

                'Prediction',

                'EKF Estimate'
            ],
            [
                True,

                True,

                True,

                True,

                True,

                True
            ]
        )

        self.check.on_clicked(self.toggle_visibility)

        # Fit-to-scale button
        fit_ax = plt.axes(
            [0.02, 0.18, 0.15, 0.06]
        )

        self.fit_button = Button(
            fit_ax,
            'Fit Scale'
        )

        self.fit_button.on_clicked(
            self.fit_scale
        )

        self.ax.legend()

        # Enable pan mode initially
        plt.get_current_fig_manager().toolbar.pan()

        # Plot timer
        self.create_timer(
            0.1,
            self.update_plot
        )

    def toggle_visibility(self, label):

        self.visibility[label] = (
            not self.visibility[label]
        )

        if label == 'Planned Path':

            self.path_line.set_visible(
                self.visibility[label]
            )

        elif label == 'Odometry':

            self.odom_line.set_visible(
                self.visibility[label]
            )

        elif label == 'IMU Estimate':

            self.imu_line.set_visible(
                self.visibility[label]
            )

        elif label == 'Marvelmind Measurements':
            
            self.meas_line.set_visible(
                self.visibility[label]
            )

        elif label == 'Prediction':

            self.pred_line.set_visible(
                self.visibility[label]
            )

        elif label == 'EKF Estimate':

            self.est_line.set_visible(
                self.visibility[label]
            )

        self.fig.canvas.draw_idle()

    def fit_scale(self, event):

        visible_lines = []

        if self.path_line.get_visible():

            visible_lines.append(
                (self.path_x, self.path_y)
            )

        if self.odom_line.get_visible():

            visible_lines.append(
                (self.odom_x, self.odom_y)
            )

        if self.imu_line.get_visible():

            visible_lines.append(
                (self.imu_x, self.imu_y)
            )

        if self.meas_line.get_visible():
            
            visible_lines.append(
                (self.meas_x, self.meas_y)
            )

        if self.pred_line.get_visible():

            visible_lines.append(
                (self.pred_x, self.pred_y)
            )

        if self.est_line.get_visible():

            visible_lines.append(
                (self.est_x, self.est_y)
            )

        if visible_lines:

            all_x = []
            all_y = []

            for x_data, y_data in visible_lines:

                all_x.extend(x_data)
                all_y.extend(y_data)

            if len(all_x) > 1 and len(all_y) > 1:

                x_min = min(all_x)
                x_max = max(all_x)

                y_min = min(all_y)
                y_max = max(all_y)

                if x_min == x_max:

                    x_min -= 1.0
                    x_max += 1.0

                if y_min == y_max:

                    y_min -= 1.0
                    y_max += 1.0

                margin = 0.2

                self.ax.set_xlim(
                    x_min - margin,
                    x_max + margin
                )

                self.ax.set_ylim(
                    y_min - margin,
                    y_max + margin
                )

                self.ax.set_aspect(
                    'equal',
                    adjustable='box'
                )

                self.fig.canvas.draw_idle()

    def odom_callback(self, msg):

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        if len(self.odom_x) > 0:
            dx = abs(x - self.odom_x[-1])
            dy = abs(y - self.odom_y[-1])

            if dx > 0.5 or dy > 0.5:
                return
        self.odom_x.append(x)
        self.odom_y.append(y)

    def path_callback(self, msg):

        self.path_x = []
        self.path_y = []

        for pose in msg.poses:

            self.path_x.append(
                pose.pose.position.x
            )

            self.path_y.append(
                pose.pose.position.y
            )

    
    def marvelmind_callback(self, msg):
        x = msg.pose.position.x
        y = msg.pose.position.y

        if len(self.meas_x) > 0:

            dx = abs(x - self.meas_x[-1])
            dy = abs(y - self.meas_y[-1])

            if dx > 0.5 or dy > 0.5:
                return

        self.meas_x.append(x)
        self.meas_y.append(y)


    def prediction_callback(self, msg):

        self.pred_x = []
        self.pred_y = []

        for pose in msg.poses:

            self.pred_x.append(
                pose.pose.position.x
            )

            self.pred_y.append(
                pose.pose.position.y
            )

    def estimate_callback(self, msg):

        self.est_x = []
        self.est_y = []

        for pose in msg.poses:

            self.est_x.append(
                pose.pose.position.x
            )

            self.est_y.append(
                pose.pose.position.y
            )

    def imu_callback(self, msg):

        current_time = (
            self.get_clock()
            .now()
            .nanoseconds
            / 1e9
        )

        if self.prev_time is None:

            self.prev_time = current_time
            return

        dt = current_time - self.prev_time

        self.prev_time = current_time

        if dt <= 0.0 or dt > 1.0:
            return

        # Body-frame acceleration
        ax_body = (
            msg.linear_acceleration.x
        )

        ay_body = (
            msg.linear_acceleration.y
        )

        # Orientation
        q = msg.orientation

        quaternion = [
            q.x,
            q.y,
            q.z,
            q.w
        ]

        _, _, yaw = (
            euler_from_quaternion(
                quaternion
            )
        )

        # Rotate into world frame
        ax_world = (
            ax_body * math.cos(yaw)
            -
            ay_body * math.sin(yaw)
        )

        ay_world = (
            ax_body * math.sin(yaw)
            +
            ay_body * math.cos(yaw)
        )

        # Integrate acceleration -> velocity
        self.imu_vel_x += (
            ax_world * dt
        )

        self.imu_vel_y += (
            ay_world * dt
        )

        # Mild damping
        self.imu_vel_x *= 0.98
        self.imu_vel_y *= 0.98

        # Integrate velocity -> position
        self.imu_pos_x += (
            self.imu_vel_x * dt
        )

        self.imu_pos_y += (
            self.imu_vel_y * dt
        )

        # Prevent IMU space program
        if (
            abs(self.imu_pos_x) > 100
            or
            abs(self.imu_pos_y) > 100
        ):
            return

        self.imu_x.append(
            self.imu_pos_x
        )

        self.imu_y.append(
            self.imu_pos_y
        )

    def update_plot(self):

        self.odom_line.set_xdata(
            self.odom_x
        )

        self.odom_line.set_ydata(
            self.odom_y
        )

        self.path_line.set_xdata(
            self.path_x
        )

        self.path_line.set_ydata(
            self.path_y
        )

        self.imu_line.set_xdata(
            self.imu_x
        )

        self.imu_line.set_ydata(
            self.imu_y
        )

        self.meas_line.set_xdata(
            self.meas_x
        )

        self.meas_line.set_ydata(
            self.meas_y
        )

        self.pred_line.set_xdata(
            self.pred_x
        )

        self.pred_line.set_ydata(
            self.pred_y
        )

        self.est_line.set_xdata(
            self.est_x
        )

        self.est_line.set_ydata(
            self.est_y
        )

        self.ax.set_xlabel(
            'X Position'
        )

        self.ax.set_ylabel(
            'Y Position'
        )

        self.ax.set_title(
            'Trajectory Comparison'
        )

        self.ax.grid(True)

        self.ax.set_aspect(
            'equal',
            adjustable='box'
        )

        self.fig.canvas.draw_idle()

        self.fig.canvas.flush_events()
def main(args=None):

    rclpy.init(args=args)

    node = OdomTracker()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()
if __name__ == '__main__':
    main()