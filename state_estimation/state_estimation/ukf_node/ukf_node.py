#!/usr/bin/env python3
import threading
import time

import numpy as np
import rclpy
import tf_transformations as tft
from ackermann_msgs.msg import AckermannDriveStamped
from filterpy.kalman import MerweScaledSigmaPoints, UnscentedKalmanFilter
from nav_msgs.msg import Odometry
from pbl_config import load_car_config_ros, load_pacejka_tire_config_ros
from rclpy.node import Node
from sensor_msgs.msg import Imu

from . import models


class UkfNode(Node):
    """
    UKF Node capable of different models

    Data recieved:
        - IMU: [psi, vpsi, ax, ay]
        - VESC: [x, y, psi, vx, vy, vpsi]
        - ACKERMANN: [accel, steer_angle]

    """

    def __init__(self):
        super().__init__('ukf_node')

        self.declare_parameter('frequency', 100)
        self.declare_parameter('racecar_version', rclpy.Parameter.Type.STRING)
        self.declare_parameter('model_type', 'single_track_model')
        self.declare_parameter('odom_topic', '/state_estimation/odom')
        self.declare_parameter('imu_topic', '/imu')
        self.declare_parameter('vesc_topic', '/vesc/odom')
        self.declare_parameter('floor', rclpy.Parameter.Type.STRING)
        self.declare_parameter('R_imu', rclpy.Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter('R_vesc', rclpy.Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter('Q', rclpy.Parameter.Type.DOUBLE_ARRAY)

        Hz = self.get_parameter('frequency').value
        self.racecar_version = self.get_parameter('racecar_version').value
        self.model_type = self.get_parameter('model_type').value
        self.ODOM_TOPIC = self.get_parameter('odom_topic').value
        self.IMU_TOPIC = self.get_parameter('imu_topic').value
        self.VESC_TOPIC = self.get_parameter('vesc_topic').value
        self.floor = self.get_parameter('floor').value  # dubi / winti / CHN / ...

        # Get covariance matrices
        self.R_imu = np.diag(self.get_parameter('R_imu').value)
        self.R_vesc = np.diag(self.get_parameter('R_vesc').value)
        self.Q = np.diag(self.get_parameter('Q').value)

        # Get Car Parameters
        tire_config = load_pacejka_tire_config_ros(self.racecar_version, self.floor)
        car_config = load_car_config_ros(self.racecar_version)
        self.p = {
            # Tire Parameters
            'Bf': tire_config.Bf,
            'Cf': tire_config.Cf,
            'Df': tire_config.Df,
            'Ef': tire_config.Ef,
            'Br': tire_config.Br,
            'Cr': tire_config.Cr,
            'Dr': tire_config.Dr,
            'Er': tire_config.Er,
            'mu': tire_config.friction_coeff,

            # Car Physical Configs
            'm': car_config.m,
            'I_z': car_config.Iz,
            'l_f': car_config.lf,
            'l_r': car_config.lr,
            'h_cg': car_config.h_cg,
            'wheelbase': car_config.wheelbase
        }

        # Initialize Model
        if self.model_type == "single_track_model":
            self.model = models.single_track_model()
        elif self.model_type == "kinematic_bicycle_model":
            self.model = models.kinematic_bicycle_model()
        else:
            self.get_logger().warn("[UKF Node] Unknown Model provided. Using single_track_model instead")
            self.model = models.single_track_model()

        # input and measurement arrays
        self.imu_data = np.zeros(4)  # [theta, dtheta, ax, ay]
        self.vesc_data = np.zeros(6)  # [x, y, theta, vx, vy, dtheta]
        self.ackermann_data = np.zeros(2)  # [accel, steer_angle]

        # Initialize Sigma Points and UKF
        self.sigmas = MerweScaledSigmaPoints(
            self.model.dim_x,
            alpha=1e-3,
            beta=2,
            kappa=0,
            subtract=self.model.residual_x)

        self.ukf = UnscentedKalmanFilter(
            dim_x=self.model.dim_x,
            dim_z=4,    # just as default
            dt=0.01,    # just as default
            fx=self.model.fx,
            hx=self.model.h_imu,  # just as default
            points=self.sigmas,
            residual_x=self.model.residual_x,
            x_mean_fn=self.model.state_mean)

        self.ukf.x = self.model.x
        self.ukf.P = self.model.P
        self.ukf.Q = self.Q

        # General Stuff
        self.lock = threading.Lock()
        self.last_filter_time = None  # time filter is at

        self.is_initialized = False

        self.fresh_imu = False
        self.fresh_vesc = False

        # IMU Exponential moving average filter
        self.alpha = 0.2
        self.prev_ax = None
        self.prev_ay = None

        # Commanded velocity deriviation
        self.last_cmd_vel = 0.0
        self.last_cmd_time = None

        # Subscriptions
        self.create_subscription(Imu, self.IMU_TOPIC, self.imu_callback, 1)
        self.create_subscription(Odometry, self.VESC_TOPIC, self.vesc_odom_callback, 1)
        self.create_subscription(AckermannDriveStamped, '/drive', self.ackermann_callback, 1)
        self.create_subscription(AckermannDriveStamped, '/manual', self.ackermann_callback, 1)

        # Publisher
        self.odom_pub = self.create_publisher(Odometry, self.ODOM_TOPIC, 1)

        # Evaulating computational expences
        self.predict_count = 0
        self.predict_mean_ms = 0.0
        self.predict_max_ms = 0.0

        self.imu_update_count = 0
        self.imu_update_mean_ms = 0.0
        self.imu_update_max_ms = 0.0

        self.vesc_update_count = 0
        self.vesc_update_mean_ms = 0.0
        self.vesc_update_max_ms = 0.0

        self.tot_update_count = 0
        self.tot_update_mean_ms = 0.0
        self.tot_update_max_ms = 0.0

        self.timer = self.create_timer(1.0 / Hz, self.timer_callback)

        self.get_logger().info(f"[UKF Node] UKF node initialized succesfully using {self.model_type}")

    # Callbacks
    def imu_callback(self, msg):
        with self.lock:
            # Exponential moving average filter
            new_ax = msg.linear_acceleration.x
            new_ay = msg.linear_acceleration.y

            if self.prev_ax is not None:
                self.imu_data[2] = self.alpha * new_ax + (1 - self.alpha) * self.prev_ax
                self.imu_data[3] = self.alpha * new_ay + (1 - self.alpha) * self.prev_ay
            else:  # if it is first measurement
                self.imu_data[2] = new_ax
                self.imu_data[3] = new_ay

            self.prev_ax = self.imu_data[2]
            self.prev_ay = self.imu_data[3]

            self.imu_data[1] = msg.angular_velocity.z

            # convert quaternion to euler angles
            q = [msg.orientation.x, msg.orientation.y,
                 msg.orientation.z, msg.orientation.w]
            _, _, yaw = tft.euler_from_quaternion(q)
            self.imu_data[0] = self.normalize_angle(yaw)

            # initialize filter state
            if not self.is_initialized:
                self.ukf.x[4] = self.imu_data[0]
                self.ukf.x[5] = self.imu_data[1]
                self.ukf.P[4, 4] = self.R_imu[0][0]
                self.ukf.P[5, 5] = self.R_imu[1][1]
                self.get_logger().info(f"[UKF Node] Initial yaw set to {self.imu_data[0]}")
                self.get_logger().info(f"[UKF Node] Initial omega set to {self.imu_data[1]}")
                self.is_initialized = True

            self.fresh_imu = True

    def vesc_odom_callback(self, msg):
        with self.lock:
            self.vesc_data[0] = msg.pose.pose.position.x
            self.vesc_data[1] = msg.pose.pose.position.y
            self.vesc_data[3] = msg.twist.twist.linear.x
            self.vesc_data[4] = msg.twist.twist.linear.y
            self.vesc_data[5] = msg.twist.twist.angular.z

            q = [msg.pose.pose.orientation.x,
                 msg.pose.pose.orientation.y,
                 msg.pose.pose.orientation.z,
                 msg.pose.pose.orientation.w]
            __, __, yaw = tft.euler_from_quaternion(q)
            self.vesc_data[2] = yaw

            self.fresh_vesc = True

    def ackermann_callback(self, msg):
        with self.lock:
            now = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            current_vel = msg.drive.speed

            if self.last_cmd_time is None:
                calculated_accel = 0.0
            else:
                dt = now - self.last_cmd_time

                if dt > 1e-4:
                    raw_accel = (current_vel - self.last_cmd_vel) / dt
                    calculated_accel = np.clip(raw_accel, -15.0, 15.0)
                else:
                    calculated_accel = self.ackermann_data[0]

            self.last_cmd_vel = current_vel
            self.last_cmd_time = now

            self.ackermann_data[0] = calculated_accel
            self.ackermann_data[1] = -msg.drive.steering_angle

    def shutdown(self):
        print(f"--- EKF Performance Report ---")
        print(f"**** Predict ****")
        print(f"Predict Steps Run: {self.predict_count}")
        print(f"Mean Execution Time: {self.predict_mean_ms:.3f} ms")
        print(f"Peak Execution Time: {self.predict_max_ms:.3f} ms")
        print(f"**** Update Total****")
        print(f"Update Steps Run: {self.tot_update_count}")
        print(f"Mean Execution Time: {self.tot_update_mean_ms:.3f} ms")
        print(f"Peak Execution Time: {self.tot_update_max_ms:.3f} ms")
        print(f"**** Update IMU****")
        print(f"Update Steps Run: {self.imu_update_count}")
        print(f"Mean Execution Time: {self.imu_update_mean_ms:.3f} ms")
        print(f"Peak Execution Time: {self.imu_update_max_ms:.3f} ms")
        print(f"**** Update VESC****")
        print(f"Update Steps Run: {self.vesc_update_count}")
        print(f"Mean Execution Time: {self.vesc_update_mean_ms:.3f} ms")
        print(f"Peak Execution Time: {self.vesc_update_max_ms:.3f} ms")

        print(f"--- System function computation time ---")
        print(f"Steps: {self.model.count}")
        print(f"Mean Execution Time: {self.model.mean_time:.3f} ms")
        print(f"Peak Execution Time: {self.model.max_time:.3f} ms")
        self.get_logger().info("Shutting down...")

    # Helper Functions
    def ensure_psd(self, P):
        """
        Ensures positive definite P by clipping eigenvalues to a minimum floor.
        """
        # ensure symetry
        P = (P + P.T) / 2.0

        min_eig = 1e-9
        eigvals, eigvecs = np.linalg.eigh(P)

        # Check if any eigenvalue is below threshold
        if np.min(eigvals) < min_eig:
            eigvals = np.maximum(eigvals, min_eig)

        return eigvecs @ np.diag(eigvals) @ eigvecs.T

    def predict(self, dt, u, p):
        """
        If dt is too big it predicts in small substeps to avoid numerical issues
        """
        self.ukf.P = self.ensure_psd(self.ukf.P)
        if dt >= 0.1:
            max_dt = 0.02
            while dt > 0:
                step = min(dt, max_dt)
                self.ukf.predict(dt=step, u=u, p=p)
                self.ukf.P = self.ensure_psd(self.ukf.P)
                dt -= step

        else:
            self.ukf.predict(dt=dt, u=u, p=p)
            self.ukf.P = self.ensure_psd(self.ukf.P)

    def normalize_angle(self, x):
        return np.arctan2(np.sin(x), np.cos(x))

    def publish_odometry(self):
        state = self.model.get_odom_state(self.ukf.x)

        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        odom.pose.pose.position.x = state[0]
        odom.pose.pose.position.y = state[1]

        q = tft.quaternion_from_euler(0, 0, state[2])
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]

        odom.twist.twist.linear.x = state[3]
        odom.twist.twist.linear.y = state[4]
        odom.twist.twist.angular.z = state[5]

        # Covariances
        pose_cov = np.zeros((6, 6))
        pose_cov[0:2, 0:2] = self.ukf.P[0:2, 0:2]
        pose_cov[0:2, 5] = self.ukf.P[0:2, 2]
        pose_cov[5, 0:2] = self.ukf.P[2, 0:2]
        pose_cov[5, 5] = self.ukf.P[2, 2]
        odom.pose.covariance = pose_cov.flatten().tolist()

        twist_cov = np.zeros((6, 6))
        twist_cov[0:2, 0:2] = self.ukf.P[3:5, 3:5]
        twist_cov[0:2, 5] = self.ukf.P[3:5, 5]
        twist_cov[5, 0:2] = self.ukf.P[5, 3:5]
        twist_cov[5, 5] = self.ukf.P[5, 5]
        odom.twist.covariance = twist_cov.flatten().tolist()

        self.odom_pub.publish(odom)

    def timer_callback(self):
        """
        Main Loop of UKF Node
        """
        with self.lock:
            # Get snapshot of newest measurement data, cmd inputs, and flags to use in prediction and update steps
            initialized = self.is_initialized

            u = np.copy(self.ackermann_data)

            fresh_imu = self.fresh_imu
            fresh_vesc = self.fresh_vesc

            z_imu = np.copy(self.imu_data)
            z_vesc = np.copy(self.vesc_data)

            # reset flags
            self.fresh_imu = False
            self.fresh_vesc = False

        if not initialized:
            self.get_logger().info(f"[UKF Node] Initial state not set yet")
            return

        now = self.get_clock().now().nanoseconds / 1e9
        if self.last_filter_time is None:  # first loop
            self.last_filter_time = now
            return

        dt = now - self.last_filter_time
        if dt == 0:
            return
        self.last_filter_time = now

        t0 = time.perf_counter_ns()
        # Prediction Step
        self.predict(dt=dt, u=u, p=self.p)

        t1 = time.perf_counter_ns()
        duration_ms = (t1 - t0) / 1_000_000.0
        if duration_ms > self.predict_max_ms:
            self.predict_max_ms = duration_ms

        self.predict_count += 1
        self.predict_mean_ms += (duration_ms - self.predict_mean_ms) / self.predict_count

        # Measurement Updates
        t2 = time.perf_counter_ns()
        if fresh_imu:
            t_imu_start = time.perf_counter_ns()

            self.ukf.residual_z = self.model.residual_imu
            self.ukf.z_mean_fn = self.model.imu_mean
            self.ukf.update(
                z=z_imu,
                R=self.R_imu,
                hx=self.model.h_imu,
                u=u,
                p=self.p)

            t_imu_end = time.perf_counter_ns()
            imu_ms = (t_imu_end - t_imu_start) / 1_000_000.0
            if imu_ms > self.imu_update_max_ms:
                self.imu_update_max_ms = imu_ms
            self.imu_update_count += 1
            self.imu_update_mean_ms += (imu_ms - self.imu_update_mean_ms) / self.imu_update_count

        if fresh_vesc:
            t_vesc_start = time.perf_counter_ns()

            self.ukf.residual_z = self.model.residual_vesc
            self.ukf.z_mean_fn = self.model.vesc_mean
            self.ukf.update(
                z=z_vesc,
                R=self.R_vesc,
                hx=self.model.h_vesc,
                u=u,
                p=self.p)

            t_vesc_end = time.perf_counter_ns()
            vesc_ms = (t_vesc_end - t_vesc_start) / 1_000_000.0
            if vesc_ms > self.vesc_update_max_ms:
                self.vesc_update_max_ms = vesc_ms
            self.vesc_update_count += 1
            self.vesc_update_mean_ms += (vesc_ms - self.vesc_update_mean_ms) / self.vesc_update_count

        t3 = time.perf_counter_ns()
        duration_ms = (t3 - t2) / 1_000_000.0
        if duration_ms > self.tot_update_max_ms:
            self.tot_update_max_ms = duration_ms

        self.tot_update_count += 1
        self.tot_update_mean_ms += (duration_ms - self.tot_update_mean_ms) / self.tot_update_count

        self.publish_odometry()


def main(args=None):
    rclpy.init(args=args)
    node = UkfNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
