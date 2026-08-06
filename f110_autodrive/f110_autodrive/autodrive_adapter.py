#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Imu
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseStamped, Quaternion
from std_msgs.msg import Float32

try:
    from vesc_msgs.msg import VescImuStamped
    HAS_VESC_MSGS = True
except ImportError:
    HAS_VESC_MSGS = False


def euler_from_quaternion(q):
    """Convert a quaternion (x, y, z, w) into Euler angles (roll, pitch, yaw)."""
    x, y, z, w = q.x, q.y, q.z, q.w
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def quaternion_from_euler(roll, pitch, yaw):
    """Convert Euler angles (roll, pitch, yaw) into a Quaternion (x, y, z, w)."""
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = Quaternion()
    q.w = cr * cp * cy + sr * sp * sy
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * sy
    q.z = cr * cp * sy - sr * sp * cy
    return q


class F110AutoDriveAdapter(Node):
    def __init__(self):
        super().__init__('autodrive_adapter')

        # Declare parameters
        self.declare_parameter('max_steer_rad', 0.4189)  # ~24 degrees
        self.declare_parameter('wheelbase', 0.25)        # ~25 cm F1Tenth wheelbase
        self.declare_parameter('use_kinematic_odom', True)
        self.declare_parameter('drive_topic', '/drive')

        # Controller tuning parameters
        self.declare_parameter('Kff_lin', 0.04)
        self.declare_parameter('Kff_quad', 0.000139)
        self.declare_parameter('K_steer', 0.15)
        self.declare_parameter('K_p', 0.0)
        self.declare_parameter('K_i', 0.0)
        self.declare_parameter('e_zone', 0.5)
        self.declare_parameter('I_max', 0.2)
        self.declare_parameter('K_d', 0.0)
        self.declare_parameter('alpha', 0.25)

        self.max_steer_rad = self.get_parameter('max_steer_rad').value
        self.wheelbase = self.get_parameter('wheelbase').value
        self.use_kinematic_odom = self.get_parameter('use_kinematic_odom').value
        self.drive_topic = self.get_parameter('drive_topic').value

        self.Kff_lin = self.get_parameter('Kff_lin').value
        self.Kff_quad = self.get_parameter('Kff_quad').value
        self.K_steer = self.get_parameter('K_steer').value
        self.K_p = self.get_parameter('K_p').value
        self.K_i = self.get_parameter('K_i').value
        self.e_zone = self.get_parameter('e_zone').value
        self.I_max = self.get_parameter('I_max').value
        self.K_d = self.get_parameter('K_d').value
        self.alpha = self.get_parameter('alpha').value

        self.current_speed = 0.0
        self.current_steering_angle = 0.0
        self.I_accum = 0.0
        self.last_callback_time = None
        self.last_error = 0.0
        self.d_error_filtered = 0.0

        # Kinematic odometry state variables
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_yaw = 0.0
        self.last_odom_time = None

        # Subscriptions from AutoDRIVE
        self.lidar_sub = self.create_subscription(
            LaserScan,
            '/autodrive/roboracer_1/lidar',
            self.lidar_callback,
            10
        )
        self.odom_sub = self.create_subscription(
            Odometry,
            '/autodrive/roboracer_1/odom',
            self.odom_callback,
            10
        )
        self.imu_sub = self.create_subscription(
            Imu,
            '/autodrive/roboracer_1/imu',
            self.imu_callback,
            10
        )

        # Subscription from Autonomy Stack
        self.drive_sub = self.create_subscription(
            AckermannDriveStamped,
            self.drive_topic,
            self.drive_callback,
            10
        )

        # Publishers to Autonomy Stack (Sensors & Odometry)
        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.vesc_odom_pub = self.create_publisher(Odometry, '/vesc/odom', 10)

        # IMU publishers
        self.imu_pub_ekf = self.create_publisher(Imu, '/sensors/imu/raw', 10)
        self.imu_pub_ctrl = self.create_publisher(Imu, '/vesc/sensors/imu/raw', 10)
        if HAS_VESC_MSGS:
            self.vesc_imu_pub = self.create_publisher(VescImuStamped, '/sensors/imu', 10)
        else:
            self.get_logger().warn('vesc_msgs not found. /sensors/imu (VescImuStamped) will be disabled.')

        # Publishers to Autonomy Stack (Car State)
        self.car_state_odom_pub = self.create_publisher(Odometry, '/car_state/odom', 10)
        self.car_state_pose_pub = self.create_publisher(PoseStamped, '/car_state/pose', 10)

        # Publishers to AutoDRIVE
        self.steer_pub = self.create_publisher(Float32, '/autodrive/roboracer_1/steering_command', 10)
        self.throttle_pub = self.create_publisher(Float32, '/autodrive/roboracer_1/throttle_command', 10)

        self.last_drive_time = self.get_clock().now()
        self.in_timeout = True
        self.watchdog_timer = self.create_timer(0.1, self.watchdog_callback)

        self.get_logger().info(f'F1Tenth AutoDRIVE Adapter Node Initialized (use_kinematic_odom={self.use_kinematic_odom})')

    def lidar_callback(self, msg: LaserScan):
        # Forward LiDAR scan with frame_id remapped to 'laser'
        msg.header.frame_id = 'laser'
        self.scan_pub.publish(msg)

    def odom_callback(self, msg: Odometry):
        # Read actual forward speed from simulator odometry
        v = msg.twist.twist.linear.x
        self.current_speed = v

        time_now = self.get_clock().now()

        if self.use_kinematic_odom:
            # Kinematic Wheel Odometry Integration (emulating vesc_to_odom_node)
            if self.last_odom_time is None:
                dt = 0.0
            else:
                dt = (time_now - self.last_odom_time).nanoseconds / 1e9
                if dt < 0.0 or dt > 0.5:
                    dt = 0.0
            self.last_odom_time = time_now

            # Angular velocity from Ackermann kinematics: omega_z = (v / L) * tan(steering_angle)
            omega_z = (v / self.wheelbase) * math.tan(self.current_steering_angle)

            # Integrate pose
            self.odom_yaw += omega_z * dt
            # Normalize yaw to [-pi, pi]
            self.odom_yaw = math.atan2(math.sin(self.odom_yaw), math.cos(self.odom_yaw))

            self.odom_x += v * math.cos(self.odom_yaw) * dt
            self.odom_y += v * math.sin(self.odom_yaw) * dt

            # Construct kinematic odometry message
            odom_msg = Odometry()
            odom_msg.header.stamp = msg.header.stamp
            odom_msg.header.frame_id = 'odom'
            odom_msg.child_frame_id = 'base_link'

            odom_msg.pose.pose.position.x = self.odom_x
            odom_msg.pose.pose.position.y = self.odom_y
            odom_msg.pose.pose.position.z = 0.0
            odom_msg.pose.pose.orientation = quaternion_from_euler(0.0, 0.0, self.odom_yaw)

            odom_msg.twist.twist.linear.x = v
            odom_msg.twist.twist.linear.y = 0.0
            odom_msg.twist.twist.angular.z = omega_z
        else:
            # Pass ground-truth odometry from simulator with remapped frame_ids
            odom_msg = msg
            odom_msg.header.frame_id = 'odom'
            odom_msg.child_frame_id = 'base_link'

        # Publish to both /odom and /vesc/odom
        self.odom_pub.publish(odom_msg)
        self.vesc_odom_pub.publish(odom_msg)

        # Republish as /car_state/odom for controller speed tracking
        self.car_state_odom_pub.publish(odom_msg)

        # Extract pose for /car_state/pose
        pose_msg = PoseStamped()
        pose_msg.header = odom_msg.header
        pose_msg.pose = odom_msg.pose.pose
        self.car_state_pose_pub.publish(pose_msg)

    def imu_callback(self, msg: Imu):
        # Forward standard IMU data with frame_id remapped to 'imu'
        msg.header.frame_id = 'imu'
        self.imu_pub_ekf.publish(msg)
        self.imu_pub_ctrl.publish(msg)

        # Synthesize VescImuStamped for /sensors/imu if vesc_msgs is available
        if HAS_VESC_MSGS:
            vesc_imu_msg = VescImuStamped()
            vesc_imu_msg.header = msg.header

            # Convert orientation quaternion to Roll, Pitch, Yaw
            roll, pitch, yaw = euler_from_quaternion(msg.orientation)
            vesc_imu_msg.imu.ypr.x = roll
            vesc_imu_msg.imu.ypr.y = pitch
            vesc_imu_msg.imu.ypr.z = yaw

            vesc_imu_msg.imu.linear_acceleration = msg.linear_acceleration
            vesc_imu_msg.imu.angular_velocity = msg.angular_velocity
            vesc_imu_msg.imu.orientation = msg.orientation

            self.vesc_imu_pub.publish(vesc_imu_msg)

    def drive_callback(self, msg: AckermannDriveStamped):
        self.last_drive_time = self.get_clock().now()
        if self.in_timeout:
            self.get_logger().info('Drive commands received. Control active.')
            self.in_timeout = False
        target_speed = msg.drive.speed
        target_steering_angle = msg.drive.steering_angle
        self.current_steering_angle = target_steering_angle

        # 1. Normalize steering command: map to [-1, 1]
        u_steer = target_steering_angle / self.max_steer_rad
        u_steer = max(-1.0, min(1.0, u_steer))

        # 2. Compute dt (time difference) for integration
        time_now = self.get_clock().now()
        if self.last_callback_time is None:
            dt = 0.0
        else:
            dt = (time_now - self.last_callback_time).nanoseconds / 1e9
            if dt <= 0.0:
                dt = 0.0
            elif dt > 0.1:
                dt = 0.1
        self.last_callback_time = time_now

        # 3. Compute throttle command using quadratic feedforward control + steering compensation + ID zone feedback
        e_v = target_speed - self.current_speed

        # Calculate derivative error with first-order low-pass filter
        if dt > 0.0:
            d_error_raw = (e_v - self.last_error) / dt
            self.d_error_filtered = self.alpha * d_error_raw + (1.0 - self.alpha) * self.d_error_filtered
        else:
            self.d_error_filtered = 0.0
        self.last_error = e_v

        if target_speed <= 0.01:
            u_throttle = 0.0
            self.I_accum = 0.0  # Reset integrator when vehicle is stopped
            self.d_error_filtered = 0.0
        else:
            # Bounded feedback zone check
            if abs(e_v) < self.e_zone:
                self.I_accum += e_v * dt
                self.I_accum = max(-self.I_max, min(self.I_max, self.I_accum))
                u_feedback = (self.K_p * e_v) + (self.K_i * self.I_accum) + (self.K_d * self.d_error_filtered)
            else:
                self.I_accum = 0.0  # Reset/disable integrator outside the zone
                self.d_error_filtered = 0.0
                u_feedback = 0.0

            u_throttle_base = (self.Kff_quad * (target_speed ** 2)) + (self.Kff_lin * target_speed)
            u_throttle_compensated = u_throttle_base * (1.0 + self.K_steer * (u_steer ** 2))
            u_throttle = u_throttle_compensated + u_feedback
            u_throttle = max(0.0, min(1.0, u_throttle))

        # Publish normalized commands to AutoDRIVE
        steer_msg = Float32()
        steer_msg.data = float(u_steer)
        self.steer_pub.publish(steer_msg)

        throttle_msg = Float32()
        throttle_msg.data = float(u_throttle)
        self.throttle_pub.publish(throttle_msg)

    def watchdog_callback(self):
        time_since_last_drive = (self.get_clock().now() - self.last_drive_time).nanoseconds / 1e9
        if time_since_last_drive > 0.2:
            # Publish safe 0.0 commands (straight wheels, active braking)
            steer_msg = Float32()
            steer_msg.data = 0.0
            self.steer_pub.publish(steer_msg)

            throttle_msg = Float32()
            throttle_msg.data = 0.0
            self.throttle_pub.publish(throttle_msg)

            if not self.in_timeout:
                self.get_logger().warn('Watchdog timeout: Lost connection to drive commands. Halting vehicle!')
                self.in_timeout = True


def main(args=None):
    rclpy.init(args=args)
    node = F110AutoDriveAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
