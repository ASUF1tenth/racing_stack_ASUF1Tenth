# F1Tenth Hardware Topics & Interface Reference (VESC & LiDAR)

This document provides a comprehensive inventory of all sensor feedback and actuation command topics, along with their exact ROS 2 message types, frame IDs, and parameters used by the physical F1Tenth hardware (VESC motor driver, steering servo, LiDAR, and IMU).

The `f110_autodrive` adapter node uses this specification to fully emulate the hardware interface when bridging the AutoDRIVE simulator.

---

## 1. Sensory Topics (Hardware Outputs / Adapter to Stack)

The table below lists all sensor feedback topics expected by the autonomy stack from the physical VESC and LiDAR hardware drivers.

| Topic Name | ROS 2 Message Type | Frame ID | Publishing Hardware / Node | Consuming Stack Nodes | Description |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **`/scan`** | `sensor_msgs/msg/LaserScan` | `laser` | LiDAR Driver (`urg_node`, `sllidar_ros2`) | `cartographer_node`, `detect` (perception), FTG / Reactive planner | 2D LiDAR laser range measurements array. |
| **`/sensors/core`** | `vesc_msgs/msg/VescStateStamped` | - | `vesc_driver` | `vesc_to_odom_node` | Raw VESC telemetry: motor current, voltage, duty cycle, ERPM, tachometer ticks, fault code. |
| **`/sensors/servo_position_command`** | `std_msgs/msg/Float64` | - | `vesc_driver` | `vesc_to_odom_node` | Echo of current commanded servo position ($0.0 - 1.0$) used for kinematic odometry calculation. |
| **`/sensors/imu/raw`** | `sensor_msgs/msg/Imu` | `imu` | `vesc_driver` / `vesc_fesc_driver` | `robot_localization` (EKF Node) | Standard ROS IMU data (linear acceleration in $m/s^2$, angular velocity in $rad/s$, orientation quaternion). |
| **`/vesc/sensors/imu/raw`** | `sensor_msgs/msg/Imu` | `imu` | `vesc_driver` (namespaced) | `controller_manager` | Mirror of `/sensors/imu/raw` under `/vesc` namespace used directly for lateral acceleration calculation. |
| **`/sensors/imu`** | `vesc_msgs/msg/VescImuStamped` | `imu` | `vesc_driver` | Diagnostics / Logging | VESC custom IMU message containing Roll/Pitch/Yaw (`ypr`), linear acceleration, angular velocity, and compass magnetometer. |
| **`/odom`** (or `/vesc/odom`) | `nav_msgs/msg/Odometry` | `odom` (child: `base_link`) | `vesc_to_odom_node` | `robot_localization` (EKF), `carstate_node` | Wheel odometry computed from motor ERPM speed and steering servo angle kinematics. |

---

## 2. Command Topics (Stack Actuation / Adapter Intercepts)

The table below lists all actuation command topics published by the autonomy stack to control the vehicle hardware.

| Topic Name | ROS 2 Message Type | Expected Range / Units | Publishing Stack Nodes | Consuming Hardware Node | Description |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **`/drive`** (or `/ackermann_cmd`) | `ackermann_msgs/msg/AckermannDriveStamped` | Speed: $m/s$<br>Angle: $rad$ | `controller_manager`, Pure Pursuit, FTG, Teleop | `ackermann_mux` | High-level target velocity and steering angle command. |
| **`/commands/motor/speed`** | `std_msgs/msg/Float64` | Electrical RPM ($\text{ERPM}$) | `ackermann_to_vesc_node`, `throttle_interpolator` | `vesc_driver` | Target motor Electrical RPM ($\text{ERPM} = \text{speed\_to\_erpm\_gain} \cdot v + \text{offset}$). |
| **`/commands/servo/position`** | `std_msgs/msg/Float64` | Normalized PWM [$0.0, 1.0$] | `ackermann_to_vesc_node`, `throttle_interpolator` | `vesc_driver` | Target steering servo position ($\text{Servo} = \text{gain} \cdot \delta + \text{offset}$, center $= 0.5$). |
| **`/commands/motor/unsmoothed_speed`** | `std_msgs/msg/Float64` | Electrical RPM ($\text{ERPM}$) | `ackermann_to_vesc_node` | `throttle_interpolator` | Raw ERPM speed before rate-limiting / acceleration smoothing. |
| **`/commands/servo/unsmoothed_position`** | `std_msgs/msg/Float64` | Normalized PWM [$0.0, 1.0$] | `ackermann_to_vesc_node` | `throttle_interpolator` | Raw servo position before slew-rate / angular velocity smoothing. |
| **`/commands/motor/duty_cycle`** | `std_msgs/msg/Float64` | Duty Cycle [$-1.0, 1.0$] | Teleop / Calibration | `vesc_driver` | Direct motor PWM duty cycle control. |
| **`/commands/motor/current`** | `std_msgs/msg/Float64` | Amperes ($A$) | Torque control / Safety | `vesc_driver` | Direct motor current control. |
| **`/commands/motor/brake`** | `std_msgs/msg/Float64` | Amperes ($A$) | Emergency stop | `vesc_driver` | Motor regenerative braking current. |

---

## 3. Physical VESC Calibration Parameters

The hardware translation logic relies on standard linear calibration parameters specified in `vesc.yaml`:

```yaml
# Speed (m/s) to ERPM Conversion
speed_to_erpm_gain: 4614.0
speed_to_erpm_offset: 0.0

# Steering Angle (rad) to Servo Position (0.0 to 1.0) Conversion
steering_angle_to_servo_gain: 0.31830988618  # 1 / pi
steering_angle_to_servo_offset: 0.5           # Center (straight) position
servo_min: 0.15                              # Left limit clamp
servo_max: 0.85                              # Right limit clamp
```

### Inverse Mathematical Conversion Equations (For Simulator Adapter Emulation)

1. **Reconstructing Steering Angle from Servo Position Command**:
   $$\delta_{\text{rad}} = \frac{\text{servo\_pos} - \text{steering\_angle\_to\_servo\_offset}}{\text{steering\_angle\_to\_servo\_gain}}$$

2. **Reconstructing Forward Velocity from Motor ERPM Command**:
   $$v_{\text{m/s}} = \frac{\text{ERPM} - \text{speed\_to\_erpm\_offset}}{\text{speed\_to\_erpm\_gain}}$$
