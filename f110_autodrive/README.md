# AutoDRIVE Simulator Adapter for the ASUF1Tenth Race Stack

`f110_autodrive` is a ROS 2 hardware abstraction layer (HAL) that bridges the
[AutoDRIVE](https://autodrive-ecosystem.github.io/) RoboRacer simulator to the
ASUF1Tenth F1TENTH race stack. The `autodrive_adapter` node presents the
simulator as a virtual VESC + LiDAR F1TENTH car, translating AutoDRIVE topics
into the exact sensor, odometry, and actuator interfaces the race stack
expects.

## System Architecture

```
AutoDRIVE Simulator                     f110_autodrive (this package)                ASUF1Tenth Race Stack
+----------------------------+          +----------------------------------+          +------------------------+
| /autodrive/roboracer_1/    | -------> | autodrive_adapter                | -------> | /scan                  |
|   lidar   (LaserScan)      |  bridge  |   - sensor/odom remapping        | remap    | /odom, /vesc/odom      |
| /autodrive/roboracer_1/    |          |   - kinematic odometry emulation |          | /sensors/imu/raw       |
|   odom    (Odometry)       |          |   - IMU forwarding               |          | /vesc/sensors/imu/raw  |
| /autodrive/roboracer_1/    |          |   - actuator command mapping     |          | /car_state/odom, pose  |
|   imu     (Imu)            |          |   - closed-loop speed control    |          | /drive                 |
+----------------------------+          +----------------------------------+          +------------------------+
       ^                                        |
       | steering/throttle commands (Float32, normalized)
       +----------------------------------------+
```

## Features

### Sensor Bridging & Frame Remapping
Converts the simulator's sensor streams into the race stack's hardware topics:

* **LiDAR**: `/autodrive/roboracer_1/lidar` → `/scan` with `frame_id = laser`.
* **Odometry**: `/autodrive/roboracer_1/odom` → `/odom` and `/vesc/odom` with
  `frame_id = odom`, `child_frame_id = base_link`, plus `/car_state/odom` and
  `/car_state/pose` for the stack's car state interface.
* **IMU**: `/autodrive/roboracer_1/imu` → `/sensors/imu/raw` and
  `/vesc/sensors/imu/raw` (`sensor_msgs/msg/Imu`, `frame_id = imu`), plus a
  synthesized `vesc_msgs/msg/VescImuStamped` on `/sensors/imu` when `vesc_msgs`
  is available.

### Kinematic Odometry Emulation (`use_kinematic_odom`)
Emulates `vesc_to_odom_node` by integrating a bicycle model using the
simulator's forward speed and the commanded steering angle:

$$\omega_z = \frac{v}{L}\tan(\delta),\quad x_{k+1} = x_k + v\cos(\theta)\,\Delta t,\quad \theta_{k+1} = \theta_k + \omega_z\,\Delta t$$

With `use_kinematic_odom := false`, the simulator's ground-truth odometry is
passed through instead.

### Actuator Command Mapping & Closed-Loop Speed Control
Subscribes to `/drive` (`ackermann_msgs/msg/AckermannDriveStamped`, topic
configurable via `drive_topic`) and produces normalized AutoDRIVE commands:

* **Steering**: $\delta$ is normalized to $u_{\text{steer}} = \delta / \delta_{\text{max}} \in [-1, 1]$.
* **Feedforward throttle**: quadratic plus linear model
  $u_{\text{ff}} = K_{\text{ff\_quad}}\,v^2 + K_{\text{ff\_lin}}\,v$.
* **Steering drag compensation**: scales feedforward throttle in turns to
  combat speed drops:
  $$u_{\text{throttle}} = u_{\text{ff}}\cdot\bigl(1 + K_{\text{steer}}\cdot u_{\text{steer}}^2\bigr)$$
* **Zone-bounded PID feedback**: active only when $|v_{\text{target}} - v_{\text{actual}}| < e_{\text{zone}}$; the integrator is clamped to $\pm I_{\text{max}}$ to prevent windup and reset outside the zone.
* **Filtered derivative damping**: the derivative error is passed through a
  first-order low-pass filter ($\alpha$) to suppress simulator noise and
  actuator chattering.
* **Watchdog safety**: if no `/drive` command arrives within 200 ms, zero
  throttle and straight steering are published to halt the vehicle.

Throttle is clamped to $[0, 1]$.

## Topic Interface

### Subscriptions

| Topic | Type | Purpose |
| :--- | :--- | :--- |
| `/autodrive/roboracer_1/lidar` | `sensor_msgs/msg/LaserScan` | Simulator LiDAR |
| `/autodrive/roboracer_1/odom` | `nav_msgs/msg/Odometry` | Simulator speed/pose |
| `/autodrive/roboracer_1/imu` | `sensor_msgs/msg/Imu` | Simulator IMU |
| `/drive` (default) | `ackermann_msgs/msg/AckermannDriveStamped` | Stack actuation command |

### Publishers

| Topic | Type | Description |
| :--- | :--- | :--- |
| `/scan` | `sensor_msgs/msg/LaserScan` | LiDAR forwarded to the stack |
| `/odom` | `nav_msgs/msg/Odometry` | Odometry (kinematic or ground truth) |
| `/vesc/odom` | `nav_msgs/msg/Odometry` | Same data as `/odom` |
| `/car_state/odom` | `nav_msgs/msg/Odometry` | Car state interface odometry |
| `/car_state/pose` | `geometry_msgs/msg/PoseStamped` | Car state interface pose |
| `/sensors/imu/raw` | `sensor_msgs/msg/Imu` | IMU for EKF (`robot_localization`) |
| `/vesc/sensors/imu/raw` | `sensor_msgs/msg/Imu` | IMU mirror for the controller |
| `/sensors/imu` | `vesc_msgs/msg/VescImuStamped` | VESC IMU (only if `vesc_msgs` is installed) |
| `/autodrive/roboracer_1/steering_command` | `std_msgs/msg/Float32` | Normalized steering to simulator |
| `/autodrive/roboracer_1/throttle_command` | `std_msgs/msg/Float32` | Normalized throttle to simulator |

## Launch Parameters

Run the adapter node with `autodrive_launch.xml`:

```bash
ros2 launch f110_autodrive autodrive_launch.xml
```

| Parameter | Default | Description |
| :--- | :--- | :--- |
| `max_steer_rad` | `0.4189` | Steering limit used to normalize commands (≈24°) |
| `wheelbase` | `0.25` | Vehicle wheelbase (m) for kinematic odometry |
| `use_kinematic_odom` | `True` | Emulate `vesc_to_odom_node` instead of using ground truth |
| `drive_topic` | `/drive` | Stack actuation command topic to subscribe to |
| `Kff_lin` | `0.04` | Linear feedforward throttle gain |
| `Kff_quad` | `0.000139` | Quadratic feedforward throttle gain |
| `K_steer` | `0.15` | Steering drag compensation gain |
| `K_p` | `0.0` | Proportional speed error gain |
| `K_i` | `0.0` | Integral speed error gain |
| `K_d` | `0.0` | Derivative speed error gain |
| `e_zone` | `0.5` | Error band for feedback activation (m/s) |
| `I_max` | `0.2` | Integral clamp |
| `alpha` | `0.25` | First-order LPF coefficient for derivative damping |

Typical tuning ranges for the feedback controller:

| Parameter | Typical range |
| :--- | :--- |
| `K_steer` | 0.0 — 0.3 |
| `K_p` | 0.0 — 1.0 |
| `K_i` | 0.0 — 0.5 |
| `K_d` | 0.0 — 0.2 |
| `e_zone` | 0.1 — 1.0 m/s |
| `I_max` | 0.05 — 0.5 |
| `alpha` | 0.05 — 0.5 |

## How to Run

1. Start the AutoDRIVE simulator and run the AutoDRIVE bridge (`autodrive_roboracer`)
   so the `/autodrive/roboracer_1/*` topics are published.
2. Build and source the workspace:
   ```bash
   colcon build --symlink-install --packages-select f110_autodrive
   source install/setup.bash
   ```
3. Launch the adapter:
   ```bash
   ros2 launch f110_autodrive autodrive_launch.xml
   ```
4. Publish a drive command to test the actuator mapping:
   ```bash
   ros2 topic pub --once /drive ackermann_msgs/msg/AckermannDriveStamped \
     '{drive: {speed: 1.5, steering_angle: 0.2}}'
   ```

The full race stack can then be launched in AutoDRIVE mode via the workspace's
wrapper launch files (e.g. `sim_mode:=autodrive`), which start the adapter
alongside the stack.

## Documentation

* [Hardware Topics & Interface Reference](docs/hardware_topics_reference.md) —
  full inventory of the F1TENTH hardware topics this adapter emulates.
* [Testing Guide](docs/testing_guide.md) — step-by-step verification of sensor
  forwarding, actuator math, and the watchdog timeout.
