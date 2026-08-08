# Testing Guide: F1Tenth AutoDRIVE Adapter

This guide outlines the systematic procedure for verifying the functionality, accuracy, and performance of the `f110_autodrive` adapter node. The adapter serves as the Hardware Abstraction Layer (HAL) translating between the AutoDRIVE simulator and the ASUF1Tenth ROS 2 Jazzy stack.

---

## 1. Prerequisites & Environment Setup

Before starting any tests, verify that your simulation environment and ROS 2 network are properly configured:

### 1a. Docker Container State
Ensure the ROS 2 Jazzy container is running:
```bash
docker ps
# Expected: nuc_forzaeth_racestack_ros2_jazzy is active in the list
```

### 1b. ROS Domain ID Alignment
To ensure cross-boundary communication between the host system and the Docker container, both must run on the same ROS domain:
* **Host System**:
  ```bash
  export ROS_DOMAIN_ID=48
  ```
* **Container**: Automatically configured to `ROS_DOMAIN_ID=48` in the devcontainer settings.

### 1c. Simulator & Host Bridge Launch
1. Launch the **AutoDRIVE Simulator** (Unity executable) on the host machine.
2. In a host terminal (sourced with the `autodrive_devkit` workspace and `ROS_DOMAIN_ID=48`), start the bridge node:
   ```bash
   source <autodrive_devkit folder>/install/setup.bash
   ros2 launch autodrive_roboracer bringup_headless.launch.py
   ```

---

## 2. Test Case 1: Sensor Topic Forwarding & Coordinate Frames

This test case verifies that raw simulator sensor outputs are correctly captured, mapped, and republished on the expected autonomy stack topics with correct TF `frame_id` headers.

### 2a. Run the Adapter Node
Inside a terminal inside the container, run the launch file:
```bash
source install/setup.bash
ros2 launch f110_autodrive autodrive_launch.xml
```

### 2b. Verify Topic Presence
Open another shell inside the container:
```bash
docker exec -it nuc_forzaeth_racestack_ros2_jazzy bash
source /opt/ros/jazzy/setup.bash
ros2 topic list
```
**Expected Output**: The following topics must appear in the list:
* `/scan`
* `/odom`
* `/vesc/odom`
* `/sensors/imu/raw`
* `/vesc/sensors/imu/raw`
* `/sensors/imu` (when `vesc_msgs` is installed)
* `/car_state/odom`
* `/car_state/pose`

### 2c. Verify LiDAR Forwarding
Echo a single scan message:
```bash
ros2 topic echo /scan --once
```
**Expected Validation**:
* `header.frame_id` must be exactly `"laser"`.
* The `ranges` list must contain active measurements (not all zeros or empty).

### 2d. Verify Dual-Topic Kinematic Odometry (`/odom` & `/vesc/odom`)
Echo single odometry messages from both topics:
```bash
ros2 topic echo /odom --once
ros2 topic echo /vesc/odom --once
```
**Expected Validation**:
* Both `/odom` and `/vesc/odom` must publish identical data.
* `header.frame_id` must be exactly `"odom"`.
* `child_frame_id` must be exactly `"base_link"`.
* Position $(x, y)$ and orientation yaw must update via Ackermann kinematics ($\omega_z = \frac{v}{L} \tan(\delta)$).

### 2e. Verify Triple-Topic IMU Forwarding (`/sensors/imu/raw`, `/vesc/sensors/imu/raw`, `/sensors/imu`)
Echo single messages from standard and VESC IMU topics:
```bash
ros2 topic echo /sensors/imu/raw --once
ros2 topic echo /sensors/imu --once
```
**Expected Validation**:
* `/sensors/imu/raw` and `/vesc/sensors/imu/raw` publish `sensor_msgs/msg/Imu` with `header.frame_id = "imu"`.
* `/sensors/imu` publishes `vesc_msgs/msg/VescImuStamped` with populated Roll/Pitch/Yaw (`ypr.x`, `ypr.y`, `ypr.z`) angles in radians.

### 2f. Verify Car State Publishing (`/car_state/odom` & `/car_state/pose`)
Echo single messages from the car state topics:
```bash
ros2 topic echo /car_state/odom --once
ros2 topic echo /car_state/pose --once
```
**Expected Validation**:
* `/car_state/odom` publishes `nav_msgs/msg/Odometry` matching `/odom` (same pose and twist).
* `/car_state/pose` publishes `geometry_msgs/msg/PoseStamped` with the same pose as `/car_state/odom`.

---

## 3. Test Case 2: Actuator Command Mapping & Mathematical Conversion

This test case verifies that the steering and throttle command conversions map correctly from physical values (rad, m/s) to normalized simulator command signals (steering in $[-1, 1]$, throttle in $[0, 1]$).

### 3a. Prepare Echo Receivers
Open two background terminals in the container to capture the outgoing simulator commands:
* **Steering Receiver**:
  ```bash
  ros2 topic echo /autodrive/roboracer_1/steering_command --once
  ```
* **Throttle Receiver**:
  ```bash
  ros2 topic echo /autodrive/roboracer_1/throttle_command --once
  ```

### 3b. Publish a Mock Drive Command
Publish a single test Ackermann drive message with defined velocity and steering:
```bash
ros2 topic pub --once /drive ackermann_msgs/msg/AckermannDriveStamped '{drive: {speed: 1.5, steering_angle: 0.2}}'
```

### 3c. Verify Conversion Math
Analyze the captured commands in the echo terminals:

#### 1. Steering Conversion Validation
* **Equation**: 
  $$u_{steer} = \frac{\delta_{target}}{\delta_{max\_limit}}$$
* **Verification**: With $\delta_{target} = 0.2$ rad and $\delta_{max\_limit} = 0.4189$ rad (default):
  * The `/autodrive/roboracer_1/steering_command` echo output must be exactly: **`0.477440...`**

#### 2. Throttle Conversion Validation (Feedforward + Steering Drag Compensation)
* **Equations**:
  $$u_{ff} = K_{ff\_quad} v_{target}^2 + K_{ff\_lin} v_{target}$$
  $$u_{throttle} = u_{ff} \cdot \bigl(1 + K_{steer} \cdot u_{steer}^2\bigr)$$
  *Clamped strictly to $[0.0, 1.0]$.* With a stationary car ($e_v = 1.5 \not< e_{zone}$), feedback stays off, so this is the exact expected value.
* **Verification**: With $v_{target} = 1.5$ m/s, $\delta_{target} = 0.2$ rad, $K_{ff\_lin} = 0.04$, $K_{ff\_quad} = 0.000139$, and $K_{steer} = 0.15$ (defaults):
  * $u_{ff} = 0.000139 \times 1.5^2 + 0.04 \times 1.5 = 0.00031275 + 0.06 = 0.06031275$
  * $u_{steer} = 0.2 / 0.4189 = 0.4774...$
  * $u_{throttle} = 0.06031275 \times (1 + 0.15 \times 0.4774^2) = 0.06031275 \times 1.03419 = 0.062375...$
  * The `/autodrive/roboracer_1/throttle_command` echo output must be exactly: **`0.062375...`**

---

## 4. Test Case 3: Watchdog Safety Timeout Verification

This test case verifies that if the autonomy stack crashes or stops publishing `/drive` commands, the adapter will detect this state within 200 ms and send zero throttle and steering commands to halt the vehicle.

### 4a. Run the Adapter Node
Verify the adapter node is active in the container:
```bash
ros2 launch f110_autodrive autodrive_launch.xml
```

### 4b. Publish High-Frequency Mock Drive Commands
Publish a `/drive` topic at a continuous high rate (e.g. 20 Hz) with non-zero values:
```bash
ros2 topic pub -r 20 /drive ackermann_msgs/msg/AckermannDriveStamped '{drive: {speed: 1.5, steering_angle: 0.15}}'
```

### 4c. Verify Active command execution
In a separate terminal inside the container, monitor the output command values:
```bash
ros2 topic echo /autodrive/roboracer_1/throttle_command
# Expected: output is non-zero (around 0.0615 with defaults, speed 1.5 / angle 0.15)
```

### 4d. Trigger Watchdog Halt
1. Terminate the `/drive` publisher (`Ctrl + C` in that terminal).
2. Within **200 milliseconds** of terminating the publisher, check the `/autodrive/roboracer_1/throttle_command` and `/autodrive/roboracer_1/steering_command` topics.
   * **Expected**: Both topics must immediately fall back to exactly **`0.0`** (applying brakes and straightening wheels).
   * **Expected Console log**: The adapter node must print:
     `[WARN] [autodrive_adapter]: Watchdog timeout: Lost connection to drive commands. Halting vehicle!`

---

## 5. Troubleshooting & Connection Diagnostics

If topics are active but data is not flowing, use this diagnostic list:

| Symptom | Root Cause | Resolution |
|---|---|---|
| **No topics appear in `ros2 topic list`** | The adapter node is not running. | Verify the launch file started without errors. Check the logs at `/home/mohany/.ros/log/` |
| **Topics exist, but `echo` blocks forever** | ROS Domain ID mismatch between host and container. | Run `export ROS_DOMAIN_ID=48` in the host terminal before starting the bridge. |
| **LiDAR or Odometry frames mismatched** | Incorrect remapping parameters inside the python node. | Verify that [f110_autodrive/autodrive_adapter.py](f110_autodrive/f110_autodrive/autodrive_adapter.py#L1) sets `frame_id` correctly. |
| **Car does not reverse** | The adapter does not support negative speeds. | Expected behavior: `u_throttle` is clamped to $[0, 1]$ and forced to `0.0` for `target_speed <= 0.01`. Reversing is not supported by this adapter. |
