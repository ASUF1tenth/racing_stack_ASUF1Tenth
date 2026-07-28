# Vedder VESC Interface

![ROS2 CI Workflow](https://github.com/f1tenth/vesc/workflows/ROS2%20CI%20Workflow/badge.svg)

Packages to interface with Vedder VESC motor controllers. See https://vesc-project.com/ for details

This is a ROS2 implementation of the ROS1 driver using the new serial driver located in [transport drivers](https://github.com/ros-drivers/transport_drivers).

## How to test

1. Clone this repository and [transport drivers](https://github.com/ros-drivers/transport_drivers) into `src`.
2. `rosdep update && rosdep install --from-paths src -i -y`
3. Plug in the VESC with a USB cable.
4. Modify `vesc/vesc_driver/params/vesc_config.yaml` to reflect any changes.
5. Build the packages `colcon build`
6. `ros2 launch vesc_driver vesc_driver_node.launch.py`
7. If prompted "permission denied" on the serial port: `sudo chmod 777 /dev/ttyACM0`

## Dual VESC/FESC Setup

If you have a dual-controller setup (e.g. VESC 6 EDU for servo/IMU, and FESC for high-current BLDC motor), you can run them simultaneously using the master launch file `vesc_fesc_system.launch.py`.

A dedicated **Bridge Node** (`vesc_fesc_bridge`) isolates each controller into its own clean namespace (`/fesc` and `/vesc`) while selectively routing standard messages between the global namespace and the drivers. This allows all downstream packages (e.g. odometry, ackermann, navigation, and SLAM) to run in the global namespace unmodified.

### Topic Routing Switchboard:

* **Global Commands ➔ Namespaced Controllers**:
  - `/commands/motor/speed` ➔ `/fesc/commands/motor/speed`
  - `/commands/motor/current` ➔ `/fesc/commands/motor/current`
  - `/commands/motor/brake` ➔ `/fesc/commands/motor/brake`
  - `/commands/motor/duty_cycle` ➔ `/fesc/commands/motor/duty_cycle`
  - `/commands/motor/position` ➔ `/fesc/commands/motor/position`
  - `/commands/servo/position` ➔ `/vesc/commands/servo/position`

* **Namespaced Telemetry ➔ Global Topics**:
  - `/fesc/sensors/core` ➔ `/sensors/core`
  - `/vesc/sensors/imu` ➔ `/sensors/imu`
  - `/vesc/sensors/imu/raw` ➔ `/sensors/imu/raw`
  - `/vesc/sensors/servo_position_command` ➔ `/sensors/servo_position_command`

### How to Run

You can launch either the full system stack (including downstream odometry and command translation nodes) or only the low-level drivers and the bridge node:

* **To run the full system**:
  ```bash
  ros2 launch vesc_driver vesc_fesc_system.launch.py
  ```

* **To run only the drivers and the topic bridge**:
  ```bash
  ros2 launch vesc_driver vesc_fesc_drivers.launch.py
  ```

If the connection order swaps (e.g., FESC gets assigned to `/dev/ttyACM1` and VESC to `/dev/ttyACM0`), you can override the ports on the fly on either launch file:

```bash
ros2 launch vesc_driver vesc_fesc_drivers.launch.py fesc_port:=/dev/ttyACM1 vesc_port:=/dev/ttyACM0
```
