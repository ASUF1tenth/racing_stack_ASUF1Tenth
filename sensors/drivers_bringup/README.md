# drivers_bringup

ROS 2 launch aggregator for F1TENTH low-level hardware drivers. Provides a single entry point to bring up all onboard hardware:

- **VESC/FESC** -- BLDC drive motor and steering servo + IMU (via serial)
- **SLLiDAR A1** -- 2D laser scan data (via serial)

## Dependencies

| Package | Purpose |
|---|---|
| `vesc_driver` | Motor/servo/IMU driver |
| `sllidar_ros2` | Slamtec SLLiDAR A1 driver |

## Usage

```bash
ros2 launch drivers_bringup drivers_bringup.launch.py
```

### Configurable Arguments

| Argument | Default | Description |
|---|---|---|
| `fesc_port` | `/dev/ttyACM0` | FESC (BLDC motor) serial port |
| `vesc_port` | `/dev/ttyACM1` | VESC (servo + IMU) serial port |
| `vesc_config` | *(vesc_driver default)* | Path to VESC configuration YAML |
| `channel_type` | `serial` | SLLiDAR connection type |
| `serial_port` | `/dev/ttyUSB0` | SLLiDAR serial port |
| `serial_baudrate` | `115200` | SLLiDAR baud rate |
| `frame_id` | `laser` | TF frame ID for scan data |
| `inverted` | `false` | Invert scan data |
| `angle_compensate` | `true` | Enable angle compensation |

Override any argument at launch time:

```bash
ros2 launch drivers_bringup drivers_bringup.launch.py \
  fesc_port:=/dev/ttyACM2 \
  serial_port:=/dev/ttyUSB1 \
  frame_id:=laser_link
```

## License

Apache-2.0
