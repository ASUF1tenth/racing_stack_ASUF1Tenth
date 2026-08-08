# Comprehensive Configurable Parameters Guide for ROS 2 ForzaETH / ASU F1TENTH Stack

This document provides an exhaustive reference of all configurable parameters across the `stack_master` orchestrator and its integrated sub-modules (Drivers, SLAM, Global Planning, Motion Control, Perception, and State Machine).

---

## Quick Reference: Online vs Static Configuration

- **Online Configurable (Dynamic):** Can be modified on the fly while the car is driving using `ros2 param set <node_name> <parameter> <value>` (supported via ROS 2 parameter callbacks).
- **Static Configuration:** Requires modifying the target YAML / Lua / INI file and restarting the corresponding ROS node or launch file.

---

## 1. Drivers & Hardware Layer (`vesc`, `urg_node`, `ackermann_mux`)

Configured via `stack_master/config/<NUCx>/vesc.yaml`, `sensors.yaml`, `mux.yaml`.

| Parameter Name | Target Node / File | Data Type | Default Value | Online Configurable? | Meaning & Behavior Impact | Recommended Tuning Strategy |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| `speed_to_erpm_gain` | `vesc.yaml` | Float | `4614.0` | ❌ Static | Multiplier converting linear velocity (m/s) to motor ERPM. Direct impact on motor top speed and acceleration accuracy. | Calculate theoretically from motor pole pairs, pinion gear teeth, spur gear teeth, and wheel radius: $\text{ERPM} = v \times \frac{\text{Poles}}{2} \times \frac{\text{Spur}}{\text{Pinion}} \times \frac{60}{2 \pi r}$. Verify on physical chassis using optical tachometer. |
| `steering_angle_to_servo_offset` | `vesc.yaml` | Float | `0.5` | ❌ Static | Servo center position (normalized $0.0 - 1.0$). If miscalibrated, the car will pull to the left or right when driving straight. | Elevate car on stand; send `0.0` rad command. Adjust offset until front wheels align perfectly straight with chassis axis. |
| `steering_angle_to_servo_gain` | `vesc.yaml` | Float | `0.3183` | ❌ Static | Ratio mapping steering angle in radians to servo PWM duty position. Affects turning radius accuracy. | Command $+0.34\text{ rad}$ ($20^\circ$) and measure physical tire angle with protractor. Adjust gain until commanded angle matches physical angle. |
| `servo_min` / `servo_max` | `vesc.yaml` | Float | `0.15` / `0.85` | ❌ Static | Hard PWM bounds for steering servo. Prevents mechanical binding and steering linkage damage. | Turn wheels manually to physical mechanical stops; set `servo_min/max` slightly inside mechanical limit. |
| `current_max` | `vesc.yaml` | Float | `100.0` | ❌ Static | Maximum current drawn by motor (Amps). Limits peak torque. | Set according to motor driver thermal rating and battery C-rating to prevent over-current shutdowns. |
| `max_acceleration` | `throttle_interpolator` | Float | `2.5` | ❌ Static | Maximum allowed linear acceleration ($\text{m/s}^2$) enforced by smooth interpolator. Prevents wheelspin on acceleration. | Set based on tire friction coefficient $\mu$. Higher value allows punchier exit acceleration; lower value avoids slip. |
| `max_servo_speed` | `throttle_interpolator` | Float | `3.2` | ❌ Static | Maximum rate of change of steering angle ($\text{rad/s}$). Prevents aggressive servo jerk. | Set near physical servo transit speed (typically $3.0 - 4.5\text{ rad/s}$). |
| `angle_min` / `angle_max` | `sensors.yaml` (`urg_node`) | Float | `-3.14` / `3.14` | ❌ Static | Angular field of view (FOV) of LiDAR scanner in radians. | Keep at $\pm 3.14$ for full $360^\circ$ FOV unless vehicle chassis obstructs rear view. |
| `navigation.priority` | `mux.yaml` | Int | `10` | ❌ Static | `ackermann_mux` priority for autonomous `/drive` commands. | Must be lower than joystick priority (`100`) so manual input always overrides autonomous control. |
| `joystick.priority` | `mux.yaml` | Int | `100` | ❌ Static | `ackermann_mux` priority for manual teleop `/teleop` commands. | Set to highest priority (`100`) for safety override. |

---

## 2. State Estimation & SLAM Layer (`cartographer_ros`, `ekf`)

Configured via `stack_master/config/<NUCx>/slam/f110_2d.lua`, `f110_2d_loc.lua`.

| Parameter Name | Target Node / File | Data Type | Default Value | Online Configurable? | Meaning & Behavior Impact | Recommended Tuning Strategy |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| `max_constraint_distance` | `f110_2d_loc.lua` | Float | `15.0` | ❌ Static | Maximum search radius (meters) for Cartographer global constraint builder loop closure. **Critical safety setting.** | **Must be set to `15.0` on large maps (>200m).** If left unconstrained (`>100m`), Cartographer will throw `std::length_error` memory allocation crash during initialization. |
| `optimize_every_n_nodes` | `f110_2d_loc.lua` | Int | `5` | ❌ Static | Number of submaps built before running global pose graph optimization. | Lower value ($5$) increases localization correction frequency at cost of CPU load; higher value ($20$) reduces CPU usage. |
| `num_range_data` | `f110_2d.lua` | Int | `80` | ❌ Static | Number of LiDAR scans per Cartographer submap during mapping. | Set between $60 - 90$. Smaller values create finer submaps for tight indoor tracks. |
| `min_range` / `max_range` | `f110_2d.lua` | Float | `0.05` / `25.0` | ❌ Static | Distance bounds (meters) for LiDAR scan points used in SLAM. | Set `min_range` to `0.15` to exclude car body reflections. Set `max_range` to `12.0 - 25.0` depending on room size. |
| `odometry_sampling_ratio` | `f110_2d_loc.lua` | Float | `0.5` | ❌ Static | Downsampling ratio for wheel odometry messages passed to SLAM. | Keep at `0.5` to balance CPU utilization and velocity update accuracy. |

---

## 3. Global Planning & Sector Velocity Layer (`global_planner`, `sector_tuner`)

Configured via `stack_master/config/global_planner/racecar_f110.ini`, `global_planner_params.yaml`, `speed_scaling.yaml`.

| Parameter Name | Target Node / File | Data Type | Default Value | Online Configurable? | Meaning & Behavior Impact | Recommended Tuning Strategy & Online Command |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| `global_limit` | `/sector_tuner` | Float | `0.2` ($20\%$) | **YES (Online)** | Master velocity multiplier ($0.0 - 1.0$) scaling all target speeds globally. | **Primary speed tuning knob.** Start at `0.2` for initial trials. Gradually scale up ($0.3 \rightarrow 0.5 \rightarrow 0.8 \rightarrow 1.0$).<br>`ros2 param set /sector_tuner global_limit 0.4` |
| `SectorX.scaling` | `/sector_tuner` | Float | `1.0` | **YES (Online)** | Velocity multiplier ($0.0 - 1.0$) applied specifically to track Sector $X$. | Tune individual corner or straight speeds live without rebuilding package.<br>`ros2 param set /sector_tuner Sector0.scaling 0.8` |
| `curv_opt_type` | `racecar_f110.ini` | String | `"mincurv"` | ❌ Static | Global trajectory optimization objective (`"mincurv"` for minimum curvature, `"mintime"` for laptime optimization). | Use `"mincurv"` for initial safe driving; switch to `"mintime"` for competitive racing laptimes. |
| `safety_width` | `global_planner_params.yaml` | Float | `0.5` | ❌ Static | Track clearance buffer (meters) including vehicle chassis width. Keeps raceline away from walls. | Set to `car_width + 0.15m` (e.g. `0.40m + 0.15m = 0.55m`) to ensure safe wall clearance margin. |
| `occupancy_grid_threshold` | `global_planner_params.yaml` | Int | `30` | ❌ Static | Map grid cell probability threshold ($0 - 100$) above which cells are marked as walls. | Set between $20 - 40$. Lower values treat faint obstacles as walls; higher values ignore minor map noise. |

---

## 4. Motion Control Layer (`controller_manager`, `map`, `pp`)

Configured via `stack_master/config/<NUCx>/l1_params.yaml`.

| Parameter Name | Target Node / File | Data Type | Default Value | Online Configurable? | Meaning & Behavior Impact | Recommended Tuning Strategy & Online Command |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| `m_l1` / `q_l1` | `/controller` | Float | `0.583` / `-0.167` | **YES (Online)** | Linear slope ($m$) and intercept ($q$) for speed-dependent Pure Pursuit lookahead distance ($L_d = m \cdot v + q$). | Increase $m_l1$ if car oscillates at high speeds; decrease if car cuts corners excessively at low speeds.<br>`ros2 param set /controller m_l1 0.6` |
| `t_clip_min` / `t_clip_max` | `/controller` | Float | `0.8` / `5.0` | **YES (Online)** | Minimum and maximum lookahead distance bounds (meters). | Set `t_clip_min` to `0.8m` to prevent acute steering lock at low speed; set `t_clip_max` to `3.5m - 5.0m`.<br>`ros2 param set /controller t_clip_min 1.0` |
| `lat_err_coeff` | `/controller` | Float | `1.0` | **YES (Online)** | Cross-track error feedback gain multiplier. Higher values aggressively force car onto reference line. | Set to `1.0`. Increase to `1.2 - 1.5` if car drifts off raceline on turns.<br>`ros2 param set /controller lat_err_coeff 1.2` |
| `acc_scaler_for_steer` | `/controller` | Float | `1.2` | **YES (Online)** | Acceleration scaling factor applied when entering turns. | Lower value ($0.9 - 1.0$) slows entry for stability; higher value ($1.2$) maintains throttle out of corners.<br>`ros2 param set /controller acc_scaler_for_steer 1.0` |
| `dec_scaler_for_steer` | `/controller` | Float | `0.9` | **YES (Online)** | Braking/deceleration factor applied when high lateral error or steering angle is detected. | Adjust between `0.8 - 0.95` to tune trail-braking behavior into sharp corners.<br>`ros2 param set /controller dec_scaler_for_steer 0.85` |
| `speed_lookahead_for_steer` | `/controller` | Float | `0.0` | **YES (Online)** | Distance ahead along trajectory (meters) used to compute upcoming curvature for steering pre-alignment. | Increase to `0.15 - 0.20m` for faster laptimes, but monitor for corner clipping.<br>`ros2 param set /controller speed_lookahead_for_steer 0.15` |
| `vel_ctr_p_gain` / `vel_ctr_d_gain` | `/controller` | Float | `8.0` / `0.5` | **YES (Online)** | Proportional ($P$) and Derivative ($D$) gains for acceleration velocity controller when `vel_accel_mode=True`. | Increase $P$ for punchier throttle response; increase $D$ to damp speed overshooting.<br>`ros2 param set /controller vel_ctr_p_gain 10.0` |

---

## 5. Perception & Opponent Tracking Layer (`perception`, `box_detector`)

Configured via `stack_master/config/opponent_tracker_params.yaml`.

| Parameter Name | Target Node / File | Data Type | Default Value | Online Configurable? | Meaning & Behavior Impact | Recommended Tuning Strategy & Online Command |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| `detect.sigma` | `/perception/detect` | Float | `0.03` | **YES (Online)** | Assumed standard deviation of LiDAR range noise (meters). | Set to match LiDAR hardware spec ($0.02 - 0.04\text{ m}$).<br>`ros2 param set /perception/detect sigma 0.02` |
| `detect.min_obs_size` | `/perception/detect` | Int | `10` | **YES (Online)** | Minimum number of raw LiDAR points required to form a valid obstacle cluster. | Lower value ($5$) detects smaller/distant opponents; higher value ($15$) filters out sensor noise.<br>`ros2 param set /perception/detect min_obs_size 8` |
| `tracking.max_dist` | `/perception/tracking` | Float | `0.5` | **YES (Online)** | Maximum association distance (meters) for matching new detections to existing tracks in EKF. | Set to `0.5m`. Increase if tracking drops fast-moving opponent cars.<br>`ros2 param set /perception/tracking max_dist 0.6` |
| `tracking.P_vs` / `tracking.P_vd` | `/perception/tracking` | Float | `0.2` / `0.2` | **YES (Online)** | EKF state covariance gains for tracking opponent longitudinal ($v_s$) and lateral ($v_d$) velocities. | Adjust to smooth out noisy velocity estimates of opponent cars.<br>`ros2 param set /perception/tracking P_vs 0.3` |

---

## 6. State Machine & High-Level Racing Layer (`state_machine`)

Configured via `stack_master/config/state_machine_params.yaml`.

| Parameter Name | Target Node / File | Data Type | Default Value | Online Configurable? | Meaning & Behavior Impact | Recommended Tuning Strategy & Online Command |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| `lateral_width_gb_m` | `/state_machine` | Float | `1.1` | **YES (Online)** | Lateral clearance width (meters) around global path within which obstacles trigger overtaking. | Set to `1.0m - 1.2m` based on track width.<br>`ros2 param set /state_machine lateral_width_gb_m 1.0` |
| `lateral_width_ot_m` | `/state_machine` | Float | `0.3` | **YES (Online)** | Lateral clearance width (meters) required to consider an overtaking line clear of opponents. | Increase for safer, wider overtaking passes; decrease for tight racing.<br>`ros2 param set /state_machine lateral_width_ot_m 0.4` |
| `splini_ttl` | `/state_machine` | Float | `2.0` | **YES (Online)** | Time-to-live counter (seconds) for validity of local spline evasion waypoints. | Lower value ($1.0 - 1.5\text{ s}$) forces frequent path re-planning around moving opponents.<br>`ros2 param set /state_machine splini_ttl 1.5` |
| `splini_hyst_timer_sec` | `/state_machine` | Float | `0.2` | **YES (Online)** | Cooldown timer (seconds) required before switching overtake trajectory from left to right side. | Prevents rapid oscillation ("chattering") between left and right evasion paths.<br>`ros2 param set /state_machine splini_hyst_timer_sec 0.3` |
| `ftg_threshold_speed` | `/state_machine` | Float | `0.1` | **YES (Online)** | Low-speed threshold (m/s) triggering dead-lock detection timer. | If vehicle speed drops below threshold for `ftg_timer_sec`, Follow-The-Gap emergency recovery activates.<br>`ros2 param set /state_machine ftg_threshold_speed 0.2` |
| `force_state` | `/state_machine` | Bool | `False` | **YES (Online)** | Forces state machine to lock into a specific state (`GB_TRACK`, `OVERTAKE`, etc.) for debugging. | Set `True` during isolated module testing.<br>`ros2 param set /state_machine force_state True` |

---

## Command Quick Sheet for Live Tuning

### 1. Adjust Master Vehicle Speed Live
```bash
ros2 param set /sector_tuner global_limit 0.4
```

### 2. Tune Sector 1 Corner Speed Live
```bash
ros2 param set /sector_tuner Sector1.scaling 0.75
```

### 3. Tune Controller Lookahead & Tracking Error Live
```bash
ros2 param set /controller m_l1 0.65
ros2 param set /controller lat_err_coeff 1.25
```

### 4. Adjust Overtaking Clearance Width Live
```bash
ros2 param set /state_machine lateral_width_gb_m 1.0
```
