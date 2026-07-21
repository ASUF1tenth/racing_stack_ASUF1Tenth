# F1Tenth Drive Commands & VESC Actuation Reference

This reference document outlines how the ForzaETH ROS 2 Jazzy stack structures driving commands, handles coordinate systems between the track trajectory and the vehicle body, and translates target velocity to physical actuator signals.

---

## 1. Drive Command Structure & Conventions
Autonomy nodes (like the state machine and controllers) publish motion commands to the `/drive` topic using the `ackermann_msgs/msg/AckermannDriveStamped` message type:

* **`drive.speed`**: Target longitudinal forward velocity of the vehicle in meters per second ($\text{m/s}$).
* **`drive.steering_angle`**: Target Ackermann steering angle in radians ($\text{rad}$), where positive is left and negative is right.

---

## 2. Velocity Coordinate Mapping: Path Tangent vs. Body Frame
A common point of confusion is how the target speed in the reference trajectory (defined in the global `map` frame) maps to the speed commanded to the vehicle (which acts along the body-frame `base_link` longitudinal X-axis).

### The Scalar Magnitude (`vx_mps`)
In the custom [Wpnt.msg](file:///home/mohany/Projects/f1tenth/highlevel/asuf1tenth/src/utilities/libraries/f110_msgs/msg/Wpnt.msg) trajectory message definition, there is **no $y$-axis speed parameter** (no `vy_mps`). Instead, the target speed is stored in a single field:
* **`vx_mps`**: A scalar value representing the target speed **magnitude** along the path tangent. The `x` in `vx_mps` refers to the longitudinal direction along the trajectory line itself, not the global map's Cartesian X-axis.

Because this target speed represents a scalar velocity magnitude, it maps directly to `drive.speed` (which is also a scalar magnitude along the vehicle's body-frame longitudinal axis) without needing coordinate frame projections.

### Non-Holonomic Constraint Alignment
An Ackermann steering vehicle is a **non-holonomic system**, meaning it can only actuate velocity along its longitudinal X-axis (`base_link` X-axis). It cannot slide or move directly sideways. 

1. **Path Tracking**: The steering controller constantly minimizes heading error ($\psi_{\text{car}} - \psi_{\text{path}}$) and lateral displacement.
2. **Alignment**: Under successful tracking, the vehicle's body-frame X-axis aligns directly with the tangent of the track centerline.
3. **Misalignment**: If the car is temporarily misaligned (e.g., returning to the racing line), the controller still commands the speed along the body X-axis. The steering controller rotates the body frame to match the tangent direction, while the throttle pushes the car forward along whatever heading it currently has to maintain momentum.

### Vector Projections inside the Controller
When the controller needs to perform 2D spatial projections in Cartesian map coordinates (such as lookahead position estimation), it transforms the body-frame speed into a map-frame velocity vector $[v_{\text{map}, x}, v_{\text{map}, y}]$ using the current heading $\psi$:
$$v_{\text{map}, x} = \cos(\psi) \cdot v_{\text{actual}}$$
$$v_{\text{map}, y} = \sin(\psi) \cdot v_{\text{actual}}$$

---

## 3. Actuator Control on the Physical Car (VESC)
On a physical F1Tenth vehicle, the target speed (m/s) is converted to motor **ERPM** (Electrical Revolutions Per Minute) for the Brushless Direct Current (BLDC) motor. 

Rather than calculating this conversion dynamically using wheel radius and gear ratio, the stack utilizes a **calibrated linear regression model** inside the `ackermann_to_vesc_node` (defined in [vesc.yaml](file:///home/mohany/Projects/f1tenth/highlevel/asuf1tenth/src/stack_master/config/NUC2/vesc.yaml)):

$$\text{ERPM} = (\text{speed\_to\_erpm\_gain} \cdot v_{\text{target}}) + \text{speed\_to\_erpm\_offset}$$

### Example Calibration Values (NUC2)
* `speed_to_erpm_gain`: `4352.0`
* `speed_to_erpm_offset`: `220.0`

### Why Linear Calibration is Used
Calculating ERPM purely through mechanical dimensions ($v / R \cdot \text{gear\_ratio} \cdot \text{pole\_pairs}$) fails to account for tire deformation under load, tire slippage, rolling resistance, and electrical losses. A linear fit based on experimental track data is much more robust.

---

## 4. Closed-Loop Feedback Data Pipeline

To enable closed-loop speed tracking, actual velocity feedback is routed from the simulator sensors back to the controller through the state estimation pipeline:

### Data Flow Diagram
```
[AutoDRIVE Unity Simulator] 
         │ (Actual physical state)
         ▼  /autodrive/roboracer_1/odom  (nav_msgs/msg/Odometry)
   [f110_autodrive_adapter]
         │ (Remaps frame headers to 'odom' / 'base_link')
         ▼  /odom  (nav_msgs/msg/Odometry)
      [EKF (robot_localization)]
         │ (Fuses IMU + Odom, filters noise)
         ▼  /early_fusion/odom  (nav_msgs/msg/Odometry)
    [carstate_node] ────► [frenet_odom_republisher]
         │                     │
         ▼                     ▼
   /car_state/odom       /car_state/frenet/odom
         │                     │
         └─────────┬───────────┘
                   ▼
          [controller_manager]
                   │
                   ▼ (Computes new target command)
                 /drive ───► [f110_autodrive_adapter] ───► [Simulator Actuators]
```

### Detail of Each Pipeline Step

1. **Step 1: Simulator to Adapter**
   The simulator publishes actual vehicle odometry to `/autodrive/roboracer_1/odom`. The `twist.twist.linear.x` field contains the ground-truth forward speed. The adapter maps frames to `odom` and `base_link` and publishes it to the standard `/odom` topic.
2. **Step 2: EKF Filtering**
   The EKF node (`robot_localization`) subscribes to `/odom`. In `ekf.yaml`, the EKF is configured to fuse the linear velocity `vx` (index 6):
   ```yaml
   odom0_config: [false, false, false,
                  false, false, true,
                  true,  false, false,  # Index 6 is vx (True)
                  ...]
   ```
   The EKF filters noise and publishes the result to `/early_fusion/odom`.
3. **Step 3: Carstate Node Aggregation**
   `carstate_node` subscribes to `/early_fusion/odom` and republishes it on `/car_state/odom` to aggregate the ego car's state.
4. **Step 4: Frenet Projection**
   The `frenet_odom_republisher` node subscribes to `/car_state/odom` and projects the Cartesian pose and velocities onto the track centerline coordinates, publishing the Frenet state to `/car_state/frenet/odom`.
5. **Step 5: Controller Loop Closure**
   The `controller_manager` subscribes to both `/car_state/odom` (saving it as `self.speed_now`) and `/car_state/frenet/odom` (saving the Frenet-aligned position and velocity `[s, d, vs, vd]`).

---

## 5. Feedback Control Objectives in the Controller
The controller uses these feedback streams for two separate tasks:

### A. Ego Speed Tracking (Open-Loop at Controller, Closed-Loop at Actuator)
For normal track-following:
* **The Controller Node is Open-Loop**: The controller manager does **not** run a PID loop to match the target speed. It simply publishes the target velocity command (in m/s) directly to the `/drive` topic.
* **Actuators close the loop**: The closed-loop speed tracking is offloaded to the hardware or simulator bridge:
  * On the physical car, the VESC motor controller runs its own internal PID control loop using physical wheel encoder ticks to match the requested speed.
  * In the AutoDRIVE simulation, your `f110_autodrive` adapter maps this speed command to the simulator's motor throttle using the quadratic feedforward equation.
* **`odom` Usage**: The controller uses `speed_now` (actual absolute Cartesian speed from `/car_state/odom`) to dynamically adjust steering limits and calculate the lateral lookahead distance:
  $$\text{L1\_distance} = q_{\text{l1}} + v_{\text{actual}} \cdot m_{\text{l1}}$$

### B. Opponent Trailing & Overtaking (Closed-Loop at Controller)
When a competitor car is detected, the state machine switches the vehicle to `"TRAILING"` mode. In this mode, the controller runs a closed-loop PID regulator internally to adjust its speed to follow the opponent at a safe distance. 

For this trailing loop, the controller **uses `frenet/odom` as feedback**:
* **Distance Error (Proportional/Integral terms)**: It calculates the gap between the cars using the progress along the track ($S$ coordinates):
  $$\text{gap} = s_{\text{opponent}} - s_{\text{ego}}$$
  *(Where $s_{\text{ego}}$ is retrieved from `position_in_map_frenet[0]`)*
* **Velocity Error (Derivative term)**: It calculates the rate of convergence using longitudinal velocities along the track ($v_s$):
  $$v_{\text{diff}} = v_{s\text{, ego}} - v_{s\text{, opponent}}$$
  *(Where $v_{s\text{, ego}}$ is retrieved from the Frenet odometry velocity `position_in_map_frenet[2]`)*

---

## 6. Cornering Drag Compensation
Because the simulator adapter uses a feedforward speed controller, the vehicle will experience speed drops in turns due to lateral tire slip and induced steering resistance. For details on how to resolve this, see: [cornering_drag_compensation.md](./cornering_drag_compensation.md).
