# Stack Master
Here is the `stack_master`, it is intended to be the main interface between the user and the PBL ForzaETH F110 system.

Additional documentation:
- [`CONFIGURABLE_PARAMETERS.md`](./CONFIGURABLE_PARAMETERS.md) — full reference of all tunable parameters (drivers, SLAM, global planning, control, perception, state machine) with live-tuning commands.
- [`MAPPING_AND_FILTERING_TUNING.md`](./MAPPING_AND_FILTERING_TUNING.md) — diagnosis and fixes for map "leaks" and the `create_map` / `map_editor` modes.

### Mapping (on the real car)
Run the mapping launch file, specifying the map name and the NUCX version:
```shell
ros2 launch stack_master mapping_launch.xml racecar_version:=<NUCX used> map_name:=<map name of choice> [remote:=true/false] [use_legacy_drivers:=true/false] [sim:=true/false] [sim_type:=autodrive/gym]
```
  - `<map name of choice>` can be any name with no white space. Conventionally we use the location name (eg, 'hangar', 'ETZ', 'icra') followed by the day of the month followed by an incremental version number. For instance, `glc_ot_ez`. (Note: Make sure that the map's folder has `speed_scaling.yaml` and `ot_sectors.yaml` files availalbe)
  - `<NUCX>` depends on which car you are using. Parameters are available for NUC2, NUC5, NUC6, SIM (the latter represents a dummy car).
  - `remote` (optional, default `false`): set to `true` if you are using a split-hardware setup where `vesc_driver` and the LiDAR node are running directly on the Raspberry Pi 4, and only the mapping/estimation stack is running on your remote laptop.
  - `use_legacy_drivers` (optional, default `false`): set to `true` if you want to use the legacy driver stack (Hokuyo `urg_node` and single standard `vesc_driver`) instead of the default `drivers_bringup` (RPLiDAR and dual motor VESC/FESC setup).
  - `sim` (optional, default `false`): set to `true` to map in a simulator instead of on the real car.
  - `sim_type` (optional, default `autodrive`): simulator backend used when `sim:=true`. Either `autodrive` (AutoDRIVE RoboRacer) or `gym` (F1TENTH Gym).

> [!IMPORTANT]
> **Mapping Best Practices & Initial Pose Alignment:**
> * **Mapping Start Position:** Cartographer sets the $(0, 0, 0)$ map origin at the location where `mapping_launch.xml` is first started. Take note of where the car was placed when starting the mapping run.
> * **Localization Startup:** When launching `base_system_launch.xml`, Cartographer will place the initial pose of the car at $(0, 0, 0)$ (the mapping start position). Always place the physical car near the mapping start line or (TODO: Allow RViz **2D Pose Estimate** to set the initial pose before driving).
> * **Large Map Safeguard:** Driving an unlocalized car on large maps (>200m) causes Cartographer to perform an unconstrained global search, which can throw `std::length_error: cannot create std::vector larger than max_size()`. Setting `POSE_GRAPH.constraint_builder.max_constraint_distance = 15.0` in your car's `slam/f110_2d_loc.lua` prevents this memory allocation crash.

After completing a lap, a GUI will popup and pressing the requested button will start the global raceline generation. 
Then two GUIs will be shown, and within them a slider can be used to select the sectors. 
Be careful as once a sector is chosen it cannot be further subdivided. 

From here on, remember to `source` the ROS 2 workspace in any new terminal.

### Base System
```shell
ros2 launch stack_master base_system_launch.xml map_name:=<name of mapped track> sim:=<true/false> racecar_version:=<NUCX used> [remote:=true/false] [use_legacy_drivers:=true/false] [sim_type:=autodrive/gym]
```
  - `<name of mapped track>` is the name of the track you want to run on. It must belong to the list of maps available in the `stack_master/maps` folder.
  - `<true/false>` is a boolean value that indicates if you want to run the simulation or the real car. 
  - `<NUCX>` depends on which car you are using. Parameters are available for NUC2, NUC5, NUC6, SIM (the latter represents a dummy car).
  - `remote` (optional, default `false`): set to `true` if you are using a split-hardware setup where VESC and LiDAR nodes are launched directly on the Pi, and only the control/localization nodes are run on the remote laptop.
  - `use_legacy_drivers` (optional, default `false`): set to `true` if you want to use the legacy driver stack (Hokuyo `urg_node` and single standard `vesc_driver`) instead of the default `drivers_bringup` (RPLiDAR and dual motor VESC/FESC setup).
  - `sim_type` (optional, default `autodrive`): simulator backend used when `sim:=true`. Either `autodrive` (AutoDRIVE RoboRacer) or `gym` (F1TENTH Gym).


### Time trials 
```shell
ros2 launch stack_master time_trials_launch.xml racecar_version:=<NUCx used> LU_table:=<Look-Up Table name> ctrl_algo:=<control algorithm> 
```
  - `<NUCx>` depends on which car you are using. Parameters are available for NUC2, NUC5, NUC6, SIM (the latter represents a dummy car).
  - `<Look-Up Table name>` is the name of the Look-Up Table you want to use. It must belong to the list of Look-Up Tables available in the `system_identification/steering_lookup/cfg` folder.
  - `<control algorithm>` is the control algorithm you want to use. Current possibilities are MAP / PP.

### Speed Scaling / Velocity Tuning
The velocities generated from the map trajectories can be scaled down or tuned using the `sector_tuner` node parameters.

* **Live Tuning (Dynamic):**
  Adjust speed scaling on the fly while running:
  ```shell
  ros2 param set /sector_tuner global_limit 0.2
  ros2 param set /sector_tuner Sector0.scaling 0.2
  ```
* **Permanent Config:**
  Modify `src/race_stack/stack_master/maps/<map_name>/speed_scaling.yaml` and rebuild using:
  ```shell
  colcon build --packages-select stack_master
  ```
  *(Tip: Rebuild with `colcon build --symlink-install` to avoid rebuilding for future YAML changes!)*

### Head to Head
```shell
ros2 launch stack_master head_to_head_launch.xml racecar_version:=<NUCx used> LU_table:=<Look-Up Table name> ctrl_algo:=<control algorithm> overtake_mode:=spliner
```
- `<NUCx>` depends on which car you are using. Parameters are available for NUC2, NUC5, NUC6, SIM (the latter represents a dummy car).
- `<Look-Up Table name>` is the name of the Look-Up Table you want to use. It must belong to the list of Look-Up Tables available in the `system_identification/steering_lookup/cfg` folder.
- `<control algorithm>` is the control algorithm you want to use. Current possibilities are MAP / PP.
- `<overtake_mode>` is the mode you want to use for overtaking. `spliner` is the only current possibility.

### Running the Spliner
The spline overtaking planner is launched through the head-to-head integration:

```bash
ros2 launch stack_master head_to_head_launch.xml racecar_version:=SIM LU_table:=SIM_linear ctrl_algo:=MAP overtake_mode:=spliner
```

Notes:
- Ensure you run `colcon build` from the workspace root and then source your workspace's `install/setup.bash` so launch files and package shares are available.
- `LU_table` must match a CSV file present in the `steering_lookup` package share (e.g. `SIM_linear`).

### Keyboard Teleoperation (Real Car / Simulation)
If a gamepad/joystick is not available, you can control the vehicle using keyboard inputs by running two nodes in a separate terminal:
```shell
ros2 run teleop_twist_keyboard teleop_twist_keyboard
ros2 run stack_master keyboard_control [--ros-args -p max_speed:=1.0 -p steering_angle:=0.34]
```
- `teleop_twist_keyboard` reads keystrokes and publishes `Twist` messages on `/cmd_vel`.
- `keyboard_control` bridges `/cmd_vel` to `AckermannDriveStamped` on `/teleop` (priority 100 in `ackermann_mux`). It can be used during mapping, testing, or simulation.
- `max_speed` (optional, default `1.0`): Maximum speed ceiling (m/s) for safety clamping on hardware.
- `steering_angle` (optional, default `1.0`): Full steering angle (rad) applied at full keyboard deflection; scale it down (e.g. `0.34` rad) for smoother control.

