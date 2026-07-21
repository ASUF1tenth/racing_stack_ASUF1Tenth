# Stack Master
Here is the `stack_master`, it is intended to be the main interface between the user and the PBL ForzaETH F110 system.

### Mapping (on the real car)
Run the mapping launch file, specifying the map name and the NUCX version:
```shell
ros2 launch stack_master mapping_launch.xml racecar_version:=<NUCX used> map_name:=<map name of choice> [remote:=true/false]
```
  - `<map name of choice>` can be any name with no white space. Conventionally we use the location name (eg, 'hangar', 'ETZ', 'icra') followed by the day of the month followed by an incremental version number. For instance, `hangar_12_v0`.
  - `<NUCX>` depends on which car you are using. Parameters are available for NUC2, NUC5, NUC6, SIM (the latter represents a dummy car).
  - `remote` (optional, default `false`): set to `true` if you are using a split-hardware setup where `vesc_driver` and the LiDAR node are running directly on the Raspberry Pi 4, and only the mapping/estimation stack is running on your remote laptop.

After completing a lap, a GUI will popup and pressing the requested button will start the global raceline generation. 
Then two GUIs will be shown, and within them a slider can be used to select the sectors. 
Be careful as once a sector is chosen it cannot be further subdivided. 

A ROS resourcing will be needed from here on. 

### Base System
```shell
ros2 launch stack_master base_system_launch.xml map_name:=<name of mapped track> sim:=<true/false> racecar_version:=<NUCX used> [remote:=true/false]
```
  - `<name of mapped track>` is the name of the track you want to run on. It must belong to the list of maps available in the `stack_master/maps` folder.
  - `<true/false>` is a boolean value that indicates if you want to run the simulation or the real car. 
  - `<NUCX>` depends on which car you are using. Parameters are available for NUC2, NUC5, NUC6, SIM (the latter represents a dummy car).
  - `remote` (optional, default `false`): set to `true` if you are using a split-hardware setup where VESC and LiDAR nodes are launched directly on the Pi, and only the control/localization nodes are run on the remote laptop.

### Time trials 
```shell
ros2 launch stack_master time_trials_launch.xml racecar_version:=<NUCx used> LU_table:=<Look-Up Table name> ctrl_algo:=<control algorithm> 
```
  - `<NUCx>` depends on which car you are using. Parameters are available for NUC2, NUC5, NUC6, SIM (the latter represents a dummy car).
  - `<Look-Up Table name>` is the name of the Look-Up Table you want to use. It must belong to the list of Look-Up Tables available in the `systm_identification/steering_lookup/cfg` folder.
  - `<control algorithm>` is the control algorithm you want to use. Current possibilities are MAP / PP.

### Head to Head
```shell
ros2 launch stack_master head_to_head_launch.xml racecar_version:=<NUCx used> LU_table:=<Look-Up Table name> ctrl_algo:=<control algorithm> overtake_mode:=spliner
```
- `<NUCx>` depends on which car you are using. Parameters are available for NUC2, NUC5, NUC6, SIM (the latter represents a dummy car).
- `<Look-Up Table name>` is the name of the Look-Up Table you want to use. It must belong to the list of Look-Up Tables available in the `systm_identification/steering_lookup/cfg` folder.
- `<control algorithm>` is the control algorithm you want to use. Current possibilities are MAP / PP.
- `<overtake_mode>` is the mode you want to use for overtaking. `spliner` is the only current possibility.

## Running the Spliners

Predictive spliner (direct package launch):

```bash
source /home/mohany/ws/install/setup.bash
ros2 launch predictive_spliner predictive_spliner_launch.xml
```

Predictive spliner (via head-to-head integration):

```bash
source /home/mohany/ws/install/setup.bash
ros2 launch stack_master head_to_head_launch.xml racecar_version:=SIM LU_table:=SIM_linear ctrl_algo:=MAP overtake_mode:=predictive_spliner
```

Default spliner (existing spliner mode via head-to-head):

```bash
source /home/mohany/ws/install/setup.bash
ros2 launch stack_master head_to_head_launch.xml racecar_version:=SIM LU_table:=SIM_linear ctrl_algo:=MAP overtake_mode:=spliner
```

Notes:
- Ensure you run `colcon build` from the workspace root and then `source /home/mohany/ws/install/setup.bash` so launch files and package shares are available.
- `LU_table` must match a CSV file present in the `steering_lookup` package share (e.g. `SIM_linear`).

### Keyboard Teleoperation (Real Car / Simulation)
If a gamepad/joystick is not available, you can control the vehicle using keyboard inputs by launching the standalone keyboard teleop launch file in a separate terminal:
```shell
ros2 launch stack_master keyboard_teleop_launch.xml [speed:=0.5] [turn:=0.34] [max_speed:=1.0]
```
- `speed` (optional, default `0.5`): Initial speed step setting for `teleop_twist_keyboard` in m/s.
- `turn` (optional, default `0.34`): Steering angle step setting in radians (~20 degrees).
- `max_speed` (optional, default `1.0`): Maximum speed ceiling for safety clamping on hardware.

This launch file starts `teleop_twist_keyboard` (in an `xterm` window) alongside the `keyboard_control` bridge node, publishing commands directly to `/teleop` (priority 100 in `ackermann_mux`). It can be used during mapping, testing, or simulation.

*Note: Requires `xterm` (`sudo apt install xterm`) to open a dedicated interactive window for keystrokes. Alternatively, run `ros2 run teleop_twist_keyboard teleop_twist_keyboard` directly in your terminal.*

