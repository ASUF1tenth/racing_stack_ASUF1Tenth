# Global Planner 

## Description
The Global Planner package is a ROS2 package designed to generate a global trajectory around a race track. It provides a node that subscribes to map and car pose information, processes this data, and publishes global waypoints, track boundaries and map informations and saves everything in a JSON file. The logic of the Global Planner is separated into three parts: ROS2 node, logic and utility functions. The logic of the Global Planner is implemented in the `global_planner_logic.py` module and the utility function are in the `global_planner_utils.py` module. This package is implemented in Python and uses the `rclpy` library for ROS interactions. For the calculation of the global trajectory, it uses the [TUM Global Race Trajectory Optimization](https://github.com/TUMFTM/global_racetrajectory_optimization) package, which was modified so that it was possible to integrate it in our race stack. The main node `global_planner_node.py` is launched with the `mapping_launch.xml` file from the `stack_master` package or if the `map_editor` is used. There is also a node `global_trajectory_publisher.py` for publishing the global waypoints and markers and track bounds markers frequently after they have been calculated which is launched every time the base system is launched.

## Parameters & Tuning Guidelines
 - `rate`: Sets the rate in hertz at which the ROS node is running.
 - `safety_width`: Sets the width of the car in meters including a safety margin for the calculation of the global trajectory. *(Default: `0.3m` for F1TENTH. Reduce to `0.3m` on narrow tracks <= 0.50m width to ensure Quadratic Programming solver constraints remain feasible).*
 - `safety_width_sp`: Sets the width of the car in meters including a safety margin for the calculation of the shortest path trajectory. *(Default: `0.3m`).*
 - `occupancy_grid_threshold`: Sets the threshold above which a cell in the occupancy grid is considered occupied and below it is considered free. *(Recommended: `20-30` for small/noisy tracks).*
 - `filter_kernel_size`: The size of the kernel for morphological opening filtering. *(Recommended: `3` for small/narrow tracks to prevent eroding away driveable track area into a completely black image).*
 - `show_plots`: Show additional plots for debugging during generation of the global trajectory.
 - `map_editor`: `True` if the map editor is used.
 - `map_name`: Sets the name of the map.
 - `create_map`: Create a map that can be used in the simulator and save a YAML file and a PNG of the map in the directory `stack_master/maps/map_name`.
 - `reverse_mapping`: Generate a global trajectory in the reverse direction.
 - `required_laps`: Number of laps required before the generation of the global trajectory can be started. **Note:** When `required_laps: 0` or when running in `map_editor` / manual mode, `cent_length` evaluates to `0.0m` as normal, expected behavior. This instructs `extract_centerline` to skip the +/-15% driven distance validation check and accept the closed contour found in the map image.

## Trajectory Optimization Fallback Behavior
The node includes automatic multi-stage fallbacks in `global_planner_logic.py` for:
1. **Global Trajectory (`mincurv_iqp`)**: If iterative minimum curvature optimization (IQP) encounters inconsistent QP constraints (e.g. at tight corner apexes on narrow tracks), the node catches `ValueError` / `RuntimeError` and automatically falls back to single-pass minimum curvature optimization (`mincurv`).
2. **Overtaking Trajectory (`mincurv_iqp`)**: Automatically falls back to single-pass `mincurv` if IQP fails on overtaking trajectory calculation.
3. **Shortest Path Trajectory (`shortest_path`)**: Automatically falls back to single-pass `mincurv` if shortest path optimization encounters infeasibility.

## Offline Pipeline Execution
To run the global planning pipeline offline on an existing map (without live SLAM or re-mapping):
```bash
ros2 launch map_editor map_editor_launch.xml map_name:=workspace_loop map_editor_mapping:=False
```

## Input/Output Topic Signature
This node subscribes to:
- `/map`: Subscribes to the occupancy grid of the map.
- `/car_state/pose`: Reads the car's pose
    
The node publishes to:
- `/global_waypoints`: Publishes global waypoints.
- `/global_waypoints/markers`: Publishes global waypoint markers.
- `/global_waypoints/shortest_path`: Publishes waypoints of the shortest path trajectory (only used for visualization of the overtaking sectors).
- `/global_waypoints/shortest_path/markers`: Publishes waypoint markers of the shortest path trajectory (only used for visualization of the overtaking sectors).
- `/centerline_waypoints`: Publishes centerline waypoints.
- `/centerline_waypoints/markers`: Publishes centerline waypoint markers.
- `/trackbounds/markers`: Publishes markers for the track boundaries.
- `/map_infos`: Publishes map infos like estimated lap time and maximum speed.
- `/estimated_lap_time`: Publishes the estimated lap time.
