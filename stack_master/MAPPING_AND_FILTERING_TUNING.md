# Comprehensive SLAM Mapping, Filtering & Map-Editor Guide

This document explains:
1. **Root Cause & Diagnosis of Map "Leaks"** (Why outer rooms and central islands turn white in binarized maps).
2. **Parameters & Modes:** `create_map` vs `map_editor` vs Automated Filtering.
3. **Step-by-Step Fixes** (Cartographer LiDAR range tuning + Manual GIMP Image Cleanup).

---

## 1. Diagnosis of Map "Leaks"

### Why do leaks happen even if physical walls look sealed?

In the binarized map images (`pf_map.png` / `ws_everest.png`), free space is represented as **White ($255$)** and walls/unexplored regions are **Black ($0$)**. 

```
[LiDAR Ray Penetration] ──> [Cartographer Occupancy < 30] ──> [Binarization: < 30 = 255 (White)]
                                                                               │
                                       [Entire Outer Room & Center Island Turn White!] ◄───┘
```

#### Causes:
1. **Laser Penetration & Gap Leakage:** LiDAR rays ($905\text{ nm}$ infrared) shoot through small gap under cardboard/fences, glass windows, dark acrylic, or table legs into the outer room or central island.
2. **Cartographer Free Space Assignment:** Cartographer marks all ray-traced cells behind the wall with low occupancy scores (e.g. $0 - 25\%$).
3. **Automated Binarization (`filter_map_occupancy_grid`):**
   ```python
   original_map = np.where(original_map == -1, 100, original_map)
   bw = np.where(original_map < occupancy_grid_threshold, 255, 0)
   ```
   Because `original_map < 30` evaluates to `True` for all those ray-traced cells behind the wall, **the entire outer room and central island turn solid white ($255$)**.
4. **Thin Wall Breaches:** If a wall is only 1-2 pixels thick, a single missing pixel allows the white drivable track to leak out and fuse with the outer white room, preventing the Watershed contour algorithm from finding closed track boundaries!

---

## 2. Modes & Parameter Reference: `map_editor` vs `create_map`

Configured in `stack_master/launch/mapping_launch.xml` and `stack_master/config/global_planner/global_planner_params.yaml`.

| Parameter Name | Config Location | Data Type | Default Value | Description & Behavior |
| :--- | :--- | :--- | :--- | :--- |
| `create_map` | `mapping_launch.xml` / `global_planner_node.py` | Bool | `True` | • `True`: Subscribes to live `/map` topic from Cartographer, processes grid, and saves PNG/YAML.<br>• `False`: Offline mode. Loads pre-existing PNG/YAML files from disk and re-calculates racelines. |
| `map_editor` | `mapping_launch.xml` / `global_planner_node.py` | Bool | `False` | • `False` (Automated): Automatically checks lap count, filters map, binarizes, skeletonizes, and computes raceline.<br>• `True` (Manual Edit): Opens a GUI window (`matplotlib`) showing the binarized map. Allows you to open the PNG file in GIMP/Photoshop and paint black boundaries to seal leaks before clicking **"Map ready; compute global trajectory"**. |
| `occupancy_grid_threshold` | `global_planner_params.yaml` | Int | `30` | Probability threshold ($0 - 100$) for binarizing grid.<br>$\text{Pixel} = \begin{cases} 255 \text{ (Free space)}, & P < \text{threshold} \\ 0 \text{ (Wall/Obstacle)}, & P \ge \text{threshold} \end{cases}$ |
| `filter_kernel_size` | `global_planner_params.yaml` | Int | `3` | Kernel size ($3 \times 3$) for morphological opening (`cv2.MORPH_OPEN` with 2 iterations). Removes speckle noise without rounding track corners. |
| `watershed` | `global_planner_logic.py` | Bool | `True` | Uses OpenCV Watershed image segmentation to extract closed inner and outer track boundaries from the binarized image. |

---

## 3. How to Fix Map Leaks: Step-by-Step Solutions

### Solution 1: Limit Cartographer LiDAR Scan Range (Hardware Fix)
In `stack_master/config/<NUCx>/slam/f110_2d.lua`:
- Lower `TRAJECTORY_BUILDER_2D.max_range = 8.0` or `10.0` (down from `25.0`).
- This stops LiDAR rays from mapping room space far outside your track barriers.

### Solution 2: Manual Image Cleaning via `map_editor` (Image Fix)
If LiDAR rays already leaked into the PNG map:

1. Launch mapping with `map_editor:=True`:
   ```bash
   ros2 launch stack_master mapping_launch.xml map_name:=my_track map_editor:=True
   ```
2. Once the matplotlib GUI pops up, open the generated PNG image in **GIMP**, **Pinta**, or **Photoshop**:
   `stack_master/maps/my_track/my_track.png`
3. Select a **solid black brush ($0,0,0$)**:
   - Paint over the entire outer room background until it is completely black.
   - Paint over the central island until it is completely black.
   - Seal any thin gaps in the inner and outer track wall lines.
4. Save the edited PNG image.
5. In the terminal / GUI window, click **"Map ready; compute global trajectory"**.
6. `global_planner` will extract the clean black/white track loop and compute your time-optimal raceline perfectly!
