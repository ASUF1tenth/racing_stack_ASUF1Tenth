# TODO: ROS 2 Jazzy Runtime Warnings Cleanup

This file tracks outstanding runtime warnings and OpenGL shader errors observed during simulation runs inside the Docker container. These do not block execution but should be resolved to clean up console logs.

---

## 1. `ROS_LOCALHOST_ONLY` Deprecation Warning
* **Warning Message**:
  ```text
  [rcl]: ROS_LOCALHOST_ONLY is deprecated but still honored if it is enabled. Use ROS_AUTOMATIC_DISCOVERY_RANGE and ROS_STATIC_PEERS instead.
  ```
* **Root Cause**: The stack configurations (likely in `devcontainer.json` or docker launch scripts) set `ROS_LOCALHOST_ONLY=1` which is deprecated in ROS 2 Jazzy.
* **Proposed Resolution**:
  * Update `devcontainer.json` and `main_dock.sh` to remove `ROS_LOCALHOST_ONLY` and set the modern Jazzy equivalents:
    ```bash
    export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
    ```

---

## 2. QoS Incompatibility on `/initialpose`
* **Warning Message**:
  ```text
  [rviz]: New subscription discovered on topic '/initialpose', requesting incompatible QoS. Last incompatible policy: RELIABILITY
  ```
* **Root Cause**: RViz2 publishes the 2D Pose Estimate topic `/initialpose` using a **Reliable** QoS profile with **Transient Local** durability, whereas `gym_bridge` subscribes with incompatible QoS parameters (likely Best Effort).
* **Proposed Resolution**:
  * Update the subscriber QoS profile in `gym_bridge` (or the publisher QoS in the RViz config file) to use compatible QoS settings (matching Reliability and Durability policies).

---

## 3. Qt Display `XDG_RUNTIME_DIR` Warning
* **Warning Message**:
  ```text
  QStandardPaths: XDG_RUNTIME_DIR not set, defaulting to '/tmp/runtime-mohany'
  ```
* **Root Cause**: The `$XDG_RUNTIME_DIR` environment variable is not defined inside the Docker container.
* **Proposed Resolution**:
  * Set a default `XDG_RUNTIME_DIR` inside the container by adding it to `devcontainer.json` environment settings:
    ```json
    "containerEnv": {
        "XDG_RUNTIME_DIR": "/tmp"
    }
    ```

---

## 4. RViz2 OpenGL/GLSL Shader Linking Error
* **Warning Message**:
  ```text
  [rviz2-1] [ERROR] [rviz2]: rviz/glsl120/indexed_8bit_image.vert
  [rviz2-1] rviz/glsl120/indexed_8bit_image.frag
  [rviz2-1] GLSL link result : active samplers with a different type refer to the same texture image unit
  ```
* **Root Cause**: A shader compilation issue inside RViz2 when rendering 8-bit grid maps using virtualized GPU acceleration drivers under Mesa inside Docker.
* **Proposed Resolution**:
  * Investigate upgrading the Mesa OpenGL libraries in the Dockerfile, or configure RViz2 rendering parameters to use a compatible texture sampler mode.

---

## 5. TF Extrapolation Warnings in Head-to-Head (`detect` node)
* **Warning Message**:
  ```text
  [detect-4] [WARN] [detect]: Could not transform between 'map' and 'car_state/laser': Lookup would require extrapolation into the future.
  ```
* **Root Cause**: The `detect` (opponent detection) node is attempting to lookup the transform between the coordinate frames `car_state/laser` and `map` at a specific timestamp that is slightly ahead of the incoming TF buffer (usually due to publication frequency latency or `/clock` simulation time sync delays).
* **Proposed Resolution**:
  * Implement a slight TF buffer lookup timeout (e.g., `tf_buffer.lookup_transform(..., rclcpp::Duration::from_seconds(0.1))`) in the `detect` node code to allow the buffer to receive the latest transforms before failing.
  * Ensure the TF publisher (odometry or simulator bridge) is publishing transforms at a sufficiently high rate.
